import os
import sys
import numpy as np
import matplotlib.pyplot as plt

LB_TO_N = 4.44822
FORCE_FRAME = "base"   # "base" or "tool"

SIGN_LEADER = +1.0
SIGN_FOLLOWER = -1.0

TOOL_OFFSET_FOLLOWER = 0.56 #0.36 (without any tool on the follower) # meters
TOOL_OFFSET_LEADER   = 0.57  # meters

# Low-pass cutoff for logged external torques (Hz)
CUTOFF_HZ = 5.0


# ---------------------------
# RC low-pass filter
# ---------------------------
def _rc_alpha(dt, cutoff_hz):
    if cutoff_hz <= 0:
        return 1.0
    tau = 1.0 / (2.0 * np.pi * cutoff_hz)
    return dt / (tau + dt)

def lowpass_rc_signal(x, t, cutoff_hz):
    x = np.asarray(x, dtype=float)
    t = np.asarray(t, dtype=float)
    if x.size == 0:
        return x
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        dt = max(t[i] - t[i - 1], 1e-12)
        alpha = _rc_alpha(dt, cutoff_hz)
        y[i] = y[i - 1] + alpha * (x[i] - y[i - 1])
    return y

def smooth_external_torques(dyn, cutoff_hz=5.0):
    if "time" not in dyn:
        raise ValueError("dynamics_data must contain 'time'")
    t = dyn["time"]
    for key in ["leader external torque", "follower external torque"]:
        if key in dyn:
            raw = dyn[key]
            filt = np.zeros_like(raw, dtype=float)
            for j in range(raw.shape[1]):
                filt[:, j] = lowpass_rc_signal(raw[:, j], t, cutoff_hz)
            dyn[f"{key} (filtered)"] = filt
    return dyn


# -------------------------------------------------
# nRMSE (mean normalization) + truncate
# -------------------------------------------------
def calculate_nrmse(predicted, logged):
    predicted = np.asarray(predicted)
    logged = np.asarray(logged)
    N = min(predicted.shape[0], logged.shape[0])
    predicted = predicted[:N]
    logged = logged[:N]

    rmse = np.sqrt(np.mean((predicted - logged) ** 2))
    mean_val = np.mean(logged)
    return rmse / mean_val if mean_val != 0 else float("inf"), N


# -------------------------------------------------
# Log reader helpers
# -------------------------------------------------
def read_config(file_path):
    kinematics_vars = []
    dynamics_vars = []

    with open(file_path, "r") as file:
        for line in file:
            line = line.strip()
            if line.startswith("Kinematics data:"):
                kinematics_vars = line.split(":")[1].strip().split(", ")
            elif line.startswith("Dynamics data:"):
                dynamics_vars = line.split(":")[1].strip().split(", ")

    return kinematics_vars, dynamics_vars


def read_data(file_path, variable_names, dof=4):
    data_dict = {name: [] for name in variable_names}

    with open(file_path, "r") as file:
        for line in file:
            values = list(map(float, line.strip().split(",")))
            data_dict[variable_names[0]].append(values[0])  # time

            idx = 1
            for var in variable_names[1:]:
                if idx + dof <= len(values):
                    data_dict[var].append(values[idx:idx + dof])
                    idx += dof

    for key in data_dict:
        data_dict[key] = np.array(data_dict[key])
    return data_dict


# -------------------------------------------------
# 4-DOF WAM DH
# -------------------------------------------------
ALPHA = np.array([-np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2])
A = np.array([0.0, 0.0, 0.045, -0.045])
D = np.array([0.0, 0.0, 0.55, 0.0])


def rotz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]], dtype=float)


def rotx(alpha):
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([[1, 0,  0, 0],
                     [0, c, -s, 0],
                     [0, s,  c, 0],
                     [0, 0,  0, 1]], dtype=float)


def transz(d):
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, d],
                     [0, 0, 0, 1]], dtype=float)


def transx(a):
    return np.array([[1, 0, 0, a],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)


def dh_T(theta, d, a, alpha):
    return rotz(theta) @ transz(d) @ transx(a) @ rotx(alpha)


def tool_T_z(L):
    T = np.eye(4)
    T[2, 3] = L
    return T


def fk_and_jacobian_wam4(q, tool_offset):
    """
    Jacobian for the point at frame4 origin + tool_offset along frame4 z-axis.
    Returns:
      T_0e (base->tool point), J (6x4) in base frame
    """
    q = np.asarray(q, dtype=float).reshape(4,)
    T = np.eye(4)

    o_list = [T[:3, 3].copy()]
    z_list = [T[:3, 2].copy()]

    for i in range(4):
        T = T @ dh_T(q[i], D[i], A[i], ALPHA[i])
        o_list.append(T[:3, 3].copy())
        z_list.append(T[:3, 2].copy())

    # Add fixed tool offset
    T_0e = T @ tool_T_z(tool_offset)
    o_e = T_0e[:3, 3]

    Jv = np.zeros((3, 4))
    Jw = np.zeros((3, 4))
    for i in range(4):
        z = z_list[i]
        o = o_list[i]
        Jv[:, i] = np.cross(z, (o_e - o))
        Jw[:, i] = z

    J = np.vstack([Jv, Jw])
    return T_0e, J


def tau_ext_from_fz(q, fz_newtons, tool_offset):
    T_0e, J = fk_and_jacobian_wam4(q, tool_offset=tool_offset)

    if FORCE_FRAME == "base":
        F_base = np.array([0.0, 0.0, fz_newtons])
    elif FORCE_FRAME == "tool":
        R = T_0e[:3, :3]
        F_base = R @ np.array([0.0, 0.0, fz_newtons])
    else:
        raise ValueError("FORCE_FRAME must be 'base' or 'tool'")

    w = np.zeros(6)
    w[:3] = F_base
    return J.T @ w


# -------------------------------------------------
# MAIN
# -------------------------------------------------
def main(folder_name, fz_leader_lb, fz_follower_lb_init, fz_follower_lb_final):
    folder_path = os.path.join("../../../data", folder_name)

    config_file = os.path.join(folder_path, "config.txt")
    kin_file = os.path.join(folder_path, "kinematics.txt")
    dyn_file = os.path.join(folder_path, "dynamics.txt")

    kin_vars, dyn_vars = read_config(config_file)
    kin = read_data(kin_file, kin_vars, dof=4)
    dyn = read_data(dyn_file, dyn_vars, dof=4)

    # Filter logged external torques (adds "... (filtered)" keys)
    dyn = smooth_external_torques(dyn, cutoff_hz=CUTOFF_HZ)

    # desired joint pos -> follower, feedback joint pos -> leader
    q_follower = kin["desired joint pos"]
    q_leader = kin["feedback joint pos"]
    t = kin["time"]

    # Forces
    fz_leader_N = SIGN_LEADER * float(fz_leader_lb) * LB_TO_N
    fz_follower_lb_avg = 0.5 * (float(fz_follower_lb_init) + float(fz_follower_lb_final))
    fz_follower_N = SIGN_FOLLOWER * fz_follower_lb_avg * LB_TO_N

    Nk = q_follower.shape[0]
    tau_pred_leader = np.zeros((Nk, 4))
    tau_pred_follower = np.zeros((Nk, 4))

    for k in range(Nk):
        tau_pred_leader[k] = tau_ext_from_fz(q_leader[k], fz_leader_N, tool_offset=TOOL_OFFSET_LEADER)
        tau_pred_follower[k] = tau_ext_from_fz(q_follower[k], fz_follower_N, tool_offset=TOOL_OFFSET_FOLLOWER)

    # Prefer filtered logs if present
    leader_key = "leader external torque (filtered)" if "leader external torque (filtered)" in dyn else "leader external torque"
    follower_key = "follower external torque (filtered)" if "follower external torque (filtered)" in dyn else "follower external torque"

    have_leader = leader_key in dyn
    have_follower = follower_key in dyn

    # nRMSE
    print(f"\n==== nRMSE (Predicted vs Logged External Torque) [cutoff={CUTOFF_HZ} Hz] ====")
    for j in range(4):
        if have_leader:
            nrmse_leader, Nl = calculate_nrmse(
                tau_pred_leader[:, j],
                dyn[leader_key][:, j]
            )
            print(f"Leader Joint {j+1}: {nrmse_leader:.4f}  (N={Nl})")

        if have_follower:
            nrmse_follower, Nf = calculate_nrmse(
                tau_pred_follower[:, j],
                dyn[follower_key][:, j]
            )
            print(f"Follower Joint {j+1}: {nrmse_follower:.4f}  (N={Nf})")

    # Plot
    plt.figure(figsize=(14, 10))
    for j in range(4):
        plt.subplot(2, 2, j + 1)

        plt.plot(t, tau_pred_leader[:, j], "--",
                 label=f"Pred Leader (offset {TOOL_OFFSET_LEADER} m)")
        plt.plot(t, tau_pred_follower[:, j],
                 label=f"Pred Follower (offset {TOOL_OFFSET_FOLLOWER} m)")

        if have_leader:
            Nl = min(len(t), dyn[leader_key].shape[0])
            plt.plot(t[:Nl], dyn[leader_key][:Nl, j],
                     alpha=0.85, label=f"Leader Log ({'filt' if 'filtered' in leader_key else 'raw'})")

        if have_follower:
            Nf = min(len(t), dyn[follower_key].shape[0])
            plt.plot(t[:Nf], dyn[follower_key][:Nf, j],
                     alpha=0.85, label=f"Follower Log ({'filt' if 'filtered' in follower_key else 'raw'})")

        plt.title(f"Joint {j+1}")
        plt.xlabel("Time (s)")
        plt.ylabel("Torque (Nm)")
        plt.legend()

    plt.suptitle(
        f"{folder_name} | Fz_leader={fz_leader_lb} lb | "
        f"Fz_follower_avg={fz_follower_lb_avg:.2f} lb (signed) | frame={FORCE_FRAME}"
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python3 plot_contact_force.py <folder_name> <Fz_leader_lb> <Fz_follower_lb_init> <Fz_follower_lb_final>")
        print("Example: python3 plot_contact_force.py u1_contact_force_3lb_1 3.0 2.5 3.5")
        sys.exit(1)

    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
