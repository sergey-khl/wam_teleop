import matplotlib.pyplot as plt
import os
import sys
import numpy as np
import pandas as pd


def read_config(file_path):
    kinematics_vars = []
    dynamics_vars = []

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith("Kinematics data:"):
                kinematics_vars = line.split(":")[1].strip().split(", ")
            elif line.startswith("Dynamics data:"):
                dynamics_vars = line.split(":")[1].strip().split(", ")

    return kinematics_vars, dynamics_vars

def read_data(file_path, variable_names, dof=4):
    data_dict = {name: [] for name in variable_names}

    with open(file_path, 'r') as file:
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

def plot_all_vars(kinematics_data, dynamics_data, folder_name, dof=4):
    total_plots = (1 + len(dynamics_data) - 1) * dof  # 1 combined position plot + other dynamics
    cols = 4
    rows = (total_plots + cols - 1) // cols
    plt.figure(figsize=(16, 3 * rows))

    plot_idx = 1

    # Plot desired and feedback joint positions together
    for j in range(dof):
        plt.subplot(rows, cols, plot_idx)
        plt.plot(kinematics_data['time'], kinematics_data['desired joint pos'][:, j], label="Desired", linestyle='--')
        plt.plot(kinematics_data['time'], kinematics_data['feedback joint pos'][:, j], label="Feedback")
        plt.title(f"Joint Position [Joint {j+1}]")
        plt.xlabel("Time (s)")
        plt.ylabel("Position")
        plt.legend()
        plot_idx += 1

    # Plot remaining dynamics data
    # Plot leader and follower external torques together
    if 'leader external torque' in dynamics_data and 'follower external torque' in dynamics_data:
        for j in range(dof):
            plt.subplot(rows, cols, plot_idx)
            plt.plot(dynamics_data['time'], dynamics_data['leader external torque'][:, j], label="Leader", linestyle='--')
            plt.plot(dynamics_data['time'], -dynamics_data['follower external torque'][:, j], label="Follower")
            plt.title(f"External Torque [Joint {j+1}]")
            plt.xlabel("Time (s)")
            plt.ylabel("Torque (Nm)")
            plt.legend()
            plot_idx += 1

    # Plot any remaining dynamics variables except the two already plotted
    for var, data in dynamics_data.items():
        if var in ['time', 'leader external torque', 'follower external torque']:
            continue
        for j in range(dof):
            plt.subplot(rows, cols, plot_idx)
            plt.plot(dynamics_data['time'], data[:, j])
            plt.title(f"{var} [Joint {j+1}]")
            plt.xlabel("Time (s)")
            plt.ylabel(var)
            plot_idx += 1


    plt.suptitle(f"{folder_name}")
    plt.subplots_adjust(top=0.95)
    plt.tight_layout()
    plt.show()

def calculate_nrmse(desired, feedback, normalization='mean'):
    rmse = np.sqrt(np.mean((desired - feedback) ** 2))
    if normalization == 'range':
        range_desired = np.max(desired) - np.min(desired)
        nrmse = rmse / range_desired if range_desired != 0 else float('inf')
    elif normalization == 'mean':
        mean_desired = np.mean(desired)
        nrmse = rmse / mean_desired if mean_desired != 0 else float('inf')
    else:
        raise ValueError("Normalization method must be 'range' or 'mean'.")
    return nrmse

def calculate_rmse(desired, feedback):
    """
    Calculate the Root Mean Square Error (RMSE) between desired and feedback signals.
    
    Parameters:
        desired (array-like): Desired values
        feedback (array-like): Actual/feedback values
    
    Returns:
        float: RMSE value
    """
    rmse = np.sqrt(np.mean((np.array(desired) - np.array(feedback)) ** 2))
    return rmse


def calculate_rms(signal):
    return np.sqrt(np.mean(np.square(signal)))

def calculate_errors(kinematics_data, dynamics_data, dof=4):
    pos_nrmse_list = []
    ext_torque_nrmse_list = []
    pos_rms_list = []
    leader_ext_rms_list = []
    follower_ext_rms_list = []
    leader_impedance_list = []

    for joint in range(dof):
        pos_des = kinematics_data['desired joint pos'][:, joint]
        pos_feedback = kinematics_data['feedback joint pos'][:, joint]
        leader_ext_torque = dynamics_data['leader external torque'][:, joint]
        follower_ext_torque = dynamics_data['follower external torque'][:, joint]
        ext_torque_nrmse = calculate_nrmse(leader_ext_torque, -follower_ext_torque)

        pos_nrmse = calculate_nrmse(pos_des, pos_feedback)
        pos_rmse = calculate_rmse(pos_des, pos_feedback)
        pos_rms = calculate_rms(pos_feedback)
        leader_ext_rms = calculate_rms(leader_ext_torque)
        follower_ext_rms = calculate_rms(follower_ext_torque)
        leader_impedance = leader_ext_rms / pos_rmse if pos_rms != 0 else float('inf')

        pos_nrmse_list.append(pos_nrmse)
        ext_torque_nrmse_list.append(ext_torque_nrmse)
        pos_rms_list.append(pos_rms)
        leader_ext_rms_list.append(leader_ext_rms)
        follower_ext_rms_list.append(follower_ext_rms)
        leader_impedance_list.append(leader_impedance)

    print("---- Leader Impedance ----")
    for joint, value in enumerate(leader_impedance_list):
        print(f"  Joint {joint+1}: {value:.4f}")

    print("\n---- nRMSE (Position) ----")
    for joint, value in enumerate(pos_nrmse_list):
        print(f"  Joint {joint+1}: {value:.4f}")

    print("\n---- nRMSE (External Torque) ----")
    for joint, value in enumerate(ext_torque_nrmse_list):
        print(f"  Joint {joint+1}: {value:.4f}")

    print("\n---- RMS Position ----")
    for joint, value in enumerate(pos_rms_list):
        print(f"  Joint {joint+1}: {value:.4f}")

    print("\n---- RMS Leader External Torque ----")
    for joint, value in enumerate(leader_ext_rms_list):
        print(f"  Joint {joint+1}: {value:.4f}")

    print("\n---- RMS Follower External Torque ----")
    for joint, value in enumerate(follower_ext_rms_list):
        print(f"  Joint {joint+1}: {value:.4f}")

def export_metrics_to_csv(kinematics_data, dynamics_data, out_path, joints=(1, 3)):
    rows = []
    for j in joints:
        pos_des = kinematics_data['desired joint pos'][:, j]
        pos_feedback = kinematics_data['feedback joint pos'][:, j]
        leader_ext_torque = dynamics_data['leader external torque'][:, j]
        follower_ext_torque = dynamics_data['follower external torque'][:, j]

        pos_nrmse = calculate_nrmse(pos_des, pos_feedback)
        ext_torque_nrmse = calculate_nrmse(leader_ext_torque, -follower_ext_torque)
        pos_rmse = calculate_rmse(pos_des, pos_feedback)
        leader_ext_rms = calculate_rms(leader_ext_torque)
        leader_impedance = leader_ext_rms / pos_rmse if pos_rmse != 0 else float('inf')

        rows.append({
            "Joint": f"Joint {j+1}",
            "nRMSE (position)": float(pos_nrmse),
            "nRMSE (external torque)": float(ext_torque_nrmse),
            "Leader impedance": float(leader_impedance),
        })

    df = pd.DataFrame(rows, columns=["Joint", "nRMSE (position)", "nRMSE (external torque)", "Leader impedance"])
    df.to_csv(out_path, index=False)



# def main(folder_name):
#     base_folder = '../data'
#     folder_path = os.path.join(base_folder, folder_name)

#     config_file = os.path.join(folder_path, 'config.txt')
#     kinematics_file = os.path.join(folder_path, 'kinematics.txt')
#     dynamics_file = os.path.join(folder_path, 'dynamics.txt')

#     kinematics_vars, dynamics_vars = read_config(config_file)
#     kinematics_data = read_data(kinematics_file, kinematics_vars, dof=4)
#     dynamics_data = read_data(dynamics_file, dynamics_vars, dof=4)

#     plot_all_vars(kinematics_data, dynamics_data, folder_name, dof=4)
#     calculate_errors(kinematics_data, dynamics_data, dof=4)

def main(folder_name):
    base_folder = '../../../data'
    folder_path = os.path.join(base_folder, folder_name)

    config_file = os.path.join(folder_path, 'config.txt')
    kinematics_file = os.path.join(folder_path, 'kinematics.txt')
    dynamics_file = os.path.join(folder_path, 'dynamics.txt')

    kinematics_vars, dynamics_vars = read_config(config_file)
    kinematics_data = read_data(kinematics_file, kinematics_vars, dof=4)
    dynamics_data = read_data(dynamics_file, dynamics_vars, dof=4)

    plot_all_vars(kinematics_data, dynamics_data, folder_name, dof=4)
    calculate_errors(kinematics_data, dynamics_data, dof=4)

    # NEW: export Excel with Joint 2 & Joint 4 metrics
    out_csv = os.path.join(folder_path, f"{folder_name}_metrics.csv")
    export_metrics_to_csv(kinematics_data, dynamics_data, out_csv, joints=(1, 3))
    print(f"Saved metrics to: {out_csv}")



if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 plot.py <folder_name>")
    else:
        main(sys.argv[1])
