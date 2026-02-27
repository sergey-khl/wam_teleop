#!/usr/bin/env python3
import matplotlib.pyplot as plt
import os
import sys
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.gridspec import GridSpec
import glob


# ---------------------------
# Paper-friendly styling
# ---------------------------
plt.rcParams.update({
    "savefig.format": "pdf",   # vector
    "font.family": "serif",
    "font.serif": ["Times", "Times New Roman", "DejaVu Serif", "STIXGeneral"],
    "axes.titlesize": 8,     # column titles
    "axes.labelsize": 5,     # (used for global row labels)
    "xtick.labelsize": 4.0,
    "ytick.labelsize": 4.0,
    "legend.fontsize": 4,
    "lines.linewidth": 0.9,  # thin lines
    "lines.markersize": 3,
})

# ---------------------------
# Shaded time intervals
# ---------------------------
# Use either:
#   - a single list of (start, end) tuples (applied to all columns), e.g. [(13, 23)]
#   - OR a list of four lists for per-column windows, e.g. [[(13,23)], [(5,12)], [], [(20,25),(30,35)]]
# SHADED_WINDOWS = [
#     [(20, 30)],          # column 0
#     [(20, 31)],          # column 1
#     [(20, 30)],          # column 2
#     [(20, 31)]           # column 3
# ]  # this was for u3

# SHADED_WINDOWS = [
#     [(21.40, 32.21)],     # column 0
#     [(20.85, 33.11)],     # column 1
#     [(21.19, 33.50)],     # column 2
#     [(21.38, 30.62)]      # column 3
# ]  # this was for u4 // this was for compression

SHADED_WINDOWS = [
    [(0.0, 0.0)],     # column 0
    [(0.0, 0.0)],     # column 1
    [(0.0, 0.0)],     # column 2
    [(0.0, 0.0)]      # column 3
]  # this was for u4

# Will be computed after cropping & time-shift:
SHADED_WINDOWS_EFF = None

SHADE_COLOR = "gray"
SHADE_ALPHA = 0.25

# ---------------------------
# I/O helpers
# ---------------------------
def read_config(file_path):
    kinematics_vars, dynamics_vars = [], []
    with open(file_path, 'r') as f:
        for line in f:
            s = line.strip()
            if s.startswith("Kinematics data:"):
                kinematics_vars = s.split(":")[1].strip().split(", ")
            elif s.startswith("Dynamics data:"):
                dynamics_vars = s.split(":")[1].strip().split(", ")
    return kinematics_vars, dynamics_vars

def read_data(file_path, variable_names, dof=4):
    data = {name: [] for name in variable_names}
    with open(file_path, 'r') as f:
        for line in f:
            vals = list(map(float, line.strip().split(",")))
            data[variable_names[0]].append(vals[0])  # time
            idx = 1
            for var in variable_names[1:]:
                if idx + dof <= len(vals):
                    data[var].append(vals[idx:idx + dof])
                    idx += dof
    for k in data:
        data[k] = np.array(data[k])
    return data

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
        dt = max(t[i] - t[i-1], 1e-12)
        alpha = _rc_alpha(dt, cutoff_hz)
        y[i] = y[i-1] + alpha * (x[i] - y[i-1])
    return y

def smooth_external_torques(dyn, cutoff_hz=5.0):
    if 'time' not in dyn:
        raise ValueError("dynamics_data must contain 'time'")
    t = dyn['time']
    for key in ['leader external torque', 'follower external torque']:
        if key in dyn:
            raw = dyn[key]
            filt = np.zeros_like(raw, dtype=float)
            for j in range(raw.shape[1]):
                filt[:, j] = lowpass_rc_signal(raw[:, j], t, cutoff_hz)
            dyn[f"{key} (filtered)"] = filt
    return dyn

# ---------------------------
# Plot helpers
# ---------------------------
def shade_intervals(ax, intervals):
    """
    Accepts either:
      - list of (start, end) tuples (applies to all columns), or
      - list of 4 lists (per-column windows); uses _CURRENT_COL to select.
    """
    col_intervals = intervals
    try:
        if intervals and not isinstance(intervals[0], tuple):
            col = globals().get("_CURRENT_COL", 0)
            if 0 <= col < len(intervals):
                col_intervals = intervals[col]
            else:
                col_intervals = []
    except Exception:
        col_intervals = intervals
    for start, end in col_intervals:
        ax.axvspan(start, end, color=SHADE_COLOR, alpha=SHADE_ALPHA, linewidth=0)

def plot_column(axs_col, kin, dyn, joints=(1, 3),
                show_leg_row1=False, show_leg_row3=False,
                show_leg_row4=False, show_leg_row6=False,
                ylims=None):
    j2, j4 = joints
    t_kin = kin['time']
    t_dyn = dyn['time']

    leader_key   = 'leader external torque (filtered)'   if 'leader external torque (filtered)'   in dyn else 'leader external torque'
    follower_key = 'follower external torque (filtered)' if 'follower external torque (filtered)' in dyn else 'follower external torque'
    has_torque = (leader_key in dyn) and (follower_key in dyn)

    # Row 0: J2 positions
    ax = axs_col[0]
    ax.plot(t_kin, kin['desired joint pos'][:, j2], '-',  color='red',  label='Follower')
    ax.plot(t_kin, kin['feedback joint pos'][:, j2], '--', color='blue', label='Leader')
    shade_intervals(ax, SHADED_WINDOWS_EFF)
    if ylims is not None: ax.set_ylim(ylims[0])
    if show_leg_row1:
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.20), ncol=2, frameon=False)

    # Row 1: J4 positions
    ax = axs_col[1]
    ax.plot(t_kin, kin['desired joint pos'][:, j4], '-',  color='red')
    ax.plot(t_kin, kin['feedback joint pos'][:, j4], '--', color='blue')
    shade_intervals(ax, SHADED_WINDOWS_EFF)
    if ylims is not None: ax.set_ylim(ylims[1])

    # Row 2: Position errors
    pos_err_j2 = kin['desired joint pos'][:, j2] - kin['feedback joint pos'][:, j2]
    pos_err_j4 = kin['desired joint pos'][:, j4] - kin['feedback joint pos'][:, j4]
    ax = axs_col[2]
    ax.plot(t_kin, pos_err_j2, '-',  color='red',  label='Joint 2')
    ax.plot(t_kin, pos_err_j4, '--', color='blue', label='Joint 4')
    shade_intervals(ax, SHADED_WINDOWS_EFF)
    if ylims is not None: ax.set_ylim(ylims[2])
    if show_leg_row3:
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.20), ncol=2, frameon=False)

    # Rows 3–5: Torques
    if has_torque:
        # Row 3: J2 torque
        ax = axs_col[3]
        ax.plot(t_dyn, -dyn[follower_key][:, j2], '-',  color='red',  label='Follower')
        ax.plot(t_dyn,  dyn[leader_key][:, j2],  '--', color='blue', label='Leader')
        shade_intervals(ax, SHADED_WINDOWS_EFF)
        if ylims is not None: ax.set_ylim(ylims[3])
        if show_leg_row4:
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.20), ncol=2, frameon=False)

        # Row 4: J4 torque
        ax = axs_col[4]
        ax.plot(t_dyn, -dyn[follower_key][:, j4], '-',  color='red')
        ax.plot(t_dyn,  dyn[leader_key][:, j4],  '--', color='blue')
        shade_intervals(ax, SHADED_WINDOWS_EFF)
        if ylims is not None: ax.set_ylim(ylims[4])

        # Row 5: Torque error
        torque_err_j2 = dyn[leader_key][:, j2] - (-dyn[follower_key][:, j2])
        torque_err_j4 = dyn[leader_key][:, j4] - (-dyn[follower_key][:, j4])
        ax = axs_col[5]
        ax.plot(t_dyn, torque_err_j2, '-',  color='red',  label='Joint 2')
        ax.plot(t_dyn, torque_err_j4, '--', color='blue', label='Joint 4')
        shade_intervals(ax, SHADED_WINDOWS_EFF)
        if ylims is not None: ax.set_ylim(ylims[5])
        ax.set_xlabel('Time (s)')
        if show_leg_row6:
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.20), ncol=2, frameon=False)
    else:
        for idx in range(3, 6):
            axs_col[idx].axis('off')
            axs_col[idx].text(0.5, 0.5, 'No torque data', ha='center', va='center',
                              transform=axs_col[idx].transAxes)

# ---------------------------
# Utilities for cropping, time shift, y-lims, filenames
# ---------------------------
def next_output_filename(base_dir="../data", prefix="compression_", ext=".pdf"):
    os.makedirs(base_dir, exist_ok=True)  # ensure folder exists
    pattern = os.path.join(base_dir, f"{prefix}*.pdf")
    files = glob.glob(pattern)
    numbers = []
    for f in files:
        name = os.path.basename(f)
        try:
            num = int(name.replace(prefix, "").replace(ext, ""))
            numbers.append(num)
        except ValueError:
            continue
    next_num = max(numbers, default=0) + 1
    return os.path.join(base_dir, f"{prefix}{next_num}{ext}")

def crop_dataset(kin, dyn, tmin=0, tmax=50):
    mask_kin = (kin['time'] >= tmin) & (kin['time'] <= tmax)
    for k in kin:
        kin[k] = kin[k][mask_kin]

    mask_dyn = (dyn['time'] >= tmin) & (dyn['time'] <= tmax)
    for k in dyn:
        dyn[k] = dyn[k][mask_dyn]
    return kin, dyn

def shift_time_zero(kin, dyn):
    """Shift time so the cropped window starts at t=0 (per column)."""
    if len(kin['time']) > 0:
        t0k = kin['time'][0]
        kin['time'] = kin['time'] - t0k
    if len(dyn['time']) > 0:
        t0d = dyn['time'][0]
        dyn['time'] = dyn['time'] - t0d
    return kin, dyn

def make_effective_shade_windows(shaded, t0):
    """
    Subtract t0 from shaded windows.
    - shaded can be a list of tuples or a list of lists (per column)
    - t0 can be a scalar (same for all) or a list of 4 t0s (per column)
    """
    if not shaded:
        return shaded
    # If per-column lists:
    if shaded and not isinstance(shaded[0], tuple):
        out = []
        for c in range(len(shaded)):
            col = shaded[c]
            t0c = t0 if np.isscalar(t0) else t0[c]
            out.append([(s - t0c, e - t0c) for (s, e) in col])
        return out
    # Single list (apply same offset)
    t0s = t0 if np.isscalar(t0) else t0[0]
    return [(s - t0s, e - t0s) for (s, e) in shaded]

def _pad_limits(ymin, ymax, frac=0.05):
    if np.isfinite(ymin) and np.isfinite(ymax) and ymax > ymin:
        pad = frac * (ymax - ymin)
        return ymin - pad, ymax + pad
    return ymin, ymax

def compute_row_ylims(loaded, joints=(1,3)):
    """
    loaded: list of 4 tuples (kin, dyn) AFTER cropping and time shifting.
    Returns list of 6 (ymin, ymax) tuples for rows 0..5.
    """
    j2, j4 = joints
    mins = [np.inf]*6
    maxs = [-np.inf]*6

    for (kin, dyn) in loaded:
        # Rows 0,1: positions
        if 'desired joint pos' in kin and 'feedback joint pos' in kin:
            # Row 0 (J2 pos)
            y = np.concatenate([kin['desired joint pos'][:, j2], kin['feedback joint pos'][:, j2]])
            mins[0] = min(mins[0], np.nanmin(y))
            maxs[0] = max(maxs[0], np.nanmax(y))
            # Row 1 (J4 pos)
            y = np.concatenate([kin['desired joint pos'][:, j4], kin['feedback joint pos'][:, j4]])
            mins[1] = min(mins[1], np.nanmin(y))
            maxs[1] = max(maxs[1], np.nanmax(y))
            # Row 2 (pos error)
            e2 = kin['desired joint pos'][:, j2] - kin['feedback joint pos'][:, j2]
            e4 = kin['desired joint pos'][:, j4] - kin['feedback joint pos'][:, j4]
            y = np.concatenate([e2, e4])
            mins[2] = min(mins[2], np.nanmin(y))
            maxs[2] = max(maxs[2], np.nanmax(y))

        # Rows 3–5: torques
        leader_key   = 'leader external torque (filtered)'   if 'leader external torque (filtered)'   in dyn else 'leader external torque'
        follower_key = 'follower external torque (filtered)' if 'follower external torque (filtered)' in dyn else 'follower external torque'
        has_torque = (leader_key in dyn) and (follower_key in dyn)
        if has_torque:
            # Row 3 (J2 torque)
            y = np.concatenate([-dyn[follower_key][:, j2], dyn[leader_key][:, j2]])
            mins[3] = min(mins[3], np.nanmin(y))
            maxs[3] = max(maxs[3], np.nanmax(y))
            # Row 4 (J4 torque)
            y = np.concatenate([-dyn[follower_key][:, j4], dyn[leader_key][:, j4]])
            mins[4] = min(mins[4], np.nanmin(y))
            maxs[4] = max(maxs[4], np.nanmax(y))
            # Row 5 (torque error)
            te2 = dyn[leader_key][:, j2] - (-dyn[follower_key][:, j2])
            te4 = dyn[leader_key][:, j4] - (-dyn[follower_key][:, j4])
            y = np.concatenate([te2, te4])
            mins[5] = min(mins[5], np.nanmin(y))
            maxs[5] = max(maxs[5], np.nanmax(y))

    # Pad and return
    ylims = []
    for r in range(6):
        if np.isfinite(mins[r]) and np.isfinite(maxs[r]):
            ylims.append(_pad_limits(mins[r], maxs[r], frac=0.05))
        else:
            ylims.append((None, None))
    return ylims

# ---------------------------
# Main multi-column plot
# ---------------------------
def plot_four_icra(folders, cutoff_hz=5.0, joints=(1, 3), base_folder="../data",
                   fig_width_in=7.16, fig_height_in=6.0):
    """
    EXACT ICRA width (7.16 in), decreased height so panels are bigger than before but not too tall.
    """
    global SHADED_WINDOWS_EFF

    if len(folders) != 4:
        raise ValueError("Exactly four folder names/paths are required.")

    # Fixed column titles (requested)
    col_titles = ["2c-GravComp", "2c-DynComp", "4c-GravComp", "4c-DynComp"]

    # Resolve folder paths & load
    loaded = []
    t0_list_for_shading = []
    for f in folders:
        folder_path = f if os.path.isabs(f) else os.path.join(base_folder, f)
        kin_vars, dyn_vars = read_config(os.path.join(folder_path, "config.txt"))
        kin = read_data(os.path.join(folder_path, "kinematics.txt"), kin_vars, dof=4)
        dyn = read_data(os.path.join(folder_path, "dynamics.txt"),   dyn_vars, dof=4)

        # Crop to [10,35]
        # kin, dyn = crop_dataset(kin, dyn, 10, 35)
        # Record original start (used to shift shaded windows)
        t0_list_for_shading.append(kin['time'][0] if len(kin['time']) > 0 else 10.0)
        # Filter torques
        smooth_external_torques(dyn, cutoff_hz=float(cutoff_hz))
        # Shift time to start from 0
        kin, dyn = shift_time_zero(kin, dyn)

        loaded.append((kin, dyn))

    # Build per-column effective shaded windows after time shift
    SHADED_WINDOWS_EFF = make_effective_shade_windows(SHADED_WINDOWS, t0_list_for_shading)

    # Compute uniform y-limits per row across all columns
    ylims_per_row = compute_row_ylims(loaded, joints=joints)

    # Figure at EXACT width; controlled height
    fig = plt.figure(figsize=(fig_width_in, fig_height_in))

    # Grid + spacing (no tight/constrained layout)
    gs = GridSpec(nrows=6, ncols=4, figure=fig, wspace=0.30, hspace=0.50)

    # Build axes grid
    axes_grid = [[fig.add_subplot(gs[r, c]) for c in range(4)] for r in range(6)]

    # Plot each column
    for c in range(4):
        # Track current column for per-column shaded windows
        global _CURRENT_COL
        _CURRENT_COL = c

        axs_col = [axes_grid[r][c] for r in range(6)]
        kin, dyn = loaded[c]
        plot_column(
            axs_col, kin, dyn, joints=joints,
            show_leg_row1=(c == 0),  # legends only in first column
            show_leg_row3=(c == 0),
            show_leg_row4=(c == 0),
            show_leg_row6=(c == 0),
            ylims=ylims_per_row
        )
        axs_col[0].set_title(col_titles[c], pad=10)

    # ---------------- Global row labels (perfect vertical alignment) ----------------
    row_texts = [
        "Joint 2 Pos (rad)",
        "Joint 4 Pos (rad)",
        "Pos Error (rad)",
        "Joint 2 Ext. Torque (Nm)",
        "Joint 4 Ext. Torque (Nm)",
        "Ext. Torque Error (Nm)",
    ]
    # Remove per-axes ylabels to avoid squeezing panels
    for r in range(6):
        for c in range(4):
            axes_grid[r][c].set_ylabel("")

    # Manual margins: leave room on left for row labels AND group labels + lines
    fig.subplots_adjust(left=0.10, right=0.995, top=0.95, bottom=0.07, wspace=0.30, hspace=0.50)

    # After layout, compute positions for aligned row labels
    fig.canvas.draw()
    left_x = axes_grid[0][0].get_position().x0
    label_x = left_x - 0.040  # row labels x (tweak if needed)

    for r in range(6):
        bbox = axes_grid[r][0].get_position()
        ymid = 0.5 * (bbox.y0 + bbox.y1)
        fig.text(label_x, ymid, row_texts[r],
                 rotation=90, va="center", ha="right",
                 fontsize=plt.rcParams["axes.labelsize"])

    # ---------------- Left-side group labels + vertical lines (kept) ----------------
    # Top group (rows 1–3)
    fig.text(label_x - 0.055, 0.74, 'Position Tracking', va='center',
             rotation=90, fontsize=8)
    fig.add_artist(Line2D((label_x - 0.030, label_x - 0.030), (0.50, 0.98),
                          color='black', linewidth=0.9))
    # Bottom group (rows 4–6)
    fig.text(label_x - 0.055, 0.25, 'Force Tracking', va='center',
             rotation=90, fontsize=8)
    fig.add_artist(Line2D((label_x - 0.030, label_x - 0.030), (0.02, 0.48),
                          color='black', linewidth=0.9))

    # Save to ../data with auto-increment name
    out_name = next_output_filename("../data", prefix="compression_", ext=".pdf")
    fig.savefig(out_name)
    print(f"Saved figure to {out_name}")
    plt.show()

# ---------------------------
# CLI
# ---------------------------
def main():
    if not (5 <= len(sys.argv) <= 6):
        print("Usage: python3 plot_four_icra.py <folder1> <folder2> <folder3> <folder4> [cutoff_hz]")
        sys.exit(1)
    folders = sys.argv[1:5]
    cutoff = float(sys.argv[5]) if len(sys.argv) == 6 else 5.0
    plot_four_icra(
        folders,
        cutoff_hz=cutoff,
        joints=(1, 3),
        base_folder="../../../data",
        fig_width_in=7.16,   # EXACT ICRA width
        fig_height_in=6.0    # your current height
    )

if __name__ == "__main__":
    main()
