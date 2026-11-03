"""
Render motion.mp4 for a recording folder made by sim_arm.py.

    python make_video.py recordings/20261101_180000_move

The sim runs this in the background after each recording; run it by hand to
re-render. Needs matplotlib (pip install matplotlib) and ffmpeg
(sudo apt install ffmpeg).

Layout: 3D view (left), side view X-Z (top right), motor torques with the
effort limits dashed and a moving time cursor (bottom right).
"""
import sys
from pathlib import Path

import numpy as np

FPS_OUT = 30
COLORS = ["#7f7f7f", "#1f77b4", "#d62728", "#2ca02c", "#9467bd"]   # one per joint
MASS_COLORS = {"motor": "#ff8c1a", "extra": "#8c8c99", "payload": "#c8a165"}


EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]


def rover_corners(rover, pose=(0.0, 0.0, 0.0)):
    """Rover body corners in the world for a rover pose (x, y, yaw_rad).
    rover = (centre x, centre y, length, width, clearance, deck) in the rover frame
    (older recordings: (nose, length, width, clearance, deck))."""
    if len(rover) == 5:
        nose, length, width, clear, deck = rover
        cx, cy = nose - length / 2, 0.0
    else:
        cx, cy, length, width, clear, deck = rover
    xs, ys, zs = (cx - length / 2, cx + length / 2), (cy - width / 2, cy + width / 2), (clear, deck)
    c = np.array([[x, y, z] for x in xs for y in ys for z in zs])
    cy, sy = np.cos(pose[2]), np.sin(pose[2])
    R = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return c @ R.T + np.array([pose[0], pose[1], 0.0])


def obb_corners(c, R, h):
    s = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)
    return c + (s * h) @ R.T


def edge_path(corners):
    """One polyline (NaN-separated) through the 12 edges of a box."""
    out = []
    for a, b in EDGES:
        out += [corners[a], corners[b], [np.nan] * 3]
    return np.array(out)


def rover_box_edges(rover):
    c = rover_corners(rover)
    return [(c[a], c[b]) for a, b in EDGES]


def main(folder):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter

    folder = Path(folder)
    d = np.load(folder / "motion.npz")
    t, tau, effort, sat = d["t"], d["tau"], d["effort"], d["sat"]
    segs, pivots, masses, tcp = d["segs"], d["pivots"], d["masses"], d["tcp"]
    kinds, kg, names = list(d["mass_kinds"]), d["mass_kg"], list(d["joint_names"])
    rover, tt_h = d["rover"], float(d["turntable"][0])
    base_pos = d["base_pos"] if "base_pos" in d.files else np.tile([0.0, 0.0, tt_h], (len(t), 1))
    prismatic = d["prismatic"] if "prismatic" in d.files else np.zeros(len(names), bool)
    title = str(d["title"])
    coll_seg = d["coll_seg"] if "coll_seg" in d.files else np.zeros((len(t), segs.shape[1]), bool)
    coll_text = d["coll_text"] if "coll_text" in d.files else np.array([""] * len(t))
    rpose = d["rover_pose"] if "rover_pose" in d.files else np.zeros((len(t), 3))
    show_objs = bool(d["show_objs"]) if "show_objs" in d.files else False
    step = d["step"] if "step" in d.files else np.array([""] * len(t))
    if show_objs:
        obj_c, obj_R, obj_h, obj_col = d["obj_c"], d["obj_R"], d["obj_h"], d["obj_col"]

    # frames to draw at FPS_OUT
    times = np.arange(0.0, t[-1] + 1e-9, 1.0 / FPS_OUT)
    frames = np.searchsorted(t, times).clip(0, len(t) - 1)

    # scene limits from everything that moves + rover
    pts = [segs.reshape(-1, 3), tcp, masses.reshape(-1, 3) if masses.size else tcp]
    pts += [rover_corners(rover, rpose[i]) for i in (0, len(t) - 1)]
    if show_objs:
        for k in range(obj_c.shape[1]):
            pts += [obj_corners for obj_corners in (obb_corners(obj_c[i, k], obj_R[i, k], obj_h[k]) for i in (0, len(t) - 1))]
    pts = np.concatenate(pts)
    lo = np.minimum(pts.min(0), [np.inf, np.inf, 0.0])
    hi = pts.max(0)
    ctr, half = (lo + hi) / 2, (hi - lo).max() / 2 + 0.1

    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    axs = fig.add_subplot(2, 2, 2)
    axt = fig.add_subplot(2, 2, 4)
    fig.suptitle(f"{title}   ({folder.name})", fontsize=12)
    fig.subplots_adjust(left=0.03, right=0.97, top=0.9, bottom=0.08, hspace=0.45, wspace=0.15)

    # --- 3D scene: rover body (moves if the rover drives), task objects
    deck_h = rover[-1]
    rpath0 = edge_path(rover_corners(rover, rpose[0]))
    has_cop = "cop" in d.files
    if has_cop:
        wheels, cop, marg = d["wheels"], d["cop"], d["margin"]
        foot3, = ax3.plot([], [], [], color="#2ca02c", lw=1.5)
        cop3, = ax3.plot([], [], [], "o", color="#2ca02c", ms=7)
        cops, = axs.plot([], [], "^", color="#2ca02c", ms=8)
        tip_lbl = ax3.text2D(0.02, 0.85, "", transform=ax3.transAxes, fontsize=9)
    rover3, = ax3.plot(rpath0[:, 0], rpath0[:, 1], rpath0[:, 2], color="0.55", lw=1)
    objs3, objss = [], []
    if show_objs:
        for k in range(obj_c.shape[1]):
            col = tuple(obj_col[k])
            objs3.append(ax3.plot([], [], [], color=col, lw=1.2)[0])
            objss.append(axs.plot([], [], color=col, lw=1.0)[0])
    post3, = ax3.plot([0, 0], [0, 0], [rover[-1], tt_h], color="0.3", lw=4)     # mount post (moves on a slide)
    if prismatic.any():                                                      # rail on the deck
        y_lo, y_hi = base_pos[:, 1].min(), base_pos[:, 1].max()
        ax3.plot([0, 0], [min(y_lo, -0.3) - 0.1, max(y_hi, 0.3) + 0.1], [rover[-1]] * 2, color="0.35", lw=2)
    g = np.linspace(-half, half, 5)
    for v in g:
        ax3.plot([ctr[0] + v] * 2, [ctr[1] - half, ctr[1] + half], [0, 0], color="0.85", lw=0.6)
        ax3.plot([ctr[0] - half, ctr[0] + half], [ctr[1] + v] * 2, [0, 0], color="0.85", lw=0.6)
    ax3.set(xlim=(ctr[0] - half, ctr[0] + half), ylim=(ctr[1] - half, ctr[1] + half), zlim=(min(lo[2], 0.0), min(lo[2], 0.0) + 2 * half),
            xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")
    ax3.set_box_aspect((1, 1, 1))
    ax3.view_init(elev=20, azim=-60)

    # --- side view static scene
    rovers, = axs.plot(rpath0[:, 0], rpath0[:, 2], color="0.6", lw=1)
    axs.axhline(0, color="0.5", lw=1)
    zlo = min(lo[2], 0.0) - 0.05
    span = max(hi[0] - lo[0], hi[2] - zlo) + 0.2           # same scale on both axes
    xc = (lo[0] + hi[0]) / 2
    axs.set(xlim=(xc - span / 2, xc + span / 2), ylim=(zlo, hi[2] + 0.1),
            xlabel="X (m)", ylabel="Z (m)", title="Side view (X-Z)")
    axs.set_aspect("equal", adjustable="box")
    axs.grid(alpha=0.3)

    # --- torque plot
    for k, n in enumerate(names):
        if np.max(np.abs(tau[:, k])) < 0.5 and n in ("base_yaw", "base_slide", "wrist_roll"):
            continue  # skip idle base / yaw motors to keep the plot readable
        unit = "N" if prismatic[k] else "N*m"
        axt.plot(t, tau[:, k], color=COLORS[k], lw=1.6, label=f"{n} (limit {effort[k]:g} {unit})")
        axt.axhline(effort[k], color=COLORS[k], ls="--", lw=0.8, alpha=0.6)
        axt.axhline(-effort[k], color=COLORS[k], ls="--", lw=0.8, alpha=0.6)
        if sat[:, k].any():
            axt.plot(t[sat[:, k]], tau[sat[:, k], k], "x", color=COLORS[k], ms=4)
    ymax = max(np.abs(tau).max() * 1.2, 1.0)
    axt.set(xlim=(0, t[-1]), ylim=(-ymax, ymax), xlabel="time (s)", ylabel="torque (N*m) / slide force (N)",
            title="Motor torques (dashed = effort limit, x = saturated)")
    axt.legend(loc="best", fontsize=8, ncol=2)
    axt.grid(alpha=0.3)
    cursor = axt.axvline(0, color="k", lw=1)
    hit = np.array([bool(x) for x in coll_text])
    if hit.any():
        axt.fill_between(t, -ymax, ymax, where=hit, color="#d62728", alpha=0.12, step="mid", label="collision")
        axt.legend(loc="best", fontsize=8, ncol=2)

    # --- moving artists
    tube3 = [ax3.plot([], [], [], color="#1f5fbf", lw=5, solid_capstyle="round")[0] for _ in range(segs.shape[1])]
    piv3, = ax3.plot([], [], [], "o", color="#d62728", ms=6)
    tcp3, = ax3.plot([], [], [], "o", color="#1ab31a", ms=6)
    trail3, = ax3.plot([], [], [], color="#1ab31a", lw=1, alpha=0.7)
    sizes = 20 + 60 * np.asarray(kg, float)
    mcols = [MASS_COLORS.get(k, "#888888") for k in kinds]
    m0 = masses[0] if masses.size else np.zeros((0, 3))
    mass3 = ax3.scatter(m0[:, 0], m0[:, 1], m0[:, 2], s=sizes, c=mcols, edgecolors="k", depthshade=False)
    tubes = [axs.plot([], [], color="#1f5fbf", lw=4, solid_capstyle="round")[0] for _ in range(segs.shape[1])]
    pivs, = axs.plot([], [], "o", color="#d62728", ms=5)
    tcps, = axs.plot([], [], "o", color="#1ab31a", ms=5)
    trails, = axs.plot([], [], color="#1ab31a", lw=1, alpha=0.7)
    masss = axs.scatter(m0[:, 0], m0[:, 2], s=sizes, c=mcols, edgecolors="k", zorder=3)
    clock = ax3.text2D(0.02, 0.95, "", transform=ax3.transAxes)
    coll_lbl = fig.text(0.5, 0.93, "", ha="center", color="#d62728", fontsize=11, fontweight="bold")
    step_lbl = ax3.text2D(0.02, 0.90, "", transform=ax3.transAxes, fontsize=9)
    writer = FFMpegWriter(fps=FPS_OUT, bitrate=4000)
    out = folder / "motion.mp4"
    with writer.saving(fig, str(out), dpi=100):
        for n, i in enumerate(frames):
            bp = base_pos[i]
            post3.set_data_3d([bp[0], bp[0]], [bp[1], bp[1]], [rover[-1], bp[2]])
            rp = edge_path(rover_corners(rover, rpose[i]))
            rover3.set_data_3d(rp[:, 0], rp[:, 1], rp[:, 2])
            rovers.set_data(rp[:, 0], rp[:, 2])
            for k in range(len(objs3)):
                op = edge_path(obb_corners(obj_c[i, k], obj_R[i, k], obj_h[k]))
                objs3[k].set_data_3d(op[:, 0], op[:, 1], op[:, 2])
                objss[k].set_data(op[:, 0], op[:, 2])
            step_lbl.set_text(str(step[i]))
            if has_cop:
                cyw, syw = np.cos(rpose[i][2]), np.sin(rpose[i][2])
                Rz = np.array([[cyw, -syw, 0], [syw, cyw, 0], [0, 0, 1]])
                wp_ = wheels[[0, 1, 3, 2, 0]] @ Rz.T + np.array([rpose[i][0], rpose[i][1], 0.0])
                col = "#2ca02c" if marg[i] > 0.05 else "#ff8c1a" if marg[i] >= 0 else "#d62728"
                foot3.set_data_3d(wp_[:, 0], wp_[:, 1], wp_[:, 2])
                foot3.set_color(col)
                cop3.set_data_3d([cop[i, 0]], [cop[i, 1]], [0.0])
                cop3.set_color(col)
                cops.set_data([cop[i, 0]], [0.0])
                cops.set_color(col)
                tip_lbl.set_text(f"tip margin {marg[i]:+.2f} m")
                tip_lbl.set_color(col)
            for s, (a, b) in enumerate(segs[i]):
                col = "#d62728" if s < coll_seg.shape[1] and coll_seg[i, s] else "#1f5fbf"
                tube3[s].set_data_3d(*zip(a, b))
                tubes[s].set_data([a[0], b[0]], [a[2], b[2]])
                tube3[s].set_color(col)
                tubes[s].set_color(col)
            piv3.set_data_3d(pivots[i, :, 0], pivots[i, :, 1], pivots[i, :, 2])
            pivs.set_data(pivots[i, :, 0], pivots[i, :, 2])
            tcp3.set_data_3d([tcp[i, 0]], [tcp[i, 1]], [tcp[i, 2]])
            tcps.set_data([tcp[i, 0]], [tcp[i, 2]])
            trail3.set_data_3d(tcp[: i + 1, 0], tcp[: i + 1, 1], tcp[: i + 1, 2])
            trails.set_data(tcp[: i + 1, 0], tcp[: i + 1, 2])
            if masses.size:
                m = masses[i]
                mass3._offsets3d = (m[:, 0], m[:, 1], m[:, 2])
                masss.set_offsets(m[:, [0, 2]])
            cursor.set_xdata([t[i], t[i]])
            clock.set_text(f"t = {t[i]:.2f} s")
            coll_lbl.set_text(f"COLLISION: {coll_text[i]}" if coll_text[i] else "")
            writer.grab_frame()
    print(f"wrote {out}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    try:
        main(sys.argv[1])
    except ModuleNotFoundError as e:
        sys.exit(f"{e}. Install it with:  pip install matplotlib")
    except FileNotFoundError as e:
        sys.exit(f"{e}\nIs ffmpeg installed?  sudo apt install ffmpeg")
