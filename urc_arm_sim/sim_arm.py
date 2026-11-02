"""
Newton sim of the URC arm (3-DOF pitch arm on a sliding base, or a turntable).

    python sim_arm.py             # opens the viewer (builds the model from arm_params.py)

Side panel
  * Joints: sliders (relative deg); the arm moves with a smooth quintic move
    limited by each joint's speed.
  * Presets and TCP target + Solve IK.
  * Record move: set a start and an end pose, press Record. The arm goes to
    the start, settles, then performs the move while logging. The result is
    saved in recordings/<time>_<name>/ :
        torques.csv   joint angles, speeds and motor torques every frame (Excel)
        summary.txt   peak torque of each motor and when it happens
        motion.mp4    animation: 3D view, side view and torque plot (made in
                      the background by make_video.py; needs ffmpeg + matplotlib)
  * Design: move a pivot, motor or extra mass inside a box around its anchor
    (the box is drawn in the 3D view). The model rebuilds when you release a
    slider. "Save to arm_params.py" writes the changes back (backup .bak) and
    regenerates urc_arm.urdf.
  * Readout: torque each motor applies vs. the static model, against its
    effort limit. "SAT" = the motor is at its limit.

Physics: MuJoCo solver, position-controlled joints with velocity feed-forward
(DRIVE_KE / DRIVE_KD), effort limits from JOINTS. The reported torque is the
actuator force, so during a move it includes acceleration and Coriolis
effects. Contacts are OFF: the rover and ground are drawn, but the arm passes
through them.

Check without a window:
    python sim_arm.py --viewer null --test --num-frames 300
"""
import csv
import datetime as dt
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import warp as wp

import newton
import newton.examples
from newton import JointTargetMode

import arm_model as am
import build_urdf
import params_io

HERE = Path(__file__).resolve().parent
PARAMS_FILE = HERE / "arm_params.py"
BOX_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def quintic(u):
    u = min(max(u, 0.0), 1.0)
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def quintic_rate(u):
    if u <= 0.0 or u >= 1.0:
        return 0.0
    return 30 * u**2 - 60 * u**3 + 30 * u**4


class Example:
    def __init__(self, viewer, args):
        self.viewer = viewer
        sys.path.insert(0, str(HERE))
        self.P = am.load_params("arm_params")
        self.fps = 60
        self.frame_dt = 1.0 / self.fps
        self.sim_substeps = 10
        self.sim_dt = self.frame_dt / self.sim_substeps
        self.sim_time = 0.0
        self.status = "ready"

        q0 = np.array(am.home_q(self.P))
        self.cmd, self.cmd_qd = q0.copy(), np.zeros(5)
        self.move_from, self.move_to = q0.copy(), q0.copy()
        self.move_t, self.move_T = 0.0, 1.0
        self.names = [am.joint_name(self.P, j) for j in am.JOINT_ORDER]   # e.g. base_slide
        self.slider_val = self._disp(q0)

        self.check_coll = True        # collision checking (readout, red overlay, IK rejection)
        self._build_model(q0)

        nose = self.P.ROVER["mount_behind_front"]
        self.presets = self.P.PRESETS          # edit them in arm_params.py
        self.ik_target = [nose + 0.20, 0.0, 0.40, -45.0]
        self.elbow_up = True

        # recording
        self.rec_start = q0.copy()
        self.rec_end = np.radians([0, 20, -40, -70, 0])
        self.rec_name = "move"
        self.rec_phase = None          # None | "approach" | "record"
        self.rec_log = []
        self.rec_settle = 0.0
        self.last_recording = None

        # design editing
        self.design_open = False
        self.design_idx = 0
        self.dirty = False
        self.pending = None          # (item index, xyz) while a design slider is dragged
        self.pending_mass = None

    # ================================================================ model
    def _build_model(self, q_now):
        P = self.P
        xml = build_urdf.build(P)                   # raises ValueError on bad geometry
        builder = newton.ModelBuilder()
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
        builder.default_joint_cfg.armature = 0.01
        builder.add_urdf(xml, floating=False, enable_self_collisions=False, collapse_fixed_joints=False)

        dof = {}
        for ji, key in enumerate(builder.joint_label):
            name = key.split("/")[-1]
            internal = {am.joint_name(P, j): j for j in am.JOINT_ORDER}
            if name in internal:
                dof[internal[name]] = builder.joint_qd_start[ji]
        self.idx = np.array([dof[j] for j in am.JOINT_ORDER])
        for k, j in enumerate(am.JOINT_ORDER):
            d = dof[j]
            builder.joint_target_ke[d] = P.DRIVE_KE
            builder.joint_target_kd[d] = P.DRIVE_KD
            builder.joint_target_mode[d] = int(JointTargetMode.POSITION)
            builder.joint_q[d] = float(q_now[k])
            builder.joint_target_q[d] = float(q_now[k])
        builder.add_ground_plane()

        self.model = builder.finalize()
        self.solver = newton.solvers.SolverMuJoCo(self.model, disable_contacts=True)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_0)
        self.target_full = self.control.joint_target_q.numpy().copy()
        mj = self.solver.mj_model                    # MuJoCo DOF index of each of our joints
        self.mj_dof = np.array([mj.jnt_dofadr[next(i for i in range(mj.njnt) if mj.joint(i).name.endswith(am.joint_name(P, j)))]
                                for j in am.JOINT_ORDER])
        self.tau_frame = None

        self.parts = am.body_parts(P)
        self.coll_elems = am.collision_elements(P, self.parts)
        self.hits = []
        self.inertials = am.body_inertials(P, self.parts)
        lim = np.array([am.joint_limits(P, j) for j in am.JOINT_ORDER])   # SI: rad or m
        self.lo, self.hi, self.effort, self.vmax = lim[:, 0], lim[:, 1], lim[:, 2], lim[:, 3]
        self.viewer.set_model(self.model)

    def rebuild(self):
        """Rebuild after a design change, keeping the current pose."""
        q, _ = self.joint_state()
        self._build_model(q)
        self.cmd = q.copy()
        self.move_from = self.move_to = q.copy()
        self.move_t = self.move_T
        # The GL viewer drops example side-panel callbacks whenever a new model
        # is set, so register this panel again (otherwise the menu disappears).
        if hasattr(self.viewer, "register_ui_callback"):
            self.viewer.register_ui_callback(lambda ui, ex=self: ex.gui(ui), position="side")

    # ================================================================ units
    def _disp(self, q):
        """SI joint vector -> display units (deg, or m for a slide)."""
        return np.array([am.from_si(self.P, j, v) for j, v in zip(am.JOINT_ORDER, q)])

    def _si(self, v):
        return np.array([am.to_si(self.P, j, x) for j, x in zip(am.JOINT_ORDER, v)])

    def _fmt_pose(self, q):
        out = []
        for j, n, v in zip(am.JOINT_ORDER, self.names, self._disp(q)):
            u = am.joint_units(self.P, j)[0]
            out.append(f"{n} {v:.3f} m" if u == "m" else f"{n} {v:.1f}")
        return ", ".join(out)

    # ================================================================ motion
    def go_to(self, q_goal, label=""):
        q_goal = np.clip(np.array(q_goal, float), self.lo, self.hi)
        dq = np.abs(q_goal - self.cmd)
        self.move_T = max(0.3, float(np.max(1.875 * dq / self.vmax)))   # quintic peak speed = 1.875 dq / T
        self.move_from, self.move_to, self.move_t = self.cmd.copy(), q_goal, 0.0
        self.slider_val = self._disp(q_goal)
        if self.rec_phase is None:
            self.status = f"moving to {label} ({self.move_T:.1f} s)" if label else f"moving ({self.move_T:.1f} s)"

    def moving(self):
        return self.move_t < self.move_T

    def _coll_fn(self):
        if not self.check_coll:
            return None
        return lambda q: am.check_collisions(self.P, q, self.coll_elems)

    def solve_ik(self, target, label="IK target"):
        x, y, z, pitch = target
        q, msg = am.ik(self.P, (x, y, z), pitch, 0.0, "up" if self.elbow_up else "down", self.cmd,
                       self._coll_fn())
        if q is None:
            self.status = f"{label}: {msg}"
            return None
        self.go_to(q, label)
        return np.array(q)

    def preset_q(self, name):
        tgt = self.presets[name]
        if isinstance(tgt, str):
            if tgt == "home":
                return np.array(am.home_q(self.P))
            if tgt == "straight":
                return np.zeros(5)
            self.status = f"{name}: unknown preset '{tgt}'"
            return None
        if isinstance(tgt, dict):
            return self._si(tgt["joints_deg"])
        q, msg = am.ik(self.P, tgt[:3], tgt[3], 0.0, "up" if self.elbow_up else "down", self.cmd,
                       self._coll_fn())
        if q is None:
            self.status = f"{name}: {msg}"
        return None if q is None else np.array(q)

    # ================================================================ sim
    def _set_command(self, t):
        u = t / self.move_T
        span = self.move_to - self.move_from
        self.cmd = self.move_from + span * quintic(u)
        self.cmd_qd = span * quintic_rate(u) / self.move_T
        # velocity feed-forward: shifting the position target by kd/ke * qd_cmd
        # makes the drive force ke*(q_cmd - q) + kd*(qd_cmd - qd)
        self.target_full[self.idx] = self.cmd + self.P.DRIVE_KD / self.P.DRIVE_KE * self.cmd_qd
        self.control.joint_target_q.assign(self.target_full.astype(np.float32))

    def step(self):
        """One frame: the command is updated every substep (no steps in the target),
        and the motor torque is averaged over the frame's substeps."""
        t0 = self.move_t
        tau_sum = np.zeros(5)
        for s in range(self.sim_substeps):
            self._set_command(t0 + (s + 1) * self.sim_dt)
            self.state_0.clear_forces()
            self.viewer.apply_forces(self.state_0)
            self.solver.step(self.state_0, self.state_1, self.control, None, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
            tau_sum += self.solver.mjw_data.qfrc_actuator.numpy().reshape(-1)[self.mj_dof]
        self.tau_frame = tau_sum / self.sim_substeps
        self.move_t = t0 + self.frame_dt
        self.sim_time += self.frame_dt
        if self.rec_phase is None and not self.moving() and self.status.startswith("moving"):
            self.status = "holding"
        if self.check_coll:
            q, _ = self.joint_state()
            self.hits = am.check_collisions(self.P, q, self.coll_elems)
        else:
            self.hits = []
        self._record_step()

    def joint_state(self):
        q = self.state_0.joint_q.numpy()[self.idx].astype(float)
        qd = self.state_0.joint_qd.numpy()[self.idx].astype(float)
        return q, qd

    def drive_torque(self):
        """Motor torque: MuJoCo's actuator force averaged over the last frame
        (already limited to the effort limit). Returns tau, saturated, q, qd."""
        q, qd = self.joint_state()
        if self.tau_frame is None:   # before the first step: PD law
            tau = self.P.DRIVE_KE * (self.cmd - q) + self.P.DRIVE_KD * (self.cmd_qd - qd)
            tau = np.clip(tau, -self.effort, self.effort)
        else:
            tau = self.tau_frame
        sat = np.abs(tau) >= self.effort * 0.999
        return tau, sat, q, qd

    # ================================================================ recording
    def start_recording(self):
        if self.rec_phase is not None:
            return
        self.rec_phase = "approach"
        self.rec_log = []
        self.go_to(self.rec_start)
        self.status = "recording: going to the start pose"

    def _record_step(self):
        if self.rec_phase == "approach":
            if not self.moving():
                self.rec_settle += self.frame_dt
                if self.rec_settle >= 1.0:
                    self.rec_settle = 0.0
                    self.rec_phase = "record"
                    self.rec_t0 = self.sim_time
                    self.go_to(self.rec_end)
                    self.status = f"recording: moving ({self.move_T:.1f} s)"
            return
        if self.rec_phase != "record":
            return
        tau, sat, q, qd = self.drive_torque()
        sk = am.skeleton(self.P, q, self.parts)
        self.rec_log.append(dict(
            t=self.sim_time - self.rec_t0, cmd=self.cmd.copy(), q=q, qd=qd, tau=tau, sat=sat,
            static=am.static_torques(self.P, q, self.inertials), tcp=sk["tcp"], hits=list(self.hits),
            base_pos=sk["base_pos"].copy(),
            segs=np.array(sk["segments"]), pivots=sk["pivots"], masses=np.array([m[2] for m in sk["masses"]])))
        if not self.moving():
            self.rec_settle += self.frame_dt
            if self.rec_settle >= 0.5:
                self.rec_settle = 0.0
                self.rec_phase = None
                try:
                    folder = self.save_recording()
                    self.status = f"saved {folder.relative_to(HERE)} (video rendering in background)"
                except Exception as e:  # keep the sim alive whatever happens
                    self.status = f"recording failed: {e}"

    def save_recording(self, make_video=True):
        P, log = self.P, self.rec_log
        name = "".join(c if c.isalnum() or c in "-_" else "_" for c in self.rec_name) or "move"
        folder = HERE / P.RECORD_DIR / f"{dt.datetime.now():%Y%m%d_%H%M%S}_{name}"
        folder.mkdir(parents=True, exist_ok=True)

        t = np.array([r["t"] for r in log])
        Q = np.array([self._disp(r["q"]) for r in log])
        C = np.array([self._disp(r["cmd"]) for r in log])
        QD = np.array([self._disp(r["qd"]) for r in log])
        TAU = np.array([r["tau"] for r in log])
        ST = np.array([r["static"] for r in log])
        SAT = np.array([r["sat"] for r in log])
        HITS = [r["hits"] for r in log]
        TCP = np.array([r["tcp"] for r in log])

        with open(folder / "torques.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            head = ["time_s"]
            for j, n in zip(am.JOINT_ORDER, self.names):
                u = am.joint_units(P, j)[0]
                f_ = "force_N" if u == "m" else "torque_Nm"
                s_ = "static_N" if u == "m" else "static_Nm"
                head += [f"{n}_cmd_{u}", f"{n}_{u}", f"{n}_vel_{u}_s", f"{n}_{f_}", f"{n}_{s_}", f"{n}_saturated"]
            w.writerow(head + ["tcp_x_m", "tcp_y_m", "tcp_z_m", "collision"])
            for i in range(len(t)):
                row = [f"{t[i]:.4f}"]
                for k in range(5):
                    row += [f"{C[i,k]:.3f}", f"{Q[i,k]:.3f}", f"{QD[i,k]:.3f}", f"{TAU[i,k]:.3f}",
                            f"{ST[i,k]:.3f}", int(SAT[i, k])]
                w.writerow(row + [f"{v:.4f}" for v in TCP[i]] +
                           ["; ".join(f"{a} hits {b}" for a, b in HITS[i]) if HITS[i] else ""])

        lines = [f"Recording '{self.rec_name}'  {dt.datetime.now():%Y-%m-%d %H:%M:%S}",
                 f"Duration {t[-1]:.2f} s, {len(t)} frames at {self.fps} fps",
                 f"L1 {P.L1} m, L2 {P.L2} m, gripper {P.L_GRIPPER} m, payload {P.PAYLOAD['mass']} kg",
                 "Start (deg; slide in m): " + self._fmt_pose(self.rec_start),
                 "End   (deg; slide in m): " + self._fmt_pose(self.rec_end),
                 "", f"{'motor':12s} {'peak |tau|':>10s} {'at t (s)':>9s} {'static peak':>11s} {'limit':>7s}  saturated"]
        for k, j in enumerate(self.names):
            i = int(np.argmax(np.abs(TAU[:, k])))
            lines.append(f"{j:12s} {abs(TAU[i,k]):10.2f} {t[i]:9.2f} {np.max(np.abs(ST[:,k])):11.2f} "
                         f"{self.effort[k]:7.1f}  {'YES' if SAT[:,k].any() else 'no'}")
        lines += ["", "Collisions" + ("" if self.check_coll else " (checking was OFF)") + ":"]
        events, start = [], None
        for i in range(len(t) + 1):
            hit = i < len(t) and bool(HITS[i])
            if hit and start is None:
                start = i
            if not hit and start is not None:
                pairs = sorted({f"{a} hits {b}" for k in range(start, i) for a, b in HITS[k]})
                events.append(f"  {t[start]:.2f} - {t[i - 1]:.2f} s: " + "; ".join(pairs))
                start = None
        lines += events or ["  none"]
        (folder / "summary.txt").write_text("\n".join(lines) + "\n")

        kinds = [m[0] for m in am.skeleton(P, log[0]["q"], self.parts)["masses"]]
        masses_kg = [m[1] for m in am.skeleton(P, log[0]["q"], self.parts)["masses"]]
        np.savez_compressed(
            folder / "motion.npz", t=t, tau=TAU, static=ST, sat=SAT, effort=self.effort, tcp=TCP,
            segs=np.array([r["segs"] for r in log]), pivots=np.array([r["pivots"] for r in log]),
            masses=np.array([r["masses"] for r in log]), mass_kinds=np.array(kinds), mass_kg=np.array(masses_kg),
            joint_names=np.array(self.names), title=np.array(self.rec_name),
            prismatic=np.array([am.is_prismatic(P, j) for j in am.JOINT_ORDER]),
            base_pos=np.array([r["base_pos"] for r in log]),
            rover=np.array([P.ROVER[k] for k in ("mount_behind_front", "length", "width",
                                                  "ground_clearance", "deck_height")]),
            turntable=np.array([P.BASE_HEIGHT]),
            coll_seg=np.array([[any(n in pair for pair in h) for n in ("link1 tube", "link2 tube", "gripper")]
                               for h in HITS]),
            coll_text=np.array([am.describe_hits(h, 2) if h else "" for h in HITS]))
        self.last_recording = folder
        if make_video:
            subprocess.Popen([sys.executable, str(HERE / "make_video.py"), str(folder)],
                             stdout=open(folder / "video_log.txt", "w"), stderr=subprocess.STDOUT)
        return folder

    # ================================================================ design editing
    def design_items(self):
        """(label, getter, setter, anchor_frame_fn) for everything movable."""
        P = self.P
        items = []

        def tup_set(holder, key, idx=None):
            def setter(v):
                if idx is None:
                    holder[key] = tuple(v)
                else:
                    holder[key][idx] = tuple(v)
            return setter

        items.append(("shoulder pivot (from base centre)", lambda: P.SHOULDER_PIVOT,
                      lambda v: setattr(P, "SHOULDER_PIVOT", tuple(v)), ("base", "pivot")))
        for link, near_lbl, far_lbl in (("link1", "shoulder", "elbow"), ("link2", "elbow", "wrist")):
            g = P.LINK_GEOMETRY[link]
            items.append((f"{link}: {near_lbl} pivot (from tube start)", (lambda g=g: g["near"]),
                          (lambda v, g=g: g.__setitem__("near", tuple(v))), (link, "tube_start")))
            items.append((f"{link}: {far_lbl} pivot (from tube end)", (lambda g=g: g["far"]),
                          (lambda v, g=g: g.__setitem__("far", tuple(v))), (link, "tube_end")))
        for group, tag in ((P.MOTORS, "motor"), (P.EXTRA_MASSES, "mass")):
            for name, d in group.items():
                items.append((f"{tag}: {name} (on {d['body']}, from {d.get('anchor', 'pivot')})",
                              (lambda d=d: d["xyz"]), (lambda v, d=d: d.__setitem__("xyz", tuple(v))),
                              (d["body"], d.get("anchor", "pivot")), d))
        return items

    def anchor_frame(self, body, anchor, q):
        """World rotation and origin of an anchor's axes."""
        frames = am.fk(self.P, q)[0]
        R, p = frames[body]
        if anchor in ("tube_start", "tube_end") and body in am.TUBE_LINKS:
            geo = am.link_geometry(self.P, body)
            o = geo["start"] if anchor == "tube_start" else geo["end"]
            return R @ geo["R"], p + R @ o
        return R, p

    def apply_design_change(self, setter, new, old):
        setter(new)
        try:
            self.rebuild()
            self.dirty = True
            self.status = "design updated (not saved)"
        except ValueError as e:
            setter(old)
            self.rebuild()
            self.status = f"rejected: {e}"

    def save_design(self):
        params_io.save(self.P, PARAMS_FILE)
        path = build_urdf.write(self.P, HERE)
        self.dirty = False
        self.status = f"saved arm_params.py (backup .bak) and {path.name}"

    def _draw_design_box(self):
        if not hasattr(self.viewer, "log_lines"):
            return
        hide = not self.design_open
        if hide and not getattr(self, "_design_drawn", False):
            return                                   # nothing drawn yet, nothing to hide
        dev = self.model.device
        item = self.design_items()[self.design_idx]
        q, _ = self.joint_state()
        R, o = self.anchor_frame(*item[3], q)
        b = self.P.DESIGN_BOX
        corners = [o + R @ np.array([sx * b, sy * b, sz * b]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        starts = wp.array([corners[i] for i, _ in BOX_EDGES], dtype=wp.vec3, device=dev)
        ends = wp.array([corners[k] for _, k in BOX_EDGES], dtype=wp.vec3, device=dev)
        line_cols = wp.array([wp.vec3(1.0, 0.85, 0.1)] * len(BOX_EDGES), dtype=wp.vec3, device=dev)
        val = np.array(item[1](), float)
        if self.pending is not None and self.pending[0] == self.design_idx:
            val = self.pending[1]
        pts = wp.array([o + R @ val], dtype=wp.vec3, device=dev)
        radii = wp.array([0.015], dtype=wp.float32, device=dev)
        pt_cols = wp.array([wp.vec3(1.0, 0.2, 0.9)], dtype=wp.vec3, device=dev)
        # The GL viewer wants Warp arrays (not tuples) for colours and radii.
        self.viewer.log_lines("/design_box", starts, ends, line_cols, hidden=hide)
        if hasattr(self.viewer, "log_points"):
            self.viewer.log_points("/design_point", pts, radii, pt_cols, hidden=hide)
        self._design_drawn = True

    # ================================================================ render + GUI
    def _draw_collisions(self):
        """Red overlay on the parts that are touching something."""
        if not hasattr(self.viewer, "log_lines"):
            return
        if not self.hits and not getattr(self, "_coll_drawn", False):
            return
        dev = self.model.device
        names = {n for pair in self.hits for n in pair}
        q, _ = self.joint_state()
        frames = am.fk(self.P, q)[0]
        segs, pts, rad = [], [], []
        for el in self.coll_elems:
            if el["name"] not in names:
                continue
            R, p = frames[el["body"]]
            if el["kind"] == "capsule":
                segs.append((p + R @ el["a"], p + R @ el["b"]))
            else:
                pts.append(p + R @ el["c"])
                rad.append(el["r"] if "r" in el else float(np.max(el["h"])))
        red = wp.vec3(1.0, 0.1, 0.1)
        if segs:
            self.viewer.log_lines("/collisions", wp.array([a for a, _ in segs], dtype=wp.vec3, device=dev),
                                  wp.array([b for _, b in segs], dtype=wp.vec3, device=dev),
                                  wp.array([red] * len(segs), dtype=wp.vec3, device=dev), hidden=False)
        elif getattr(self, "_coll_drawn", False):
            self.viewer.log_lines("/collisions", None, None, None, hidden=True)
        if hasattr(self.viewer, "log_points"):
            if pts:
                self.viewer.log_points("/collision_parts", wp.array(pts, dtype=wp.vec3, device=dev),
                                       wp.array([r * 1.05 for r in rad], dtype=wp.float32, device=dev),
                                       wp.array([red] * len(pts), dtype=wp.vec3, device=dev), hidden=False)
            elif getattr(self, "_coll_drawn", False):
                self.viewer.log_points("/collision_parts", None)
        self._coll_drawn = bool(self.hits)

    def render(self):
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        self._draw_design_box()
        self._draw_collisions()
        self.viewer.end_frame()

    def _slider_box(self, ui, label, v, lo, hi, fmt="%.3f"):
        """A slider plus a box you can type a value into (press Enter to apply).
        Returns (value, changed, committed): changed = any edit this frame,
        committed = slider released or Enter pressed in the box."""
        avail = ui.get_content_region_avail().x
        ui.set_next_item_width(max(80.0, avail * 0.45))
        ch1, v1 = ui.slider_float(f"##slider_{label}", float(v), float(lo), float(hi), fmt)
        released = ui.is_item_deactivated_after_edit()
        ui.same_line()
        ui.set_next_item_width(max(60.0, avail * 0.22))
        ch2, v2 = ui.input_float(label, float(v1), 0.0, 0.0, fmt, ui.InputTextFlags_.enter_returns_true)
        value = v2 if ch2 else v1
        return value, ch1 or ch2, released or ch2

    def gui(self, ui):
        P = self.P
        ui.text_wrapped(f"Status: {self.status}")
        ch, self.check_coll = ui.checkbox("Check collisions##coll", self.check_coll)
        if ch and not self.check_coll:
            self.hits = []
        ui.separator()

        busy = self.rec_phase is not None
        ui.text("Joints (relative deg; slide in m)")
        changed_any = False
        for k, j in enumerate(am.JOINT_ORDER):
            lo, hi = am.from_si(P, j, self.lo[k]), am.from_si(P, j, self.hi[k])
            fmt = "%.3f" if am.is_prismatic(P, j) else "%.1f"
            v, ch, _ = self._slider_box(ui, f"{self.names[k]}##jt", self.slider_val[k], lo, hi, fmt)
            if ch and not busy:
                self.slider_val[k] = float(np.clip(v, lo, hi))
                changed_any = True
        if changed_any:
            self.go_to(self._si(self.slider_val), "slider pose")

        ui.text("Presets")
        for name in self.presets:
            if ui.button(f"{name}##preset") and not busy:
                q = self.preset_q(name)
                if q is not None:
                    self.go_to(q, name)
            ui.same_line()
        ui.new_line()

        if ui.collapsing_header("TCP target / IK"):
            t = self.ik_target
            t[0] = self._slider_box(ui, "X (m)##ik", t[0], -0.5, 2.5)[0]
            t[1] = self._slider_box(ui, "Y (m)##ik", t[1], -2.0, 2.0)[0]
            t[2] = self._slider_box(ui, "Z (m)##ik", t[2], -0.5, 2.6)[0]
            t[3] = self._slider_box(ui, "Pitch (deg)##ik", t[3], -90.0, 90.0, "%.1f")[0]
            _, self.elbow_up = ui.checkbox("Elbow up##ik", self.elbow_up)
            if ui.button("Solve IK##ik") and not busy:
                self.solve_ik(list(t))

        if ui.collapsing_header("Record move"):
            ui.text_wrapped("Start: " + self._fmt_pose(self.rec_start))
            ui.text_wrapped("End:   " + self._fmt_pose(self.rec_end))
            if ui.button("Set start = current pose##rec"):
                self.rec_start = self.move_to.copy()
            ui.same_line()
            if ui.button("Set end = current pose##rec"):
                self.rec_end = self.move_to.copy()
            _, self.rec_name = ui.input_text("Name##rec", self.rec_name)
            if busy:
                ui.text(f"... {self.rec_phase} ({len(self.rec_log)} frames)")
            elif ui.button("Record##rec"):
                self.start_recording()
            if self.last_recording is not None:
                ui.text(f"Last: {self.last_recording.relative_to(HERE)}")

        self.design_open = ui.collapsing_header("Design (pivots, motors, masses)")
        if self.design_open:
            items = self.design_items()
            self.design_idx = min(self.design_idx, len(items) - 1)
            ch, self.design_idx = ui.combo("Item##design", self.design_idx, [it[0] for it in items])
            if ch:
                self.pending, self.pending_mass = None, None
            item = items[self.design_idx]
            cur = np.array(item[1](), float)
            if self.pending is not None and self.pending[0] == self.design_idx:
                cur = self.pending[1]               # value being dragged (applied on release)
            b = P.DESIGN_BOX
            new = cur.copy()
            released = False
            for a, axis in enumerate("xyz"):
                lo, hi = min(-b, cur[a]), max(b, cur[a])
                new[a], _, done = self._slider_box(ui, f"{axis} (m)##design{a}", cur[a], lo, hi, "%.4f")
                released |= done
            if np.any(new != cur) or self.pending is not None:
                self.pending = (self.design_idx, new.copy())
            if released and self.pending is not None:
                old = np.array(item[1](), float)
                self.pending = None
                self.apply_design_change(item[2], new, old)
            if len(item) > 4:                       # motors / extras: mass too
                d = item[4]
                shown = self.pending_mass if self.pending_mass is not None else d["mass"]
                m, _, done = self._slider_box(ui, "mass (kg)##design", shown, 0.0, max(5.0, float(d["mass"])))
                m = max(0.0, m)
                if m != d["mass"]:
                    self.pending_mass = m
                if done and self.pending_mass is not None:
                    old_m, self.pending_mass = d["mass"], None
                    self.apply_design_change(lambda v, d=d: d.__setitem__("mass", float(v)), m, old_m)
            ui.text_wrapped(f"Drag a slider or type a value and press Enter. Box: +/-{b:.2f} m around the "
                            "anchor (yellow); typed values may go outside it. Body/anchor: edit arm_params.py.")
            if ui.button("Save to arm_params.py##design"):
                try:
                    self.save_design()
                except Exception as e:
                    self.status = f"save failed: {e}"
            ui.same_line()
            ui.text("unsaved changes" if self.dirty else "saved")

        ui.separator()
        tau, sat, q, _ = self.drive_torque()
        _, _, tcp = am.fk(P, q)
        ui.text(f"TCP x {tcp[0]:.3f}  y {tcp[1]:.3f}  z {tcp[2]:.3f} m  (past nose {tcp[0] - P.ROVER['mount_behind_front']:.3f})")
        if not self.check_coll:
            ui.text("Collisions: not checked")
        elif self.hits:
            ui.push_text_wrap_pos(0.0)
            ui.text_colored(ui.ImVec4(1.0, 0.3, 0.3, 1.0), "COLLISION: " + am.describe_hits(self.hits, 4))
            ui.pop_text_wrap_pos()
        else:
            ui.text_colored(ui.ImVec4(0.4, 0.9, 0.4, 1.0), "Collisions: clear")
        ui.text("Torque N*m (slide: force N):  motor | static | limit")
        static = am.static_torques(P, q, self.inertials)
        pos = self._disp(q)
        for k, j in enumerate(am.JOINT_ORDER):
            unit = am.joint_units(P, j)[0]
            where = f"{pos[k]:+.3f} m" if unit == "m" else f"{pos[k]:+6.1f} deg"
            ui.text(f"  {self.names[k]:11s} {where:>11s}  {tau[k]:7.2f} | {static[k]:7.2f} | "
                    f"{self.effort[k]:6.1f}{'  SAT' if sat[k] else ''}")

    # ================================================================ headless check
    def test_final(self):
        """Hold the home pose: motor torque must match the static model."""
        tau, sat, q, _ = self.drive_torque()
        static = am.static_torques(self.P, q, self.inertials)
        print("joint        motor   static   error (deg; slide in m)")
        err = self._disp(self.cmd) - self._disp(q)
        for k in range(len(am.JOINT_ORDER)):
            print(f"  {self.names[k]:11s} {tau[k]:7.3f} {static[k]:7.3f}  {err[k]:9.4f}")
        if not np.all(np.isfinite(q)):
            raise ValueError("non-finite joint state")
        if np.max(np.abs(tau - static)) > 0.05 * max(1.0, np.max(np.abs(static))):
            raise ValueError("motor torque differs from the static model")
        print("OK: sim holding torques match the static model")


if __name__ == "__main__":
    parser = newton.examples.create_parser()
    viewer, args = newton.examples.init(parser)
    newton.examples.run(Example(viewer, args), args)
