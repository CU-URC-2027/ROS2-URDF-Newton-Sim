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
  * Task: box -> drawer: place the box, set grasps and rover poses, then
    Check mission (fast, no physics), Find rover poses, Reach map, or Run
    mission (physics; recorded like a move, with the box, drawer and rover in
    the video). See README section 4.7.
  * Design: move a pivot, motor or extra mass inside a box around its anchor
    (the box is drawn in the 3D view). The model rebuilds when you release a
    slider. "Save to arm_params.py" writes the changes back (backup .bak) and
    regenerates urc_arm.urdf.
  * Readout: torque each motor applies vs. the static model, against its
    effort limit. "SAT" = the motor is at its limit.

Physics: MuJoCo solver, position-controlled joints with velocity feed-forward
(DRIVE_KE / DRIVE_KD) plus gravity feed-forward (GRAVITY_FF), effort limits
from JOINTS. The reported torque is the actuator force, so during a move it
includes acceleration and Coriolis effects. The rover drives on three
position-controlled planar joints. Contacts are OFF: collisions are checked
and reported, but nothing physically stops the arm.

Check without a window:
    python sim_arm.py --viewer null --test --num-frames 300
"""
import csv
import datetime as dt
import math
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
import warp as wp

import newton
import newton.examples
from newton import JointTargetMode

import arm_model as am
import build_urdf
import mission as ms
import params_io
import scene as sc

HERE = Path(__file__).resolve().parent
PARAMS_FILE = HERE / "arm_params.py"
BOX_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
REACH_COLORS = [(0.1, 0.8, 0.2), (0.5, 0.5, 0.5), (0.9, 0.1, 0.1), (1.0, 0.6, 0.0), (0.6, 0.2, 0.9)]


def quat_of(R):
    """Rotation matrix -> warp quaternion (x, y, z, w)."""
    return wp.quat_from_matrix(wp.mat33(*[float(v) for v in np.asarray(R).reshape(-1)]))


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
        # rover (driven by planar joints rover_x / rover_y / rover_yaw)
        self.rcmd, self.rcmd_qd = np.zeros(3), np.zeros(3)
        self.rmove_from, self.rmove_to, self.rmove_t, self.rmove_T = np.zeros(3), np.zeros(3), 1.0, 1.0
        # task scene (box -> drawer)
        self.show_task = True
        self.att = ms.initial_state(self.P)
        self.mission, self.mseg_i, self.mseg_t = None, 0, 0.0
        self.task_lines, self.reach, self.bg = [], None, None
        self.rec_mission = True
        self._build_model(q0, np.zeros(3))

        nose = am.rover_geom(self.P)["nose"]
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
    def _build_model(self, q_now, rover_now):
        P = self.P
        xml = build_urdf.build(P, mobile=True)      # raises ValueError on bad geometry
        builder = newton.ModelBuilder()
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
        builder.default_joint_cfg.armature = 0.01
        builder.add_urdf(xml, floating=False, enable_self_collisions=False, collapse_fixed_joints=False)

        dof, rdof = {}, {}
        internal = {am.joint_name(P, j): j for j in am.JOINT_ORDER}
        for ji, key in enumerate(builder.joint_label):
            name = key.split("/")[-1]
            if name in internal:
                dof[internal[name]] = builder.joint_qd_start[ji]
            if name in build_urdf.ROVER_JOINTS:
                rdof[name] = builder.joint_qd_start[ji]
        self.idx = np.array([dof[j] for j in am.JOINT_ORDER])
        self.ridx = np.array([rdof[n] for n in build_urdf.ROVER_JOINTS])
        for k, d in enumerate(self.ridx):
            scale = 0.1 if k == 2 else 1.0        # yaw gains are per radian
            builder.joint_target_ke[d] = P.ROVER_KE * scale
            builder.joint_target_kd[d] = P.ROVER_KD * scale
            builder.joint_target_mode[d] = int(JointTargetMode.POSITION)
            builder.joint_q[d] = float(rover_now[k])
            builder.joint_target_q[d] = float(rover_now[k])
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
        self.model.set_gravity(tuple(float(v) for v in am.gravity_vec(P)))     # a slope tilts gravity
        self.solver.notify_model_changed(newton.ModelFlags.MODEL_PROPERTIES)
        self._mp_hist, self._acc = [], None
        self.stab, self.stab_drive = None, None
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_0)
        self.target_full = self.control.joint_target_q.numpy().copy()
        mj = self.solver.mj_model                    # MuJoCo DOF index of each of our joints
        self.mj_dof = np.array([mj.jnt_dofadr[next(i for i in range(mj.njnt) if mj.joint(i).name.endswith(am.joint_name(P, j)))]
                                for j in am.JOINT_ORDER])
        self.tau_frame = None
        # gripper body: its original mass properties (the box is added to them when held)
        self.g_body = next(i for i, l in enumerate(self.model.body_label) if l.split("/")[-1] == "gripper")
        self.g_mass0 = float(self.model.body_mass.numpy()[self.g_body])
        self.g_com0 = self.model.body_com.numpy()[self.g_body].astype(float).copy()
        self.g_I0 = self.model.body_inertia.numpy()[self.g_body].astype(float).copy()

        self.parts = am.body_parts(P)
        self.coll_elems = am.collision_elements(P, self.parts)
        self.hits = []
        self.inertials = am.body_inertials(P, self.parts)
        lim = np.array([am.joint_limits(P, j) for j in am.JOINT_ORDER])   # SI: rad or m
        self.lo, self.hi, self.effort, self.vmax = lim[:, 0], lim[:, 1], lim[:, 2], lim[:, 3]
        self.rcmd, self.rcmd_qd = np.array(rover_now, float), np.zeros(3)
        self._apply_grip_inertia()
        self.viewer.set_model(self.model)

    def rebuild(self):
        """Rebuild after a design change, keeping the current pose."""
        q, _ = self.joint_state()
        self._build_model(q, self.rover_state())
        self.rmove_from = self.rmove_to = self.rcmd.copy()
        self.rmove_t = self.rmove_T
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
        r = self.rover_state()
        if self.show_task:
            return lambda q: ms.collisions(self.P, q, r, self.att, self.coll_elems)
        return lambda q: am.check_collisions(self.P, q, self.coll_elems, rover=r)

    def solve_ik(self, target, label="IK target"):
        x, y, z, pitch = target
        q, msg = am.ik(self.P, (x, y, z), pitch, 0.0, "up" if self.elbow_up else "down", self.cmd,
                       self._coll_fn(), rover=self.rover_state())
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
                       self._coll_fn(), rover=self.rover_state())
        if q is None:
            self.status = f"{name}: {msg}"
        return None if q is None else np.array(q)

    # ================================================================ sim
    def _set_command(self, h):
        """Commands at h seconds past the start of this frame (arm and rover)."""
        if self.mission is not None:
            sg = self.mission[self.mseg_i]
            u = min((self.mseg_t + h) / sg.T, 1.0)
            s, sdot = quintic(u), quintic_rate(u) / sg.T
            self.cmd, self.cmd_qd = sg.q(s), sg.dq_ds(s) * sdot
            self.rcmd, self.rcmd_qd = sg.rover(s), sg.drover_ds() * sdot
        else:
            u = (self.move_t + h) / self.move_T
            span = self.move_to - self.move_from
            self.cmd = self.move_from + span * quintic(u)
            self.cmd_qd = span * quintic_rate(u) / self.move_T
            u = (self.rmove_t + h) / self.rmove_T
            span = self.rmove_to - self.rmove_from
            self.rcmd = self.rmove_from + span * quintic(u)
            self.rcmd_qd = span * quintic_rate(u) / self.rmove_T
        # velocity feed-forward: shifting the position target by kd/ke * qd_cmd
        # makes the drive force ke*(q_cmd - q) + kd*(qd_cmd - qd); gravity feed-forward
        # adds the static torque the pose needs (shift by tau_g / ke)
        ff = 0.0
        if getattr(self.P, "GRAVITY_FF", False):
            ff = am.static_torques(self.P, self.cmd, self.inertials_now(), rover=self.rcmd) / self.P.DRIVE_KE
        self.target_full[self.idx] = self.cmd + self.P.DRIVE_KD / self.P.DRIVE_KE * self.cmd_qd + ff
        self.target_full[self.ridx] = self.rcmd + self.P.ROVER_KD / self.P.ROVER_KE * self.rcmd_qd
        self.control.joint_target_q.assign(self.target_full.astype(np.float32))

    def step(self):
        """One frame: the command is updated every substep (no steps in the target),
        and the motor torque is averaged over the frame's substeps."""
        tau_sum = np.zeros(5)
        for s in range(self.sim_substeps):
            self._set_command((s + 1) * self.sim_dt)
            self.state_0.clear_forces()
            self.viewer.apply_forces(self.state_0)
            self.solver.step(self.state_0, self.state_1, self.control, None, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
            tau_sum += self.solver.mjw_data.qfrc_actuator.numpy().reshape(-1)[self.mj_dof]
        self.tau_frame = tau_sum / self.sim_substeps
        self.sim_time += self.frame_dt
        if self.mission is not None:
            self.mseg_t += self.frame_dt
        else:
            self.move_t += self.frame_dt
            self.rmove_t += self.frame_dt
        if self.rec_phase is None and self.mission is None and not self.moving() \
                and self.status.startswith("moving"):
            self.status = "holding"
        q, _ = self.joint_state()
        r = self.rover_state()
        if not self.check_coll:
            self.hits = []
        elif self.show_task:
            self.hits = ms.collisions(self.P, q, r, self.att, self.coll_elems)
        else:
            self.hits = am.check_collisions(self.P, q, self.coll_elems, rover=r)
        self._update_stability(q, r)
        self._record_step()
        if self.mission is not None:
            self._advance_mission()
        self._poll_background()

    def joint_state(self):
        q = self.state_0.joint_q.numpy()[self.idx].astype(float)
        qd = self.state_0.joint_qd.numpy()[self.idx].astype(float)
        return q, qd

    def _update_stability(self, q, r):
        """Tipping margin, wheel loads and mount loads, with the accelerations of every
        mass (from their positions over the last frames, lightly smoothed)."""
        ins = self.inertials_now()
        pts, _ = am.mass_points(self.P, q, r, ins, self.parts)
        pos = np.array([p for _, p in pts])
        self._mp_hist = (self._mp_hist + [pos])[-3:]
        acc = None
        if len(self._mp_hist) == 3:
            a = (self._mp_hist[2] - 2 * self._mp_hist[1] + self._mp_hist[0]) / self.frame_dt ** 2
            self._acc = a if self._acc is None or self._acc.shape != a.shape else 0.6 * self._acc + 0.4 * a
            acc = list(self._acc)
        self.stab = am.stability(self.P, q, r, ins, self.parts, acc)
        self.stab_drive = am.worst_drive(self.P, q, r, ins, self.parts, acc)

    def _stab_record(self, r):
        st, wd = self.stab, self.stab_drive
        R0, p0 = am.rover_frame(r)
        cop_w = p0 + R0 @ np.array([st["cop"][0], st["cop"][1], 0.0])
        return dict(margin=st["margin"], edge=st["edge"], N=st["N"].copy(), cop_w=cop_w,
                    mount_F=st["mount_F"].copy(), mount_M=st["mount_M"].copy(),
                    drive_margin=wd["margin"], drive_edge=wd["edge"])

    def rover_state(self):
        """Actual rover pose (x, y, yaw_rad)."""
        return self.state_0.joint_q.numpy()[self.ridx].astype(float)

    def inertials_now(self):
        """Body inertials including a held box (for the static torques)."""
        return ms.held_box_inertials(self.P, self.inertials, self.att)

    def _apply_grip_inertia(self):
        """Put the held box's mass into the gripper body (or take it out)."""
        g = dict(mass=self.g_mass0, com=self.g_com0, inertia=self.g_I0)
        new = ms.held_box_inertials(self.P, {"gripper": g}, self.att)["gripper"]
        i = self.g_body
        m = self.model
        for arr, val in ((m.body_mass, new["mass"]), (m.body_inv_mass, 1.0 / new["mass"])):
            a = arr.numpy()
            a[i] = val
            arr.assign(a)
        for arr, val in ((m.body_com, new["com"]),):
            a = arr.numpy()
            a[i] = val
            arr.assign(a)
        for arr, val in ((m.body_inertia, new["inertia"]), (m.body_inv_inertia, np.linalg.inv(new["inertia"]))):
            a = arr.numpy()
            a[i] = val
            arr.assign(a)
        self.solver.notify_model_changed(newton.ModelFlags.BODY_INERTIAL_PROPERTIES)

    # ================================================================ task: box -> drawer
    def reset_task(self, q_start=None):
        """Box back on the ground at BOX_POSE, drawer closed, rover at the start, arm at home."""
        self.mission = None
        self.att = ms.initial_state(self.P)
        q = np.array(am.home_q(self.P)) if q_start is None else q_start
        self._build_model(q, np.zeros(3))
        self.cmd, self.cmd_qd = q.copy(), np.zeros(5)
        self.move_from = self.move_to = q.copy()
        self.move_t = self.move_T
        self.rmove_from = self.rmove_to = np.zeros(3)
        self.rmove_t = self.rmove_T
        self.slider_val = self._disp(q)
        if hasattr(self.viewer, "register_ui_callback"):
            self.viewer.register_ui_callback(lambda ui, ex=self: ex.gui(ui), position="side")

    def start_mission(self, record=True):
        q_home = np.array(am.home_q(self.P))
        try:
            segs, _ = ms.build_mission(self.P, q_home, self.coll_elems)
        except ms.PlanError as e:
            self.status = f"can't run the mission: {e}"
            return
        self.reset_task(q_home)
        self.mission, self.mseg_i, self.mseg_t = segs, 0, 0.0
        self.mission_T = sum(sg.T for sg in segs)
        self._segment_started()
        if record:
            self.rec_phase, self.rec_log, self.rec_t0 = "mission", [], self.sim_time
            self.rec_name = "mission"
        self.status = f"mission: {segs[0].phase}: {segs[0].label}"

    def _segment_started(self):
        sg = self.mission[self.mseg_i]
        if sg.kind == "wait" and sg.at == "start":
            self._event(sg.event)
        self.status = f"mission ({self.mseg_i + 1}/{len(self.mission)}): {sg.phase}: {sg.label}"

    def _event(self, event):
        q, _ = self.joint_state()
        ms.apply_event(self.P, event, q, self.rover_state(), self.att)
        if event in ("grip box", "release box"):
            self._apply_grip_inertia()
            self._mp_hist, self._acc = [], None   # the gripper's COM jumps; don't read that as acceleration

    def _advance_mission(self):
        sg = self.mission[self.mseg_i]
        if self.mseg_t < sg.T - 1e-9:
            return
        if sg.kind == "wait" and sg.at == "end":
            self._event(sg.event)
        self.mseg_i += 1
        self.mseg_t = 0.0
        if self.mseg_i < len(self.mission):
            self._segment_started()
            return
        # finished: hold the last command
        self.mission = None
        self.move_from = self.move_to = self.cmd.copy()
        self.move_t = self.move_T
        self.rmove_from = self.rmove_to = self.rcmd.copy()
        self.rmove_t = self.rmove_T
        self.status = "mission done"
        if self.rec_phase == "mission":
            self.rec_phase = None
            try:
                folder = self.save_recording()
                self.status = f"mission done; saved {folder.relative_to(HERE)} (video rendering in background)"
            except Exception as e:
                self.status = f"mission done; recording failed: {e}"

    def drive_rover(self, r_goal):
        r_goal = np.array(r_goal, float)
        v, w = self.P.MISSION["rover_speed"]
        d = np.abs(r_goal - self.rcmd)
        self.rmove_T = max(0.3, 1.875 * math.hypot(d[0], d[1]) / v, 1.875 * d[2] / math.radians(w))
        self.rmove_from, self.rmove_to, self.rmove_t = self.rcmd.copy(), r_goal, 0.0

    def _run_bg(self, kind, fn):
        """Run a slow check in a background thread so the window stays responsive."""
        if self.bg is not None:
            return
        box = dict(kind=kind, msg=f"{kind}...", result=None, error=None)

        def work():
            try:
                box["result"] = fn(lambda m: box.__setitem__("msg", m))
            except Exception as e:  # report, don't crash the sim
                box["error"] = repr(e)
        box["thread"] = threading.Thread(target=work, daemon=True)
        self.bg = box
        box["thread"].start()

    def _poll_background(self):
        b = self.bg
        if b is None:
            return
        if b["thread"].is_alive():
            self.status = b["msg"]
            return
        self.bg = None
        if b["error"]:
            self.status = f"{b['kind']} failed: {b['error']}"
            return
        r = b["result"]
        if b["kind"] == "check":
            ok, self.task_lines = r[0], r[1]
            self.status = "mission check: " + self.task_lines[0]
        elif b["kind"] == "rover poses":
            poses, lines = r
            if poses:
                self.P.ROVER_POSES = poses
            self.task_lines = ["rover poses:"] + lines
            self.status = "rover pose search done" + ("" if poses else " (stopped)")
        elif b["kind"] == "reach map":
            xs, ys, codes, labels, secs = r
            self.reach = (xs, ys, codes)
            folder = self._save_reach_map(xs, ys, codes, labels)
            n_ok = int((codes == 0).sum())
            self.task_lines = [f"reach map: {n_ok}/{codes.size} box positions OK ({secs:.0f} s)",
                               f"saved {folder.relative_to(HERE)}"] + \
                              [f"  {lab}: {int((codes == k).sum())}" for k, lab in enumerate(labels) if (codes == k).any()]
            self.status = self.task_lines[0]

    def _save_reach_map(self, xs, ys, codes, labels):
        P = self.P
        folder = HERE / P.RECORD_DIR / f"{dt.datetime.now():%Y%m%d_%H%M%S}_reachmap"
        folder.mkdir(parents=True, exist_ok=True)
        with open(folder / "reach_map.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["box_x_m", "box_y_m", "result"])
            for iy, y in enumerate(ys):
                for ix, x in enumerate(xs):
                    w.writerow([f"{x:.3f}", f"{y:.3f}", labels[codes[iy, ix]]])
        r = P.ROVER_POSES["pick_up"]
        (folder / "settings.txt").write_text(
            f"rover at {r}, box yaw {P.BOX_POSE[2]} deg, box grasp {P.BOX_GRASP}, base {P.BASE_JOINT}\n"
            f"L1 {P.L1}, L2 {P.L2}, gripper {P.L_GRIPPER}, box {P.TASK_BOX['mass']} kg\n")
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from matplotlib.colors import ListedColormap
            fig, ax = plt.subplots(figsize=(7, 6))
            step = P.MISSION["reach_step"]
            ax.imshow(codes, origin="lower", cmap=ListedColormap(REACH_COLORS), vmin=-0.5, vmax=4.5,
                      extent=(xs[0] - step / 2, xs[-1] + step / 2, ys[0] - step / 2, ys[-1] + step / 2))
            nose = am.rover_geom(P)["nose"]
            ax.axvline(nose, color="k", ls="--", lw=1)
            ax.text(nose, ys[0], " rover nose", va="bottom", fontsize=8, rotation=90)
            for k, lab in enumerate(labels):
                ax.plot([], [], "s", color=REACH_COLORS[k], label=lab)
            ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
            ax.set(xlabel="box x (m, forward)", ylabel="box y (m, left)",
                   title=f"Can the arm pick up the box? (rover at x {r[0]}, y {r[1]}, yaw {r[2]})")
            fig.tight_layout()
            fig.savefig(folder / "reach_map.png", dpi=120)
            plt.close(fig)
        except Exception as e:
            (folder / "plot_error.txt").write_text(repr(e))
        return folder

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
        if self.rec_phase not in ("record", "mission"):
            return
        tau, sat, q, qd = self.drive_torque()
        r = self.rover_state()
        sk = am.skeleton(self.P, q, self.parts, rover=r)
        rs = ms.resolve(self.P, q, r, self.att)
        objs = sc.scene_objects(self.P, dict(drawer=rs["drawer"], box_T=rs["box_T"]))
        sg = self.mission[self.mseg_i] if self.mission is not None else None
        self.rec_log.append(dict(
            t=self.sim_time - self.rec_t0, cmd=self.cmd.copy(), q=q, qd=qd, tau=tau, sat=sat,
            static=am.static_torques(self.P, q, self.inertials_now(), rover=r), tcp=sk["tcp"], hits=list(self.hits),
            stab=self._stab_record(r),
            base_pos=sk["base_pos"].copy(), rover=r, drawer=rs["drawer"], box_held=self.att["box"] == "gripper",
            step=(f"{sg.phase}: {sg.label}" if sg else ""),
            obj_c=np.array([o["c"] for o in objs]), obj_R=np.array([o["R"] for o in objs]),
            obj_h=np.array([o["h"] for o in objs]), obj_col=np.array([o["color"] for o in objs]),
            segs=np.array(sk["segments"]), pivots=sk["pivots"], masses=np.array([m[2] for m in sk["masses"]])))
        if self.rec_phase == "mission":
            return
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
            w.writerow(head + ["tcp_x_m", "tcp_y_m", "tcp_z_m", "rover_x_m", "rover_y_m", "rover_yaw_deg",
                               "drawer_open_m", "box_held", "tip_margin_m", "tip_edge", "tip_margin_driving_m",
                               "wheel_FL_N", "wheel_FR_N", "wheel_RL_N", "wheel_RR_N",
                               "mount_Fz_N", "mount_shear_N", "mount_bending_Nm", "mount_twist_Nm",
                               "step", "collision"])
            for i in range(len(t)):
                row = [f"{t[i]:.4f}"]
                for k in range(5):
                    row += [f"{C[i,k]:.3f}", f"{Q[i,k]:.3f}", f"{QD[i,k]:.3f}", f"{TAU[i,k]:.3f}",
                            f"{ST[i,k]:.3f}", int(SAT[i, k])]
                rv = log[i]["rover"]
                sb = log[i]["stab"]
                mF, mM = sb["mount_F"], sb["mount_M"]
                w.writerow(row + [f"{v:.4f}" for v in TCP[i]] +
                           [f"{rv[0]:.4f}", f"{rv[1]:.4f}", f"{math.degrees(rv[2]):.2f}", f"{log[i]['drawer']:.4f}",
                            int(log[i]["box_held"]), f"{sb['margin']:.4f}", sb["edge"], f"{sb['drive_margin']:.4f}"] +
                           [f"{v:.1f}" for v in sb["N"]] +
                           [f"{mF[2]:.1f}", f"{math.hypot(mF[0], mF[1]):.1f}", f"{math.hypot(mM[0], mM[1]):.2f}",
                            f"{mM[2]:.2f}", log[i]["step"]] +
                           ["; ".join(f"{a} hits {b}" for a, b in HITS[i]) if HITS[i] else ""])

        lines = [f"Recording '{self.rec_name}'  {dt.datetime.now():%Y-%m-%d %H:%M:%S}",
                 f"Duration {t[-1]:.2f} s, {len(t)} frames at {self.fps} fps",
                 f"L1 {P.L1} m, L2 {P.L2} m, gripper {P.L_GRIPPER} m, payload {P.PAYLOAD['mass']} kg",
                 ]
        steps = [r["step"] for r in log]
        if any(steps):
            box = P.TASK_BOX
            lines += [f"Task: box {box['mass']} kg at {P.BOX_POSE}, box grasp {P.BOX_GRASP}, "
                      f"drawer grasp {P.DRAWER_GRASP}",
                      "Rover poses: " + ", ".join(f"{k} {v}" for k, v in P.ROVER_POSES.items())]
        else:
            lines += ["Start (deg; slide in m): " + self._fmt_pose(self.rec_start),
                      "End   (deg; slide in m): " + self._fmt_pose(self.rec_end)]
        lines += ["", f"{'motor':12s} {'peak |tau|':>10s} {'at t (s)':>9s} {'static peak':>11s} {'limit':>7s}  saturated"
                  + ("   during" if any(steps) else "")]
        for k, j in enumerate(self.names):
            i = int(np.argmax(np.abs(TAU[:, k])))
            lines.append(f"{j:12s} {abs(TAU[i,k]):10.2f} {t[i]:9.2f} {np.max(np.abs(ST[:,k])):11.2f} "
                         f"{self.effort[k]:7.1f}  {'YES' if SAT[:,k].any() else 'no ':3s}"
                         + (f"   {steps[i]}" if any(steps) else ""))
        if any(steps):
            lines += ["", "Per step: time span, peak |torque| (N*m; slide N) per motor"]
            order = []
            for s_ in steps:
                if not order or order[-1] != s_:
                    order.append(s_)
            for s_ in order:
                ii = [i for i, x in enumerate(steps) if x == s_]
                pk = np.max(np.abs(TAU[ii]), axis=0)
                lines.append(f"  {t[ii[0]]:6.2f}-{t[ii[-1]]:6.2f} s  {s_:42s} " +
                             " ".join(f"{n}={v:.1f}" for n, v in zip(self.names, pk)))
        SB = [r["stab"] for r in log]
        im = int(np.argmin([b["margin"] for b in SB]))
        idm = int(np.argmin([b["drive_margin"] for b in SB]))
        NW = np.array([b["N"] for b in SB])
        iw = np.unravel_index(int(np.argmin(NW)), NW.shape)
        bend = np.array([math.hypot(b["mount_M"][0], b["mount_M"][1]) for b in SB])
        ib = int(np.argmax(bend))
        lines += ["", f"Tipping (rover {P.CHASSIS['mass']} kg, wheelbase {P.WHEELS['wheelbase']} m, "
                      f"track {P.WHEELS['track']} m, mount at {P.ARM_MOUNT}, slope {P.SLOPE['deg']} deg):",
                  f"  least margin           {SB[im]['margin']:6.3f} m toward the {SB[im]['edge']} at {t[im]:.2f} s"
                  + (f"  ({steps[im]})" if steps[im] else "") + ("   TIPS OVER" if SB[im]['margin'] < 0 else ""),
                  f"  least margin if braking/turning at {P.DRIVE_ACCEL} m/s^2: {SB[idm]['drive_margin']:6.3f} m "
                  f"toward the {SB[idm]['drive_edge']} at {t[idm]:.2f} s",
                  f"  lightest wheel         {am.WHEEL_NAMES[iw[1]]} {NW[iw]:7.1f} N at {t[iw[0]]:.2f} s"
                  + ("   LIFTS OFF" if NW[iw] < 0 else ""),
                  f"  arm mount: peak bending {bend[ib]:.1f} N*m at {t[ib]:.2f} s, peak vertical "
                  f"{max(b['mount_F'][2] for b in SB):.1f} N, peak twist {max(abs(b['mount_M'][2]) for b in SB):.1f} N*m"]
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
        # the rover-mounted masses move with the rover (already in world coordinates)
        np.savez_compressed(
            folder / "motion.npz", t=t, tau=TAU, static=ST, sat=SAT, effort=self.effort, tcp=TCP,
            segs=np.array([r["segs"] for r in log]), pivots=np.array([r["pivots"] for r in log]),
            masses=np.array([r["masses"] for r in log]), mass_kinds=np.array(kinds), mass_kg=np.array(masses_kg),
            joint_names=np.array(self.names), title=np.array(self.rec_name),
            prismatic=np.array([am.is_prismatic(P, j) for j in am.JOINT_ORDER]),
            base_pos=np.array([r["base_pos"] for r in log]),
            rover=np.array([*am.rover_geom(P)["center"], P.CHASSIS["length"], P.CHASSIS["width"],
                            P.CHASSIS["clearance"], P.CHASSIS["deck_height"]]),
            turntable=np.array([am.base_height(P)]),
            wheels=am.wheel_points(P), cop=np.array([r["stab"]["cop_w"] for r in log]),
            margin=np.array([r["stab"]["margin"] for r in log]),
            rover_pose=np.array([r["rover"] for r in log]),
            obj_c=np.array([r["obj_c"] for r in log]), obj_R=np.array([r["obj_R"] for r in log]),
            obj_h=log[0]["obj_h"], obj_col=log[0]["obj_col"], show_objs=np.array(any(r["step"] for r in log)),
            step=np.array([r["step"] for r in log]),
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
        frames = am.fk(self.P, q, rover=self.rover_state())[0]
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
        frames = am.fk(self.P, q, rover=self.rover_state())[0]
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

    def _draw_task(self):
        """Box, drawer cabinet, box zone and reach map."""
        if not hasattr(self.viewer, "log_shapes"):
            return
        hide = not self.show_task
        if hide and not getattr(self, "_task_drawn", False):
            return
        dev = self.model.device
        q, _ = self.joint_state()
        rs = ms.resolve(self.P, q, self.rover_state(), self.att)
        objs = sc.scene_objects(self.P, dict(drawer=rs["drawer"], box_T=rs["box_T"]))
        for o in objs:
            self.viewer.log_shapes(f"/task/{o['name'].replace(' ', '_')}", newton.GeoType.BOX, tuple(float(v) for v in o["h"]),
                                   wp.array([wp.transform(wp.vec3(*o["c"]), quat_of(o["R"]))], dtype=wp.transform, device=dev),
                                   wp.array([wp.vec3(*o["color"])], dtype=wp.vec3, device=dev), hidden=hide)
        Z = self.P.BOX_ZONE
        c = [np.array([x, y, 0.005]) for x, y in ((Z["x"][0], Z["y"][0]), (Z["x"][1], Z["y"][0]),
                                                  (Z["x"][1], Z["y"][1]), (Z["x"][0], Z["y"][1]))]
        self.viewer.log_lines("/task/zone", wp.array(c, dtype=wp.vec3, device=dev),
                              wp.array(c[1:] + c[:1], dtype=wp.vec3, device=dev),
                              wp.array([wp.vec3(0.2, 0.5, 1.0)] * 4, dtype=wp.vec3, device=dev), hidden=hide)
        if self.reach is not None and hasattr(self.viewer, "log_points"):
            xs, ys, codes = self.reach
            pts = [wp.vec3(x, y, 0.01) for y in ys for x in xs]
            cols = [wp.vec3(*REACH_COLORS[k]) for k in codes.reshape(-1)]
            self.viewer.log_points("/task/reach", wp.array(pts, dtype=wp.vec3, device=dev),
                                   wp.array([0.012] * len(pts), dtype=wp.float32, device=dev),
                                   wp.array(cols, dtype=wp.vec3, device=dev), hidden=hide)
        self._task_drawn = True

    def _draw_stability(self):
        """Wheel footprint on the ground and the centre of pressure (green / orange / red)."""
        if not hasattr(self.viewer, "log_lines") or self.stab is None:
            return
        dev = self.model.device
        R0, p0 = am.rover_frame(self.rover_state())
        w = [p0 + R0 @ v + np.array([0, 0, 0.006]) for v in am.wheel_points(self.P)[[0, 1, 3, 2]]]
        m = self.stab["margin"]
        col = (0.1, 0.8, 0.2) if m > 0.05 else (1.0, 0.6, 0.0) if m >= 0 else (1.0, 0.1, 0.1)
        self.viewer.log_lines("/stability/footprint", wp.array(w, dtype=wp.vec3, device=dev),
                              wp.array(w[1:] + w[:1], dtype=wp.vec3, device=dev),
                              wp.array([wp.vec3(*col)] * 4, dtype=wp.vec3, device=dev))
        if hasattr(self.viewer, "log_points"):
            c = p0 + R0 @ np.array([self.stab["cop"][0], self.stab["cop"][1], 0.01])
            self.viewer.log_points("/stability/cop", wp.array([c], dtype=wp.vec3, device=dev),
                                   wp.array([0.025], dtype=wp.float32, device=dev),
                                   wp.array([wp.vec3(*col)], dtype=wp.vec3, device=dev))

    def render(self):
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        self._draw_stability()
        self._draw_task()
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

    def _platform_gui(self, ui, busy):
        """Chassis size / height / mass / COM, arm mount position, wheels, slope.
        Changes rebuild the model when a slider is released (or Enter is pressed)."""
        P = self.P
        C, W, S = P.CHASSIS, P.WHEELS, P.SLOPE
        items = [
            ("chassis length (m)", C, "length", 0.3, 2.0), ("chassis width (m)", C, "width", 0.3, 1.5),
            ("deck height (m)", C, "deck_height", 0.2, 1.2), ("ground clearance (m)", C, "clearance", 0.05, 0.6),
            ("chassis mass (kg)", C, "mass", 5.0, 150.0),
            ("chassis COM x (m)", C, ("com", 0), -0.6, 0.6), ("chassis COM y (m)", C, ("com", 1), -0.5, 0.5),
            ("chassis COM height (m)", C, ("com", 2), 0.05, 1.0),
            ("arm mount x (m)", P, ("ARM_MOUNT", 0), -0.8, 0.8), ("arm mount y (m)", P, ("ARM_MOUNT", 1), -0.6, 0.6),
            ("wheelbase (m)", W, "wheelbase", 0.2, 2.0), ("track (m)", W, "track", 0.2, 1.5),
            ("wheel diameter (m)", W, "diameter", 0.05, 0.6), ("rocker pivot x (m)", W, "pivot_x", -0.8, 0.8),
            ("slope (deg)", S, "deg", 0.0, 30.0), ("downhill direction (deg)", S, "downhill_deg", -180.0, 180.0),
        ]
        changed = False
        for label, holder, key, lo, hi in items:
            if isinstance(key, tuple):
                name, idx = key
                cur_t = getattr(holder, name) if holder is P else holder[name]
                cur = cur_t[idx]
            else:
                cur = holder[key]
            v, _, done = self._slider_box(ui, f"{label}##plat", cur, lo, hi)
            if done and abs(v - cur) > 1e-9:
                if isinstance(key, tuple):
                    lst = list(cur_t)
                    lst[idx] = round(float(v), 4)
                    if holder is P:
                        setattr(P, name, tuple(lst))
                    else:
                        holder[name] = tuple(lst)
                else:
                    holder[key] = round(float(v), 4)
                changed = True
        P.DRIVE_ACCEL = round(float(self._slider_box(ui, "braking/turning check (m/s^2)##plat", P.DRIVE_ACCEL,
                                                     0.0, 5.0)[0]), 3)
        if changed and not busy:
            try:
                self.rebuild()
                self.status = "platform updated (not saved)"
            except Exception as e:
                self.status = f"platform change failed: {e}"
        ui.text_wrapped("World origin stays under the arm mount: moving the mount moves the chassis and wheels. "
                        "The slope tilts gravity; the view stays level.")
        if ui.button("Save platform to arm_params.py##plat"):
            try:
                params_io.save(P, PARAMS_FILE, params_io.platform_blocks_for(P))
                build_urdf.write(P, HERE)
                self.status = "saved platform to arm_params.py (backup .bak) and urc_arm.urdf"
            except Exception as e:
                self.status = f"save failed: {e}"

    def _task_gui(self, ui, busy):
        P = self.P
        idle = not busy and self.bg is None
        _, self.show_task = ui.checkbox("Show task scene (and check collisions with it)##task", self.show_task)
        ui.text("Box (centre on the ground, world m; yaw deg)")
        Z = P.BOX_ZONE
        bx, by, byaw = P.BOX_POSE
        nbx, c1, _ = self._slider_box(ui, "box x##task", bx, Z["x"][0], Z["x"][1])
        nby, c2, _ = self._slider_box(ui, "box y##task", by, Z["y"][0], Z["y"][1])
        nyaw, c3, _ = self._slider_box(ui, "box yaw##task", byaw, -180.0, 180.0, "%.1f")
        if (c1 or c2 or c3) and idle:
            P.BOX_POSE = (round(float(nbx), 4), round(float(nby), 4), round(float(nyaw), 2))
            if self.att["box"] == "ground":
                self.att["box_T"] = sc.box_ground_pose(P)
        if ui.button("Random spot in zone##task") and idle:
            rng = np.random.default_rng()
            P.BOX_POSE = (round(float(rng.uniform(*Z["x"])), 3), round(float(rng.uniform(*Z["y"])), 3), P.BOX_POSE[2])
            if self.att["box"] == "ground":
                self.att["box_T"] = sc.box_ground_pose(P)
        jn = self.names[4]                     # wrist_roll
        _, v = ui.checkbox("auto wrist roll (square the jaws to each handle, box square to the tray)##task",
                           bool(P.MISSION.get("auto_roll", True)))
        P.MISSION["auto_roll"] = v
        auto = v
        ui.text(f"Grasps: gripper pitch (deg, -90 = down)" + ("" if auto else f" / {jn} (deg)"))
        for name, gr in (("box", P.BOX_GRASP), ("drawer", P.DRAWER_GRASP)):
            gr["pitch"] = round(float(self._slider_box(ui, f"{name} pitch##task", gr["pitch"], -90.0, 90.0, "%.1f")[0]), 2)
            if not auto:
                lo, hi = am.from_si(P, "gripper", self.lo[4]), am.from_si(P, "gripper", self.hi[4])
                gr["gripper"] = round(float(self._slider_box(ui, f"{name} {jn}##task", gr["gripper"], lo, hi, "%.1f")[0]), 2)
        ui.text("Rover pose per phase (x, y m; yaw deg)")
        for ph in ("open_drawer", "pick_up", "place"):
            x, y, yaw = P.ROVER_POSES[ph]
            ui.text(f"  {ph}")
            nx = self._slider_box(ui, f"x##{ph}", x, -2.0, 3.0)[0]
            ny = self._slider_box(ui, f"y##{ph}", y, -2.0, 2.0)[0]
            nyaw = self._slider_box(ui, f"yaw##{ph}", yaw, -180.0, 180.0, "%.1f")[0]
            P.ROVER_POSES[ph] = (round(float(nx), 4), round(float(ny), 4), round(float(nyaw), 2))
            ui.same_line()
            if ui.button(f"drive here##{ph}") and idle:
                self.drive_rover((nx, ny, math.radians(nyaw)))
        q_home = np.array(am.home_q(P))
        if ui.button("Check mission##task") and idle:
            self._run_bg("check", lambda prog: ms.check_mission(P, q_home, self.coll_elems, self.inertials))
        ui.same_line()
        if ui.button("Find rover poses##task") and idle:
            self._run_bg("rover poses", lambda prog: ms.find_rover_poses(P, q_home, self.coll_elems, self.inertials, prog))
        ui.same_line()
        if ui.button("Reach map##task") and idle:
            rv = np.array(P.ROVER_POSES["pick_up"], float)
            rv[2] = math.radians(rv[2])
            self._run_bg("reach map", lambda prog: ms.reach_map(P, q_home, self.coll_elems, self.inertials, rv, prog))
        if self.mission is not None:
            done = sum(sg.T for sg in self.mission[:self.mseg_i]) + self.mseg_t
            ui.text(f"running: {done:.1f} / {self.mission_T:.1f} s")
            if ui.button("Stop mission##task"):
                self.mission = None
                self.rec_phase = None
                self.move_from = self.move_to = self.cmd.copy()
                self.move_t = self.move_T
                self.rmove_from = self.rmove_to = self.rcmd.copy()
                self.rmove_t = self.rmove_T
                self.status = "mission stopped"
        else:
            if ui.button("Run mission##task") and idle:
                self.start_mission(self.rec_mission)
            ui.same_line()
            _, self.rec_mission = ui.checkbox("record##task", self.rec_mission)
            ui.same_line()
            if ui.button("Reset scene##task") and idle:
                self.reset_task()
                self.status = "scene reset"
        if ui.button("Save task settings to arm_params.py##task"):
            try:
                params_io.save(P, PARAMS_FILE, params_io.task_blocks_for(P))
                self.status = "saved task settings to arm_params.py (backup .bak)"
            except Exception as e:
                self.status = f"save failed: {e}"
        for line in self.task_lines[:14]:
            col = (1.0, 0.4, 0.4, 1.0) if any(k in line for k in ("COLLISION", "over limit", "STOPPED", "jaws", "no clear")) \
                else (0.85, 0.85, 0.85, 1.0)
            ui.push_text_wrap_pos(0.0)
            ui.text_colored(ui.ImVec4(*col), line)
            ui.pop_text_wrap_pos()

    def gui(self, ui):
        P = self.P
        ui.text_wrapped(f"Status: {self.status}")
        ch, self.check_coll = ui.checkbox("Check collisions##coll", self.check_coll)
        if ch and not self.check_coll:
            self.hits = []
        ui.separator()

        busy = self.rec_phase is not None or self.mission is not None
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

        if ui.collapsing_header("Platform and stability"):
            self._platform_gui(ui, busy)

        if ui.collapsing_header("Task: box -> drawer"):
            self._task_gui(ui, busy)

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
        r = self.rover_state()
        _, _, tcp = am.fk(P, q, rover=r)
        ui.text(f"TCP x {tcp[0]:.3f}  y {tcp[1]:.3f}  z {tcp[2]:.3f} m (world)")
        ui.text(f"Rover x {r[0]:+.3f}  y {r[1]:+.3f} m  yaw {math.degrees(r[2]):+.1f} deg"
                + ("   box held" if self.att["box"] == "gripper" else ""))
        if not self.check_coll:
            ui.text("Collisions: not checked")
        elif self.hits:
            ui.push_text_wrap_pos(0.0)
            ui.text_colored(ui.ImVec4(1.0, 0.3, 0.3, 1.0), "COLLISION: " + am.describe_hits(self.hits, 4))
            ui.pop_text_wrap_pos()
        else:
            ui.text_colored(ui.ImVec4(0.4, 0.9, 0.4, 1.0), "Collisions: clear")
        if self.stab is not None:
            st, wd = self.stab, self.stab_drive
            m = st["margin"]
            col = (0.4, 0.9, 0.4, 1.0) if m > 0.05 else (1.0, 0.7, 0.2, 1.0) if m >= 0 else (1.0, 0.3, 0.3, 1.0)
            ui.text_colored(ui.ImVec4(*col), f"Tip margin {m:+.3f} m ({st['edge']})" + ("  TIPPING" if m < 0 else "")
                            + f"   braking/turning {P.DRIVE_ACCEL} m/s^2: {wd['margin']:+.3f} m ({wd['edge']})")
            ui.text("Wheels N:  " + "  ".join(f"{n} {v:5.0f}" for n, v in zip(am.WHEEL_NAMES, st["N"]))
                    + ("  LIFT" if st["lift"] else ""))
            mF, mM = st["mount_F"], st["mount_M"]
            ui.text(f"Arm mount: Fz {mF[2]:.0f} N, shear {math.hypot(mF[0], mF[1]):.0f} N, "
                    f"bending {math.hypot(mM[0], mM[1]):.1f} N*m, twist {mM[2]:.1f} N*m")
        ui.text("Torque N*m (slide: force N):  motor | static | limit")
        static = am.static_torques(P, q, self.inertials_now(), rover=r)
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
        static = am.static_torques(self.P, q, self.inertials, rover=self.rover_state())
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
