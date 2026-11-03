"""
Box -> drawer mission: plan, kinematic check, rover-pose search and reach map.

The mission is a list of segments. Each segment knows how long it takes and,
for any progress s in [0, 1], the arm joint values and the rover pose. The
sim plays the same segments through the physics; check_mission() replays
them kinematically (no physics) to find problems quickly.

Phases (rover pose per phase from ROVER_POSES):
  open_drawer : approach the drawer handle, grip, pull it open, release, back off
  pick_up     : stow, drive, approach above the box handle, grip, lift
  place       : (drive with the box), rise over the tray rim, move over the tray,
                lower, release, back off, stow

Segment kinds:
  "joint"  : straight line in joint space (quintic timing)
  "linear" : straight line of the TCP at fixed gripper pitch (IK along the path)
  "rover"  : the rover drives between poses; the arm holds
  "wait"   : gripper closes / opens; an event attaches or releases an object
"""
import math
import time

import numpy as np

import arm_model as am
import scene as sc

GRIP_PARTS = {"gripper", "gripper_mechanism", "camera"}
HANDLES = {"box handle", "drawer handle"}


def skip_pair(arm_part, scene_part):
    """The gripper's own parts may overlap a handle (the jaws go around it)."""
    return scene_part in HANDLES and arm_part in GRIP_PARTS


# ------------------------------------------------------------------ attachment state
def initial_state(P, box_pose=None):
    return dict(box="ground", box_T=sc.box_ground_pose(P, box_pose), box_rel=None,
                drawer_d=0.0, drawer_held=False, drawer_ref=None, drawer_d0=0.0)


def resolve(P, q, rover, att, frames=None):
    """Positions of everything for joint values q, rover pose and attachments."""
    if frames is None:
        frames = am.fk(P, q, rover=rover)[0]
    Rt, tcp = sc.tcp_frame(P, frames)
    d = att["drawer_d"]
    if att["drawer_held"]:
        Rd, _ = sc.drawer_frame(P)
        d = att["drawer_d0"] + float((tcp - att["drawer_ref"]) @ Rd[:, 0])
        d = min(max(d, 0.0), sc.drawer_dims(P)["open"])
    box_T = None
    if att["box"] == "ground":
        box_T = att["box_T"]
    elif att["box"] == "gripper":
        Rg, pg = frames["gripper"]
        Rr, pr = att["box_rel"]
        box_T = (Rg @ Rr, pg + Rg @ pr)
    elif att["box"] == "drawer":
        Rd, o = sc.drawer_frame(P)
        Rr, pr = att["box_rel"]
        box_T = (Rd @ Rr, o + Rd @ (pr + np.array([d, 0, 0])))
    return dict(frames=frames, tcp=tcp, drawer=d, box_T=box_T)


def held_box_inertials(P, inertials, att):
    """Body inertials with the held box added to the gripper."""
    if att["box"] != "gripper":
        return inertials
    m, com, I = sc.box_inertia_local(P)
    Rr, pr = att["box_rel"]
    g = inertials["gripper"]
    c_box = pr + Rr @ com
    M = g["mass"] + m
    c = (g["mass"] * g["com"] + m * c_box) / M
    Ib = Rr @ I @ Rr.T

    def shift(Ic, mass, r):
        return Ic + mass * (r @ r * np.eye(3) - np.outer(r, r))
    out = dict(inertials)
    out["gripper"] = dict(mass=M, com=c, inertia=shift(g["inertia"], g["mass"], g["com"] - c) + shift(Ib, m, c_box - c))
    return out


def collisions(P, q, rover, att, elems, rs=None):
    rs = rs or resolve(P, q, rover, att)
    st = dict(drawer=rs["drawer"], box_T=None if att["box"] == "gripper" else rs["box_T"])
    extra = sc.gripper_box_elements(P, att["box_rel"]) if att["box"] == "gripper" else None
    return am.check_collisions(P, q, elems, frames=rs["frames"], extra=extra,
                               scene=sc.scene_objects(P, st), skip=skip_pair)


# ------------------------------------------------------------------ segments
def _quintic_T(dist, vmax):
    return 1.875 * abs(dist) / vmax


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Segment:
    def __init__(self, kind, phase, label, T, q0, q1=None, qs=None, r0=None, r1=None, event=None, at="end"):
        self.kind, self.phase, self.label, self.T = kind, phase, label, max(T, 1e-3)
        self.q0 = np.array(q0, float)
        self.q1 = np.array(q1 if q1 is not None else q0, float)
        self.qs = qs                      # linear: (N, 5) samples along s
        self.r0 = np.array(r0, float)
        self.r1 = np.array(r1 if r1 is not None else r0, float)
        if kind == "rover":
            self.r1[2] = self.r0[2] + _wrap(self.r1[2] - self.r0[2])
        self.event, self.at = event, at
        if qs is not None:
            self.sgrid = np.linspace(0, 1, len(qs))
            self.dqs = np.gradient(qs, self.sgrid, axis=0)

    def q(self, s):
        if self.qs is not None:
            return np.array([np.interp(s, self.sgrid, self.qs[:, k]) for k in range(self.qs.shape[1])])
        return self.q0 + (self.q1 - self.q0) * s

    def dq_ds(self, s):
        if self.qs is not None:
            return np.array([np.interp(s, self.sgrid, self.dqs[:, k]) for k in range(self.qs.shape[1])])
        return self.q1 - self.q0

    def rover(self, s):
        return self.r0 + (self.r1 - self.r0) * s

    def drover_ds(self):
        return self.r1 - self.r0


class PlanError(Exception):
    def __init__(self, phase, label, msg):
        super().__init__(f"{phase}: {label}: {msg}")
        self.phase, self.label, self.msg = phase, label, msg


class Planner:
    """Builds the segments in order, tracking the arm, rover and attachments."""

    def __init__(self, P, q, rover, att, elems, elbow="up"):
        self.P, self.q, self.rover, self.att = P, np.array(q, float), np.array(rover, float), dict(att)
        self.elems, self.elbow, self.segs = elems, elbow, []
        self.vmax = np.array([am.joint_limits(P, j)[3] for j in am.JOINT_ORDER])
        self.lo = np.array([am.joint_limits(P, j)[0] for j in am.JOINT_ORDER])
        self.hi = np.array([am.joint_limits(P, j)[1] for j in am.JOINT_ORDER])
        self.phase = ""

    def gdir(self, pitch_deg, rover=None):
        """World direction the gripper points for a pitch (deg), ignoring gripper yaw."""
        R0, _ = am.rover_frame(rover if rover is not None else self.rover)
        p = math.radians(pitch_deg)
        return R0 @ np.array([math.cos(p), 0, math.sin(p)])

    def _ik(self, p, pitch, gj, label, ref=None, elbow=None):
        def coll(qq):
            return collisions(self.P, qq, self.rover, self.att, self.elems)
        mode = elbow or self.elbow
        q, msg = am.ik(self.P, p, pitch, gj, mode, ref if ref is not None else self.q,
                       None if mode == "closest" else coll, rover=self.rover)
        if q is None:
            raise PlanError(self.phase, label, msg)
        return np.array(q)

    def joint_to(self, label, p, pitch, gj):
        q1 = self._ik(p, pitch, gj, label)
        self.joint_q(label, q1)

    def joint_q(self, label, q1):
        q1 = np.clip(q1, self.lo, self.hi)
        T = max(0.3, float(np.max(_quintic_T(q1 - self.q, self.vmax))))
        self.segs.append(Segment("joint", self.phase, label, T, self.q, q1, r0=self.rover))
        self.q = q1

    def linear_to(self, label, p1, pitch, gj):
        p0 = sc.tcp_frame(self.P, am.fk(self.P, self.q, rover=self.rover)[0])[1]
        L = float(np.linalg.norm(np.asarray(p1) - p0))
        n = max(10, int(L / 0.005) + 1)
        qs, ref = [], self.q
        for i, s in enumerate(np.linspace(0, 1, n)):
            q = self._ik(p0 + (np.asarray(p1) - p0) * s, pitch, gj, f"{label} ({s * L:.2f} of {L:.2f} m)",
                         ref=ref, elbow="closest")
            if i > 0 and np.max(np.abs(q - ref)) > 0.5:
                raise PlanError(self.phase, label, "the arm would have to flip to another solution mid-path")
            qs.append(q)
            ref = q
        qs = np.array(qs)
        dq = np.max(np.abs(np.gradient(qs, np.linspace(0, 1, n), axis=0)), axis=0)
        T = max(0.3, _quintic_T(L, self.P.MISSION["linear_speed"]), float(np.max(1.875 * dq / self.vmax)))
        self.segs.append(Segment("linear", self.phase, label, T, qs[0], qs[-1], qs=qs, r0=self.rover))
        self.q = qs[-1]

    def drive_to(self, label, r1):
        r1 = np.array(r1, float)
        if np.allclose(r1, self.rover, atol=1e-6):
            return
        v, w = self.P.MISSION["rover_speed"]
        dist = math.hypot(*(r1[:2] - self.rover[:2]))
        T = max(0.3, _quintic_T(dist, v), _quintic_T(_wrap(r1[2] - self.rover[2]), math.radians(w)))
        self.segs.append(Segment("rover", self.phase, label, T, self.q, r0=self.rover, r1=r1))
        self.rover = r1

    def wait(self, label, event, at):
        self.segs.append(Segment("wait", self.phase, label, self.P.MISSION["grip_time"], self.q,
                                 r0=self.rover, event=event, at=at))
        apply_event(self.P, event, self.q, self.rover, self.att)

    def tcp(self):
        return sc.tcp_frame(self.P, am.fk(self.P, self.q, rover=self.rover)[0])[1]


def apply_event(P, event, q, rover, att):
    """Attach / release objects (changes att in place)."""
    frames = am.fk(P, q, rover=rover)[0]
    rs = resolve(P, q, rover, att, frames)
    if event == "grip drawer":
        att.update(drawer_held=True, drawer_ref=rs["tcp"].copy(), drawer_d0=rs["drawer"])
    elif event == "release drawer":
        att.update(drawer_held=False, drawer_d=rs["drawer"])
    elif event == "grip box":
        Rg, pg = frames["gripper"]
        Rb, pb = rs["box_T"]
        att.update(box="gripper", box_rel=(Rg.T @ Rb, Rg.T @ (pb - pg)))
    elif event == "release box":
        Rd, o = sc.drawer_frame(P)
        Rb, pb = rs["box_T"]
        att.update(box="drawer", box_rel=(Rd.T @ Rb, Rd.T @ (pb - o) - np.array([rs["drawer"], 0, 0])),
                   drawer_d=rs["drawer"])


def rover_poses_rad(P, override=None):
    poses = dict(P.ROVER_POSES)
    if override:
        poses.update(override)
    return {k: np.array([v[0], v[1], math.radians(v[2])]) for k, v in poses.items()}


def build_mission(P, q_home, elems, poses=None, phases=("open_drawer", "pick_up", "place"), start=None):
    """Compile the mission. Returns (segments, final planner). Raises PlanError at the first
    step that can't be planned (unreachable, outside limits, arm would flip)."""
    poses = poses or rover_poses_rad(P)
    M = P.MISSION
    if start is None:
        start = dict(q=q_home, rover=poses[phases[0]] * 0, att=initial_state(P))
    pl = Planner(P, start["q"], start["rover"], start["att"], elems)
    d_open = sc.drawer_dims(P)["open"]
    Rd, _ = sc.drawer_frame(P)
    u = Rd[:, 0]
    box_d = sc.box_dims(P)
    dg, bg = P.DRAWER_GRASP, P.BOX_GRASP
    try:
        _plan_phases(P, pl, phases, poses, q_home, M, d_open, u, box_d, dg, bg)
    except PlanError as e:
        e.segs = pl.segs
        raise
    return pl.segs, pl


def _auto_roll(P):
    return P.MISSION.get("auto_roll", True)


def _closest_in_limits(P, cands_deg, ref_deg=0.0):
    lo, hi, _, _ = am.joint_limits(P, "gripper")
    ok = [c for c in cands_deg if math.radians(c) >= lo - 1e-9 and math.radians(c) <= hi + 1e-9]
    return min(ok, key=lambda c: abs(c - ref_deg)) if ok else None


def grip_roll(P, pl, p, pitch, bar, default, label):
    """Wrist roll (deg) that closes the jaws squarely on a bar at TCP target p."""
    if not _auto_roll(P):
        return default
    q = pl._ik(p, pitch, 0.0, label)
    Rg0 = am.fk(P, q, rover=pl.rover)[0]["gripper"][0]          # gripper frame at roll 0
    e = np.eye(3)
    jl = e["xyz".index(P.GRIPPER["jaw_axis"])]
    a, b = Rg0 @ jl, Rg0 @ np.cross(e[0], jl)                   # jaw(theta) = a cos + b sin
    bar = np.asarray(bar, float)
    th = math.degrees(math.atan2(-(a @ bar), b @ bar))
    r = _closest_in_limits(P, [th, th - 180, th + 180])
    if r is None:
        raise PlanError(pl.phase, label, f"the wrist roll can't reach the {th:.0f} deg needed to square the jaws")
    return r


def _box_tray_error(P, pl, q):
    """Box heading vs the drawer, mod 90 deg (the box is square), in [-45, 45)."""
    rs = resolve(P, q, pl.rover, pl.att)
    Rb, _ = rs["box_T"]
    Rd, _ = sc.drawer_frame(P)
    ang = math.degrees(math.atan2(Rb[1, 0], Rb[0, 0]) - math.atan2(Rd[1, 0], Rd[0, 0]))
    return (ang + 45.0) % 90.0 - 45.0


def place_roll(P, pl, p, pitch, r_now, label):
    """Wrist roll (deg) that lines the held (square) box up with the tray at TCP target p."""
    if not _auto_roll(P):
        return r_now
    def err_at(r):
        """Box-to-tray misalignment at roll r, or None if that pose isn't reachable / collides."""
        try:
            return _box_tray_error(P, pl, pl._ik(p, pitch, r, label))
        except PlanError:
            return None
    # the error changes 1:1 with roll (sign depends on the pose): find it from two rolls
    e0 = err_at(r_now)
    e1 = err_at(r_now + 1.0) if e0 is not None else None
    if e0 is not None and e1 is not None:
        slope = (e1 - e0) if abs(e1 - e0) < 10 else 1.0
        base = [r_now - e0 / slope]
    else:
        base = [r_now + k for k in range(-45, 46, 5)]
    best = None
    for r in [b + k * 90 for b in base for k in (0, 1, -1, 2, -2)]:
        rr = _closest_in_limits(P, [r], r_now)
        if rr is None:
            continue
        e = err_at(rr)
        if e is None:
            continue
        err = abs(e)
        key = (round(err, 2), abs(rr - r_now))
        if best is None or key < best[0]:
            best = (key, rr)
    if best is None or best[0][0] > 1.0:
        raise PlanError(pl.phase, label, "the wrist roll can't line the box up with the tray")
    return best[1]


def _plan_phases(P, pl, phases, poses, q_home, M, d_open, u, box_d, dg, bg):
    for ph in phases:
        pl.phase = ph
        r = poses[ph]
        if ph == "open_drawer":
            if not np.allclose(r, pl.rover):
                pl.joint_q("stow", q_home)
                pl.drive_to("drive to the drawer", r)
            h, bar = sc.drawer_handle(P, pl.att["drawer_d"])
            g = pl.gdir(dg["pitch"])
            roll = grip_roll(P, pl, h, dg["pitch"], bar, dg["gripper"], "square the jaws to the drawer handle")
            pl.joint_to("approach drawer handle", h - g * M["approach"], dg["pitch"], roll)
            pl.linear_to("reach drawer handle", h, dg["pitch"], roll)
            pl.wait("grip drawer handle", "grip drawer", "end")
            pl.linear_to("pull drawer open", h + u * (d_open - pl.att["drawer_d0"]), dg["pitch"], roll)
            pl.wait("release drawer handle", "release drawer", "start")
            pl.linear_to("back off", pl.tcp() - g * M["retract"], dg["pitch"], roll)
        elif ph == "pick_up":
            if not np.allclose(r, pl.rover):
                pl.joint_q("stow", q_home)
                pl.drive_to("drive to the box", r)
            Rb, pb = pl.att["box_T"]
            grasp = pb + Rb @ box_d["grasp"]
            g = pl.gdir(bg["pitch"])
            roll = grip_roll(P, pl, grasp, bg["pitch"], Rb[:, 0], bg["gripper"], "square the jaws to the box handle")
            pl.joint_to("approach box handle", grasp - g * M["approach"], bg["pitch"], roll)
            pl.linear_to("reach box handle", grasp, bg["pitch"], roll)
            pl.wait("grip box", "grip box", "end")
            pl.linear_to("lift box", grasp + np.array([0, 0, M["lift"]]), bg["pitch"], roll)
        elif ph == "place":
            D = P.DRAWER
            roll = math.degrees(pl.q[4])          # keep the grip's roll
            rim = D["floor_z"] + D["wall_height"]
            hang = _tcp_above_box_bottom(P, pl)
            z_over = rim + M["over_rim"] + hang
            tcp = pl.tcp()
            if tcp[2] < z_over:     # lift to rim height first, then drive with the box up
                pl.linear_to("raise box over the rim height", np.array([tcp[0], tcp[1], z_over]),
                             bg["pitch"], roll)
            pl.drive_to("drive to the drawer with the box", r)
            tray = sc.tray_place_point(P, pl.att["drawer_d"])
            over = np.array([tray[0], tray[1], z_over])
            roll = place_roll(P, pl, over, bg["pitch"], roll, "line the box up with the tray")
            pl.joint_to("move over the tray", over, bg["pitch"], roll)
            pl.linear_to("lower into the drawer", np.array([tray[0], tray[1], D["floor_z"] + M["place_gap"] + hang]),
                         bg["pitch"], roll)
            pl.wait("release box", "release box", "start")
            pl.linear_to("back off", pl.tcp() - pl.gdir(bg["pitch"]) * M["retract"], bg["pitch"], roll)
            # move out from over the drawer (toward its pull-out side) before folding up
            Rd, _ = sc.drawer_frame(P)
            out = sc.drawer_dims(P)["inner"][0] + D["panel"] + M["approach"]   # past the tray's front
            pl.linear_to("move clear of the drawer", pl.tcp() + Rd[:, 0] * out, bg["pitch"], roll)
            pl.joint_q("stow", q_home)


def _tcp_above_box_bottom(P, pl):
    """Height of the TCP above the held box's lowest point."""
    rs = resolve(P, pl.q, pl.rover, pl.att)
    Rb, pb = rs["box_T"]
    corners = []
    for o in sc.box_parts(P, (Rb, pb)):
        for sgn in [(i, j, k) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]:
            corners.append(o["c"] + o["R"] @ (o["h"] * np.array(sgn)))
    return float(rs["tcp"][2] - min(c[2] for c in corners))


# ------------------------------------------------------------------ kinematic check
def replay(P, segs, att0, inertials, elems, dt=0.05, stop_on_problem=False):
    """Step through the segments kinematically. Returns a list of per-segment reports:
    dict(phase, label, T, collisions:set, over_limit:dict, jaw_deg).
    stop_on_problem: return as soon as a collision or torque limit is found (for searches)."""
    att = dict(att0)
    eff = np.array([am.joint_limits(P, j)[2] for j in am.JOINT_ORDER])
    names = [am.joint_name(P, j) for j in am.JOINT_ORDER]
    reports = []
    for sg in segs:
        rep = dict(phase=sg.phase, label=sg.label, T=sg.T, kind=sg.kind, collisions=set(), over_limit={},
                   jaw_deg=None, peak=np.zeros(5), margin=np.inf, edge="", lift=False,
                   drive_margin=np.inf, drive_edge="")
        if sg.kind == "wait" and sg.at == "start":
            apply_event(P, sg.event, sg.q0, sg.r0, att)
        n = max(2, int(sg.T / dt) + 1) if sg.kind != "wait" else 2
        for s in np.linspace(0, 1, n):
            q, r = sg.q(s), sg.rover(s)
            rs = resolve(P, q, r, att)
            for a, b in collisions(P, q, r, att, elems, rs):
                rep["collisions"].add(f"{a} hits {b}")
            ins = held_box_inertials(P, inertials, att)
            tau = am.static_torques(P, q, ins, rover=r)
            st = am.stability(P, q, r, ins)
            if st["margin"] < rep["margin"]:
                rep["margin"], rep["edge"] = st["margin"], st["edge"]
            rep["lift"] |= st["lift"]
            if sg.kind == "rover":      # driving: also check braking / turning
                wd = am.worst_drive(P, q, r, ins)
                if wd["margin"] < rep["drive_margin"]:
                    rep["drive_margin"], rep["drive_edge"] = wd["margin"], wd["edge"]
            rep["peak"] = np.maximum(rep["peak"], np.abs(tau))
            for k in range(5):
                if abs(tau[k]) > eff[k]:
                    rep["over_limit"][names[k]] = max(rep["over_limit"].get(names[k], 0), abs(tau[k]))
            if stop_on_problem and (rep["collisions"] or rep["over_limit"] or rep["margin"] < 0):
                reports.append(rep)
                return reports, att
        if sg.kind == "wait":
            rs = resolve(P, sg.q0, sg.r0, att)
            if sg.event == "grip drawer":
                _, axis = sc.drawer_handle(P, rs["drawer"])
                rep["jaw_deg"] = sc.jaw_error_deg(P, rs["frames"], axis)
                rep["handle_d"] = P.DRAWER["handle"]["diameter"]
            elif sg.event == "grip box":
                Rb, _ = rs["box_T"]
                rep["jaw_deg"] = sc.jaw_error_deg(P, rs["frames"], Rb[:, 0])
                rep["handle_d"] = P.TASK_BOX["handle"]["diameter"]
        if sg.kind == "wait" and sg.at == "end":
            apply_event(P, sg.event, sg.q0, sg.r0, att)
        reports.append(rep)
    return reports, att


def summarize(P, reports, plan_error=None):
    """Short text lines for the panel / summary file."""
    lines = []
    tol = P.MISSION["jaw_tol_deg"]
    ok = plan_error is None
    for i, r in enumerate(reports):
        notes = []
        if r["collisions"]:
            notes.append("COLLISION: " + "; ".join(sorted(r["collisions"])[:3]) +
                         (f" (+{len(r['collisions']) - 3})" if len(r["collisions"]) > 3 else ""))
        for n, v in r["over_limit"].items():
            notes.append(f"{n} over limit ({v:.1f})")
        if r.get("margin", np.inf) < 0:
            notes.append(f"TIPS OVER toward the {r['edge']} ({r['margin']:.2f} m)")
        elif r.get("lift"):
            notes.append("a wheel lifts off")
        if r.get("drive_margin", np.inf) < 0:
            notes.append(f"tips {r['drive_edge']} if braking/turning at {P.DRIVE_ACCEL} m/s^2 ({r['drive_margin']:.2f} m)")
        if r["jaw_deg"] is not None:
            if r["jaw_deg"] > tol:
                notes.append(f"jaws {r['jaw_deg']:.0f} deg off the handle (tol {tol:.0f})")
            if r.get("handle_d", 0) > P.GRIPPER["max_opening"]:
                notes.append(f"handle {r['handle_d'] * 100:.1f} cm > jaw opening {P.GRIPPER['max_opening'] * 100:.1f} cm")
        if notes:
            ok = False
            lines.append(f"{r['phase']}: {r['label']}: " + "; ".join(notes))
    if plan_error is not None:
        lines.append(f"STOPPED at {plan_error}")
    total = sum(r["T"] for r in reports)
    head = ("OK" if ok else "PROBLEMS") + f": {len(reports)} steps, {total:.1f} s"
    if reports:
        i = int(np.argmin([r.get("margin", np.inf) for r in reports]))
        r = reports[i]
        if np.isfinite(r.get("margin", np.inf)):
            dm = [x.get("drive_margin", np.inf) for x in reports]
            head += f"; least tip margin {r['margin']:.2f} m ({r['edge']}, {r['label']})"
            if np.isfinite(min(dm)):
                head += f", {min(dm):.2f} m while driving"
    return ok, [head] + lines


def check_mission(P, q_home, elems, inertials, poses=None):
    """Plan + replay. Returns (ok, lines, segments or None, reports)."""
    try:
        segs, _ = build_mission(P, q_home, elems, poses)
    except PlanError as e:
        # report the steps that could be planned before the failure
        reports, _ = replay(P, e.segs, initial_state(P), inertials, elems)
        ok, lines = summarize(P, reports, str(e))
        return False, lines, None, reports
    reports, _ = replay(P, segs, initial_state(P), inertials, elems)
    ok, lines = summarize(P, reports)
    return ok, lines, segs, reports


# ------------------------------------------------------------------ rover pose search
def find_rover_poses(P, q_home, elems, inertials, progress=None, max_tries=400, y_max=0.5, x_span=1.5):
    """Search each phase's rover pose (in order) for one where that phase plans and
    replays with no collisions or torque limits, as close as possible to the previous
    pose. Returns (poses dict in params units (x, y, yaw_deg), lines)."""
    poses = rover_poses_rad(P)
    lines = []
    start = dict(q=np.array(q_home, float), rover=np.zeros(3), att=initial_state(P))
    prev = np.zeros(3)
    for ph in ("open_drawer", "pick_up", "place"):
        # try driving straight forward/back first, then sideways, then turning
        dxs = sorted(np.arange(-x_span, x_span + 1e-9, 0.05), key=abs)
        dys = sorted(np.arange(-y_max, y_max + 1e-9, 0.05), key=abs)
        cands = [(0, prev + np.array([dx, 0, 0])) for dx in dxs]
        cands += [(1, prev + np.array([dx, dy, 0])) for dy in dys[1:] for dx in dxs[:21]]
        cands += [(2, prev + np.array([dx, 0, math.radians(a)])) for a in (15, -15, 30, -30) for dx in dxs[:21]]
        found = None
        for i, (_, c) in enumerate(cands[:max_tries]):
            if progress and i % 20 == 0:
                progress(f"{ph}: trying rover pose {i}/{min(len(cands), max_tries)}")
            trial = dict(poses)
            trial[ph] = c
            try:
                segs, pl = build_mission(P, q_home, elems, trial, phases=(ph,), start=start)
            except PlanError:
                continue
            reps, _ = replay(P, segs, start["att"], inertials, elems, dt=0.25, stop_on_problem=True)
            good = lambda rr: all(not x["collisions"] and not x["over_limit"] and x["margin"] >= 0 for x in rr)
            if good(reps) and good(replay(P, segs, start["att"], inertials, elems, dt=0.1, stop_on_problem=True)[0]):
                found = (c, pl)
                break
        if found is None:
            lines.append(f"{ph}: no clear rover pose found near ({prev[0]:.2f}, {prev[1]:.2f}); kept the current one")
            try:
                _, pl = build_mission(P, q_home, elems, poses, phases=(ph,), start=start)
            except PlanError as e:
                return None, lines + [f"stopped: {e}"]
        else:
            poses[ph] = found[0]
            pl = found[1]
            lines.append(f"{ph}: rover at x {found[0][0]:+.2f} m, y {found[0][1]:+.2f} m, yaw {math.degrees(found[0][2]):+.0f} deg")
        start = dict(q=pl.q, rover=pl.rover, att=pl.att)
        prev = poses[ph]
    out = {k: (round(float(v[0]), 3), round(float(v[1]), 3), round(math.degrees(v[2]), 1)) for k, v in poses.items()}
    return out, lines


# ------------------------------------------------------------------ reach map
def reach_map(P, q_home, elems, inertials, rover=None, progress=None):
    """For every box position in BOX_ZONE (current box yaw and grasp), can the arm pick the
    box up from the given rover pose? Returns grid xs, ys, codes, labels.
    codes: 0 ok, 1 unreachable / limits, 2 collision, 3 torque over limit, 4 jaws misaligned."""
    Z = P.BOX_ZONE
    step = P.MISSION["reach_step"]
    xs = np.arange(Z["x"][0], Z["x"][1] + 1e-9, step)
    ys = np.arange(Z["y"][0], Z["y"][1] + 1e-9, step)
    rover = np.zeros(3) if rover is None else np.array(rover, float)
    codes = np.zeros((len(ys), len(xs)), int)
    yaw = P.BOX_POSE[2]
    poses = rover_poses_rad(P)
    poses["pick_up"] = rover
    n, t0 = len(xs) * len(ys), time.time()
    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            if progress and (iy * len(xs) + ix) % 10 == 0:
                progress(f"reach map {iy * len(xs) + ix}/{n}")
            att = initial_state(P, (x, y, yaw))
            start = dict(q=np.array(q_home, float), rover=rover, att=att)
            try:
                segs, _ = build_mission(P, q_home, elems, poses, phases=("pick_up",), start=start)
            except PlanError:
                codes[iy, ix] = 1
                continue
            reps, _ = replay(P, segs, att, inertials, elems, dt=0.15)
            if any(r["collisions"] for r in reps):
                codes[iy, ix] = 2
            elif any(r["over_limit"] for r in reps):
                codes[iy, ix] = 3
            elif any(r["jaw_deg"] is not None and r["jaw_deg"] > P.MISSION["jaw_tol_deg"] for r in reps):
                codes[iy, ix] = 4
    labels = ["OK", "out of reach / limits", "collision", "torque over limit", "jaws misaligned"]
    return xs, ys, codes, labels, time.time() - t0
