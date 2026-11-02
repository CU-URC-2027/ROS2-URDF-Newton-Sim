"""
Shared arm model: geometry (offset pivots), mass properties, forward
kinematics, static holding torques and IK. Used by build_urdf.py, sim_arm.py
and make_video.py.

Frames
  * Each moving body's frame has its origin on its own joint axis.
  * link1 / link2: x points along the PIVOT LINE (this axis -> next axis), so
    the next joint is at (L, dy, 0). Joint angles are between pivot lines,
    exactly like the MATLAB sim, so the planar IK is unchanged.
  * The tube is placed relative to the pivots (LINK_GEOMETRY): it may start
    behind the pivot, stop short of the next one, and sit above/below the
    pivot line (then it is tilted relative to that line).

Static torque = moment of gravity about each motor axis from every mass
beyond it (MATLAB README section 4.5).
"""
import importlib
import math

import numpy as np

G = 9.81
JOINT_ORDER = ["base_yaw", "shoulder", "elbow", "wrist_pitch", "gripper_yaw"]
BODY_ORDER = ["turntable", "link1", "link2", "wrist", "gripper"]   # child of each joint
TUBE_LINKS = ["link1", "link2"]
JOINT_AXIS = {  # in the child frame
    "base_yaw": (0.0, 0.0, 1.0),
    "shoulder": (0.0, -1.0, 0.0),
    "elbow": (0.0, -1.0, 0.0),
    "wrist_pitch": (0.0, -1.0, 0.0),
    "gripper_yaw": (0.0, 0.0, 1.0),
}


def load_params(name="arm_params"):
    mod = importlib.import_module(name)
    return importlib.reload(mod)


def _ry(a):
    """Rotation about +y by a (URDF pitch convention): x -> (cos a, 0, -sin a)."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def link_length(P, link):
    return {"link1": P.L1, "link2": P.L2}[link]


def link_geometry(P, link):
    """Tube placement for link1/link2 in the link frame.
    Returns dict: T (tube length), gamma (tube tilt, rad, URDF pitch),
    R (tube axes -> link axes), start/end/center (link frame), dy (next joint y)."""
    L = link_length(P, link)
    g = P.LINK_GEOMETRY[link]
    n = np.array(g["near"], float)
    f = np.array(g["far"], float)
    dz = f[2] - n[2]
    if abs(dz) >= L:
        raise ValueError(f"{link}: near/far z offsets differ by {abs(dz):.3f} m, more than L = {L} m")
    T = n[0] - f[0] + math.sqrt(L * L - dz * dz)
    if T <= 0.01:
        raise ValueError(f"{link}: x offsets leave a tube length of {T:.3f} m")
    # pivot line in tube axes: from n to (T + fx, *, fz); planar angle:
    ang = math.atan2(dz, T + f[0] - n[0])
    R = _ry(ang)                       # tube axes -> link axes (rotates the pivot line onto +x)

    def to_link(p_tube):
        return R @ (np.array(p_tube, float) - n)

    start, end = to_link((0, 0, 0)), to_link((T, 0, 0))
    return dict(T=T, gamma=ang, R=R, start=start, end=end, center=0.5 * (start + end),
                dy=f[1] - n[1], to_link=to_link, near=n, far=f)


def anchor_to_body(P, body, anchor, xyz):
    """Convert an item's anchor + xyz to its position in the body frame."""
    xyz = np.array(xyz, float)
    if anchor in (None, "pivot"):
        return xyz
    if body not in TUBE_LINKS:
        raise ValueError(f"anchor '{anchor}' only works on link1/link2 (got body '{body}')")
    geo = link_geometry(P, body)
    if anchor == "tube_start":
        return geo["to_link"](xyz)
    if anchor == "tube_end":
        return geo["to_link"](np.array([geo["T"], 0, 0]) + xyz)
    raise ValueError(f"unknown anchor '{anchor}'")


def joint_origins(P):
    """Origin of each joint in its parent body's frame."""
    sx, sy, sz = P.SHOULDER_PIVOT
    return {
        "base_yaw": (0.0, 0.0, P.TURNTABLE_HEIGHT),            # in rover frame
        "shoulder": (sx, sy, sz),                               # in turntable frame
        "elbow": (P.L1, link_geometry(P, "link1")["dy"], 0.0),  # in link1 frame
        "wrist_pitch": (P.L2, link_geometry(P, "link2")["dy"], 0.0),
        "gripper_yaw": (0.0, 0.0, 0.0),
    }


def shoulder_height(P):
    return P.TURNTABLE_HEIGHT + P.SHOULDER_PIVOT[2]


def lateral_offset(P):
    """Sideways (y) offset of the arm plane from the turntable axis."""
    return P.SHOULDER_PIVOT[1] + link_geometry(P, "link1")["dy"] + link_geometry(P, "link2")["dy"]


# ------------------------------------------------------------------ mass parts
def _box_inertia(m, sx, sy, sz):
    return np.diag([m * (sy**2 + sz**2), m * (sx**2 + sz**2), m * (sx**2 + sy**2)]) / 12.0


def _tube_inertia(rho, L, b, h, t):
    """Rectangular tube along x: width b (y), height h (z), wall t. Returns mass, inertia."""
    m_out = rho * L * b * h
    I = _box_inertia(m_out, L, b, h)
    if t > 0 and b > 2 * t and h > 2 * t:
        bi, hi = b - 2 * t, h - 2 * t
        m_in = rho * L * bi * hi
        return m_out - m_in, I - _box_inertia(m_in, L, bi, hi)
    return m_out, I


def _cyl_inertia(m, r, l, axis):
    ia, it = 0.5 * m * r**2, m * (3 * r**2 + l**2) / 12.0
    d = [it, it, it]
    d["xyz".index(axis)] = ia
    return np.diag(d)


def body_parts(P):
    """Every mass on every body: dicts with name, kind, mass, com (body frame),
    inertia (about com, body axes), drawing info."""
    parts = {b: [] for b in ["rover"] + BODY_ORDER}
    tubes = {}
    for link in TUBE_LINKS:
        geo = link_geometry(P, link)
        tubes[link] = (geo["T"], geo["center"], geo["R"], geo["gamma"])
    tubes["gripper"] = (P.L_GRIPPER, np.array([P.L_GRIPPER / 2, 0, 0]), np.eye(3), 0.0)
    total_len = sum(t[0] for t in tubes.values())
    for body, (T, c, R, gam) in tubes.items():
        s = P.SECTION_MM[body]
        b, h, t = s["width"] / 1000, s["height"] / 1000, s["wall"] / 1000
        m, I = _tube_inertia(P.DENSITY, T, b, h, t)
        parts[body].append(dict(name=f"{body}_tube", kind="tube", mass=m, com=c, inertia=R @ I @ R.T,
                                size=(T, b, h), rpy=(0.0, gam, 0.0)))
        mw = P.WIRING_MASS * T / total_len
        if mw > 0:
            parts[body].append(dict(name=f"{body}_wiring", kind="wiring", mass=mw, com=c,
                                    inertia=R @ _box_inertia(mw, T, 0, 0) @ R.T, size=None))
    for name, mo in P.MOTORS.items():
        ax = "z" if JOINT_AXIS[mo["drives"]][2] != 0 else "y"
        parts[mo["body"]].append(dict(
            name=f"motor_{name}", kind="motor", mass=mo["mass"],
            com=anchor_to_body(P, mo["body"], mo.get("anchor", "pivot"), mo["xyz"]),
            inertia=_cyl_inertia(mo["mass"], mo["radius"], mo["length"], ax),
            size=(mo["radius"], mo["length"]), axis=ax))
    for name, ex in P.EXTRA_MASSES.items():
        parts[ex["body"]].append(dict(
            name=name, kind="extra", mass=ex["mass"],
            com=anchor_to_body(P, ex["body"], ex.get("anchor", "pivot"), ex["xyz"]),
            inertia=_box_inertia(ex["mass"], *ex["size"]), size=tuple(ex["size"])))
    pl = P.PAYLOAD
    if pl["mass"] > 0:
        cx = P.L_GRIPPER + pl["size"][0] / 2 - pl["inset"]
        parts["gripper"].append(dict(name="payload", kind="payload", mass=pl["mass"], com=np.array([cx, 0, 0]),
                                     inertia=_box_inertia(pl["mass"], *pl["size"]), size=tuple(pl["size"])))
    return parts


def body_inertials(P, parts=None):
    """Combined mass, COM and inertia (about COM, body axes) of each moving body."""
    parts = parts or body_parts(P)
    out = {}
    for body in BODY_ORDER:
        ps = parts[body]
        m = sum(p["mass"] for p in ps)
        if m <= 0:
            out[body] = dict(mass=1e-3, com=np.zeros(3), inertia=np.eye(3) * 1e-6)
            continue
        com = sum(p["mass"] * p["com"] for p in ps) / m
        if body in P.COM_OVERRIDE:
            com = np.array(P.COM_OVERRIDE[body], float)
        I = np.zeros((3, 3))
        for p in ps:
            d = p["com"] - com
            I += p["inertia"] + p["mass"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        out[body] = dict(mass=m, com=com, inertia=I + np.eye(3) * 1e-6)
    return out


# ------------------------------------------------------------------ kinematics
def _rot(axis, q):
    c, s = math.cos(q), math.sin(q)
    if axis[2] != 0:   # about +z
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])   # about -y: x -> (c, 0, s)


def fk(P, q, org=None):
    """q: 5 joint angles (rad). Returns (frames body->(R,p), joints name->(axis,p), tcp)."""
    org = org or joint_origins(P)
    R, p = np.eye(3), np.zeros(3)
    frames, joints = {"rover": (np.eye(3), np.zeros(3))}, {}
    for j, body, qj in zip(JOINT_ORDER, BODY_ORDER, q):
        p = p + R @ np.array(org[j])
        R = R @ _rot(JOINT_AXIS[j], qj)
        joints[j] = (R @ np.array(JOINT_AXIS[j]), p.copy())
        frames[body] = (R.copy(), p.copy())
    Rg, pg = frames["gripper"]
    return frames, joints, pg + Rg @ np.array([P.L_GRIPPER, 0, 0])


def static_torques(P, q, inertials=None, org=None):
    """Motor holding torques (N*m) about each joint axis, sign = joint positive direction."""
    inertials = inertials or body_inertials(P)
    frames, joints, _ = fk(P, q, org)
    coms = {b: frames[b][1] + frames[b][0] @ inertials[b]["com"] for b in BODY_ORDER}
    tau = np.zeros(len(JOINT_ORDER))
    for k, j in enumerate(JOINT_ORDER):
        a, pk = joints[j]
        moment = np.zeros(3)
        for b in BODY_ORDER[k:]:
            moment += np.cross(coms[b] - pk, np.array([0, 0, -inertials[b]["mass"] * G]))
        tau[k] = -np.dot(a, moment)
    return tau


def skeleton(P, q, parts=None):
    """World-space points for drawing: tube segments, pivots, motor and extra COMs, TCP."""
    parts = parts or body_parts(P)
    frames, joints, tcp = fk(P, q)
    segs = []
    for link in TUBE_LINKS:
        R, p = frames[link]
        geo = link_geometry(P, link)
        segs.append((p + R @ geo["start"], p + R @ geo["end"]))
    Rg, pg = frames["gripper"]
    segs.append((pg, tcp))
    pivots = np.array([joints[j][1] for j in JOINT_ORDER[1:4]])
    masses = []
    for body in ["rover"] + BODY_ORDER:
        R, p = frames[body]
        for part in parts[body]:
            if part["kind"] in ("motor", "extra", "payload"):
                masses.append((part["kind"], part["mass"], p + R @ part["com"]))
    return dict(segments=segs, pivots=pivots, masses=masses, tcp=tcp, frames=frames)


def ik(P, xyz, pitch_deg, gripper_yaw_deg=0.0, elbow="up", q_current=None):
    """TCP target (world, m) with gripper pitch (deg) -> joint angles (rad) or (None, reason).
    Base yaw turns the arm plane onto the target (accounting for sideways pivot offsets)."""
    x, y, z = xyz
    phi, psi = math.radians(pitch_deg), math.radians(gripper_yaw_deg)
    c = lateral_offset(P) + P.L_GRIPPER * math.sin(psi)
    rxy = math.hypot(x, y)
    if rxy <= abs(c) + 1e-6:
        return None, "target too close to the turntable axis"
    r = math.sqrt(rxy * rxy - c * c)               # distance along the arm plane
    base = math.atan2(y, x) - math.atan2(c, r)
    sx, _, _ = P.SHOULDER_PIVOT
    lg = P.L_GRIPPER * math.cos(psi)
    wx = r - lg * math.cos(phi) - sx
    wz = z - lg * math.sin(phi) - shoulder_height(P)
    cb = (wx * wx + wz * wz - P.L1**2 - P.L2**2) / (2 * P.L1 * P.L2)
    if abs(cb) > 1:
        return None, "out of reach"
    sols = []
    for sgn in (1, -1):
        beta = sgn * math.acos(cb)
        t1 = math.atan2(wz, wx) - math.atan2(P.L2 * math.sin(beta), P.L1 + P.L2 * math.cos(beta))
        qq = _wrap_into_limits(P, [base, t1, beta, phi - t1 - beta, psi], q_current)
        if qq is not None:
            sols.append(qq)
    if not sols:
        return None, "outside joint limits"
    if elbow == "closest" and q_current is not None:
        return min(sols, key=lambda s: np.linalg.norm(np.array(s) - q_current)), "ok"
    key = (lambda s: -math.sin(s[1])) if elbow == "up" else (lambda s: math.sin(s[1]))
    return min(sols, key=key), "ok"


def _wrap_into_limits(P, q, q_current=None):
    out = []
    for k, j in enumerate(JOINT_ORDER):
        lo, hi = math.radians(P.JOINTS[j][0]), math.radians(P.JOINTS[j][1])
        ref = q_current[k] if q_current is not None else 0.0
        cands = [q[k] + 2 * math.pi * n for n in (-2, -1, 0, 1, 2)]
        cands = [c for c in cands if lo - 1e-9 <= c <= hi + 1e-9]
        if not cands:
            return None
        out.append(min(cands, key=lambda c: abs(c - ref)))
    return out


def home_q(P):
    return [math.radians(P.HOME_DEG[j]) for j in JOINT_ORDER]
