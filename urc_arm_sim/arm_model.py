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
JOINT_ORDER = ["base", "shoulder", "elbow", "wrist_pitch", "gripper"]   # internal keys
BODY_ORDER = ["base", "link1", "link2", "wrist", "gripper"]   # child of each joint
TUBE_LINKS = ["link1", "link2"]
JOINT_AXIS = {  # in the child frame ("base" depends on BASE_JOINT, see joint_axis)
    "shoulder": (0.0, -1.0, 0.0),
    "elbow": (0.0, -1.0, 0.0),
    "wrist_pitch": (0.0, -1.0, 0.0),
}


def load_params(name="arm_params"):
    mod = importlib.import_module(name)
    return importlib.reload(mod)


# ------------------------------------------------------------------ joints
def base_slides(P):
    kind = getattr(P, "BASE_JOINT", "yaw")
    if kind not in ("slide_y", "yaw"):
        raise ValueError(f"BASE_JOINT must be 'slide_y' or 'yaw' (got '{kind}')")
    return kind == "slide_y"


def is_prismatic(P, j):
    return j == "base" and base_slides(P)


def joint_name(P, j):
    """Name used in the URDF, CSV and readout."""
    if j == "base":
        return "base_slide" if base_slides(P) else "base_yaw"
    if j == "gripper":
        return "wrist_roll"
    return j


def internal_key(P, name):
    """Internal joint key from an internal key or a joint name (e.g. 'wrist_roll' -> 'gripper')."""
    if name in JOINT_ORDER:
        return name
    for j in JOINT_ORDER:
        if joint_name(P, j) == name:
            return j
    if name in ("base_yaw", "base_slide"):
        return "base"
    raise KeyError(f"unknown joint '{name}'")


def joint_axis(P, j):
    j = internal_key(P, j)
    if j == "base":
        return (0.0, 1.0, 0.0) if base_slides(P) else (0.0, 0.0, 1.0)
    if j == "gripper":   # wrist roll: about the gripper's own axis
        return (1.0, 0.0, 0.0)
    return JOINT_AXIS[j]


def joint_limits(P, j):
    """(lower, upper, effort, max speed) in SI: rad or m, N*m or N, rad/s or m/s."""
    lo, hi, eff, vel = P.JOINTS[joint_name(P, j)]
    if is_prismatic(P, j):
        return float(lo), float(hi), float(eff), float(vel)
    return math.radians(lo), math.radians(hi), float(eff), math.radians(vel)


def joint_units(P, j):
    """(position unit, display scale from SI, effort unit) for the UI and files."""
    return ("m", 1.0, "N") if is_prismatic(P, j) else ("deg", 180.0 / math.pi, "N*m")


def to_si(P, j, v):
    return float(v) if is_prismatic(P, j) else math.radians(v)


def from_si(P, j, v):
    return float(v) if is_prismatic(P, j) else math.degrees(v)


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
        "base": (0.0, 0.0, base_height(P)),                     # in rover frame
        "shoulder": (sx, sy, sz),                               # in base frame
        "elbow": (P.L1, link_geometry(P, "link1")["dy"], 0.0),  # in link1 frame
        "wrist_pitch": (P.L2, link_geometry(P, "link2")["dy"], 0.0),
        "gripper": (0.0, 0.0, 0.0),
    }


def base_height(P):
    """Top of the carriage / turntable above the ground."""
    return P.CHASSIS["deck_height"] + P.BASE_ABOVE_DECK


def shoulder_height(P):
    return base_height(P) + P.SHOULDER_PIVOT[2]


def rover_geom(P):
    """Chassis geometry in the rover frame (origin on the ground under the arm mount)."""
    C = P.CHASSIS
    cx, cy = -P.ARM_MOUNT[0], -P.ARM_MOUNT[1]
    return dict(center=(cx, cy), length=C["length"], width=C["width"], clearance=C["clearance"],
                deck=C["deck_height"], nose=cx + C["length"] / 2)


def wheel_points(P):
    """Wheel contact points in the rover frame: FL, FR, RL, RR."""
    g = rover_geom(P)
    cx, cy = g["center"]
    a, b = P.WHEELS["wheelbase"] / 2, P.WHEELS["track"] / 2
    return np.array([[cx + a, cy + b, 0], [cx + a, cy - b, 0], [cx - a, cy + b, 0], [cx - a, cy - b, 0]])


def gravity_vec(P):
    """Gravity in the world frame (tilted for a slope)."""
    S = getattr(P, "SLOPE", dict(deg=0.0, downhill_deg=0.0))
    s, d = math.radians(S["deg"]), math.radians(S["downhill_deg"])
    return G * np.array([math.sin(s) * math.cos(d), math.sin(s) * math.sin(d), -math.cos(s)])


def lateral_offset(P):
    """Sideways (y) offset of the arm plane from the base centre."""
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
        a = joint_axis(P, mo["drives"])
        ax = "xyz"[int(np.argmax(np.abs(a)))]
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
    if axis[0] != 0:   # about +x (wrist roll)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis[2] != 0:   # about +z
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])   # about -y: x -> (c, 0, s)


def rover_frame(rover=None):
    """Rover pose (x, y, yaw_rad) on the ground -> (R, p) of the rover frame in the world."""
    if rover is None:
        return np.eye(3), np.zeros(3)
    x, y, yaw = rover
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), np.array([x, y, 0.0])


def fk(P, q, org=None, rover=None):
    """q: 5 joint values (rad, or m for a slide). rover: (x, y, yaw_rad) or None (= start pose).
    Returns (frames body->(R,p), joints name->(axis,p), tcp), all in the world frame."""
    org = org or joint_origins(P)
    R, p = rover_frame(rover)
    frames, joints = {"rover": (R.copy(), p.copy())}, {}
    for j, body, qj in zip(JOINT_ORDER, BODY_ORDER, q):
        p = p + R @ np.array(org[j])
        ax = np.array(joint_axis(P, j))
        if is_prismatic(P, j):
            p = p + R @ ax * qj
        else:
            R = R @ _rot(ax, qj)
        joints[j] = (R @ ax, p.copy())
        frames[body] = (R.copy(), p.copy())
    Rg, pg = frames["gripper"]
    return frames, joints, pg + Rg @ np.array([P.L_GRIPPER, 0, 0])


def static_torques(P, q, inertials=None, org=None, rover=None):
    """Motor holding torques (N*m) about each joint axis, sign = joint positive direction.
    For a sliding base it is the holding force (N) along the slide: zero on a level rail."""
    inertials = inertials or body_inertials(P)
    frames, joints, _ = fk(P, q, org, rover=rover)
    g = gravity_vec(P)
    coms = {b: frames[b][1] + frames[b][0] @ inertials[b]["com"] for b in BODY_ORDER}
    tau = np.zeros(len(JOINT_ORDER))
    for k, j in enumerate(JOINT_ORDER):
        a, pk = joints[j]
        if is_prismatic(P, j):
            weight = sum(inertials[b]["mass"] for b in BODY_ORDER[k:]) * g
            tau[k] = -np.dot(a, weight)
            continue
        moment = np.zeros(3)
        for b in BODY_ORDER[k:]:
            moment += np.cross(coms[b] - pk, inertials[b]["mass"] * g)
        tau[k] = -np.dot(a, moment)
    return tau


def skeleton(P, q, parts=None, rover=None):
    """World-space points for drawing: tube segments, pivots, motor and extra COMs, TCP."""
    parts = parts or body_parts(P)
    frames, joints, tcp = fk(P, q, rover=rover)
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
    return dict(segments=segs, pivots=pivots, masses=masses, tcp=tcp, frames=frames,
                base_pos=frames["base"][1])


def ik(P, xyz, pitch_deg, gripper_deg=0.0, elbow="up", q_current=None, collision_check=None,
       rover=None):
    """TCP target (world, m) with gripper pitch (deg) -> joint values or (None, reason).
    The base (slide or turntable) brings the arm plane onto the target, allowing for
    sideways pivot offsets. rover: the rover's pose (x, y, yaw_rad), None = start pose.
    collision_check: optional f(q) -> list of hits; colliding solutions are dropped and,
    if one elbow choice collides, the other is used."""
    R0, p0 = rover_frame(rover)
    x, y, z = R0.T @ (np.asarray(xyz, float) - p0)          # target in the rover frame
    phi, psi = math.radians(pitch_deg), math.radians(gripper_deg)
    c = lateral_offset(P)        # the roll leaves the TCP on the gripper axis
    if base_slides(P):
        base = y - c                               # slide so the arm plane passes through the target
        r = x                                      # forward distance in the arm plane
    else:
        rxy = math.hypot(x, y)
        if rxy <= abs(c) + 1e-6:
            return None, "target too close to the turntable axis"
        r = math.sqrt(rxy * rxy - c * c)           # distance along the arm plane
        base = math.atan2(y, x) - math.atan2(c, r)
    sx, _, _ = P.SHOULDER_PIVOT
    lg = P.L_GRIPPER
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
        if base_slides(P):
            lo, hi, _, _ = joint_limits(P, "base")
            if not lo - 1e-9 <= y - c <= hi + 1e-9:
                return None, f"needs the slide at {y - c:+.3f} m (travel {lo:+.2f} to {hi:+.2f} m)"
        return None, "outside joint limits"
    note = "ok"
    if collision_check is not None:
        hits = [collision_check(s_) for s_ in sols]
        clear = [s_ for s_, h in zip(sols, hits) if not h]
        if not clear:
            return None, "every solution collides: " + describe_hits(hits[0], 2)
        if len(clear) < len(sols):
            note = "ok (other elbow choice collides)"
        sols = clear
    if elbow == "closest" and q_current is not None:
        return min(sols, key=lambda s: np.linalg.norm(np.array(s) - q_current)), note
    key = (lambda s: -math.sin(s[1])) if elbow == "up" else (lambda s: math.sin(s[1]))
    return min(sols, key=key), note


def _wrap_into_limits(P, q, q_current=None):
    out = []
    for k, j in enumerate(JOINT_ORDER):
        lo, hi, _, _ = joint_limits(P, j)
        if is_prismatic(P, j):
            if not lo - 1e-9 <= q[k] <= hi + 1e-9:
                return None
            out.append(q[k])
            continue
        ref = q_current[k] if q_current is not None else 0.0
        cands = [q[k] + 2 * math.pi * n for n in (-2, -1, 0, 1, 2)]
        cands = [c for c in cands if lo - 1e-9 <= c <= hi + 1e-9]
        if not cands:
            return None
        out.append(min(cands, key=lambda c: abs(c - ref)))
    return out


def home_q(P):
    H = P.HOME_DEG
    return [to_si(P, j, H.get(j, H.get(joint_name(P, j), 0.0))) for j in JOINT_ORDER]


# ------------------------------------------------------------------ collision checking
# Simplified shapes (like the MATLAB collision model, section 4.8):
#   tubes and the gripper  -> capsules (centre segment + radius = half the larger side)
#   motors                 -> spheres (radius = max(radius, length/2))
#   extra masses           -> spheres (radius = half the largest side)
#   payload                -> oriented box
#   rover body             -> box; mount post -> vertical capsule; ground -> z = 0
# Pairs that are checked: everything moving vs ground; rover body / post / rover
# motors vs link1 and beyond; arm parts vs arm parts two or more bodies apart in
# the chain (adjacent bodies touch at their joint by design); payload vs
# everything not on the gripper.
CHAIN_INDEX = {"rover": 0, "base": 1, "link1": 2, "link2": 3, "wrist": 4, "gripper": 4}


def collision_elements(P, parts=None):
    """Collision shapes in their body frames."""
    parts = parts or body_parts(P)
    els = []
    for link in TUBE_LINKS:
        geo = link_geometry(P, link)
        s = P.SECTION_MM[link]
        els.append(dict(name=f"{link} tube", body=link, kind="capsule", a=geo["start"], b=geo["end"],
                        r=max(s["width"], s["height"]) / 2000))
    s = P.SECTION_MM["gripper"]
    els.append(dict(name="gripper", body="gripper", kind="capsule", a=np.zeros(3),
                    b=np.array([P.L_GRIPPER, 0, 0]), r=max(s["width"], s["height"]) / 2000))
    for body, ps in parts.items():
        for p in ps:
            if p["kind"] == "motor":
                els.append(dict(name=p["name"].replace("motor_", "motor "), body=body, kind="sphere",
                                c=p["com"], r=max(p["size"][0], p["size"][1] / 2)))
            elif p["kind"] == "extra":
                els.append(dict(name=p["name"], body=body, kind="sphere", c=p["com"], r=max(p["size"]) / 2))
            elif p["kind"] == "payload":
                els.append(dict(name="payload", body=body, kind="obb", c=p["com"], h=np.array(p["size"]) / 2))
    deck, bh = P.CHASSIS["deck_height"], base_height(P)
    if base_slides(P):   # the post rides on the carriage (base frame origin is at the base height)
        els.append(dict(name="mount post", body="base", chain=0, kind="capsule",
                        a=np.array([0, 0, deck - bh + 0.04]), b=np.array([0, 0, -0.06]), r=0.04))
    else:
        els.append(dict(name="mount post", body="rover", kind="capsule", a=np.array([0, 0, deck]),
                        b=np.array([0, 0, bh - 0.04]), r=0.04))
    return els


def rover_box(P):
    g = rover_geom(P)
    cx, cy = g["center"]
    lo = np.array([cx - g["length"] / 2, cy - g["width"] / 2, g["clearance"]])
    hi = np.array([cx + g["length"] / 2, cy + g["width"] / 2, g["deck"]])
    return lo, hi


def _seg_seg(p1, q1, p2, q2):
    """Closest distance between segments p1q1 and p2q2 (Ericson 5.1.9)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a <= 1e-12 and e <= 1e-12:
        return np.linalg.norm(r)
    if a <= 1e-12:
        s, t = 0.0, np.clip(f / e, 0, 1)
    else:
        c = d1 @ r
        if e <= 1e-12:
            s, t = np.clip(-c / a, 0, 1), 0.0
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0:
                t, s = 0.0, np.clip(-c / a, 0, 1)
            elif t > 1:
                t, s = 1.0, np.clip((b - c) / a, 0, 1)
    return np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t))


def _pt_seg(p, a, b):
    ab = b - a
    t = np.clip((p - a) @ ab / max(ab @ ab, 1e-12), 0, 1)
    return np.linalg.norm(p - (a + ab * t))


def _pt_aabb(p, lo, hi):
    return np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0))


def _seg_aabb_hit(a, b, lo, hi, r):
    """Segment within r of the box: slab test against the box grown by r."""
    lo, hi = lo - r, hi + r
    d = b - a
    t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if a[k] < lo[k] or a[k] > hi[k]:
                return False
        else:
            u0, u1 = (lo[k] - a[k]) / d[k], (hi[k] - a[k]) / d[k]
            t0, t1 = max(t0, min(u0, u1)), min(t1, max(u0, u1))
            if t0 > t1:
                return False
    return True


def _pt_obb(p, c, R, h):
    loc = R.T @ (p - c)
    return np.linalg.norm(np.maximum(np.abs(loc) - h, 0))


def _seg_obb(a, b, c, R, h):
    """Segment-to-box distance by ternary search (the distance is convex along the segment)."""
    lo_t, hi_t = 0.0, 1.0
    for _ in range(24):
        m1, m2 = lo_t + (hi_t - lo_t) / 3, hi_t - (hi_t - lo_t) / 3
        if _pt_obb(a + (b - a) * m1, c, R, h) < _pt_obb(a + (b - a) * m2, c, R, h):
            hi_t = m2
        else:
            lo_t = m1
    return _pt_obb(a + (b - a) * (lo_t + hi_t) / 2, c, R, h)


def _obb_aabb_overlap(c, R, h, lo, hi):
    """Separating-axis test, oriented box vs axis-aligned box."""
    bc, bh = (lo + hi) / 2, (hi - lo) / 2
    T = c - bc
    axes = [np.eye(3)[i] for i in range(3)] + [R[:, i] for i in range(3)]
    axes += [np.cross(np.eye(3)[i], R[:, j]) for i in range(3) for j in range(3)]
    for L in axes:
        n = np.linalg.norm(L)
        if n < 1e-9:
            continue
        L = L / n
        ra = np.sum(bh * np.abs(L))
        rb = np.sum(h * np.abs(R.T @ L))
        if abs(T @ L) > ra + rb:
            return False
    return True


def _obb_obb_overlap(c1, R1, h1, c2, R2, h2):
    """Separating-axis test between two oriented boxes."""
    T = c2 - c1
    axes = [R1[:, i] for i in range(3)] + [R2[:, i] for i in range(3)]
    axes += [np.cross(R1[:, i], R2[:, j]) for i in range(3) for j in range(3)]
    for L in axes:
        n = np.linalg.norm(L)
        if n < 1e-9:
            continue
        L = L / n
        if abs(T @ L) > np.sum(h1 * np.abs(R1.T @ L)) + np.sum(h2 * np.abs(R2.T @ L)):
            return False
    return True


def _world(el, frames):
    R, p = frames[el["body"]]
    w = dict(el)
    if el["kind"] == "capsule":
        w["a"], w["b"] = p + R @ el["a"], p + R @ el["b"]
    elif el["kind"] == "sphere":
        w["c"] = p + R @ el["c"]
    else:
        w["c"], w["R"] = p + R @ el["c"], R @ el.get("R_local", np.eye(3))
    return w


def _bsphere(w):
    """Bounding sphere (centre, radius) of a world-space element, cached on the dict."""
    if "_bs" not in w:
        if w["kind"] == "capsule":
            w["_bs"] = ((w["a"] + w["b"]) / 2, np.linalg.norm(w["b"] - w["a"]) / 2 + w["r"])
        elif w["kind"] == "sphere":
            w["_bs"] = (w["c"], w["r"])
        else:
            w["_bs"] = (w["c"], float(np.linalg.norm(w["h"])))
    return w["_bs"]


def _pair_hit(x, y, margin):
    (cx, rx), (cy, ry) = _bsphere(x), _bsphere(y)
    if np.linalg.norm(cx - cy) > rx + ry + margin:
        return False
    kx, ky = x["kind"], y["kind"]
    if kx == "obb" and ky != "obb":
        x, y, kx, ky = y, x, ky, kx
    if kx == "capsule" and ky == "capsule":
        return _seg_seg(x["a"], x["b"], y["a"], y["b"]) < x["r"] + y["r"] + margin
    if kx == "capsule" and ky == "sphere":
        return _pt_seg(y["c"], x["a"], x["b"]) < x["r"] + y["r"] + margin
    if kx == "sphere" and ky == "capsule":
        return _pt_seg(x["c"], y["a"], y["b"]) < x["r"] + y["r"] + margin
    if kx == "sphere" and ky == "sphere":
        return np.linalg.norm(x["c"] - y["c"]) < x["r"] + y["r"] + margin
    if ky == "obb" and kx == "capsule":
        return _seg_obb(x["a"], x["b"], y["c"], y["R"], y["h"]) < x["r"] + margin
    if ky == "obb" and kx == "sphere":
        return _pt_obb(x["c"], y["c"], y["R"], y["h"]) < x["r"] + margin
    if kx == "obb" and ky == "obb":
        return _obb_obb_overlap(x["c"], x["R"], x["h"] + margin, y["c"], y["R"], y["h"])
    return False


def _to_frame(w, R0, p0):
    """World-space element -> the frame (R0, p0)."""
    v = dict(w)
    if w["kind"] == "capsule":
        v["a"], v["b"] = R0.T @ (w["a"] - p0), R0.T @ (w["b"] - p0)
    elif w["kind"] == "sphere":
        v["c"] = R0.T @ (w["c"] - p0)
    else:
        v["c"], v["R"] = R0.T @ (w["c"] - p0), R0.T @ w["R"]
    return v


def _env_hit(w, lo, hi, margin):
    """(ground hit, rover-body hit) for one world-space element."""
    k = w["kind"]
    if k == "capsule":
        g = min(w["a"][2], w["b"][2]) - w["r"] - margin < 0
        rv = _seg_aabb_hit(w["a"], w["b"], lo, hi, w["r"] + margin)
    elif k == "sphere":
        g = w["c"][2] - w["r"] - margin < 0
        rv = _pt_aabb(w["c"], lo, hi) < w["r"] + margin
    else:
        corners = [w["c"] + w["R"] @ (w["h"] * np.array(s)) for s in
                   [(i, j, k2) for i in (-1, 1) for j in (-1, 1) for k2 in (-1, 1)]]
        g = min(c[2] for c in corners) - margin < -w.get("ground_tol", 0.0)
        rv = _obb_aabb_overlap(w["c"], w["R"], w["h"] + margin, lo, hi)
    return g, rv


FOLD_PAIRS = {"link1 tube": {"link2 tube"}, "link2 tube": {"gripper"}}


def _fold_hit(x, y, margin):
    """Two jointed tubes folding together: trim 2*(r1+r2) off the ends that meet
    at the joint, then test the rest. Side-by-side tubes (offset >= r1+r2) never hit."""
    trim = 2 * (x["r"] + y["r"])
    ends = [(x["a"], y["a"]), (x["a"], y["b"]), (x["b"], y["a"]), (x["b"], y["b"])]
    i = int(np.argmin([np.linalg.norm(a - b) for a, b in ends]))
    xj, yj = (x["a"] if i < 2 else x["b"]), (y["a"] if i % 2 == 0 else y["b"])
    xo, yo = (x["b"] if i < 2 else x["a"]), (y["b"] if i % 2 == 0 else y["a"])

    def cut(j, o):
        L = np.linalg.norm(o - j)
        return None if L <= trim else (j + (o - j) * trim / L, o)
    xs, ys = cut(xj, xo), cut(yj, yo)
    if xs is None or ys is None:
        return False
    return _seg_seg(xs[0], xs[1], ys[0], ys[1]) < x["r"] + y["r"] + margin


def check_collisions(P, q, elements=None, margin=None, frames=None, rover=None, extra=None,
                     scene=None, skip=None):
    """List of (part, other) pairs that touch in pose q. Empty list = clear.
    rover : rover pose (x, y, yaw_rad); extra: more arm elements (e.g. a held box, in the
    gripper frame); scene: world-fixed boxes (task objects); skip(arm_part, scene_part) ->
    True to ignore a pair (e.g. the gripper around a handle)."""
    elements = elements if elements is not None else collision_elements(P)
    if extra:
        elements = list(elements) + list(extra)
    margin = getattr(P, "COLLISION_MARGIN", 0.0) if margin is None else margin
    frames = frames or fk(P, q, rover=rover)[0]
    R0, p0 = frames["rover"]
    W = [_world(e, frames) for e in elements]
    lo, hi = rover_box(P)
    hits = []
    ci = lambda w: w.get("chain", CHAIN_INDEX[w["body"]])
    for w in W:
        if w["body"] == "rover" or ci(w) == 0:
            continue
        g, rv = _env_hit(_to_frame(w, R0, p0), lo, hi, margin)
        if g:
            hits.append((w["name"], "ground"))
        if rv and (ci(w) >= 2 or w["kind"] == "obb"):
            hits.append((w["name"], "rover body"))
    for i in range(len(W)):
        for j in range(i + 1, len(W)):
            x, y = W[i], W[j]
            if ci(x) == 0 and ci(y) == 0:
                continue
            if x["kind"] == "obb" or y["kind"] == "obb":
                other = y if x["kind"] == "obb" else x
                if other["body"] == "gripper":
                    continue          # the box is held by the gripper
            elif abs(ci(x) - ci(y)) < 2:
                # adjacent bodies meet at their joint; only check two tubes folding onto
                # each other, ignoring the region around the shared joint
                if x["name"] in FOLD_PAIRS and y["name"] in FOLD_PAIRS[x["name"]]:
                    if _fold_hit(x, y, margin):
                        hits.append((x["name"], y["name"]))
                continue
            if _pair_hit(x, y, margin):
                hits.append((x["name"], y["name"]))
    if scene:
        rover_obb = dict(name="rover body", kind="obb", c=p0 + R0 @ ((lo + hi) / 2), R=R0, h=(hi - lo) / 2)
        for sc in scene:
            if _pair_hit(rover_obb, sc, margin):
                hits.append(("rover body", sc["name"]))
            for w in W:
                if skip is not None and skip(w["name"], sc["name"]):
                    continue
                if _pair_hit(w, sc, margin):
                    hits.append((w["name"], sc["name"]))
    return hits


def describe_hits(hits, n=3):
    if not hits:
        return "clear"
    s = "; ".join(f"{a} hits {b}" for a, b in hits[:n])
    return s + (f" (+{len(hits) - n} more)" if len(hits) > n else "")



# ------------------------------------------------------------------ stability (tipping) and mount loads
WHEEL_NAMES = ("FL", "FR", "RL", "RR")


def mass_points(P, q, rover=None, inertials=None, parts=None):
    """Every mass on the rover as (mass, world position): chassis, rover-mounted parts,
    then the arm bodies (base onward). Returns (list, number of rover-side entries)."""
    inertials = inertials or body_inertials(P)
    parts = parts or body_parts(P)
    frames = fk(P, q, rover=rover)[0]
    R0, p0 = frames["rover"]
    g = rover_geom(P)
    cx, cy = g["center"]
    C = P.CHASSIS
    pts = [(C["mass"], p0 + R0 @ np.array([cx + C["com"][0], cy + C["com"][1], C["com"][2]]))]
    for part in parts["rover"]:
        pts.append((part["mass"], p0 + R0 @ part["com"]))
    n_rover = len(pts)
    for b in BODY_ORDER:
        Rb, pb = frames[b]
        pts.append((inertials[b]["mass"], pb + Rb @ inertials[b]["com"]))
    return pts, n_rover


def wheel_loads(P, rover, Fr, Mo):
    """Ground normal force on each wheel (N; FL, FR, RL, RR) for the total ground reaction
    Fr (world) with moment Mo about the world origin. Rocker-differential suspension:
    the two side rockers carry equal moments about their pivots."""
    R0, p0 = rover_frame(rover)
    F = R0.T @ Fr
    M = R0.T @ (Mo - np.cross(p0, Fr))          # about the rover origin, rover axes
    w = wheel_points(P)
    px = rover_geom(P)["center"][0] + P.WHEELS["pivot_x"]
    A = np.array([
        [1, 1, 1, 1],
        w[:, 1],                                   # sum N y = Mx
        -w[:, 0],                                  # -sum N x = My
        [w[0, 0] - px, -(w[1, 0] - px), w[2, 0] - px, -(w[3, 0] - px)],   # left rocker = right rocker
    ])
    rhs = np.array([F[2], M[0], M[1], 0.0])
    return np.linalg.solve(A, rhs), F, M


def support_margin(P, cop):
    """Distance (m) from the centre of pressure to the nearest edge of the wheel
    footprint (rover frame), positive inside, and that edge's name."""
    w = wheel_points(P)
    x0, x1 = w[:, 0].min(), w[:, 0].max()
    y0, y1 = w[:, 1].min(), w[:, 1].max()
    d = {"front": x1 - cop[0], "rear": cop[0] - x0, "left": y1 - cop[1], "right": cop[1] - y0}
    edge = min(d, key=d.get)
    return d[edge], edge


def stability(P, q, rover=None, inertials=None, parts=None, acc=None, extra_acc=None):
    """Tipping check. acc: per-mass accelerations (world, list matching mass_points) or None
    (static); extra_acc: a uniform acceleration of the whole rover (e.g. braking).
    Returns dict(N, cop, margin, edge, lift, mount_F, mount_M, Fz)."""
    pts, n_rover = mass_points(P, q, rover, inertials, parts)
    g = gravity_vec(P)
    Fr, Mo = np.zeros(3), np.zeros(3)
    Fa, Ma = np.zeros(3), np.zeros(3)
    R0, p0 = rover_frame(rover)
    mount = p0 + R0 @ np.array([0, 0, P.CHASSIS["deck_height"]])
    for i, (m, r) in enumerate(pts):
        a = np.zeros(3) if acc is None else acc[i]
        if extra_acc is not None:
            a = a + extra_acc
        f = m * (a - g)                         # force the support must supply to this mass
        Fr += f
        Mo += np.cross(r, f)
        if i >= n_rover:                        # arm: carried by the mount
            Fa += f
            Ma += np.cross(r - mount, f)
    N, F, M = wheel_loads(P, rover, Fr, Mo)
    cop = np.array([-M[1] / F[2], M[0] / F[2]]) if F[2] > 1e-6 else np.array([np.nan, np.nan])
    margin, edge = support_margin(P, cop)
    return dict(N=N, cop=cop, margin=margin, edge=edge, lift=bool((N < 0).any()), Fz=F[2],
                mount_F=R0.T @ Fa, mount_M=R0.T @ Ma)


def drive_cases(P, rover=None):
    """Uniform rover accelerations for the worst-case driving check (world)."""
    a = P.DRIVE_ACCEL
    R0, _ = rover_frame(rover)
    return [R0 @ np.array(v) for v in ((a, 0, 0), (-a, 0, 0), (0, a, 0), (0, -a, 0))]


def worst_drive(P, q, rover=None, inertials=None, parts=None, acc=None):
    """Smallest margin over the drive cases (braking, accelerating, turning both ways)."""
    worst = None
    for ea in drive_cases(P, rover):
        st = stability(P, q, rover, inertials, parts, acc, ea)
        if worst is None or st["margin"] < worst["margin"]:
            worst = st
    return worst
