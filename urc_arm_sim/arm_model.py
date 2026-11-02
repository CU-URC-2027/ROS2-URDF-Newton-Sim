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
JOINT_ORDER = ["base", "shoulder", "elbow", "wrist_pitch", "gripper_yaw"]   # internal keys
BODY_ORDER = ["base", "link1", "link2", "wrist", "gripper"]   # child of each joint
TUBE_LINKS = ["link1", "link2"]
JOINT_AXIS = {  # in the child frame ("base" depends on BASE_JOINT, see joint_axis)
    "shoulder": (0.0, -1.0, 0.0),
    "elbow": (0.0, -1.0, 0.0),
    "wrist_pitch": (0.0, -1.0, 0.0),
    "gripper_yaw": (0.0, 0.0, 1.0),
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
    return j


def joint_axis(P, j):
    if j == "base":
        return (0.0, 1.0, 0.0) if base_slides(P) else (0.0, 0.0, 1.0)
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
        "base": (0.0, 0.0, P.BASE_HEIGHT),                      # in rover frame
        "shoulder": (sx, sy, sz),                               # in base frame
        "elbow": (P.L1, link_geometry(P, "link1")["dy"], 0.0),  # in link1 frame
        "wrist_pitch": (P.L2, link_geometry(P, "link2")["dy"], 0.0),
        "gripper_yaw": (0.0, 0.0, 0.0),
    }


def shoulder_height(P):
    return P.BASE_HEIGHT + P.SHOULDER_PIVOT[2]


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
        ax = np.array(joint_axis(P, j))
        if is_prismatic(P, j):
            p = p + R @ ax * qj
        else:
            R = R @ _rot(ax, qj)
        joints[j] = (R @ ax, p.copy())
        frames[body] = (R.copy(), p.copy())
    Rg, pg = frames["gripper"]
    return frames, joints, pg + Rg @ np.array([P.L_GRIPPER, 0, 0])


def static_torques(P, q, inertials=None, org=None):
    """Motor holding torques (N*m) about each joint axis, sign = joint positive direction.
    For a sliding base it is the holding force (N) along the slide: zero on a level rail."""
    inertials = inertials or body_inertials(P)
    frames, joints, _ = fk(P, q, org)
    coms = {b: frames[b][1] + frames[b][0] @ inertials[b]["com"] for b in BODY_ORDER}
    tau = np.zeros(len(JOINT_ORDER))
    for k, j in enumerate(JOINT_ORDER):
        a, pk = joints[j]
        if is_prismatic(P, j):
            weight = sum(inertials[b]["mass"] for b in BODY_ORDER[k:]) * G
            tau[k] = -np.dot(a, np.array([0, 0, -weight]))
            continue
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
    return dict(segments=segs, pivots=pivots, masses=masses, tcp=tcp, frames=frames,
                base_pos=frames["base"][1])


def ik(P, xyz, pitch_deg, gripper_yaw_deg=0.0, elbow="up", q_current=None, collision_check=None):
    """TCP target (world, m) with gripper pitch (deg) -> joint angles (rad) or (None, reason).
    Base yaw turns the arm plane onto the target (accounting for sideways pivot offsets).
    collision_check: optional f(q) -> list of hits; colliding solutions are dropped and,
    if one elbow choice collides, the other is used."""
    x, y, z = xyz
    phi, psi = math.radians(pitch_deg), math.radians(gripper_yaw_deg)
    c = lateral_offset(P) + P.L_GRIPPER * math.sin(psi)
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
    return [to_si(P, j, P.HOME_DEG[j]) for j in JOINT_ORDER]


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
    R = P.ROVER
    if base_slides(P):   # the post rides on the carriage (base frame origin is at BASE_HEIGHT)
        els.append(dict(name="mount post", body="base", chain=0, kind="capsule",
                        a=np.array([0, 0, R["deck_height"] - P.BASE_HEIGHT + 0.04]),
                        b=np.array([0, 0, -0.06]), r=0.04))
    else:
        els.append(dict(name="mount post", body="rover", kind="capsule", a=np.array([0, 0, R["deck_height"]]),
                        b=np.array([0, 0, P.BASE_HEIGHT - 0.04]), r=0.04))
    return els


def rover_box(P):
    R = P.ROVER
    nose = R["mount_behind_front"]
    lo = np.array([nose - R["length"], -R["width"] / 2, R["ground_clearance"]])
    hi = np.array([nose, R["width"] / 2, R["deck_height"]])
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
    for _ in range(40):
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


def _world(el, frames):
    R, p = frames[el["body"]]
    w = dict(el)
    if el["kind"] == "capsule":
        w["a"], w["b"] = p + R @ el["a"], p + R @ el["b"]
    elif el["kind"] == "sphere":
        w["c"] = p + R @ el["c"]
    else:
        w["c"], w["R"] = p + R @ el["c"], R
    return w


def _pair_hit(x, y, margin):
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
    return False


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
        g = min(c[2] for c in corners) - margin < 0
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


def check_collisions(P, q, elements=None, margin=None, frames=None):
    """List of (part, other) pairs that touch in pose q. Empty list = clear."""
    elements = elements if elements is not None else collision_elements(P)
    margin = getattr(P, "COLLISION_MARGIN", 0.0) if margin is None else margin
    frames = frames or fk(P, q)[0]
    W = [_world(e, frames) for e in elements]
    lo, hi = rover_box(P)
    hits = []
    ci = lambda w: w.get("chain", CHAIN_INDEX[w["body"]])
    for w in W:
        if w["body"] == "rover" or ci(w) == 0:
            continue
        g, rv = _env_hit(w, lo, hi, margin)
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
    return hits


def describe_hits(hits, n=3):
    if not hits:
        return "clear"
    s = "; ".join(f"{a} hits {b}" for a, b in hits[:n])
    return s + (f" (+{len(hits) - n} more)" if len(hits) > n else "")
