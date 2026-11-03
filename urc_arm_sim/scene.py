"""
Task scene for the box -> drawer mission: the box with its handle, and the
drawer cabinet with a sliding tray. Pure geometry (no physics): poses, grasp
points, collision boxes and drawing data. Used by mission.py, sim_arm.py and
make_video.py.

All objects are oriented boxes (OBBs): dict(name, group, kind="obb", c, R, h,
color), with c = centre (world), R = rotation (columns = box axes), h = half
sizes.
"""
import math

import numpy as np

import arm_model as am

COLORS = {
    "box": (0.80, 0.62, 0.35), "handle": (0.25, 0.25, 0.25), "cabinet": (0.55, 0.60, 0.68),
    "tray": (0.75, 0.78, 0.82), "panel": (0.42, 0.47, 0.55), "drawer handle": (0.15, 0.15, 0.15),
}


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _obb(name, group, c, R, size, color=None):
    return dict(name=name, group=group, kind="obb", c=np.asarray(c, float), R=np.asarray(R, float),
                h=np.asarray(size, float) / 2, color=color or COLORS.get(group, (0.6, 0.6, 0.6)))


# ------------------------------------------------------------------ box
def box_dims(P):
    """Body size, handle size and the grasp point (handle centre) in the box frame
    (origin at the centre of the bottom face)."""
    B = P.TASK_BOX
    sx, sy, sz = B["size"]
    hd = B["handle"]
    body = np.array([sx, sy, sz - hd["height"]])
    grasp = np.array([0.0, 0.0, body[2] + hd["height"] - hd["diameter"] / 2])
    return dict(body=body, handle_d=hd["diameter"], handle_len=hd["length"], grasp=grasp, mass=B["mass"])


def box_ground_pose(P, pose=None):
    """Box frame (R, p) when sitting on the ground at BOX_POSE = (x, y, yaw_deg)."""
    x, y, yaw = pose if pose is not None else P.BOX_POSE
    return rz(math.radians(yaw)), np.array([x, y, 0.0])


def box_parts(P, T=None, local=False):
    """The box's OBBs: body and handle bar. T = (R, p) of the box frame in the world;
    local=True gives them in the box frame."""
    d = box_dims(P)
    R, p = (np.eye(3), np.zeros(3)) if (local or T is None) else T
    body_c = np.array([0, 0, d["body"][2] / 2])
    bar_c = d["grasp"]
    return [
        _obb("box", "box", p + R @ body_c, R, d["body"]),
        _obb("box handle", "handle", p + R @ bar_c, R, (d["handle_len"], d["handle_d"], d["handle_d"])),
    ]


def box_inertia_local(P):
    """Mass, COM and inertia (about the COM) of the box in its own frame (handle mass ignored)."""
    d = box_dims(P)
    m = d["mass"]
    sx, sy, sz = d["body"]
    I = np.diag([m * (sy ** 2 + sz ** 2), m * (sx ** 2 + sz ** 2), m * (sx ** 2 + sy ** 2)]) / 12.0
    return m, np.array([0, 0, sz / 2]), I


# ------------------------------------------------------------------ drawer
def drawer_frame(P):
    """Drawer frame: origin at the cabinet front face, tray floor height; x = pull-out direction."""
    D = P.DRAWER
    R = rz(math.radians(D["pull_deg"]))
    return R, np.array([D["front"][0], D["front"][1], D["floor_z"]])


def drawer_dims(P):
    D = P.DRAWER
    bx, by, _ = P.TASK_BOX["size"]
    inner = np.array([bx + 2 * D["clearance"], by + 2 * D["clearance"]])   # (depth along pull, width)
    open_dist = D["panel"] + inner[0]          # "exact fit": the tray's inside just clears the cabinet
    return dict(inner=inner, open=open_dist)


def drawer_handle(P, d):
    """Handle centre (world) and bar axis (world) with the drawer pulled out by d."""
    D = P.DRAWER
    R, o = drawer_frame(P)
    h = D["handle"]
    panel_h = D["wall_height"] + 0.03 + D["wall"]
    c = o + R @ np.array([d + h["standoff"], 0.0, -D["wall"] + panel_h / 2])
    axis = R[:, 1] if h["orientation"] == "horizontal" else R[:, 2]
    return c, axis


def tray_place_point(P, d=None):
    """Centre of the tray floor (inside) in the world, with the drawer open by d."""
    D = P.DRAWER
    dd = drawer_dims(P)
    d = dd["open"] if d is None else d
    R, o = drawer_frame(P)
    return o + R @ np.array([d - D["panel"] - dd["inner"][0] / 2, 0.0, 0.0])


def drawer_parts(P, d):
    """Cabinet + tray + panel + handle OBBs (world) with the drawer pulled out by d (m)."""
    D = P.DRAWER
    dd = drawer_dims(P)
    R, o = drawer_frame(P)
    w, t, wh, tp = D["wall"], D["wall"], D["wall_height"], D["panel"]
    din, win = dd["inner"]
    W = win + 2 * w
    out = []

    def add(name, group, c_local, size):
        out.append(_obb(name, group, o + R @ np.asarray(c_local, float), R, size))

    # cabinet: solid, from the ground to above the tray, behind the front face
    top = wh + 0.15
    cab_h = o[2] + top
    add("cabinet", "cabinet", (-D["cabinet_depth"] / 2, 0, cab_h / 2 - o[2]), (D["cabinet_depth"], W + 0.06, cab_h))
    # tray (moves with d)
    x_in0, x_in1 = d - tp - din, d - tp                      # inside of the tray along x
    add("drawer floor", "tray", ((x_in0 + x_in1) / 2 - w / 2, 0, -t / 2), (din + w, W, t))
    add("drawer back wall", "tray", (x_in0 - w / 2, 0, wh / 2), (w, W, wh))
    add("drawer left wall", "tray", ((x_in0 + x_in1) / 2, win / 2 + w / 2, wh / 2), (din, w, wh))
    add("drawer right wall", "tray", ((x_in0 + x_in1) / 2, -win / 2 - w / 2, wh / 2), (din, w, wh))
    panel_h = wh + 0.03 + t
    add("drawer front", "panel", (d - tp / 2, 0, -t + panel_h / 2), (tp, W + 0.04, panel_h))
    hc, _ = drawer_handle(P, d)
    h = D["handle"]
    size = (h["diameter"], h["length"], h["diameter"]) if h["orientation"] == "horizontal" else \
        (h["diameter"], h["diameter"], h["length"])
    out.append(_obb("drawer handle", "drawer handle", hc, R, size))
    return out


# ------------------------------------------------------------------ whole scene
def scene_objects(P, st):
    """All task objects in the world for scene state st:
    st = dict(drawer=d, box_T=(R, p) | None (held), box_in_drawer=(R, p in drawer-tray frame) | None)."""
    objs = drawer_parts(P, st["drawer"])
    if st.get("box_T") is not None:
        objs += box_parts(P, st["box_T"])
    return objs


def gripper_box_elements(P, box_in_gripper):
    """Held box as arm collision elements in the gripper frame."""
    R, p = box_in_gripper
    els = []
    for o in box_parts(P, (R, p)):
        # ground_tol: a box just lifted off the ground isn't "hitting" it
        els.append(dict(name=o["name"], body="gripper", kind="obb", c=o["c"], h=o["h"], R_local=o["R"], ground_tol=0.01))
    return els


def jaw_error_deg(P, frames, bar_axis):
    """How far the jaws are from closing squarely on a bar (deg): the jaw axis and the
    gripper axis should both be perpendicular to the bar."""
    Rg, _ = frames["gripper"]
    jaw = Rg[:, "xyz".index(P.GRIPPER["jaw_axis"])]
    g = Rg[:, 0]
    b = bar_axis / np.linalg.norm(bar_axis)
    e = max(abs(jaw @ b), abs(g @ b))
    return math.degrees(math.asin(min(1.0, e)))


def tcp_frame(P, frames):
    Rg, pg = frames["gripper"]
    return Rg, pg + Rg @ np.array([P.L_GRIPPER, 0, 0])
