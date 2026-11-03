"""
Link deflection (bending + torsion + shear) and stress for the arm, from the
live loads. Used by sim_arm.py, mission.py, build_urdf.py and compare_designs.py.

Model
  * Link 1, link 2 and the gripper are beams along their pivot lines (shoulder
    -> elbow, elbow -> wrist, wrist -> TCP), each clamped at its proximal
    joint. Joints, motors and the chassis are rigid.
  * Every mass on the arm is a point load m (g - a): tubes are split into short
    pieces along their length, motors / extra masses / a held box sit at their
    centres of mass. a = acceleration (from the sim) or 0 (static).
  * At each station s along a beam, the internal forces come from every load
    beyond s (further along the same link, plus everything on later links):
        V(s) = sum F_i,     M(s) = sum (r_i - p(s)) x F_i
    expressed in the link frame: x along the link (torsion), y across the arm
    (in-plane bending), z in the plane of motion (out-of-plane bending).
  * Integrating from the clamped end:
        theta' = (Mx / GJ, My / EIy, Mz / EIz)          (rotation)
        u'     = theta x x_hat + (0, Vy / Sy, Vz / Sz)  (bending + shear deflection)
  * A link's tip deflection and rotation carry every later link with them, so
    the TCP moves by the sum over links of  u_tip + theta_tip x (TCP - tip).

Small deflections; the joints' own play (backlash, gearbox wind-up) is not
included.
"""
import math

import numpy as np

import arm_model as am
import sections

BEAMS = (("link1", "link1", 1), ("link2", "link2", 2), ("gripper", "gripper", 4))   # name, body, body index
N_STATIONS = 31
TUBE_PIECES = 12


def load_points(P, q, rover=None, parts=None, box=None):
    """Point masses on the arm (body index, mass, world position). Tubes and wiring are
    split into pieces along their length. box: (mass, com in the gripper frame) or None."""
    parts = parts or am.body_parts(P)
    frames = am.fk(P, q, rover=rover)[0]
    bi, ms, rs = [], [], []
    for k, body in enumerate(am.BODY_ORDER):
        R, p = frames[body]
        for part in parts[body]:
            if part["kind"] in ("tube", "wiring") and part.get("size"):
                T = part["size"][0]
                Rt = part.get("R", np.eye(3))
                for x in (np.arange(TUBE_PIECES) + 0.5) / TUBE_PIECES * T - T / 2:
                    bi.append(k), ms.append(part["mass"] / TUBE_PIECES)
                    rs.append(p + R @ (part["com"] + Rt @ np.array([x, 0, 0])))
            elif part["kind"] == "wiring":
                bi.append(k), ms.append(part["mass"]), rs.append(p + R @ part["com"])
            else:
                bi.append(k), ms.append(part["mass"]), rs.append(p + R @ part["com"])
    if box is not None:
        Rg, pg = frames["gripper"]
        bi.append(4), ms.append(box[0]), rs.append(pg + Rg @ np.asarray(box[1], float))
    return np.array(bi), np.array(ms), np.array(rs), frames


def box_load(P, att):
    """(mass, com in gripper frame) of a held task box, else None."""
    if att is None or att.get("box") != "gripper":
        return None
    import scene as sc
    m, com, _ = sc.box_inertia_local(P)
    Rr, pr = att["box_rel"]
    return m, pr + Rr @ com


def deflect(P, q, rover=None, parts=None, box=None, acc=None, pts=None):
    """Deflections and stresses. acc: accelerations of the load points (N x 3) or None.
    Returns dict with tcp (world vector, m), per-beam results, shapes for drawing."""
    if pts is None:
        pts = load_points(P, q, rover, parts, box)
    bidx, m, r, frames = pts
    g = am.gravity_vec(P)
    a = np.zeros_like(r) if acc is None else acc
    F = m[:, None] * (g[None, :] - a)                  # load each point puts on the arm
    rxF = np.cross(r, F)
    Rg, pg = frames["gripper"]
    tcp = pg + Rg @ np.array([P.L_GRIPPER, 0, 0])
    lengths = {"link1": P.L1, "link2": P.L2, "gripper": P.L_GRIPPER}

    beams, total = [], np.zeros(3)
    upstream = []                                      # (u_tip, theta_tip, tip) of earlier beams
    for name, body, k in BEAMS:
        R, p0 = frames[body]
        L = lengths[name]
        pr = sections.props(P, name)
        s = np.linspace(0.0, L, N_STATIONS)
        ps = p0[None, :] + s[:, None] * R[:, 0][None, :]
        xloc = (r - p0) @ R[:, 0]                      # position along this link of every point
        mask = (bidx[None, :] > k) | ((bidx[None, :] == k) & (xloc[None, :] > s[:, None]))
        V = mask.astype(float) @ F                     # (N_STATIONS, 3) world
        M = mask.astype(float) @ rxF - np.cross(ps, V)
        Vb, Mb = V @ R, M @ R                          # link frame
        kappa = Mb / np.array([pr["GJ"], pr["EIy"], pr["EIz"]])
        ds = np.diff(s)
        theta = np.vstack([np.zeros(3), np.cumsum(0.5 * (kappa[1:] + kappa[:-1]) * ds[:, None], axis=0)])
        slope = np.cross(theta, np.array([1.0, 0, 0])) + Vb * np.array([0.0, 1 / pr["Sy"], 1 / pr["Sz"]])
        u = np.vstack([np.zeros(3), np.cumsum(0.5 * (slope[1:] + slope[:-1]) * ds[:, None], axis=0)])
        # stresses (axial force N = component along the link)
        worst = dict(fos=math.inf, mode="", s=0.0)
        for i in range(N_STATIONS):
            st = sections.stress(pr, Vb[i, 0], Vb[i, 1:], Mb[i])
            if st["fos"] < worst["fos"]:
                worst = dict(fos=st["fos"], mode=st["mode"], s=float(s[i]))
        # deflected shape of this beam (world), including what earlier beams did to it
        base_shift = np.zeros((N_STATIONS, 3))
        for (uu, th, tip) in upstream:
            base_shift += uu[None, :] + np.cross(th[None, :], ps - tip[None, :])
        shape = ps + base_shift + u @ R.T
        u_tip, th_tip, tip = R @ u[-1], R @ theta[-1], ps[-1]
        total += u_tip + np.cross(th_tip, tcp - tip)
        upstream.append((u_tip, th_tip, tip))
        beams.append(dict(name=name, tip_mm=1000 * float(np.linalg.norm(u[-1])), tip_vec=u_tip,
                          bend_deg=math.degrees(float(np.hypot(theta[-1, 1], theta[-1, 2]))),
                          twist_deg=math.degrees(float(theta[-1, 0])),
                          fos=worst["fos"], fos_mode=worst["mode"], fos_at=worst["s"],
                          M_root=Mb[0], shape=shape, line=ps))
    return dict(tcp=total, tcp_mm=1000 * float(np.linalg.norm(total)), tcp_down_mm=-1000 * float(total[2]),
                beams=beams, fos=min(b["fos"] for b in beams),
                fos_where=min(beams, key=lambda b: b["fos"])["name"])


def summary_line(d):
    b = {x["name"]: x for x in d["beams"]}
    return (f"TCP {d['tcp_mm']:.1f} mm (down {d['tcp_down_mm']:.1f}); link1 tip {b['link1']['tip_mm']:.1f} mm, "
            f"twist {b['link1']['twist_deg']:.2f} deg; link2 tip {b['link2']['tip_mm']:.1f} mm; "
            f"min safety factor {d['fos']:.1f} ({d['fos_where']})")
