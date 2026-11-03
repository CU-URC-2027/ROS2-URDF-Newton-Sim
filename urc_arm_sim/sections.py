"""
Link cross-sections: stiffness, mass, stress and drawing for each link.

Two section types (sizes in mm in arm_params.py):

  "rect_tube"   (SECTION_MM, the default for every link)
      width (across the arm, y), height (in the plane of motion, z), wall.
      wall = 0 -> solid bar.

  "corner_bars" (LINK_SECTIONS, e.g. the 4-bar link 1)
      Four rectangular bars running the length of the link at the corners of
      a width x height rectangle (centre to centre), tied together by single
      diagonal lacing on all four faces (one diagonal per `bay` on each face).
        width, height : bar centre spacing (mm)
        bar_w, bar_h, bar_wall : corner bars, size across the arm (y) and in the
                                 plane of motion (z); wall 0 = solid
        lace_w, lace_h, lace_wall : lacing diagonals (rectangular): width in the
                                 face, depth out of the face; wall 0 = solid (flat bar)
        bay : lacing pitch along the link (mm)

  "battened"    (LINK_SECTIONS) four corner bars like "corner_bars", but tied
      together by flat batten plates instead of diagonals: at every station
      (one per `spacing` along the link, plus both ends) a plate is welded
      across each of the four faces, joining every bar to its neighbours, so
      each station is a closed rectangular frame. Shear is carried by frame
      (Vierendeel) action: the bars and plates bend between stations.
        width, height, bar_w, bar_h, bar_wall : as for corner_bars
        batten_w : plate width along the link (mm)
        batten_t : plate thickness (mm)
        spacing  : plate pitch along the link, centre to centre (mm)

  "two_bars"    (LINK_SECTIONS, e.g. arm_params_2bar.py) two rectangular bars in
      the plane of the arm, one above the other (height apart, centre to
      centre, in z), joined in that plane by lacing or batten plates. Stiff in
      in-plane bending; out-of-plane bending is just the two bars, and torsion
      has no closed cell (an open section), so it is soft in twist and can
      buckle sideways (lateral-torsional buckling) under in-plane bending.
        height, bar_w, bar_h, bar_wall : as for corner_bars (no width)
        bracing = "laced"    : lace_w, lace_h, lace_wall, bay  (as corner_bars)
        bracing = "battened" : batten_w, batten_t, spacing     (as battened)

Conventions (link frame): x along the link, y across the arm (pitch-axis
direction), z in the plane of motion. "In-plane" bending = about y (uses the
height), "out-of-plane" bending = about z (uses the width), torsion = about x.

Stiffness used by deflection.py (SI units):
  EIy, EIz  bending stiffness (N m^2)      GJ  torsional stiffness (N m^2)
  Sy, Sz    shear stiffness (N): 1/S = shear strain per newton of shear force
            (finite for the laced and battened sections, where the bracing's
            flexibility matters; large for a tube)
"""
import math

import numpy as np

STEEL_LIKE = 1e12   # "very stiff" placeholder for terms that don't matter


def _rect(b, h, t):
    """Rectangular bar or tube, b wide and h deep (wall t, 0 = solid).
    Returns area, I_b (bending across h, i.e. about the axis along b), I_h (about the
    axis along h), and the torsion constant J."""
    hollow = t > 0 and b > 2 * t and h > 2 * t
    bi, hi = (b - 2 * t, h - 2 * t) if hollow else (0.0, 0.0)
    A = b * h - bi * hi
    Ib = (b * h ** 3 - bi * hi ** 3) / 12.0
    Ih = (h * b ** 3 - hi * bi ** 3) / 12.0
    if hollow:
        J = 4 * ((b - t) * (h - t)) ** 2 * t / (2 * ((b - t) + (h - t)))
    else:
        lo, hi_ = min(b, h), max(b, h)
        J = hi_ * lo ** 3 * (1 / 3 - 0.21 * lo / hi_ * (1 - lo ** 4 / (12 * hi_ ** 4)))
    return A, Ib, Ih, J


def _bars(s):
    """Corner bar properties: area, I about y (bending in z), I about z, J."""
    for k in ("bar_w", "bar_h"):
        if k not in s:
            raise ValueError(f"corner bars need bar_w and bar_h (mm); got {sorted(s)}")
    return _rect(s["bar_w"], s["bar_h"], s.get("bar_wall", 0.0))


def material(P):
    M = getattr(P, "MATERIAL", dict(E=69e9, G=26e9, yield_MPa=276.0))
    return M["E"], M["G"], M["yield_MPa"] * 1e6


def spec(P, link):
    """The section spec for a link ("link1", "link2", "gripper"), sizes in metres."""
    over = getattr(P, "LINK_SECTIONS", {}).get(link)
    if over is not None:
        s = {k: (v / 1000.0 if isinstance(v, (int, float)) else v) for k, v in over.items()}
        s["type"] = over["type"]
        return s
    s = P.SECTION_MM[link]
    return dict(type="rect_tube", width=s["width"] / 1000, height=s["height"] / 1000, wall=s["wall"] / 1000)


def props(P, link):
    """Section properties of a link (SI)."""
    s = spec(P, link)
    E, G, fy = material(P)
    rho = P.DENSITY
    if s["type"] == "rect_tube":
        b, h, t = s["width"], s["height"], s["wall"]
        hollow = t > 0 and b > 2 * t and h > 2 * t
        bi, hi = (b - 2 * t, h - 2 * t) if hollow else (0.0, 0.0)
        A = b * h - bi * hi
        Iy = (b * h ** 3 - bi * hi ** 3) / 12.0
        Iz = (h * b ** 3 - hi * bi ** 3) / 12.0
        if hollow:
            Am = (b - t) * (h - t)
            J = 4 * Am ** 2 * t / (2 * ((b - t) + (h - t)))
            Sz, Sy = G * 2 * h * t, G * 2 * b * t           # webs carry the shear
        else:
            lo, hi_ = min(b, h), max(b, h)
            J = hi_ * lo ** 3 * (1 / 3 - 0.21 * lo / hi_ * (1 - lo ** 4 / (12 * hi_ ** 4)))
            Am = None
            Sz = Sy = G * A * 5 / 6
        return dict(type="rect_tube", spec=s, A=A, mass_per_m=rho * A, EIy=E * Iy, EIz=E * Iz, GJ=G * J,
                    Sy=Sy, Sz=Sz, envelope=(b, h), Iy=Iy, Iz=Iz, Am=Am, E=E, G=G, fy=fy)
    if s["type"] == "corner_bars":
        w, h, bay = s["width"], s["height"], s["bay"]
        Ab, Iby, Ibz, Jb = _bars(s)
        Ad, Id1, Id2, _ = _rect(s["lace_w"], s["lace_h"], s.get("lace_wall", 0.0))
        Id = min(Id1, Id2)                                             # lacing buckles about its weak axis
        EIy = E * (4 * Iby + 4 * Ab * (h / 2) ** 2)
        EIz = E * (4 * Ibz + 4 * Ab * (w / 2) ** 2)
        # single lacing: a face of width b with diagonals at angle phi to the link axis has
        # shear stiffness S = E Ad cos(phi) sin(phi)^2   (Timoshenko, built-up members)
        phi_s, phi_t = math.atan2(h, bay), math.atan2(w, bay)
        S_side = E * Ad * math.cos(phi_s) * math.sin(phi_s) ** 2      # faces in the plane of motion
        S_tb = E * Ad * math.cos(phi_t) * math.sin(phi_t) ** 2        # top / bottom faces
        Sz, Sy = 2 * S_side, 2 * S_tb
        # torsion: closed cell of four shear panels (Bredt): GJ = 4 A^2 / sum(b_i^2 / S_i)
        A_enc = w * h
        GJ = 4 * A_enc ** 2 / (2 * h ** 2 / S_side + 2 * w ** 2 / S_tb)
        GJ += 4 * G * Jb                                               # the bars' own (small) torsion
        d_side, d_tb = math.hypot(bay, h), math.hypot(bay, w)
        lace_per_m = Ad * (2 * d_side + 2 * d_tb) / bay
        return dict(type="corner_bars", spec=s, A=4 * Ab, Ab=Ab, Iby=Iby, Ibz=Ibz, Ad=Ad, Id=Id,
                    mass_per_m=rho * (4 * Ab + lace_per_m), lace_mass_per_m=rho * lace_per_m,
                    EIy=EIy, EIz=EIz, GJ=GJ, Sy=Sy, Sz=Sz, d_side=d_side, d_tb=d_tb,
                    envelope=(w + s["bar_w"], h + s["bar_h"]), E=E, G=G, fy=fy)
    if s["type"] == "battened":
        w, h, a = s["width"], s["height"], s["spacing"]
        bw, bt = s["batten_w"], s["batten_t"]
        if a <= bw:
            raise ValueError(f"{link}: batten spacing must be larger than the batten width")
        Ab, Iby, Ibz, Jb = _bars(s)
        Ip = bt * bw ** 3 / 12.0                       # plate bending in its own plane
        EIy = E * (4 * Iby + 4 * Ab * (h / 2) ** 2)
        EIz = E * (4 * Ibz + 4 * Ab * (w / 2) ** 2)

        # shear stiffness of one battened face with two chords h0 apart (EN 1993-1-1, 6.4.3.1):
        #   Sv = 24 E Ich / (a^2 (1 + 2 Ich h0 / (Ip a)))  <=  2 pi^2 E Ich / a^2
        # side faces: chords h apart, bending in z (Iby); top/bottom: w apart, bending in y (Ibz)
        def face(h0, Ich):
            return min(24 * E * Ich / (a * a * (1 + 2 * Ich * h0 / (Ip * a))), 2 * math.pi ** 2 * E * Ich / (a * a))
        S_side, S_tb = face(h, Iby), face(w, Ibz)
        Sz, Sy = 2 * S_side, 2 * S_tb
        GJ = 4 * (w * h) ** 2 / (2 * h ** 2 / S_side + 2 * w ** 2 / S_tb) + 4 * G * Jb
        plate_per_m = bw * bt * (2 * h + 2 * w) / a
        return dict(type="battened", spec=s, A=4 * Ab, Ab=Ab, Iby=Iby, Ibz=Ibz, Ip=Ip,
                    mass_per_m=rho * (4 * Ab + plate_per_m),
                    lace_mass_per_m=rho * plate_per_m, EIy=EIy, EIz=EIz, GJ=GJ, Sy=Sy, Sz=Sz,
                    envelope=(w + s["bar_w"], h + s["bar_h"]), E=E, G=G, fy=fy)
    if s["type"] == "two_bars":
        return _two_bars(P, link, s, E, G, fy, rho)
    raise ValueError(f"{link}: unknown section type '{s['type']}'")


def _two_bars(P, link, s, E, G, fy, rho):
    h = s["height"]
    Ab, Iby, Ibz, Jb = _bars(s)
    L = {"link1": P.L1, "link2": P.L2}.get(link, P.L_GRIPPER)
    br = s.get("bracing", "laced")
    EIy = E * (2 * Iby + 2 * Ab * (h / 2) ** 2)
    EIz = E * 2 * Ibz                                  # the bars side by side bend on their own
    GJ = 2 * G * Jb                                    # open section: only the bars' own torsion
    Iw = Ibz * h ** 2 / 2                              # warping constant (like an I-beam's flanges)
    Sy = 2 * G * Ab * 0.5                              # each bar's own shear (bending dominates)
    out = dict(type="two_bars", spec=s, A=2 * Ab, Ab=Ab, Iby=Iby, Ibz=Ibz, Jb=Jb, Iw=Iw, L=L,
               EIy=EIy, EIz=EIz, GJ=GJ, Sy=Sy, envelope=(s["bar_w"], h + s["bar_h"]), E=E, G=G, fy=fy,
               bracing=br)
    if br == "laced":
        bay = s["bay"]
        Ad, Id1, Id2, _ = _rect(s["lace_w"], s["lace_h"], s.get("lace_wall", 0.0))
        phi = math.atan2(h, bay)
        d = math.hypot(bay, h)
        brace_per_m = Ad * d / bay
        out.update(Sz=E * Ad * math.cos(phi) * math.sin(phi) ** 2, Ad=Ad, Id=min(Id1, Id2), d=d, node=bay)
    elif br == "battened":
        a, bw, bt = s["spacing"], s["batten_w"], s["batten_t"]
        if a <= bw:
            raise ValueError(f"{link}: batten spacing must be larger than the batten width")
        Ip = bt * bw ** 3 / 12.0
        Sz = min(24 * E * Iby / (a * a * (1 + 2 * Iby * h / (Ip * a))), 2 * math.pi ** 2 * E * Iby / (a * a))
        brace_per_m = bw * bt * h / a
        out.update(Sz=Sz, Ip=Ip, node=a)
    else:
        raise ValueError(f"{link}: two_bars bracing must be 'laced' or 'battened' (got '{br}')")
    out.update(mass_per_m=rho * (2 * Ab + brace_per_m), lace_mass_per_m=rho * brace_per_m)
    return out


def mass_and_inertia(P, link, T):
    """Mass and inertia about the centre (tube axes) of a length T of the link's section."""
    pr = props(P, link)
    m = pr["mass_per_m"] * T
    if pr["type"] == "rect_tube":
        b, h = pr["spec"]["width"], pr["spec"]["height"]
        t = pr["spec"]["wall"]
        m_out = P.DENSITY * T * b * h

        def box(mm, sx, sy, sz):
            return np.diag([mm * (sy ** 2 + sz ** 2), mm * (sx ** 2 + sz ** 2), mm * (sx ** 2 + sy ** 2)]) / 12.0
        I = box(m_out, T, b, h)
        if t > 0 and b > 2 * t and h > 2 * t:
            bi, hi = b - 2 * t, h - 2 * t
            I = I - box(P.DENSITY * T * bi * hi, T, bi, hi)
        return m, I
    # built-up sections: all mass lumped on the bars (bracing included)
    h = pr["spec"]["height"]
    ys = (0.0,) if pr["type"] == "two_bars" else (-pr["spec"]["width"] / 2, pr["spec"]["width"] / 2)
    mb = m / (2 * len(ys))
    I = np.zeros((3, 3))
    for y in ys:
        for z in (-h / 2, h / 2):
            I += np.diag([mb * (y * y + z * z), mb * (T * T / 12 + z * z), mb * (T * T / 12 + y * y)])
    return m, I


def stress(pr, N, V, M):
    """Worst stress safety factors at one section for internal axial force N, shear V
    (y, z) and moments M (x torsion, y in-plane, z out-of-plane), all in the link frame.
    Returns dict(fos, mode) where mode says what limits it."""
    fy, E = pr["fy"], pr["E"]
    Mx, My, Mz = abs(M[0]), abs(M[1]), abs(M[2])
    Vy, Vz = abs(V[0]), abs(V[1])
    if pr["type"] == "rect_tube":
        b, h = pr["spec"]["width"], pr["spec"]["height"]
        sig = abs(N) / pr["A"] + My * (h / 2) / pr["Iy"] + Mz * (b / 2) / pr["Iz"]
        t = pr["spec"]["wall"]
        if pr["Am"]:
            tau = Mx / (2 * pr["Am"] * t) + max(Vz / (2 * h * t), Vy / (2 * b * t))
        else:
            tau = 1.5 * max(Vy, Vz) / pr["A"]
        vm = math.sqrt(sig ** 2 + 3 * tau ** 2)
        return dict(fos=fy / vm if vm > 0 else math.inf, mode="yield")
    if pr["type"] == "battened":
        return _stress_battened(pr, N, Mx, My, Mz, Vy, Vz)
    if pr["type"] == "two_bars":
        return _stress_two_bars(pr, N, Mx, My, Mz, Vy, Vz)
    # corner bars: bar axial force from bending + axial; lacing from shear + torsion
    w, h, bay = pr["spec"]["width"], pr["spec"]["height"], pr["spec"]["bay"]
    Fb = abs(N) / 4 + My / (2 * h) + Mz / (2 * w)
    q = Mx / (2 * w * h)                                     # torsion shear flow
    Fd = max((Vz / 2 + q * h) * pr["d_side"] / h, (Vy / 2 + q * w) * pr["d_tb"] / w)
    cands = []
    if Fb > 0:
        cands.append((fy * pr["Ab"] / Fb, "corner bar yield"))
        cands.append((math.pi ** 2 * E * min(pr["Iby"], pr["Ibz"]) / bay ** 2 / Fb,
                      "corner bar buckling (between lacing)"))
    if Fd > 0:
        cands.append((fy * pr["Ad"] / Fd, "lacing yield"))
        dmax = max(pr["d_side"], pr["d_tb"])
        cands.append((math.pi ** 2 * E * pr["Id"] / dmax ** 2 / Fd, "lacing buckling"))
    if not cands:
        return dict(fos=math.inf, mode="")
    fos, mode = min(cands)
    return dict(fos=fos, mode=mode)


def _stress_battened(pr, N, Mx, My, Mz, Vy, Vz):
    """Battened corner bars (frame action, EN 1993-1-1 6.4.3): each face's shear V_f bends
    its two chords between plates (contraflexure mid-way: M_ch = V_f a / 4) and its plates
    (M = V_f a / 2, shear V_f a / h0)."""
    s, fy, E = pr["spec"], pr["fy"], pr["E"]
    w, h, a = s["width"], s["height"], s["spacing"]
    bw, bt = s["batten_w"], s["batten_t"]
    Fb = abs(N) / 4 + My / (2 * h) + Mz / (2 * w)
    q = Mx / (2 * w * h)                                     # torsion shear flow
    # (face shear, chord spacing, chord bending stress per N*m): side faces bend the bars in z,
    # top / bottom faces bend them in y
    faces = ((Vz / 2 + q * h, h, (s["bar_h"] / 2) / pr["Iby"]),
             (Vy / 2 + q * w, w, (s["bar_w"] / 2) / pr["Ibz"]))
    # a corner bar is a chord of one side face and one top/bottom face
    sb = sum(f[0] * a / 4 * f[2] for f in faces)            # chord bending stress (M_ch = V_f a / 4)
    cands = []
    sig = Fb / pr["Ab"] + sb
    if sig > 0:
        cands.append((fy / sig, "corner bar yield (axial + frame bending)"))
    if Fb > 0:
        Pcr = math.pi ** 2 * E * min(pr["Iby"], pr["Ibz"]) / a ** 2
        cands.append((1.0 / (Fb / Pcr + sb / fy), "corner bar buckling (between battens)"))
    Vf, h0, _ = max(faces, key=lambda f: f[0] / f[1])       # plate shear V_f a / h0 is worst here
    Vm = max(f[0] for f in faces)
    Vp, Mp = Vf * a / max(h0, 1e-9), Vm * a / 2
    sp = Mp / (bt * bw ** 2 / 6)
    tp = 1.5 * Vp / (bt * bw)
    vm = math.sqrt(sp ** 2 + 3 * tp ** 2)
    if vm > 0:
        cands.append((fy / vm, "batten plate yield"))
    if not cands:
        return dict(fos=math.inf, mode="")
    fos, mode = min(cands)
    return dict(fos=fos, mode=mode)


def _stress_two_bars(pr, N, Mx, My, Mz, Vy, Vz):
    """Two bars in the arm plane. Each bar: axial N/2 + My/h, plus its own out-of-plane
    bending Mz/2, torsion Mx/2 and (battened) frame bending V a/4. Bracing: lacing force
    Vz d/h or plate bending. Lateral-torsional buckling of the whole link under My:
    M_cr = (pi/L) sqrt(E Iz G J) sqrt(1 + pi^2 E Iw / (G J L^2)), with L the link length
    (uniform moment, ends held against twist: conservative for a cantilever's moment)."""
    s, fy, E, G = pr["spec"], pr["fy"], pr["E"], pr["G"]
    h, bw_, bh_ = s["height"], s["bar_w"], s["bar_h"]
    t = s.get("bar_wall", 0.0)
    Fb = abs(N) / 2 + My / h
    sz = (Mz / 2) * (bw_ / 2) / pr["Ibz"]                     # each bar's out-of-plane bending
    hollow = t > 0 and bw_ > 2 * t and bh_ > 2 * t
    tau = (Mx / 2) / (2 * (bw_ - t) * (bh_ - t) * t) if hollow else (Mx / 2) / (0.21 * max(bw_, bh_) * min(bw_, bh_) ** 2)
    node = pr["node"]
    cands = []
    if pr["bracing"] == "battened":
        a = node
        sframe = Vz * a / 4 * (bh_ / 2) / pr["Iby"]
        Vp, Mp = Vz * a / h, Vz * a / 2
        sp = Mp / (s["batten_t"] * s["batten_w"] ** 2 / 6)
        tp = 1.5 * Vp / (s["batten_t"] * s["batten_w"])
        vmp = math.sqrt(sp ** 2 + 3 * tp ** 2)
        if vmp > 0:
            cands.append((fy / vmp, "batten plate yield"))
    else:
        sframe = 0.0
        Fd = Vz * pr["d"] / h
        if Fd > 0:
            cands.append((fy * pr["Ad"] / Fd, "lacing yield"))
            cands.append((math.pi ** 2 * E * pr["Id"] / pr["d"] ** 2 / Fd, "lacing buckling"))
    sig = Fb / pr["Ab"] + sz + sframe
    vm = math.sqrt(sig ** 2 + 3 * tau ** 2)
    if vm > 0:
        cands.append((fy / vm, "bar yield"))
    if Fb > 0:
        Pcr = math.pi ** 2 * E * pr["Iby"] / node ** 2           # in plane, between bracing points
        cands.append((1.0 / (Fb / Pcr + (sz + sframe) / fy), "bar buckling (between bracing)"))
    if My > 0:
        L, GJ = pr["L"], G * 2 * pr["Jb"]
        Mcr = math.pi / L * math.sqrt(E * 2 * pr["Ibz"] * GJ) * math.sqrt(1 + math.pi ** 2 * E * pr["Iw"] / (GJ * L * L))
        cands.append((Mcr / My, "lateral-torsional buckling"))
    if not cands:
        return dict(fos=math.inf, mode="")
    fos, mode = min(cands)
    return dict(fos=fos, mode=mode)


# ------------------------------------------------------------------ drawing (URDF visuals)
def _rpy(R):
    """Rotation matrix -> URDF rpy (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll, yaw = math.atan2(-R[1, 2], R[1, 1]), 0.0
    return roll, pitch, yaw


def _frame(x, n):
    """Rotation whose x axis is x and whose y axis is the face normal n (n is perpendicular to x)."""
    x = x / np.linalg.norm(x)
    y = n - (n @ x) * x
    y = y / np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


def built_up_visuals(P, link, T, R_tube, c_tube):
    """Boxes (centre, rpy, size) in the link frame for a corner-bar or battened link of tube
    length T placed at c_tube with rotation R_tube: the four bars, and the lacing or plates."""
    pr = props(P, link)
    s = pr["spec"]
    h = s["height"]
    two = pr["type"] == "two_bars"
    w = 0.0 if two else s["width"]
    ys = (0.0,) if two else (-w / 2, w / 2)
    if two:                        # one braced face, in the arm plane
        kind = "battened" if pr["bracing"] == "battened" else "laced"
    else:
        kind = "battened" if pr["type"] == "battened" else "laced"
    out = []

    def bar(a, b, n, size_y, size_z):
        """A box from a to b (tube axes), with its local y along the face normal n."""
        d = b - a
        R = R_tube @ _frame(d, n)
        out.append((c_tube + R_tube @ ((a + b) / 2), _rpy(R), (float(np.linalg.norm(d)), size_y, size_z)))
    ey, ez = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
    for y in ys:
        for z in (-h / 2, h / 2):
            bar(np.array([-T / 2, y, z]), np.array([T / 2, y, z]), ey, s["bar_w"], s["bar_h"])
    if kind == "battened":
        bw, bt = s["batten_w"], s["batten_t"]
        n = max(1, int(round(T / s["spacing"])))
        rpy = _rpy(R_tube)
        for x in np.linspace(-T / 2 + bw / 2, T / 2 - bw / 2, n + 1):
            for y in ys:                         # side faces
                out.append((c_tube + R_tube @ np.array([x, y, 0.0]), rpy, (bw, bt, h)))
            if two:
                continue
            for z in (-h / 2, h / 2):            # top / bottom faces
                out.append((c_tube + R_tube @ np.array([x, 0.0, z]), rpy, (bw, w, bt)))
        return out
    bay = s["bay"]
    n = max(1, int(round(T / bay)))
    xs = np.linspace(-T / 2, T / 2, n + 1)
    lw, lh = s["lace_w"], s["lace_h"]
    for i in range(n):
        x0, x1 = xs[i], xs[i + 1]
        flip = 1 if i % 2 == 0 else -1
        for y in ys:                       # side faces (plane of motion): normal y
            bar(np.array([x0, y, -flip * h / 2]), np.array([x1, y, flip * h / 2]), ey, lh, lw)
        if two:
            continue
        for z in (-h / 2, h / 2):          # top / bottom faces: normal z
            bar(np.array([x0, -flip * w / 2, z]), np.array([x1, flip * w / 2, z]), ez, lh, lw)
    return out
