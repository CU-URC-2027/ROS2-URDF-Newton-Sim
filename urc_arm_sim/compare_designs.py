"""
Compare arm designs side by side (link sections, masses, deflection, stress, torques, tipping).

    python compare_designs.py                          # tube vs 4-bar vs 2-bar
    python compare_designs.py arm_params arm_params_4bar my_params
"""
import math
import sys

import numpy as np

import arm_model as am
import deflection as dfl
import sections


def report(name):
    P = am.load_params(name)
    inert = am.body_inertials(P)
    box = P.TASK_BOX["mass"]
    tip_box = (box, np.array([P.L_GRIPPER, 0, 0]))
    poses = {
        "straight out, level": [0, 0, 0, 0, 0],
        "straight out, gripper down": [0, 0, 0, -math.pi / 2, 0],
        "home (folded)": am.home_q(P),
        "reach 45 deg up": [0, math.radians(45), 0, -math.radians(45), 0],
    }
    m1, _ = sections.mass_and_inertia(P, "link1", P.L1)
    pr = sections.props(P, "link1")
    out = {
        "design": getattr(P, "DESIGN_NAME", name),
        "link1 section": pr["type"],
        "link1 mass (kg)": f"{m1:.3f}",
        "arm moving mass (kg)": f"{sum(v['mass'] for v in inert.values()):.2f}",
        "link1 EI in-plane (N*m^2)": f"{pr['EIy']:.0f}",
        "link1 EI out-of-plane (N*m^2)": f"{pr['EIz']:.0f}",
        "link1 GJ (N*m^2)": f"{pr['GJ']:.0f}",
    }
    for pname, q in poses.items():
        for tag, b in (("empty", None), (f"{box:g} kg box", tip_box)):
            d = dfl.deflect(P, q, box=b)
            l1 = [x for x in d["beams"] if x["name"] == "link1"][0]
            ins = dict(inert)
            if b is not None:
                g = inert["gripper"]
                M = g["mass"] + box
                ins["gripper"] = dict(mass=M, com=(g["mass"] * g["com"] + box * b[1]) / M, inertia=g["inertia"])
            tau = am.static_torques(P, q, ins)
            st = am.stability(P, q, None, ins)
            key = f"{pname} / {tag}"
            out[f"{key}: TCP deflection (mm)"] = f"{d['tcp_mm']:.1f}"
            out[f"{key}: link1 tip / twist"] = f"{l1['tip_mm']:.1f} mm / {l1['twist_deg']:+.3f} deg"
            out[f"{key}: least safety factor"] = f"{d['fos']:.1f} ({d['fos_where']}: {[x for x in d['beams'] if x['name'] == d['fos_where']][0]['fos_mode']})"
            out[f"{key}: shoulder / elbow torque (N*m)"] = f"{tau[1]:.1f} / {tau[2]:.1f}"
            out[f"{key}: tip margin (m)"] = f"{st['margin']:.3f} ({st['edge']})"
    return out


def main():
    names = sys.argv[1:] or ["arm_params", "arm_params_4bar", "arm_params_2bar"]
    reps = [report(n) for n in names]
    keys = list(reps[0].keys())
    w0 = max(len(k) for k in keys)
    ws = [max(len(n), max(len(str(r.get(k, ""))) for k in keys)) for n, r in zip(names, reps)]
    print(f"{'':{w0}s}  " + "  ".join(f"{n:{w}s}" for n, w in zip(names, ws)))
    last_group = None
    for k in keys:
        group = k.split(":")[0] if ":" in k else None
        if group != last_group:
            print()
            last_group = group
        print(f"{k:{w0}s}  " + "  ".join(f"{str(r.get(k, '')):{w}s}" for r, w in zip(reps, ws)))


if __name__ == "__main__":
    main()
