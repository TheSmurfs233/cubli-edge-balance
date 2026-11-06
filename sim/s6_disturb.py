"""How big a constant disturbance does it take to wind the wheel up?

The closed-loop runs on 2026-11-05 held the body for ~14 s while the wheel
climbed to ~1370 rpm (about 91 rpm/s).  This script inverts that observation:
sweep a constant body torque and a zero-offset error, and report the wheel speed
the loop settles at.  It answers "is the USB cable force big enough to explain
it, or is the loop simply too weak against ANY constant torque".
"""

import numpy as np

import s4_design as d

SCALE = 0.7          # BAL_SCALE, the value in the firmware today
BAL_MAX = 28.0       # BAL_MAX_MNM in board_cfg.h
KP = d.KP * SCALE
KD = d.KD * SCALE
KW = d.KW * SCALE

T_END = 60.0


def run(ext_mnm=0.0, off_deg=0.0):
    h = d.simulate(KP, KD, KW, d.NOMINAL_I, 0.0, t_end=T_END,
                   u_max_mnm=BAL_MAX, ext_mnm=ext_mnm,
                   phi_offset_deg=off_deg)
    phi = h[:, 1]
    rpm = h[:, 3]
    tau = h[:, 4]
    # The last 10 s are the steady state we care about.
    tail = h[h[:, 0] > T_END - 10.0]
    fell = np.max(np.abs(phi)) > 30.0
    return dict(fell=fell,
                rpm_end=float(np.mean(tail[:, 3])),
                phi_end=float(np.mean(tail[:, 1])),
                tau_end=float(np.mean(np.abs(tail[:, 4]))))


print("nominal gains x %.2f: kp %.4f  kd %.5f  kw %.6f" % (SCALE, KP, KD, KW))
print("friction %.2f mN.m, ceiling %.1f mN.m" % (d.TAU_FRIC * 1e3, BAL_MAX))

print()
print("A. constant body torque (what a cable would do)")
print("   ext[mN.m]   phi[deg]   steady rpm   tau[mN.m]   fell")
for ext in (0.0, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 15.0):
    r = run(ext_mnm=ext)
    print("   %8.1f   %8.2f   %10.0f   %9.2f   %s"
          % (ext, r["phi_end"], r["rpm_end"], r["tau_end"], "YES" if r["fell"] else "no"))

print()
print("B. zero-offset error (compiled ATT_OFFSET_DEG vs real balance)")
print("   off[deg]    phi[deg]   steady rpm   tau[mN.m]   fell")
for off in (0.0, 0.2, 0.35, 0.5, 1.0, 1.5, 2.0):
    r = run(off_deg=off)
    print("   %8.2f   %8.2f   %10.0f   %9.2f   %s"
          % (off, r["phi_end"], r["rpm_end"], r["tau_end"], "YES" if r["fell"] else "no"))

print()
print("Target from the 2026-11-05 runs: about 1370 rpm after 14 s (91 rpm/s).")
