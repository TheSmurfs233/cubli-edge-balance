"""How much wheel speed a constant external torque costs, for two kw values.

The steady wheel speed of the three-state law is fixed by the torque balance
   tau = kw*omega + ff   (at phi = 0)
so a constant push (cable pull, zero error) shows up as a constant wheel speed
and the only lever is kw.  This script quantifies it before touching firmware.
"""

import numpy as np
import s4_design as d

EXT_MNM = 15.0          # cable pull + zero error, from the on-board wheel slope
T_END = 60.0


def run(kp, kd, kw, tag):
    h = d.simulate(kp, kd, kw, d.NOMINAL_I, 0.0, t_end=T_END,
                   ext_mnm=EXT_MNM, u_max_mnm=28.0, trip_rpm=1700.0,
                   ff_mnm=d.TAU_FRIC * 1000.0)
    rpm = h[:, 3]
    phi = h[:, 1]
    # time until the wheel reaches the trip line, if it does
    over = np.nonzero(np.abs(rpm) >= 1700.0)[0]
    t_trip = h[over[0], 0] if len(over) else float("nan")
    print("%-10s kp %.4f kd %.5f kw %.4e | max|phi| %5.2f deg | "
          "rpm@10s %7.1f  rpm@30s %7.1f  rpm@end %7.1f | trip %s"
          % (tag, kp, kd, kw, np.max(np.abs(phi)),
             np.interp(10.0, h[:, 0], rpm),
             np.interp(30.0, h[:, 0], rpm),
             rpm[-1],
             ("%.1f s" % t_trip) if not np.isnan(t_trip) else "none"))


print("ext torque = %.1f mN.m, ceiling 28 mN.m, trip 1700 rpm" % EXT_MNM)
run(d.KP, d.KD, d.KW, "current 50rpm")

for sigma_rpm in (20.0, 10.0):
    sig = sigma_rpm * 2.0 * np.pi / 60.0
    k = d.design_lqr(d.NOMINAL_I, 2.0e-3, sig_w=sig)
    run(k[0], k[1], k[2], "sigma %2.0frpm" % sigma_rpm)

print()
print("capture window for each candidate (no external torque):")
for sigma_rpm in (50.0, 20.0, 10.0):
    sig = sigma_rpm * 2.0 * np.pi / 60.0
    k = d.design_lqr(d.NOMINAL_I, 2.0e-3, sig_w=sig)
    got = []
    for phi0 in (2.0, 3.0, 4.0, 4.5, 5.0):
        h = d.simulate(k[0], k[1], k[2], d.NOMINAL_I, phi0, t_end=8.0,
                       u_max_mnm=28.0, ff_mnm=d.TAU_FRIC * 1000.0)
        ok = abs(h[-1, 1]) < 0.5
        got.append("%.1f:%s" % (phi0, "ok" if ok else "fall"))
    print("  sigma %2.0f rpm -> %s  (kp %.4f kd %.5f kw %.4e)"
          % (sigma_rpm, " ".join(got), k[0], k[1], k[2]))
