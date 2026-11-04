"""S6 pre-flight: does the derated law still converge, and can we recognise a
sign error at all?

The S6 orchestration is not simulated here (the host tests cover it). What is
checked is the quantitative side of docs/45 section 6.1:

  1. the derated gains still recover from the ARM window,
  2. flipping the sign diverges -- otherwise "it fell over" in S6a would not
     mean "the sign is wrong",
  3. 10 ms of extra latency (5 ms ZOH + 5 ms jitter) still recovers.

Numbers behind the chosen derating: sim/s6_scale_scan.py.

Run: .tmp_env\\venv\\Scripts\\python.exe sim\\s6_check.py
"""

import numpy as np

import s4_design as d

SCALE = 0.7
PHI0 = 2.5          # = BAL_ARM_DEG, the worst case that may be armed
SETTLE_DEG = 0.3


def run(kp, kd, kw, delay_steps=0, t_end=12.0):
    h = d.simulate(kp, kd, kw, d.NOMINAL_I, PHI0, t_end=t_end, delay_steps=delay_steps)
    phi = h[:, 1]
    rpm = h[:, 3]
    settled = None
    for i in range(len(phi)):
        if abs(phi[i]) > SETTLE_DEG:
            settled = None
        elif settled is None:
            settled = h[i, 0]
    recovered = (settled is not None) and (abs(phi[-1]) < SETTLE_DEG)
    return recovered, settled, float(np.max(np.abs(rpm)))


def report(tag, ok, settled, peak, expect):
    verdict = "PASS" if (ok == expect) else "FAIL"
    when = "n/a" if settled is None else "%.2f s" % settled
    print("  %-32s %-4s  settle %-7s peak %6.0f rpm  (expected %s)"
          % (tag, verdict, when, peak, "recover" if expect else "fall"))
    return verdict == "PASS"


def main():
    kp = d.KP * SCALE
    kd = d.KD * SCALE
    kw = d.KW * SCALE

    print("S6 pre-flight: %.1fx of the nominal gains, release at %.1f deg" % (SCALE, PHI0))
    print("  kp = %.4f N.m/rad  kd = %.5f  kw = %.3e" % (kp, kd, kw))
    print("")

    ok = True
    ok &= report("%.1fx gains" % SCALE, *run(kp, kd, kw), expect=True)
    ok &= report("sign flipped (-1x)", *run(-kp, -kd, -kw), expect=False)
    ok &= report("%.1fx + 10 ms latency" % SCALE, *run(kp, kd, kw, delay_steps=2), expect=True)

    print("")
    print("RESULT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
