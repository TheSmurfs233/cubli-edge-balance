"""Pick the S6a derating from data instead of by feel.

docs/45 originally said 0.4x. The scan shows 0.4x cannot catch anything at
all, so the ARM window it was paired with (3 deg) would have armed a law that
was already beaten. This prints the two candidates side by side.
"""

import numpy as np

import s4_design as d

SETTLE_DEG = 0.3


def recovers(kp, kd, kw, phi0, delay_steps=0, t_end=12.0):
    h = d.simulate(kp, kd, kw, d.NOMINAL_I, phi0, t_end=t_end, delay_steps=delay_steps)
    phi = h[:, 1]
    tail = phi[int(len(phi) * 0.75):]
    return bool(np.max(np.abs(tail)) < SETTLE_DEG)


def capture(scale, delay_steps=0):
    kp, kd, kw = d.KP * scale, d.KD * scale, d.KW * scale
    best = 0.0
    for phi0 in np.arange(0.25, 6.01, 0.25):
        if recovers(kp, kd, kw, float(phi0), delay_steps):
            best = float(phi0)
        else:
            break
    return best


def settle(scale, phi0, delay_steps=0):
    kp, kd, kw = d.KP * scale, d.KD * scale, d.KW * scale
    h = d.simulate(kp, kd, kw, d.NOMINAL_I, phi0, t_end=12.0, delay_steps=delay_steps)
    phi = h[:, 1]
    t_set = None
    for i in range(len(phi)):
        if abs(phi[i]) > SETTLE_DEG:
            t_set = None
        elif t_set is None:
            t_set = h[i, 0]
    peak = float(np.max(np.abs(h[:, 3])))
    return t_set, peak


def main():
    print("S6a derating candidates (release from rest, nominal inertia)")
    print("  scale  capture  capture+10ms  settle@2.5deg  peak@2.5deg  peak torque")
    for scale in (0.5, 0.6, 0.7):
        cap = capture(scale)
        cap_d = capture(scale, delay_steps=2)
        t_set, peak = settle(scale, 2.5)
        kp = d.KP * scale
        tau = kp * np.radians(2.5) * 1000.0
        print("  %4.1f   %5.2f    %5.2f          %-8s      %6.0f rpm    %5.2f mN.m"
              % (scale, cap, cap_d,
                 "n/a" if t_set is None else "%.2f s" % t_set, peak, tau))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
