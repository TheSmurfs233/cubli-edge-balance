"""Does the flywheel accelerate too slowly for this cube?

The user's question after S6a: "maybe the reaction wheel spins up too slowly".
This answers it with numbers instead of opinion.

For a reaction wheel the body receives exactly the motor torque -- Newton's
third law -- so the wheel inertia does not appear in the body equation at all.
It only sets how fast the wheel speed ramps (and therefore how soon the wheel
runs out of headroom). This script checks that claim by re-deriving the LQR for
several wheel inertias and re-running the capture scan.

s4_design keeps J_W as a module global, so the sweep just rebinds it.
"""

import numpy as np

import s4_design as d

SETTLE_DEG = 0.3
J_NOMINAL = d.J_W


def set_jw(j_w):
    d.J_W = j_w


def capture(u0=2e-3):
    kp, kd, kw = d.design_lqr(d.NOMINAL_I, u0)
    best = 0.0
    for phi0 in np.arange(0.25, 6.01, 0.25):
        h = d.simulate(kp, kd, kw, d.NOMINAL_I, float(phi0), t_end=12.0)
        tail = h[:, 1][int(len(h) * 0.75):]
        if np.max(np.abs(tail)) < SETTLE_DEG:
            best = float(phi0)
        else:
            break
    return best, kp, kd, kw


def spin_up(j_w, tau=34.7e-3):
    alpha = tau / j_w
    return alpha, alpha * 60.0 / (2.0 * np.pi)


def main():
    print("Body sees only the motor torque; J_w sets the spin-up rate.")
    print("")
    print("  J_w [kg.m^2]     kp      kd       kw        capture   full-torque spin-up")
    for factor in (0.5, 1.0, 2.0, 5.0):
        j_w = J_NOMINAL * factor
        set_jw(j_w)
        cap, kp, kd, kw = capture()
        alpha, rpm_s = spin_up(j_w)
        print("  %.2e (%.1fx)  %.4f  %.5f  %.2e  %5.2f deg  %4.0f rad/s^2 = %5.0f rpm/s"
              % (j_w, factor, kp, kd, kw, cap, alpha, rpm_s))

    set_jw(J_NOMINAL)
    alpha, rpm_s = spin_up(J_NOMINAL)
    print("")
    print("  nominal: full torque gives %.0f rad/s^2 = %.0f rpm/s" % (alpha, rpm_s))
    measured_alpha = 957.0 * 2.0 * np.pi / 60.0 / 0.75
    print("  S6a run: wheel went 222 -> 1179 rpm in 0.75 s => %.0f rad/s^2"
          % measured_alpha)
    print("           which is %.1f mN.m of torque, out of 33 allowed"
          % (measured_alpha * J_NOMINAL * 1000.0))
    print("  => the wheel was not torque limited: the body was held still, so the")
    print("     command stayed small (+-few mN.m) and the wheel just coasted up.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
