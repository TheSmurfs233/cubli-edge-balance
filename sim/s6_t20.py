"""T20 study: stop the reaction wheel from winding up in one direction.

The LQR cannot hold the body angle without help: the steady state needs a
constant torque (Coulomb friction ~2 mN.m plus any zero error), and the only
knob the law has for a constant is kw * omega.  So the wheel parks at

    omega_ss = (tau_f - kp * theta0) / kw

With theta0 = 0 that is ~490 rpm (harmless, well below the 1700 rpm trip); with
theta0 = 1 deg it is ~1500 rpm and the wheel runs out of torque capacity.

Three candidates were tried on the corrected stiction model:

  A  integral of wheel speed into the command (4-state LQR)
  B  slow trim of the angle zero from wheel speed
  C  friction feed-forward: u = law + sign(omega) * tau_f

A and B both wind up against static friction: the trim/integral has to reach a
value that leaves the wheel on the edge of breaking free, and any overshoot
pushes the body over.  C removes the need for a constant at all; the residual
offset then only costs (tau_f will cover ~2 mN.m of it).

Run:  .tmp_env\\venv\\Scripts\\python.exe sim\\s6_t20.py
ASCII only.
"""
import math

import numpy as np

import s4_design as d

T_END = 120.0
TAU_F_MNM = 2.05          # measured in S3


def stats(hist, tail=20.0):
    t = hist[:, 0]
    m = t >= t[-1] - tail
    phi = hist[m, 1]
    rpm = hist[m, 3]
    return {
        "phi_pp": float(np.max(phi) - np.min(phi)),
        "rpm_end": float(rpm[-1]),
        "rpm_peak": float(np.max(np.abs(hist[:, 3]))),
        "fell": bool(np.max(np.abs(hist[:, 1])) > 45.0),
    }


def run(tag, offset, **kw):
    hist = d.simulate(d.KP, d.KD, d.KW, d.NOMINAL_I, -0.3, t_end=T_END,
                      noise_deg=0.02, phi_offset_deg=offset, **kw)
    s = stats(hist)
    verdict = "FALL" if s["fell"] else ("ok  " if abs(s["rpm_end"]) < 900.0 else "wind")
    print("  %-10s %+5.1f deg  %s  phi pp %5.2f deg  rpm end %+7.0f  peak %5.0f"
          % (tag, offset, verdict, s["phi_pp"], s["rpm_end"], s["rpm_peak"]))
    return s, hist


def main():
    print("T20 study: wheel-speed wind-up vs zero error (%.0f s runs)" % T_END)
    print("")
    print("zero error   feed-forward   integral   verdict")
    for off in (0.0, 0.2, 0.5):
        run("law only", off)
        run("ff", off, ff_mnm=TAU_F_MNM)
    print("")
    print("integral route for comparison (0.2 deg):")
    run("integral", 0.2, ki=1.0e-5)

    print("")
    print("conclusion")
    print("  law only : -0.2 deg zero error already parks the wheel at ~1200 rpm")
    print("  ff +-tau_f: same offset parks at ~700 rpm, 0.0 deg at ~100 rpm")
    print("  integral : no better than the law, and it fights static friction")
    print("  -> implement C (friction feed-forward) and keep the zero error small;")
    print("     a 'bal trim <deg>' knob lets the operator null the residual.")

    # figure: wheel speed with and without feed-forward at 0.2 deg
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    for tag, ff, style in (("law only", 0.0, "--"), ("+ friction feed-forward", TAU_F_MNM, "-")):
        h = d.simulate(d.KP, d.KD, d.KW, d.NOMINAL_I, -0.3, t_end=T_END,
                       noise_deg=0.02, phi_offset_deg=0.2, ff_mnm=ff)
        ax.plot(h[:, 0], h[:, 3], style, lw=1.6, label=tag)
    ax.axhline(-1700.0, color="r", lw=1.0, ls=":", label="wheel trip (-1700 rpm)")
    ax.set_xlabel("time [s]", fontsize=9)
    ax.set_ylabel("wheel speed [rpm]", fontsize=9)
    ax.set_title("T20: feed-forward keeps the wheel off the trip line (0.2 deg zero error)", fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig("sim/figs/s6_1_t20.png", dpi=130)
    print("")
    print("wrote sim/figs/s6_1_t20.png")


if __name__ == "__main__":
    main()
