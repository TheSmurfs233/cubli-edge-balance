"""S6b study: a slow zero trim that keeps the wheel from winding up.

The 2026-11-04 run balanced for 6.1 s but the wheel reached the 1700 rpm trip
because the USB cable pulls the body with a constant torque: the LQR can only
answer a constant disturbance with a constant command, and a constant command
is wheel speed.  Moving the zero (trim) makes gravity supply that torque
instead, so the wheel stops accumulating.

This script scans the trim rate with a 4 mN.m cable pull and reports whether
the wheel stops winding inside 120 s without tilting the body out of its window.

Run:  .tmp_env\\venv\\Scripts\\python.exe sim\\t20_trim.py
"""
import math
import s4_design as d

SCALE = 0.7                      # BAL_SCALE in board_cfg.h
KP = d.KP * SCALE
KD = d.KD * SCALE
KW = d.KW * SCALE


def run(rate_dps, ext_mnm, offset_deg, t_end=120.0, gate=2.0, clamp=2.0, db=150.0):
    hist = d.simulate(KP, KD, KW, d.NOMINAL_I, 0.5, t_end=t_end,
                      phi_offset_deg=offset_deg,
                      slew_mnm_step=3.0, ff_mnm=2.05, ff_deadband_rps=0.5,
                      trim_rate_dps=rate_dps, trim_db_rpm=db,
                      trim_gate_deg=gate, trim_clamp_deg=clamp,
                      ext_mnm=ext_mnm)
    t, phi, rpm, u, trim = hist[:, 0], hist[:, 1], hist[:, 3], hist[:, 4], hist[:, 6]
    late = t > 5.0
    fell = bool((abs(phi) > 30.0).any())
    return {
        "rate": rate_dps,
        "fallen": fell,
        "max_phi": float(abs(phi[late]).max()) if late.any() else float("nan"),
        "max_rpm": float(abs(rpm).max()),
        "end_rpm": float(rpm[-1]),
        "end_trim": float(trim[-1]),
        "mean_u_late": float(u[late].mean()) if late.any() else float("nan"),
    }


def main():
    print("slow zero trim vs constant cable pull (4 mN.m), 120 s, 0.5 deg release")
    print("rate   fallen  max_phi  max_rpm  end_rpm  end_trim  mean_u(>5s)")
    for rate in (0.0, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5):
        r = run(rate, 4.0, 0.0)
        print("%-6.2f %-7s %7.2f %8.0f %8.0f %9.2f %11.2f" % (
            r["rate"], r["fallen"], r["max_phi"], r["max_rpm"],
            r["end_rpm"], r["end_trim"], r["mean_u_late"]))

    print("\nno cable pull (ext = 0), same scan: does the trim stay put?")
    print("rate   fallen  max_phi  max_rpm  end_rpm  end_trim  mean_u(>5s)")
    for rate in (0.0, 0.05, 0.2):
        r = run(rate, 0.0, 0.0)
        print("%-6.2f %-7s %7.2f %8.0f %8.0f %9.2f %11.2f" % (
            r["rate"], r["fallen"], r["max_phi"], r["max_rpm"],
            r["end_rpm"], r["end_trim"], r["mean_u_late"]))

    print("\n0.3 deg zero error + 4 mN.m pull (worst combination)")
    print("rate   fallen  max_phi  max_rpm  end_rpm  end_trim  mean_u(>5s)")
    for rate in (0.0, 0.05, 0.2):
        r = run(rate, 4.0, 0.3)
        print("%-6.2f %-7s %7.2f %8.0f %8.0f %9.2f %11.2f" % (
            r["rate"], r["fallen"], r["max_phi"], r["max_rpm"],
            r["end_rpm"], r["end_trim"], r["mean_u_late"]))


if __name__ == "__main__":
    main()
