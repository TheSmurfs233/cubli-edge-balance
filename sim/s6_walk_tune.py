"""Pick the wheel-walk gain for the real start-up error.

The button start takes the pose in the operator's hand as zero, so a fresh run
begins with a few degrees of reference error (measured: 43.99 vs 46.3 -> 2.3 deg).
At kp = 22.9 mN.m/deg even 1.2 deg needs more than the 28 mN.m ceiling, so the
loop starts saturated and only the wheel walk can remove the offset.

This script scans the walk gain and the clamp for that start-up case and reports
how long the offset takes to disappear, how much of it is spent saturated, and
how far the body has to lean.

Run:  python sim/s6_walk_tune.py
"""
import math
import s6_auto_trim2 as sim

RPM = sim.RPM


def run_case(zero_deg, k, clamp_deg, t_end=90.0):
    """Re-run the plant but also report saturation duty and settle time."""
    orig = sim.run

    # sim.run already integrates the full loop; re-implement the scan on top of
    # its internals by calling it and re-deriving the metrics from a trace.
    trace = []
    z = math.radians(zero_deg)
    phi = 0.0
    v = 0.0
    omega = 0.0
    trim = 0.0
    tau_cmd = 0.0
    t = 0.0
    sat_steps = 0
    steps = 0
    settle = None
    fell = False
    max_phi = 0.0

    while t < t_end:
        rpm_now = omega * RPM
        if abs(rpm_now) > 1700.0:
            fell = True
            break
        ff = sim.TAU_F if omega > sim.OMEGA_STICK else (-sim.TAU_F if omega < -sim.OMEGA_STICK else 0.0)
        tau = sim.KP * (phi - trim) + sim.KD * v + sim.KW * omega + ff
        if abs(tau) < 1e-3:
            tau = 0.0
        tau = max(-sim.CEIL, min(sim.CEIL, tau))
        step = max(-sim.SLEW, min(sim.SLEW, tau - tau_cmd))
        tau_cmd += step
        tau = tau_cmd

        # trim walk: wheel speed drives the reference, clamped, gated on |phi|
        if abs(phi) < math.radians(4.0):
            trim -= math.radians(k * (omega * RPM)) * sim.DT
            trim = max(-math.radians(clamp_deg), min(math.radians(clamp_deg), trim))

        steps += 1
        if abs(tau_cmd) > sim.CEIL * 0.99:
            sat_steps += 1
        max_phi = max(max_phi, abs(phi))

        n = 8
        h = sim.DT / n
        for _ in range(n):
            if abs(omega) < 0.05 and abs(tau) <= sim.TAU_F:
                fr = tau
                v_dot = (sim.MGH * math.sin(phi + z) - tau + fr) / (sim.I_BODY + sim.J_W)
                w_dot = 0.0
            else:
                fr = sim.TAU_F if omega >= 0.0 else -sim.TAU_F
                v_dot = (sim.MGH * math.sin(phi + z) - tau + fr) / sim.I_BODY
                w_dot = (tau - fr) / sim.J_W - v_dot
            v += v_dot * h
            phi += v * h
            omega += w_dot * h
        t += sim.DT
        if settle is None and abs(omega * RPM) < 60.0:
            settle = t
        trace.append((t, phi, omega * RPM, trim, tau_cmd))

    return {
        "fell": fell,
        "settle_s": settle,
        "sat_pct": 100.0 * sat_steps / max(steps, 1),
        "max_phi_deg": math.degrees(max_phi),
        "end_rpm": omega * RPM,
        "end_trim_deg": math.degrees(trim),
        "end_phi_deg": math.degrees(phi),
    }


def main():
    print("start-up reference error 2.3 deg (button start in the hand)")
    print()
    print("  k      clamp   settle  sat%   maxphi   end_rpm  end_trim  fell")
    for clamp in (3.0, 5.0):
        for k in (3e-4, 6e-4, 1e-3, 2e-3):
            r = run_case(2.3, k, clamp)
            print(" %-7.0e %-6.1f %-7s %-6.1f %-8.2f %-8.0f %-9.2f %s" % (
                k, clamp,
                "-" if r["settle_s"] is None else "%.1f" % r["settle_s"],
                r["sat_pct"], r["max_phi_deg"], r["end_rpm"],
                r["end_trim_deg"], "YES" if r["fell"] else "no"))
    print()
    print("start-up reference error 4.0 deg (operator held it well off balance)")
    print()
    print("  k      clamp   settle  sat%   maxphi   end_rpm  end_trim  fell")
    for clamp in (5.0, 6.0):
        for k in (6e-4, 1e-3, 2e-3):
            r = run_case(4.0, k, clamp)
            print(" %-7.0e %-6.1f %-7s %-6.1f %-8.2f %-8.0f %-9.2f %s" % (
                k, clamp,
                "-" if r["settle_s"] is None else "%.1f" % r["settle_s"],
                r["sat_pct"], r["max_phi_deg"], r["end_rpm"],
                r["end_trim_deg"], "YES" if r["fell"] else "no"))


if __name__ == "__main__":
    main()
