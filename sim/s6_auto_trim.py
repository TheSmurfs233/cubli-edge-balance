"""Can the wheel be brought to zero by moving the reference, not by leaning?

The steady state of the closed loop pins the body at the true balance point
(phi = -z) and parks the wheel at

    omega_ss = (kp/kw) * (z + trim)

so a zero error z is paid for entirely in wheel speed.  The only knob that
changes it is `trim`: bring trim to -z and the wheel goes to zero while the body
stays exactly where it already was.

This script answers two questions:

  1. how fast may trim be walked without tipping the cube or saturating, and
  2. what does the wheel trajectory look like at that rate.

Run:  python sim/s6_auto_trim.py
"""
import math
import s4_design as d

SCALE = 0.7
KP = d.KP * SCALE
KD = d.KD * SCALE
KW = d.KW * SCALE
CEILING = 28.0
TRIP_RPM = 1700.0


def run(offset_deg, k, clamp_deg=5.0, t_end=90.0):
    h = d.simulate(KP, KD, KW, d.NOMINAL_I, 0.0, t_end=t_end, dt=5e-3,
                   phi_offset_deg=offset_deg, ff_mnm=d.TAU_FRIC * 1e3,
                   trim_w_k=k, trim_gate_deg=4.0, trim_clamp_deg=clamp_deg,
                   slew_mnm_step=3.0, u_max_mnm=CEILING, trip_rpm=TRIP_RPM,
                   noise_deg=0.02, bias_dps=0.25)
    t = h[:, 0]
    phi = h[:, 1]
    rpm = h[:, 3]
    trim = h[:, 6]
    return t, phi, rpm, trim


def describe(tag, offset, k):
    t, phi, rpm, trim = run(offset, k)
    end_max_rpm = max(abs(rpm[len(rpm) // 2:]))
    fell = abs(phi[-1]) > 20.0
    settle = None
    for i in range(len(t)):
        if abs(rpm[i]) < 50.0 and all(abs(rpm[j]) < 100.0 for j in range(i, len(t))):
            settle = t[i]
            break
    print("%-34s peak_rpm=%7.0f  tail_rpm=%7.0f  max|phi|=%5.2f  trim_end=%5.2f  %s"
          % (tag, max(abs(rpm)), end_max_rpm, max(abs(phi)), math.degrees(trim[-1]),
             "FELL" if fell else ("settles %.1fs" % settle if settle else "still turning")))


def main():
    print("plant: mgh=%.4f N.m/rad  J=%.3g  tau_f=%.2f mN.m  ceiling=%.0f mN.m"
          % (d.MGH, d.J_W, d.TAU_FRIC * 1e3, CEILING))
    print("gains x%.1f: kp=%.4f kd=%.4f kw=%.3e (%.1f rpm per deg of (trim-phi))"
          % (SCALE, KP, KD, KW, KP / KW / (2 * math.pi / 60 * 60)))
    print()

    print("== zero error 2.3 deg, trim walked by the wheel speed ==")
    for k in (0.0, 5e-6, 1e-5, 2e-5, 5e-5, 1e-4):
        describe("k_w=%.0e" % k, 2.3, k)
    print()

    print("== same speed, different zero errors (k_w=2e-5) ==")
    for off in (0.5, 1.0, 2.3, 3.0, 4.0):
        describe("zero error %.1f deg" % off, off, 2e-5)
    print()

    print("== timeline for the chosen rate (zero 2.3 deg, k_w=2e-5) ==")
    t, phi, rpm, trim = run(2.3, 2e-5)
    print("  t_s   phi_deg   wheel_rpm   trim_deg")
    for s in range(0, 61, 5):
        i = int(s / 0.005)
        if i >= len(t):
            break
        print("%5.1f %9.2f %11.0f %10.2f"
              % (t[i], math.degrees(phi[i]), rpm[i], math.degrees(trim[i])))


if __name__ == "__main__":
    main()
