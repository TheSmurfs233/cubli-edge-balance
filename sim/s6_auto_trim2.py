"""Close the loop by hand and walk the balance reference with the wheel speed.

Why not reuse s4_design.simulate(): its trim branches never clamp (trim ran to
133 deg in the first attempt), so its numbers cannot be trusted for this
question.  This file integrates the same plant with its own trim law.

Steady state of the real loop (derived from the equations, and matching the
bench: phi = -2.32 deg <-> 1340 rpm):

    body  : mgh * sin(phi_true) - tau + fr = 0
    wheel : tau - fr = 0
    law   : tau = kp*(phi - trim) + kw*omega + ff

  =>  phi_true = 0  (the body is pinned at the true balance point)
      omega_ss = (kp/kw) * (trim - phi_ss),   phi_ss = -(zero error)

so the wheel can only stop when trim == phi_ss, i.e. when the reference is moved
onto the angle the loop is already holding.  There is no separate brake: braking
the wheel means leaning the body, and gravity pays the lean back.

Run:  python sim/s6_auto_trim2.py
"""
import math
import random

# plant
M = 0.459
G = 9.81
H = 0.073
MGH = M * G * H                 # N.m/rad
I_BODY = 3.310e-3               # kg.m^2 (solid cube about an edge)
J_W = 5.12e-5                   # kg.m^2 (wheel + rotor, measured)
TAU_F = 2.05e-3                 # N.m
CEIL = 28e-3                    # firmware ceiling
SLEW = 3e-3                     # per 5 ms frame
OMEGA_STICK = 0.5               # rad/s

# gains, 0.7x nominal
KP = 0.9197
KD = 0.1229
KW = 2.674e-4

DT = 5e-3
RPM = 60.0 / (2.0 * math.pi)


def run(zero_deg, k_trim, t_end=90.0, clamp_deg=3.0, gate_deg=4.0,
        trip_rpm=1700.0, noise_deg=0.02, bias_dps=0.25, verbose=False):
    z = math.radians(zero_deg)
    phi = 0.0
    v = 0.0
    omega = 0.0
    trim = 0.0
    tau_cmd = 0.0
    t = 0.0
    log = []
    fell = False
    rng = random.Random(7)

    while t < t_end:
        rpm_now = omega * RPM
        if abs(rpm_now) > trip_rpm:
            fell = True
            break

        # measurement noise and gyro bias, as on the bench
        phi_m = phi + math.radians(rng.gauss(0.0, noise_deg))
        v_m = v + math.radians(bias_dps) + math.radians(rng.gauss(0.0, noise_deg) / DT * 0.001)

        ff = TAU_F if omega > OMEGA_STICK else (-TAU_F if omega < -OMEGA_STICK else 0.0)
        tau = KP * (phi_m - trim) + KD * v_m + KW * omega + ff
        if abs(tau) < 1e-3:
            tau = 0.0
        tau = max(-CEIL, min(CEIL, tau))
        step = tau - tau_cmd
        step = max(-SLEW, min(SLEW, step))
        tau_cmd += step
        tau = tau_cmd

        n = 8
        h = DT / n
        for _ in range(n):
            if abs(omega) < 0.05 and abs(tau) <= TAU_F:
                fr = tau
                v_dot = (MGH * math.sin(phi + z) - tau + fr) / (I_BODY + J_W)
                w_dot = 0.0
            else:
                fr = TAU_F if omega >= 0.0 else -TAU_F
                v_dot = (MGH * math.sin(phi + z) - tau + fr) / I_BODY
                w_dot = (tau - fr) / J_W - v_dot
            v += v_dot * h
            phi += v * h
            omega += w_dot * h

        if k_trim > 0.0 and abs(math.degrees(phi)) < gate_deg:
            trim -= math.radians(k_trim) * (omega * RPM) * DT
            lim = math.radians(clamp_deg)
            trim = max(-lim, min(lim, trim))

        t += DT
        log.append((t, math.degrees(phi), omega * RPM, math.degrees(trim)))
        if verbose and abs((t % 5.0)) < DT * 0.5:
            print("  t=%5.1f  phi=%6.2f  rpm=%7.0f  trim=%6.2f"
                  % (t, math.degrees(phi), omega * RPM, math.degrees(trim)))
    return log, fell


def summarise(tag, log, fell):
    rpm = [r for _, _, r, _ in log]
    phi = [p for _, p, _, _ in log]
    trim = [x for _, _, _, x in log]
    t = [x for x, _, _, _ in log]
    settle = None
    for i in range(len(rpm)):
        if all(abs(rpm[j]) < 60.0 for j in range(i, len(rpm))):
            settle = t[i]
            break
    print("%-22s peak=%6.0f tail=%6.0f  phi[%5.2f,%5.2f] trim_end=%5.2f  %s"
          % (tag, max(abs(r) for r in rpm), rpm[-1], min(phi), max(phi), trim[-1],
             "FELL" if fell else ("settles %.0fs" % settle if settle else "still turning")))


def main():
    print("plant: mgh=%.4f N.m/rad  I=%.3g  J=%.3g  tau_f=%.2f mN.m  ceiling=%.0f mN.m"
          % (MGH, I_BODY, J_W, TAU_F * 1e3, CEIL * 1e3))
    print("gains x0.7: kp=%.4f  kw=%.3e  ->  %.0f rpm per deg of (trim - phi_ss)"
          % (KP, KW, KP / KW * RPM))
    print()
    print("== zero error 2.3 deg, trim walked by the wheel speed ==")
    for k in (0.0, 5e-5, 1.5e-4, 3e-4, 5e-4, 1e-3):
        log, fell = run(2.3, k)
        summarise("k_trim=%.0e" % k, log, fell)
    print()
    print("== k_trim=1.5e-4, different zero errors ==")
    for z in (0.5, 1.0, 2.3, 3.0):
        log, fell = run(z, 1.5e-4)
        summarise("zero %4.1f deg" % z, log, fell)
    print()
    print("== timeline, zero 2.3 deg, k_trim=1.5e-4 ==")
    log, fell = run(2.3, 1.5e-4, verbose=True)


if __name__ == "__main__":
    main()
