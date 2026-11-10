"""Why the wheel-speed walk needs a brake, and what a stable form looks like.

Bench fact (2026-11-11 01:00, Code=89156): with BAL_TRIM_W_K = 3e-4 the
reference walked straight into its clamp (trim = -3.00 deg) while the wheel
sat at +405 rpm, and the cube settled into a ~1.6 s limit cycle between
phi = -0.0 and -3.9 deg.  Freezing the walk and setting trim = phi made the
cube stand quietly (phi -2.2 deg, wheel decaying 174 -> 75 rpm).

The law under test (same as sim/s6_auto_trim2.py):

  body  : I * phi_ddot = mgh * sin(phi + zero) - tau + fr
  wheel : J * (phi_ddot + omega_dot) = tau - fr
  law   : tau = kp*(phi - trim) + kd*phi_dot + kw*omega + ff, slew + ceiling
  walk  : trim -= k * omega * dt        (current firmware)

Trim is an INPUT to the loop, not a state; phi follows trim, so (trim - phi)
barely changes while the walk keeps pushing.  This script scans the walk gain
and compares three structures:

  A  trim -= k * omega * dt                (firmware today)
  B  trim -= k * (phi - trim) * dt         (reference chases the angle it holds)
  C  trim -= k * sgn(omega) * dt           (bang-bang, immune to the gain)

Run:  .tmp_env\\venv\\Scripts\\python.exe sim\\s6_walk_stab.py
"""
import math
import random

M, G, H = 0.459, 9.81, 0.073
MGH = M * G * H
I_BODY = 3.310e-3
J_W = 5.12e-5
TAU_F = 2.05e-3
CEIL = 28e-3
SLEW = 3e-3
OMEGA_STICK = 0.5

KP, KD, KW = 0.9197, 0.1229, 2.674e-4
DT = 5e-3
RPM = 60.0 / (2.0 * math.pi)


def run(mode, k, zero_deg=2.3, t_end=120.0, clamp_deg=3.0, gate_deg=4.0,
        trip_rpm=1700.0, noise_deg=0.02, bias_dps=0.25):
    rng = random.Random(11)
    z = math.radians(zero_deg)
    phi = 0.0
    v = 0.0
    omega = 0.0
    trim = 0.0
    tau_cmd = 0.0
    t = 0.0
    hist = []

    while t < t_end:
        if abs(omega * RPM) > trip_rpm:
            return hist, True

        phi_m = phi + math.radians(rng.gauss(0.0, noise_deg))
        v_m = v + math.radians(bias_dps) + math.radians(rng.gauss(0.0, noise_deg) * 0.001 / DT)

        ff = TAU_F if omega > OMEGA_STICK else (-TAU_F if omega < -OMEGA_STICK else 0.0)
        tau = KP * (phi_m - trim) + KD * v_m + KW * omega + ff
        if abs(tau) < 1e-3:
            tau = 0.0
        tau = max(-CEIL, min(CEIL, tau))
        tau_cmd += max(-SLEW, min(SLEW, tau - tau_cmd))
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

        if abs(math.degrees(phi)) < gate_deg:
            lim = math.radians(clamp_deg)
            if mode == "A":
                trim -= math.radians(k) * (omega * RPM) * DT
            elif mode == "B":
                trim -= math.radians(k) * math.degrees(phi) * DT
            elif mode == "C":
                trim -= math.radians(k) * (1.0 if omega > 0.0 else -1.0) * DT
            trim = max(-lim, min(lim, trim))

        t += DT
        hist.append((t, math.degrees(phi), omega * RPM, math.degrees(trim)))

    return hist, False


def summarise(tag, hist, fell):
    if not hist:
        print("%-26s empty" % tag)
        return
    rpm = [r for _, _, r, _ in hist]
    phi = [p for _, p, _, _ in hist]
    trim = [x for _, _, _, x in hist]
    t = [x for x, _, _, _ in hist]
    settle = None
    for i in range(len(rpm)):
        if all(abs(rpm[j]) < 60.0 for j in range(i, len(rpm))):
            settle = t[i]
            break
    print("%-26s peak=%6.0f tail=%6.0f phi[%5.2f,%5.2f] trim_end=%5.2f  %s"
          % (tag, max(abs(r) for r in rpm), rpm[-1], min(phi), max(phi), trim[-1],
             "FELL" if fell else ("settles %3.0fs" % settle if settle else "spinning")))


def main():
    print("plant mgh=%.4f I=%.3g J=%.3g | gains x0.7 kp=%.3f kw=%.2e (%.0f rpm per deg)"
          % (MGH, I_BODY, J_W, KP, KW, KP / KW * RPM))
    print()
    print("== A: trim -= k*omega*dt  (firmware today) ==")
    for k in (5e-5, 1.5e-4, 3e-4, 5e-4, 1e-3, 3e-3):
        h, fell = run("A", k)
        summarise("A k=%.0e" % k, h, fell)

    print()
    print("== B: trim -= k*(phi - trim)*dt  (reference chases the held angle) ==")
    for k in (0.2, 0.5, 1.0, 2.0):
        h, fell = run("B", k)
        summarise("B k=%.1f" % k, h, fell)

    print()
    print("== C: trim -= k*sgn(omega)*dt  (bang-bang brake) ==")
    for k in (0.05, 0.1, 0.2):
        h, fell = run("C", k)
        summarise("C k=%.2f" % k, h, fell)

    print()
    print("== A timeline, k=3e-4 (what the bench did) ==")
    h, fell = run("A", 3e-4)
    for (t, p, r, tr) in h:
        if abs(t % 10.0) < DT * 0.5:
            print("  t=%5.1f  phi=%6.2f  rpm=%7.0f  trim=%6.2f" % (t, p, r, tr))


if __name__ == "__main__":
    main()
