"""Compare zero-walk laws on the 2-DOF plant, including the trim -> phi coupling.

    python sim\\s7_trim_law.py

Plant (same numbers as sim/s6_auto_trim2.py):
    I * dphi2 = mgh*sin(phi) - tau + tau_f_body
    J * (dphi2 + domega) = tau - tau_f_wheel
    tau = clamp(kp*(phi - trim) + kd*dphi + kw*omega, +-ceiling), slew limited

The point of this script is the *law*, so the controller runs at the real 200 Hz
and the walk law runs at a chosen update rate.
"""
import numpy as np

MGH = 0.3287
I_TOT = 3.31e-3
J_W = 5.12e-5
TAU_F = 2.05e-3
KP = 1.313830 * 0.7
KD = 0.175517 * 0.7
KW = 3.819719e-04 * 0.7
CEIL = 0.028
DT = 5e-3
SLEW = 0.003


def run(law, k_deg_per_rpm, walk_hz, seed_wheel_rpm=1400.0, trim0=0.0,
        phi0=1.66, deadband_rpm=40.0, t_end=300.0, freeze_sat=True):
    """law: 'trim' (move the reference) or 'bias' (offset on phi only)."""
    phi = np.radians(phi0)
    dphi = 0.0
    omega = seed_wheel_rpm * 2.0 * np.pi / 60.0
    trim = trim0
    bias = 0.0
    tau_prev = 0.0
    walk_dt = 1.0 / walk_hz
    next_walk = 0.0

    hist_t, hist_w, hist_phi, hist_trim = [], [], [], []
    t = 0.0
    while t < t_end:
        # ---- controller ----
        ref = trim if law == "trim" else 0.0
        tau_cmd = KP * (phi - np.radians(ref)) + KD * dphi + KW * omega
        if law == "bias":
            tau_cmd -= KP * np.radians(bias)
        if tau_cmd > CEIL:
            tau_cmd = CEIL
            sat = 1
        elif tau_cmd < -CEIL:
            tau_cmd = -CEIL
            sat = 1
        else:
            sat = 0
        d = tau_cmd - tau_prev
        if d > SLEW:
            tau_cmd = tau_prev + SLEW
        elif d < -SLEW:
            tau_cmd = tau_prev - SLEW
        tau_prev = tau_cmd

        # ---- plant ----
        grav = MGH * np.sin(phi)
        friction_b = 0.0
        friction_w = TAU_F if abs(omega) > 1e-3 else 0.0
        if abs(dphi) < 1e-4:
            friction_b = 0.0
        dphi2 = (grav - tau_cmd) / I_TOT
        domega = (tau_cmd - friction_w) / J_W - dphi2
        dphi += dphi2 * DT
        omega += domega * DT
        phi += dphi * DT
        t += DT

        # ---- walk law ----
        if t >= next_walk:
            next_walk += walk_dt
            rpm = omega * 60.0 / (2.0 * np.pi)
            frozen = abs(rpm) < deadband_rpm or abs(np.degrees(phi)) > 2.0
            if freeze_sat and sat:
                frozen = True
            if not frozen:
                step = -k_deg_per_rpm * rpm
                lim = k_deg_per_rpm * 1500.0 * 0.25 if walk_dt < 1.0 else 0.05
                lim = max(lim, 0.002)
                step = float(np.clip(step, -lim, lim))
                if law == "trim":
                    trim = float(np.clip(trim + step, -3.0, 3.0))
                else:
                    bias = float(np.clip(bias + step, -3.0, 3.0))

        hist_t.append(t)
        hist_w.append(omega * 60.0 / (2.0 * np.pi))
        hist_phi.append(np.degrees(phi))
        hist_trim.append(trim if law == "trim" else bias)

    return np.array(hist_t), np.array(hist_w), np.array(hist_phi), np.array(hist_trim)


def report(name, k, hz, law="trim", **kw):
    t, w, phi, tr = run(law, k, hz, **kw)
    tail = w[t > t[-1] - 30.0]
    settled = None
    for i in range(len(t)):
        if np.all(np.abs(w[i:]) < 60.0):
            settled = t[i]
            break
    print("%-28s k=%-7.1e hz=%-4.0f tail |w|max=%7.1f rpm  tail |phi|max=%5.2f deg  "
          "trim=%6.2f  settle=%s" % (
              name, k, hz, np.abs(tail).max(), np.abs(phi[t > t[-1] - 30]).max(),
              tr[-1], ("%.0f s" % settled) if settled is not None else "never"))


print("=== start from omega = 1400 rpm, trim = 0, phi = 1.66 deg ===")
report("old firmware walk", 3.0e-4, 200.0)
report("host loop (tonight)", 2.0e-4, 0.5)
report("proposed slow", 1.0e-4, 2.0)
report("proposed slower", 5.0e-5, 2.0)
print()
print("=== start from omega = 300 rpm ===")
report("old firmware walk", 3.0e-4, 200.0, seed_wheel_rpm=300.0)
report("host loop (tonight)", 2.0e-4, 0.5, seed_wheel_rpm=300.0)
report("proposed slow", 1.0e-4, 2.0, seed_wheel_rpm=300.0)
