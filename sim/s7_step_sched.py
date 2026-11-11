"""Check the error-scheduled zero walk on the 2-DOF plant.

The firmware walk runs at 2 Hz and currently moves the reference by
    step = -k * omega      (k = 5e-5 deg per rpm per step, capped at 0.05 deg)
so recovering a 2.6 deg reference error at 1500 rpm takes about 9 s and a
small residual takes a minute.  This script checks a schedule that raises the
gain and the step cap while the loop error is large:

    rel = phi - trim
    gain   = k * (|rel| >= gate ? big_x : 1)
    stepcap= (|rel| >= gate ? big_step : step)

It reports settling time and residual amplitude for a few initial errors, and a
pure-transient case (reference already right, body pushed) to make sure the
schedule does not turn a disturbance into a sustained swing.

    python sim\\s7_step_sched.py
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
PPR_RPM_PER_DEG = 573.0
FF_MAX = 2.05e-3
FF_BAND = 40.0 * 2.0 * np.pi / 60.0
FF_VISC = 3.0e-6
TRIM_CLAMP = 5.0


def run(k, walk_hz=2.0, step_small=0.05, step_big=0.15, gate=0.5, big_x=5.0,
        rel0=0.0, push0=0.0, t_end=120.0):
    """rel0: reference is off by this many degrees, wheel starts at 573*rel0."""
    trim = -rel0
    phi = np.radians(push0)
    dphi = 0.0
    omega = (trim - phi) * PPR_RPM_PER_DEG * 2.0 * np.pi / 60.0
    tau_prev = 0.0
    walk_dt = 1.0 / walk_hz
    next_walk = 0.0
    omega_f = omega
    t = 0.0

    hist_t, hist_w, hist_phi, hist_trim, hist_tau = [], [], [], [], []
    while t < t_end:
        rpm = omega * 60.0 / (2.0 * np.pi)
        ff = FF_MAX * max(-1.0, min(1.0, omega / FF_BAND)) + FF_VISC * omega
        tau_cmd = KP * (phi - np.radians(trim)) + KD * dphi + KW * omega + ff
        sat = 0
        if tau_cmd > CEIL:
            tau_cmd, sat = CEIL, 1
        elif tau_cmd < -CEIL:
            tau_cmd, sat = -CEIL, 1
        d = tau_cmd - tau_prev
        if d > SLEW:
            tau_cmd = tau_prev + SLEW
        elif d < -SLEW:
            tau_cmd = tau_prev - SLEW
        tau_prev = tau_cmd

        grav = MGH * np.sin(np.radians(phi))
        friction_w = TAU_F * (1.0 if omega > 1e-3 else (-1.0 if omega < -1e-3 else 0.0))
        dphi2 = (grav - tau_cmd) / I_TOT
        domega = (tau_cmd - friction_w) / J_W - dphi2
        dphi += dphi2 * DT
        omega += domega * DT
        phi += dphi * DT
        omega_f += (omega - omega_f) * DT / (8.0 + DT)
        t += DT

        if t >= next_walk:
            next_walk += walk_dt
            rel = phi - trim
            big = abs(rel) >= gate
            gain = k * (big_x if big else 1.0)
            cap = step_big if big else step_small
            step = -gain * (omega_f * 60.0 / (2.0 * np.pi))
            step = float(np.clip(step, -cap, cap))
            trim = float(np.clip(trim + step, -TRIM_CLAMP, TRIM_CLAMP))

        hist_t.append(t)
        hist_w.append(omega * 60.0 / (2.0 * np.pi))
        hist_phi.append(phi)
        hist_trim.append(trim)
        hist_tau.append(tau_cmd)

    return (np.array(hist_t), np.array(hist_w), np.array(hist_phi),
            np.array(hist_trim), np.array(hist_tau))


def report(tag, **kw):
    t, w, phi, trim, tau = run(**kw)
    tail = t > t[-1] - 20.0
    settled = None
    for i in range(len(t)):
        if np.all(np.abs(w[i:]) < 60.0):
            settled = t[i]
            break
    sat_frac = 100.0 * np.mean(np.abs(tau) >= CEIL - 1e-9)
    print("%-34s settle=%-8s tail|w|max=%6.1f rpm  tail|phi-trim|=%5.2f deg  "
          "|phi|max=%5.2f  sat=%4.1f%%" % (
              tag, ("%.0f s" % settled) if settled is not None else "never",
              np.abs(w[tail]).max(), np.abs(phi[tail] - trim[tail]).max(),
              np.abs(phi[tail]).max(), sat_frac))


print("=== reference error only (wheel starts at 573 * error) ===")
for err in (0.5, 1.0, 2.6):
    report("plain  k=5e-5 cap .05   err %.1f" % err, k=5.0e-5, rel0=err)
    report("sched  gate .5 x5 cap .15 err %.1f" % err, k=5.0e-5, rel0=err)
    print()

print("=== transient only (reference right, body pushed 1.0 deg) ===")
report("plain  k=5e-5 cap .05", k=5.0e-5, push0=1.0)
report("sched  gate .5 x5 cap .15", k=5.0e-5, push0=1.0)
print()

print("=== steady state noise free run, 1200 rpm residual ===")
report("plain  k=5e-5 cap .05", k=5.0e-5, rel0=2.1)
report("sched  gate .5 x5 cap .15", k=5.0e-5, rel0=2.1)
