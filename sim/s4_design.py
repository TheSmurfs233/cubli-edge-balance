#!/usr/bin/env python3
# -*- coding: ascii -*-
"""S4 design study: edge-balancing inverted pendulum on one reaction wheel.

Plant, taken about the pivot edge (the wheel axis is parallel to that edge):

    I_tot * phi_dd = m*g*h*sin(phi) - tau + tau_f
    J_w  *(phi_dd + omega_dd) = tau - tau_f
    =>  omega_dd = (tau - tau_f)/J_w - phi_dd

    phi    body tilt away from the balance point [rad]
    omega  wheel speed relative to the body [rad/s]
    tau    motor torque, positive = spool the wheel up [N.m]
           the reaction on the body is -tau, which is why a positive tilt
           needs a positive command:  u = kp*phi + kd*phi_d + kw*omega

Everything SI.  Run:  python3 sim/s4_design.py
"""

import math
import numpy as np
from control import lqr

# ---------------------------------------------------------------- plant data
G = 9.81
A_CUBE = 0.104           # edge length [m]
M_CUBE = 0.301 + 0.158   # incl. battery [kg]
H_COM = 0.073            # COM to pivot, measured by user [m]
J_W = 5.12e-5            # wheel + rotor, MEASURED on the bench (S3)
KT = 0.02891             # N.m/A
R_PHASE = 2.3            # ohm
V_LIM = 5.7              # V left after the modulation limit
I_MAX = 1.20             # A
TAU_MAX = KT * I_MAX     # 34.7 mN.m
TAU_FRIC = 2.05e-3       # N.m, measured (S3)
TAU_MIN = 1.0e-3         # below this the firmware would round the command to zero
OMEGA_STICK = 0.5        # |omega| below this may be held by static friction [rad/s]

MGH = M_CUBE * G * H_COM
I_SOLID = (2.0 / 3.0) * M_CUBE * A_CUBE ** 2      # uniform solid cube
I_HOLLOW = (7.0 / 6.0) * M_CUBE * A_CUBE ** 2     # thin-walled shell
NOMINAL_I = I_HOLLOW     # conservative pick until we measure it

# controller gains (see docs/44): u = kp*phi + kd*phi_d + kw*omega
# 2026-11-05: the wheel-speed weight moved from 50 rpm to the 50 rpm set below
# after the S6b wind-up runs.  The old set (kp 0.728, kw 4.0e-5) needed only
# ~1.5 mN.m of constant torque to walk the wheel into the 1700 rpm trip in a few
# seconds, because omega_ss = mgh*sin(err)/kw.  With this set the same 1.5 mN.m
# settles near 300 rpm and the capture window is unchanged (3 deg recovers,
# 4 deg does not), see docs/44 4.3 and docs/45 4.13.
KP, KD, KW = 1.313830, 0.175517, 3.819719e-04
SIGMA_OMEGA_RPM = 50.0


def tau_limit(omega):
    """Wheel torque still available at a given wheel speed (12 V bus).

    The back-EMF depends on the rotor speed in the absolute frame (omega + phi_d).
    phi_d stays below ~0.2 rad/s while balancing vs omega up to 200 rad/s, so the
    approximation abs(omega) is good to well under 1%.
    """
    emf = KT * abs(omega)
    i_cap = (V_LIM - emf) / R_PHASE
    if i_cap > I_MAX:
        i_cap = I_MAX
    if i_cap < 0.0:
        i_cap = 0.0
    return KT * i_cap


def design_lqr(inertia, u0):
    """LQR with Bryson-style weights; u0 = torque we are willing to spend."""
    i_tot = inertia + J_W
    a = np.array([[0.0, 1.0, 0.0],
                  [MGH / i_tot, 0.0, 0.0],
                  [-MGH / i_tot, 0.0, 0.0]])
    b = np.array([[0.0],
                  [-1.0 / i_tot],
                  [1.0 / J_W + 1.0 / i_tot]])
    # sigma_omega = 50 rpm: the wheel-speed state is what the 1700 rpm trip
    # watches, so it is weighted far above the old 1/50^2 placeholder.
    sig_w = SIGMA_OMEGA_RPM * 2.0 * math.pi / 60.0   # rpm -> rad/s
    q = np.diag([1 / 0.05 ** 2, 1 / 0.5 ** 2, 1 / sig_w ** 2])
    r = np.array([[1 / u0 ** 2]])
    k, _, _ = lqr(a, b, q, r)
    return -k.ravel()     # u = -K x; our u uses the opposite sign, so negate


def simulate(kp, kd, kw, inertia, phi0_deg, t_end=10.0, dt=5e-3,
             noise_deg=0.0, bias_dps=0.25, delay_steps=0, omega0=0.0,
             ki=0.0, z_max=400.0, phi_offset_deg=0.0, ki_deadband_rpm=0.0,
             dither_mnm=0.0, dither_hz=60.0, trim_k=0.0, slew_mnm_step=0.0,
             trim_gate_deg=2.0, trim_clamp_deg=1.0,
             bias_k_mnm_per_rps=0.0, bias_clamp_mnm=3.0, bias_gate_deg=2.0,
             ff_mnm=0.0, ff_deadband_rps=0.5,
             trim_db_rpm=0.0, ext_mnm=0.0, trim_rate_dps=0.0,
             trim_u_k=0.0, trim_u_tau_s=0.5, trim_w_k=0.0,
             u_max_mnm=0.0, trip_rpm=0.0):
    """One run at 200 Hz with ZOH, torque saturation, friction, gyro bias.

    ki != 0 adds the optional fourth state z = integral(omega) [rad], used by the
    S6 study in docs/44 4.7.  With ki = 0 this is exactly the S4 controller.

    Extra knobs for the T20 study (docs/46), all default-off:
      phi_offset_deg  constant zero error seen by the controller (deg)
      ki_deadband_rpm only integrate omega outside this band (wheel speed, rpm)
      dither_mnm      peak amplitude of a square dither added to the command
      dither_hz       dither frequency
      trim_k          angle-zero adaptation: trim' = trim_k * omega [rad/s]
      trim_db_rpm     if > 0, adapt with sign(omega) instead and only above this
                      wheel speed (the slow trim used for S6b, docs/46)
      ext_mnm         constant external torque on the body in mN.m (cable pull)
      slew_mnm_step   max change of the command per control period [mN.m]
    """
    i_tot = inertia + J_W
    phi = math.radians(phi0_deg)
    dphi = 0.0
    omega = omega0
    z = 0.0
    trim = 0.0
    u_lp = 0.0
    bias = 0.0
    ff_sgn = 0.0
    u_prev = 0.0
    hist = []
    rng = np.random.RandomState(1)
    steps = int(t_end / dt)
    n_sub = 10
    h = dt / n_sub
    delay_buf = [0.0] * (delay_steps + 1)
    for k in range(steps):
        phi_m = phi + math.radians(phi_offset_deg + rng.randn() * noise_deg)
        dphi_m = dphi + math.radians(bias_dps + rng.randn() * noise_deg * 10.0)
        u_law = kp * (phi_m - trim) + kd * dphi_m + kw * omega + ki * z + bias * 1e-3
        u = u_law
        if ff_mnm > 0.0:
            # friction feed-forward: the motor must supply the friction torque
            # itself, so the LQR does not have to buy it with wheel speed.
            # Sign follows the wheel with a hysteresis band (docs/46).
            if omega > ff_deadband_rps:
                ff_sgn = 1.0
            elif omega < -ff_deadband_rps:
                ff_sgn = -1.0
            u += ff_sgn * ff_mnm * 1e-3
        if dither_mnm > 0.0:
            u += (dither_mnm * 1e-3) * (1.0 if math.sin(2.0 * math.pi * dither_hz * k * dt) >= 0.0 else -1.0)
        if slew_mnm_step > 0.0:
            step = slew_mnm_step * 1e-3
            u = max(u_prev - step, min(u_prev + step, u))
        u_prev = u
        if abs(u) < TAU_MIN:
            u = 0.0
        u = max(-TAU_MAX, min(TAU_MAX, u))
        if u_max_mnm > 0.0:
            # firmware ceiling (BAL_MAX_MNM), below the physical torque limit
            um = u_max_mnm * 1e-3
            u = max(-um, min(um, u))
        delay_buf.append(u)
        u = delay_buf.pop(0) if len(delay_buf) > delay_steps else 0.0
        for _ in range(n_sub):
            cap = tau_limit(omega)
            tau = max(-cap, min(cap, u))
            ext = ext_mnm * 1e-3
            stick = (abs(omega) < OMEGA_STICK) and (abs(tau) <= TAU_FRIC)
            if stick:
                # static friction holds the wheel to the body: no relative motion,
                # bearing carries the motor torque, the body+wheel act as one rigid body
                fr = tau
                phi_dd = (MGH * math.sin(phi) - tau + fr + ext) / i_tot
                omega_dd = 0.0
            else:
                fr = TAU_FRIC * (1.0 if omega >= 0.0 else -1.0)
                phi_dd = (MGH * math.sin(phi) - tau + fr + ext) / i_tot
                omega_dd = (tau - fr) / J_W - phi_dd
            phi += dphi * h
            dphi += phi_dd * h
            omega = 0.0 if stick else omega + omega_dd * h
        rpm = omega * 60.0 / (2.0 * math.pi)
        if abs(rpm) >= ki_deadband_rpm:
            # brake wind-up: while the wheel turns forward we must LOWER the
            # command, so the integral accumulates -omega (see docs/46)
            z -= omega * dt
        if trim_u_k > 0.0:
            # Lean on the disturbance instead of fighting it: a constant external
            # torque shows up as a constant command, so move the zero until the
            # command (and therefore the wheel speed it buys) goes away.
            u_lp = u_lp + (u_law * 1e3 - u_lp) * dt / trim_u_tau_s
            if abs(math.degrees(phi)) < trim_gate_deg:
                # sign: a positive command means the loop is holding the body
                # against a forward push, so the zero must move BACKWARD to let
                # gravity carry that load (mgh*sin(trim) = -ext - tau_f).
                trim += math.radians(-trim_u_k * u_lp) * dt
                lim = math.radians(trim_clamp_deg)
                trim = max(-lim, min(lim, trim))
        if trim_rate_dps > 0.0 and abs(math.degrees(phi)) < trim_gate_deg and abs(rpm) > trim_db_rpm:
            # Slow zero trim used by S6b (docs/46): while the wheel keeps turning
            # one way the zero has to move, otherwise the LQR pays for the cable
            # disturbance with wheel speed until the 1700 rpm trip. Rate limited
            # in deg/s, gated on a small body angle, clamped, so it cannot push
            # the body over during a capture.
            trim += math.radians(trim_rate_dps) * (1.0 if omega > 0.0 else -1.0) * dt
            lim = math.radians(trim_clamp_deg)
            trim = max(-lim, min(lim, trim))
        elif trim_w_k > 0.0 and abs(math.degrees(phi)) < trim_gate_deg:
            # wheel-speed trim, the physically signed version: trim' = -k*omega
            # (rad/s per rad/s).  Because dtau/dtrim is positive (about
            # +0.93 mN.m/deg near upright), a positive wheel speed can only be
            # killed by LOWERING the trim.  trim_k above is the sign-flipped
            # experiment and is unstable on purpose.
            trim -= trim_w_k * omega * dt
            lim = math.radians(trim_clamp_deg)
            trim = max(-lim, min(lim, trim))
        elif trim_k > 0.0 and abs(math.degrees(phi)) < trim_gate_deg:
            # angle-zero adaptation: raise the trim while the wheel turns
            # forward so the command drops and the wheel slows down.
            # Gated on the body angle: with a large tilt the wheel-speed loop
            # must stay out of the way (it would otherwise wind up to the clamp
            # during the capture transient, see docs/46).
            trim += trim_k * omega * dt
            lim = math.radians(trim_clamp_deg)
            trim = max(-lim, min(lim, trim))
        if bias_k_mnm_per_rps > 0.0 and abs(math.degrees(phi)) < bias_gate_deg:
            # wheel-speed loop -> torque bias, much slower than the balance law
            # and clamped to roughly the measured friction, so a wind-up cannot
            # turn into a body push (docs/46)
            bias += -bias_k_mnm_per_rps * rpm * dt
            bias = max(-bias_clamp_mnm, min(bias_clamp_mnm, bias))
        z = max(-z_max, min(z_max, z))
        hist.append((k * dt, math.degrees(phi), dphi,
                     rpm, u * 1e3, z, math.degrees(trim), bias))
    return np.array(hist)


def report(tag, hist, limit_deg=0.3):
    t, phi = hist[:, 0], hist[:, 1]
    rpm, u = hist[:, 3], hist[:, 4]
    ok = abs(phi[-1]) < limit_deg and np.max(np.abs(phi)) < 30.0
    idx = np.where(np.abs(phi) > limit_deg)[0]
    settle = t[idx[-1]] if len(idx) else 0.0
    print("  %-26s %s  settle=%5.2fs  peak|phi|=%5.2f  peak|rpm|=%5.0f  "
          "peak|u|=%5.1f  end(phi=%+.2f, rpm=%+.0f)"
          % (tag, "RECOVER" if ok else "FALL   ", settle,
             np.max(np.abs(phi)), np.max(np.abs(rpm)), np.max(np.abs(u)),
             phi[-1], rpm[-1]))
    return ok


def main():
    print("plant: m=%.3f kg  h=%.3f m  mgh=%.4f N.m/rad  Jw=%.3e kg.m2"
          % (M_CUBE, H_COM, MGH, J_W))
    print("       I_solid=%.3e  I_hollow=%.3e  tau_max=%.1f mN.m  friction=%.2f mN.m"
          % (I_SOLID, I_HOLLOW, TAU_MAX * 1e3, TAU_FRIC * 1e3))
    print("       unstable pole: solid %.2f rad/s, hollow %.2f rad/s"
          % (math.sqrt(MGH / I_SOLID), math.sqrt(MGH / I_HOLLOW)))
    print("       static fall limit  asin(tau_max/mgh) = %.2f deg"
          % math.degrees(math.asin(TAU_MAX / MGH)))
    print("       command saturates at tau_max/kp        = %.2f deg"
          % math.degrees(TAU_MAX / KP))

    print("\nLQR reference (hollow inertia, u0 = torque budget per unit state):")
    for u0 in (3e-4, 9e-4, 2e-3):
        k = design_lqr(NOMINAL_I, u0)
        print("  u0=%5.1f mN.m -> kp=%.3f  kd=%.4f  kw=%+.2e"
              % (u0 * 1e3, k[0], k[1], k[2]))
    print("  chosen: kp=%.3f  kd=%.4f  kw=%+.2e  (u0 = 2 mN.m row)"
          % (KP, KD, KW))
    k = design_lqr(NOMINAL_I, 2e-3)
    same = (abs(k[0] - KP) < 5e-4) and (abs(k[1] - KD) < 5e-4) \
        and (abs(k[2] - KW) < 5e-7)
    print("  chosen gains match the LQR solve: %s"
          % ("yes" if same else "NO -- docs/44 is out of date"))

    print("\n1) capture range (release from rest, nominal plant)")
    for phi0 in (1.0, 2.0, 3.0, 4.0, 4.5, 5.0, 6.0):
        report("phi0 = %.1f deg" % phi0,
               simulate(KP, KD, KW, NOMINAL_I, phi0, t_end=15.0))

    print("\n2) plant uncertainty (release from 3 deg)")
    for tag, inertia in (("solid  (0.67 m a^2)", I_SOLID),
                         ("nominal (1.17 m a^2)", NOMINAL_I),
                         ("hollow x 1.5", I_HOLLOW * 1.5)):
        report(tag, simulate(KP, KD, KW, inertia, 3.0, t_end=15.0))

    print("\n3) sensor noise and gyro bias (release from 3 deg)")
    for n in (0.02, 0.05, 0.10):
        report("noise %.2f deg" % n,
               simulate(KP, KD, KW, NOMINAL_I, 3.0, t_end=15.0, noise_deg=n))

    print("\n4) loop delay (release from 3 deg)")
    for d in (0, 1, 2, 4):
        report("delay %2d ms" % (d * 5),
               simulate(KP, KD, KW, NOMINAL_I, 3.0, t_end=15.0, delay_steps=d))

    print("\n5) wheel already turning (release from 1 deg)")
    for rpm0 in (300, 600, 900):
        report("wheel %3d rpm" % rpm0,
               simulate(KP, KD, KW, NOMINAL_I, 1.0, t_end=15.0,
                        omega0=rpm0 * 2 * math.pi / 60.0))

    print("\n6) torque available vs wheel speed (12 V bus, modulation limit 5.7 V)")
    for rpm in (0, 500, 1000, 1250, 1500, 1700, 1900):
        w = rpm * 2 * math.pi / 60.0
        print("   %5d rpm -> %5.1f mN.m  (%.2f A)"
              % (rpm, tau_limit(w) * 1e3, tau_limit(w) / KT))


if __name__ == "__main__":
    main()
