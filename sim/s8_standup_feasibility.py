#!/usr/bin/env python3
# -*- coding: ascii -*-
"""S8 feasibility study: can this cube stand itself up from a flat face?

Question asked on 2026-11-12: "can we do automatic stand-up and balance?"
This script answers it with the same plant the balance loop was designed on,
before any firmware is written (workflow rule: sim first, board second).

Model
-----
Rotation about the bottom edge, reaction wheel on the same axis:

    I_tot * phi_dd = mgh * sin(phi) - tau + tau_f
    J_w  *(phi_dd + omega_dd) = tau - tau_f

    phi = 0 at the balance point (standing on that edge)
    phi = -45 deg is the cube lying flat on its face
    tau > 0 spools the wheel up, the reaction on the body is -tau

While the cube lies flat the ground absorbs any torque below the static
tipping torque

    tau_tip = mgh * sin(45 deg) = m*g*(a/2) = 232 mN.m

so the cube only starts to rotate once the motor can beat that number.
Two independent budgets decide the question, and BOTH have to pass:

  torque    any lifting torque must exceed tau_tip for a finite time
  momentum  a wheel can only hand the body J_w*omega of angular momentum,
            and reaching the barrier takes I_body*omega_barrier

Run:  .tmp_env\\venv\\Scripts\\python.exe sim/s8_standup_feasibility.py
"""

import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ------------------------------------------------------------------ constants
G = 9.81
A_CUBE = 0.104                 # edge [m]
M_CUBE = 0.301 + 0.158         # incl. battery [kg]
H_COM = 0.073                  # COM above the pivot edge at balance [m]
J_W = 5.12e-5                  # wheel + rotor, measured in S3 [kg.m2]
KT = 0.02891                   # N.m/A
R_PHASE = 2.3                  # ohm
V_LIM = 5.7                    # V left after the modulation limit
V_BUS = 14.0                   # 4S pack
I_MAX = 1.20                   # A, firmware reference limit
TAU_HW = KT * I_MAX            # 34.7 mN.m, the drive's ceiling
TAU_BAL = 28.0e-3              # BAL_MAX_MNM, what the balance loop may use
TAU_FRIC = 2.05e-3             # measured in S3
OMEGA_TRIP = 178.0             # rad/s, 1700 rpm trip

MGH = M_CUBE * G * H_COM       # 0.3287 N.m
PHI_FLAT = -math.pi / 4.0      # cube on its face
TAU_TIP = MGH * math.sin(-PHI_FLAT)      # 0.2324 N.m
D_ENERGY = M_CUBE * G * (A_CUBE / math.sqrt(2.0) - A_CUBE / 2.0)   # barrier [J]

# body inertia about the bottom edge, two bounds: all mass at the surface
# (hollow shell) and uniform solid cube.  The real cube is in between.
I_HOLLOW = (7.0 / 6.0) * M_CUBE * A_CUBE ** 2
I_SOLID = (2.0 / 3.0) * M_CUBE * A_CUBE ** 2
I_EDGE = I_HOLLOW              # conservative for the tipping effort

# balance loop as compiled into v0.10-speed-zero (docs/01 2.6)
KP, KD, KW = 1.313830, 0.175517, 3.819719e-04


def tau_limit(omega, i_max=I_MAX, v_lim=V_LIM):
    """Wheel torque still available at a given wheel speed (back-EMF)."""
    i_cap = (v_lim - KT * abs(omega)) / R_PHASE
    if i_cap > i_max:
        i_cap = i_max
    if i_cap < 0.0:
        i_cap = 0.0
    return KT * i_cap


def hold_band(tau):
    """|phi| below which a torque tau can still hold the cube."""
    s = tau / MGH
    if s >= 1.0:
        return math.pi / 2.0
    return math.asin(s)


def simulate_release(phi0_deg, tau_cap=TAU_BAL, t_end=8.0, dt=2e-4, law=True):
    """Release the cube at phi0 (0 = balance) and let the loop try to hold it."""
    phi = math.radians(phi0_deg)
    phi_d = 0.0
    omega = 0.0
    t = np.arange(0.0, t_end, dt)
    ph = np.zeros_like(t)
    om = np.zeros_like(t)

    for k, _tk in enumerate(t):
        u = (KP * phi + KD * phi_d + KW * omega) if law else 0.0
        cap = min(tau_cap, tau_limit(omega))
        u = max(-cap, min(cap, u))
        if abs(u) < 1.0e-3:            # firmware rounds two-digit mN.m
            u = 0.0
        tau = u

        phi_dd = (MGH * math.sin(phi) - tau
                  + TAU_FRIC * np.sign(omega)) / I_EDGE
        omega_d = (tau - TAU_FRIC * np.sign(omega)) / J_W - phi_dd
        phi_d += phi_dd * dt
        phi += phi_d * dt
        omega += omega_d * dt
        if phi < PHI_FLAT:             # hit the face, inelastic stop
            phi = PHI_FLAT
            phi_d = 0.0
        ph[k], om[k] = phi, omega

    return {"t": t, "phi": ph, "omega": om,
            "phi_max_deg": math.degrees(ph.max()),
            "phi_end_deg": math.degrees(ph[-1]),
            "fell": ph[-1] < math.radians(-30.0)}


def simulate_flat_attempt(tau_cap=TAU_BAL, t_end=4.0, dt=1e-4):
    """Cube lying flat: spool the wheel up for 2 s, then brake hard for 2 s.

    While flat the ground takes the reaction as long as the motor cannot beat
    TAU_TIP, so phi stays at PHI_FLAT and only the wheel moves.  If the wheel
    ever reaches the brake phase with torque above TAU_TIP the body would
    start to rise - that is exactly what we are testing for.
    """
    phi = PHI_FLAT
    phi_d = 0.0
    omega = 0.0
    t = np.arange(0.0, t_end, dt)
    ph = np.zeros_like(t)
    om = np.zeros_like(t)
    tu = np.zeros_like(t)

    for k, tk in enumerate(t):
        u = +tau_cap if tk < 2.0 else (-tau_cap if tk < 4.0 else 0.0)
        cap = min(tau_cap, tau_limit(omega))
        u = max(-cap, min(cap, u))
        tau = u

        if phi <= PHI_FLAT + 1e-9 and abs(tau) <= TAU_TIP:
            phi = PHI_FLAT
            phi_d = 0.0
            omega_d = (tau - TAU_FRIC * np.sign(omega)) / J_W
        else:
            phi_dd = (MGH * math.sin(phi) - tau
                      + TAU_FRIC * np.sign(omega)) / I_EDGE
            omega_d = (tau - TAU_FRIC * np.sign(omega)) / J_W - phi_dd
            phi_d += phi_dd * dt
            phi += phi_d * dt
        omega += omega_d * dt
        if phi < PHI_FLAT:
            phi = PHI_FLAT
            phi_d = 0.0
        ph[k], om[k], tu[k] = phi, omega, tau

    return {"t": t, "phi": ph, "omega": om, "tau": tu,
            "lifted": ph.max() > PHI_FLAT + math.radians(0.5),
            "phi_max_deg": math.degrees(ph.max()),
            "omega_peak_rpm": float(np.max(np.abs(om))) * 60.0 / (2 * math.pi)}


def main():
    print("=" * 74)
    print("S8 stand-up feasibility")
    print("=" * 74)
    print("m = %.3f kg   a = %.1f mm   h_com = %.1f mm   J_w = %.2e kg.m2"
          % (M_CUBE, A_CUBE * 1e3, H_COM * 1e3, J_W))
    print("mgh = %.4f N.m   barrier energy = %.1f mJ   tipping torque = %.1f mN.m"
          % (MGH, D_ENERGY * 1e3, TAU_TIP * 1e3))
    print("I_edge: hollow %.3e, solid %.3e kg.m2   (using hollow)"
          % (I_HOLLOW, I_SOLID))
    print()

    # ---------------------------------------------------------- budget 1
    tau_stall_57 = KT * (V_LIM / R_PHASE)
    tau_stall_14 = KT * (V_BUS / R_PHASE)
    print("-- budget 1: torque needed to break contact at flat -------------")
    print("  needed                     %.1f mN.m" % (TAU_TIP * 1e3))
    print("  balance loop cap           %.1f mN.m  (%.2f x needed)"
          % (TAU_BAL * 1e3, TAU_BAL / TAU_TIP))
    print("  firmware current limit     %.1f mN.m  (%.2f x needed)"
          % (TAU_HW * 1e3, TAU_HW / TAU_TIP))
    print("  electrical stall, 5.7 V    %.1f mN.m  (%.2f x needed)"
          % (tau_stall_57 * 1e3, tau_stall_57 / TAU_TIP))
    print("  electrical stall, 14 V     %.1f mN.m  (%.2f x needed)"
          % (tau_stall_14 * 1e3, tau_stall_14 / TAU_TIP))
    print("  -> no voltage or current setting inside the winding's rating")
    print("     reaches %.0f mN.m; %0.1f A would be needed at Kt = %.5f"
          % (TAU_TIP * 1e3, TAU_TIP / KT, KT))
    print()

    # ---------------------------------------------------------- budget 2
    print("-- budget 2: momentum a wheel can hand over ---------------------")
    w_barrier = math.sqrt(2.0 * D_ENERGY / I_EDGE)
    l_needed = I_EDGE * w_barrier
    print("  angular velocity needed at the barrier %.2f rad/s (%.0f rpm body)"
          % (w_barrier, w_barrier * 60.0 / (2 * math.pi)))
    print("  momentum needed            %.5f kg.m2/s" % l_needed)
    for rpm in (1700.0, 2000.0, 2300.0):
        w = rpm * 2 * math.pi / 60.0
        have = J_W * w
        print("  wheel at %4.0f rpm        %.5f kg.m2/s  (%.2f x needed, %.1f mJ)"
              % (rpm, have, have / l_needed,
                 0.5 * J_W * w * w * 1e3))
    print("  -> energy is NOT the problem: the wheel holds %.0f mJ vs %.0f mJ"
          % (0.5 * J_W * OMEGA_TRIP ** 2 * 1e3, D_ENERGY * 1e3))
    print("     needed, but momentum is %.2f x short and, worse, the only way"
          % (l_needed / (J_W * OMEGA_TRIP)))
    print("     to extract it (motor torque) is %.1f x below the tipping torque"
          % (TAU_TIP / TAU_HW))
    print()

    # ------------------------------------------------- what the loop can hold
    band_bal = 45.0 - math.degrees(hold_band(TAU_BAL))
    band_hw = 45.0 - math.degrees(hold_band(TAU_HW))
    print("-- how far from flat the loop can hold --------------------------")
    print("  with 28 mN.m:   %.1f deg or more from flat" % band_bal)
    print("  with 34.7 mN.m: %.1f deg or more from flat" % band_hw)
    print()

    print("-- release sweep with the compiled firmware ---------------------")
    for rel in (40.0, 40.5, 41.0, 42.0, 43.0, 44.0):
        r = simulate_release(-45.0 + rel)
        print("  release at %4.1f deg: peak %+6.2f deg, end %+7.2f deg  %s"
              % (rel, r["phi_max_deg"], r["phi_end_deg"],
                 "FELL BACK" if r["fell"] else "held"))
    print()

    print("-- flat-face attempt with the real drive (spin up, then brake) --")
    r = simulate_flat_attempt(TAU_BAL)
    print("  28.0 mN.m: lifted = %-5s peak phi = %+6.2f deg, wheel peak %.0f rpm"
          % (r["lifted"], r["phi_max_deg"], r["omega_peak_rpm"]))
    r = simulate_flat_attempt(TAU_HW)
    print("  34.7 mN.m: lifted = %-5s peak phi = %+6.2f deg, wheel peak %.0f rpm"
          % (r["lifted"], r["phi_max_deg"], r["omega_peak_rpm"]))
    print()

    # ------------------------------------------------------------- figure
    fig, ax = plt.subplots(2, 2, figsize=(11.5, 7.6))

    # (a) torque map
    ph = np.linspace(-45.0, 0.0, 400)
    need = np.abs(MGH * np.sin(np.radians(ph)))
    ax[0, 0].plot(ph, need * 1e3, "k", lw=2, label="|gravity torque|")
    ax[0, 0].axhline(TAU_BAL * 1e3, color="tab:orange", ls="--",
                     label="balance cap 28 mN.m")
    ax[0, 0].axhline(TAU_HW * 1e3, color="tab:red", ls=":",
                     label="current limit 34.7 mN.m")
    ax[0, 0].axhline(tau_stall_14 * 1e3, color="tab:purple", ls="-.",
                     label="14 V stall 176 mN.m")
    ax[0, 0].axvline(-45.0 + band_bal, color="tab:blue", lw=1,
                     label="holdable from %.0f deg" % band_bal)
    ax[0, 0].set_xlabel("tilt from balance [deg]   (-45 = flat on a face)")
    ax[0, 0].set_ylabel("torque [mN.m]")
    ax[0, 0].set_title("(a) lifting needs 232 mN.m at flat, drive has 35")
    ax[0, 0].grid(alpha=0.3)
    ax[0, 0].legend(fontsize=7, loc="lower left")

    # (b) release sweep
    for rel, c in ((44.0, "tab:green"), (43.0, "tab:olive"),
                   (42.0, "tab:orange"), (41.0, "tab:red"),
                   (40.5, "tab:purple"), (40.0, "tab:brown")):
        r = simulate_release(-45.0 + rel)
        ax[0, 1].plot(r["t"], np.degrees(r["phi"]), color=c,
                      label="release at %.1f deg" % rel)
    ax[0, 1].axhline(0.0, color="k", lw=0.8)
    ax[0, 1].set_xlim(0, 6)
    ax[0, 1].set_ylim(-46, 6)
    ax[0, 1].set_xlabel("time [s]")
    ax[0, 1].set_ylabel("phi [deg]")
    ax[0, 1].set_title("(b) hand-off: catchable only inside ~4.9 deg")
    ax[0, 1].grid(alpha=0.3)
    ax[0, 1].legend(fontsize=7)

    # (c) flat-face attempt
    r = simulate_flat_attempt(TAU_BAL)
    ax[1, 0].plot(r["t"], np.degrees(r["phi"]), "tab:blue",
                  label="phi (stays at -45 deg)")
    ax[1, 0].plot(r["t"], r["tau"] * 1e3, "tab:orange", label="torque [mN.m]")
    ax[1, 0].plot(r["t"], r["omega"] * 60.0 / (2 * math.pi), "tab:green",
                  label="wheel rpm")
    ax[1, 0].set_xlabel("time [s]")
    ax[1, 0].set_title("(c) real drive: spin up then brake, nothing happens")
    ax[1, 0].grid(alpha=0.3)
    ax[1, 0].legend(fontsize=7)

    # (d) budgets as bars
    labels = ["torque\n[mN.m]", "momentum\n[kg.m2/s x100]", "energy\n[mJ]"]
    have = [TAU_HW * 1e3, l_needed * 0.0 + J_W * OMEGA_TRIP * 1e2, 0.5 * J_W * OMEGA_TRIP ** 2 * 1e3]
    need_v = [TAU_TIP * 1e3, l_needed * 1e2, D_ENERGY * 1e3]
    x = np.arange(len(labels))
    ax[1, 1].bar(x - 0.2, have, 0.38, label="have at 1700 rpm", color="tab:gray")
    ax[1, 1].bar(x + 0.2, need_v, 0.38, label="needed", color="tab:red")
    for i in range(len(labels)):
        ax[1, 1].text(x[i] - 0.2, have[i] * 1.02, "%.1f" % have[i],
                      ha="center", fontsize=8)
        ax[1, 1].text(x[i] + 0.2, need_v[i] * 1.02, "%.1f" % need_v[i],
                      ha="center", fontsize=8)
    ax[1, 1].set_xticks(x, labels)
    ax[1, 1].set_title("(d) torque and momentum fall short; energy is not")
    ax[1, 1].grid(alpha=0.3, axis="y")
    ax[1, 1].legend(fontsize=8)

    fig.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs",
                       "s8_standup_feasibility.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=120)
    print("figure -> %s" % out)


if __name__ == "__main__":
    main()
