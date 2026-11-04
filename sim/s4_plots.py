#!/usr/bin/env python3
# -*- coding: ascii -*-
"""Plots for the S4 design study.

Reuses the plant, gains and simulator from s4_design.py, so every curve here is
the same model that produced docs/44 6.1.  Writes PNGs to sim/figs/.

Run (Windows uv venv, matplotlib needed):
    .tmp_env\\venv\\Scripts\\python.exe sim\\s4_plots.py
"""

import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import s4_design as d

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs")
DPI = 130


def mkdir_out():
    if not os.path.isdir(OUT_DIR):
        os.makedirs(OUT_DIR)


def save(fig, name):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print("wrote %s" % path)
    return path


def style(ax, title, ylabel):
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("time [s]", fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=8)


def fig_capture():
    """Release from several tilt angles: angle and wheel speed."""
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.2, 6.4), sharex=True)
    cases = [1.0, 2.0, 3.0, 4.0, 4.5]
    colors = plt.cm.viridis(np.linspace(0.05, 0.85, len(cases)))
    for phi0, c in zip(cases, colors):
        h = d.simulate(d.KP, d.KD, d.KW, d.NOMINAL_I, phi0, t_end=6.0)
        a1.plot(h[:, 0], h[:, 1], color=c, lw=1.4, label="release %.1f deg" % phi0)
        a2.plot(h[:, 0], h[:, 3], color=c, lw=1.4, label="release %.1f deg" % phi0)
    a1.axhline(0.0, color="k", lw=0.8)
    a1.axhline(2.73, color="r", ls="--", lw=1.0)
    a1.text(0.15, 2.9, "command saturates (2.73 deg)", color="r", fontsize=8)
    style(a1, "Capture: body tilt recovers from rest (nominal plant)", "tilt phi [deg]")
    a1.legend(fontsize=7, loc="upper right")
    a2.axhline(0.0, color="k", lw=0.8)
    style(a2, "Wheel speed during the same runs", "wheel speed [rpm]")
    a2.legend(fontsize=7, loc="upper right")
    return save(fig, "s4_1_capture.png")


def fig_boundary():
    """Where it stops working: 4.5 deg recovers, 5 and 6 deg fall.

    The model has no ground, so a fallen cube just keeps swinging; the plot is
    clipped to the first 4 s to make the divergence visible.
    """
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    for phi0, ls in ((4.5, "-"), (5.0, "--"), (6.0, ":")):
        h = d.simulate(d.KP, d.KD, d.KW, d.NOMINAL_I, phi0, t_end=5.0)
        ax.plot(h[:, 0], h[:, 1], ls=ls, lw=1.6,
                label="release %.1f deg  (peak %.0f rpm)" % (phi0, np.max(np.abs(h[:, 3]))))
    ax.axhline(0.0, color="k", lw=0.8)
    ax.set_xlim(0.0, 4.0)
    ax.set_ylim(-3.0, 40.0)
    ax.annotate("no ground in the model:\npast the capture window it just swings",
                (2.6, 30.0), fontsize=8, color="dimgray")
    style(ax, "Capture boundary: 4.5 deg recovers, 5 deg and above fall", "tilt phi [deg]")
    ax.legend(fontsize=8, loc="upper left")
    return save(fig, "s4_2_boundary.png")


def fig_slow_mode():
    """The 20 s view: angle settles, wheel speed does not come back to zero."""
    h = d.simulate(d.KP, d.KD, d.KW, d.NOMINAL_I, 3.0, t_end=25.0)
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(7.2, 7.6), sharex=True)
    a1.plot(h[:, 0], h[:, 1], lw=1.4, color="#1f77b4")
    a1.axhline(0.0, color="k", lw=0.8)
    style(a1, "3 deg release, 25 s: angle recovers in ~0.4 s", "tilt phi [deg]")
    a2.plot(h[:, 0], h[:, 3], lw=1.4, color="#ff7f0e")
    a2.axhline(h[-1, 3], color="r", ls="--", lw=1.0)
    a2.text(1.0, h[-1, 3] + 40.0, "steady %.0f rpm (friction + gyro bias)" % h[-1, 3],
            color="r", fontsize=8)
    style(a2, "Wheel speed: no restoring force, so it stops wherever friction holds it", "wheel [rpm]")
    a3.plot(h[:, 0], h[:, 4], lw=1.2, color="#2ca02c")
    a3.axhline(0.0, color="k", lw=0.8)
    a3.axhline(d.TAU_FRIC * 1e3, color="gray", ls=":", lw=1.0)
    a3.text(3.0, d.TAU_FRIC * 1e3 + 0.6, "coulomb friction %.2f mN.m" % (d.TAU_FRIC * 1e3),
            color="gray", fontsize=8)
    style(a3, "Commanded torque (deadband + limit applied)", "torque [mN.m]")
    fig.tight_layout(h_pad=1.6)
    return save(fig, "s4_3_slow_mode.png")


def fig_robustness():
    """Delay, sensor noise and plant uncertainty on the same axes."""
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.6, 3.8))
    h = d.simulate(d.KP, d.KD, d.KW, d.NOMINAL_I, 3.0, t_end=4.0)
    a1.plot(h[:, 0], h[:, 1], lw=2.2, color="k", label="nominal")
    for tag, kw in (("delay 20 ms", dict(delay_steps=4)),
                    ("noise 0.10 deg", dict(noise_deg=0.10)),
                    ("J x1.5", dict())):
        if tag.startswith("J"):
            hh = d.simulate(d.KP, d.KD, d.KW, d.I_HOLLOW * 1.5, 3.0, t_end=4.0)
        else:
            hh = d.simulate(d.KP, d.KD, d.KW, d.NOMINAL_I, 3.0, t_end=4.0, **kw)
        a1.plot(hh[:, 0], hh[:, 1], lw=1.1, label=tag)
    a1.axhline(0.0, color="k", lw=0.6)
    style(a1, "Robustness: angle, 3 deg release", "tilt phi [deg]")
    a1.legend(fontsize=7)

    for tag, kw in (("delay 20 ms", dict(delay_steps=4)),
                    ("noise 0.10 deg", dict(noise_deg=0.10))):
        hh = d.simulate(d.KP, d.KD, d.KW, d.NOMINAL_I, 3.0, t_end=4.0, **kw)
        a2.plot(hh[:, 0], hh[:, 3], lw=1.1, label=tag)
    a2.plot(h[:, 0], h[:, 3], lw=2.2, color="k", label="nominal")
    style(a2, "Robustness: wheel speed, 3 deg release", "wheel [rpm]")
    a2.legend(fontsize=7)
    return save(fig, "s4_4_robustness.png")


def fig_torque_budget():
    """Available torque vs wheel speed, and where the balance loop operates."""
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    rpm = np.linspace(0.0, 2000.0, 400)
    tau = [d.tau_limit(r * 2 * math.pi / 60.0) * 1e3 for r in rpm]
    ax.plot(rpm, tau, lw=2.0, color="#1f77b4", label="available torque (12 V bus, 5.7 V limit)")
    ax.axhline(d.TAU_MAX * 1e3, color="r", ls="--", lw=1.0,
               label="current limit 34.7 mN.m (1.20 A)")
    for r, c in ((1000, "gray"), (1250, "orange"), (1500, "orangered"), (1700, "firebrick")):
        t = d.tau_limit(r * 2 * math.pi / 60.0) * 1e3
        ax.plot([r], [t], "o", color=c, ms=6)
        ax.annotate("%.1f mN.m" % t, (r, t), textcoords="offset points",
                    xytext=(6, 6), fontsize=8, color=c)
    ax.annotate("balance works here\n(steady wheel ~390 rpm)", (390.0, 33.5),
                textcoords="offset points", xytext=(10, -30), fontsize=8,
                arrowprops=dict(arrowstyle="->", lw=0.8))
    style(ax, "Torque budget vs wheel speed: back-EMF eats the command", "torque [mN.m]")
    ax.set_xlabel("wheel speed [rpm]", fontsize=9)
    ax.set_xlim(0.0, 2000.0)
    ax.set_ylim(0.0, 40.0)
    ax.legend(fontsize=7, loc="lower left")
    return save(fig, "s4_5_torque_budget.png")


def main():
    mkdir_out()
    print("plant: m=%.3f kg  h=%.3f m  mgh=%.4f N.m/rad  Jw=%.3e"
          % (d.M_CUBE, d.H_COM, d.MGH, d.J_W))
    for f in (fig_capture, fig_boundary, fig_slow_mode, fig_robustness,
              fig_torque_budget):
        f()


if __name__ == "__main__":
    main()
