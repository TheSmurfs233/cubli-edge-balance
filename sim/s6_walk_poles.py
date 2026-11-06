"""Eigenvalues of the closed loop with the wheel walk added as a 4th state.

The walk is an integrator on the wheel speed that moves the reference angle:

    trim' = -k_d * omega_rpm        (k_d in deg per rpm per second, firmware units)
    u     = kp*(phi - trim) + kd*phi' + kw*omega

so the state vector is [phi, phi', omega, trim].  This script prints the poles
for a range of k_d, in the same units the firmware uses, and reports the damping
ratio of the slow pair, which is what "slow oscillation that grows" looks like.

Run:  python sim/s6_walk_poles.py
"""
import numpy as np

MGH = 0.459 * 9.81 * 0.073      # 0.3287 N.m/rad
I_BODY = 3.310e-3
J_W = 5.12e-5
KP = 1.31383 * 0.7
KD = 0.175517 * 0.7
KW = 3.819719e-4 * 0.7

RAD2DEG = 180.0 / np.pi
RPM_PER_RPS = 60.0 / (2.0 * np.pi)


def poles(k_deg):
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [MGH / I_BODY, 0.0, 0.0, 0.0],
        [-MGH / I_BODY, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ])
    B = np.array([[0.0], [-1.0 / I_BODY], [1.0 / J_W + 1.0 / I_BODY], [0.0]])
    K = np.array([[KP, KD, KW, -KP]])      # u = kp*(phi - trim) + kd*phi' + kw*w
    Acl = A + B @ K
    # trim' = -k_d [deg/(rpm*s)] * omega_rpm  ->  rad/s per rad/s
    k_rad = k_deg * (np.pi / 180.0) * RPM_PER_RPS
    Acl[3, 2] = -k_rad
    return np.linalg.eigvals(Acl)


def report(k_deg):
    w = poles(k_deg)
    out = []
    for z in sorted(w, key=lambda c: c.real):
        re, im = z.real, z.imag
        if abs(im) < 1e-9:
            out.append("%+.3f" % re)
        else:
            wn = abs(z)
            zeta = -re / wn
            out.append("%+.3f%+.3fj (wn %.3f, zeta %.3f)" % (re, im, wn, zeta))
    print("k = %-8.1e  %s" % (k_deg, "  |  ".join(out)))


print("firmware gains (0.7x):  kp %.4f  kd %.4f  kw %.4e" % (KP, KD, KW))
print()
for k in (0.0, 5e-5, 1e-4, 2e-4, 3e-4, 6e-4, 1e-3, 2e-3):
    report(k)
print()
print("'slow oscillation that grows' == a pole pair crossing into the right half")
print("plane, or a pair whose zeta is so small that friction/deadband finishes it.")
