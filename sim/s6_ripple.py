"""S6: does the current ripple fit under the over-current trip?

The trip compares the instantaneous phase peak against FOC_I_TRIP_A with a
5-frame debounce, so the useful headroom is the commanded iq minus half the
switching ripple, not iq alone. At 12 V the margin was ~0.05 A; a 4S pack
raises the ripple by 17 % and the law started tripping at full torque.

Run:  .tmp_env/venv/Scripts/python.exe sim/s6_ripple.py
"""

FSW_HZ = 20000.0
L_H = 0.5e-3            # phase inductance, motor_cfg.h MOTOR_L_UH 500
KT = 0.02891            # N.m/A
I_TRIP = 1.28           # board_cfg.h FOC_I_TRIP_A
I_REF_MAX = 1.20        # board_cfg.h FOC_I_MAX_A
V_BUSES = (12.0, 14.03, 16.8)
CEILINGS_MNM = (33.0, 30.0, 28.0, 26.0)


def ripple_pp(vbus, duty=0.5):
    """Peak-to-peak inductor current at the worst-case duty cycle."""
    return vbus * duty * (1.0 - duty) / (FSW_HZ * L_H)


def main():
    print("ripple = Vbus*D*(1-D)/(fsw*L), worst case D=0.5, L=0.5mH, fsw=20kHz")
    print()
    for vbus in V_BUSES:
        r = ripple_pp(vbus)
        print("Vbus %5.2f V -> ripple %.3f A pp (%.3f A peak)" % (vbus, r, 0.5 * r))
    print()
    print("head room left for the command: iq_max = trip - ripple/2")
    head = ("bus  " + "".join("%9.2fV" % v for v in V_BUSES))
    print(head)
    for vbus in V_BUSES:
        row = "     "
        for _ in (0,):
            row += "%10.3f" % (I_TRIP - 0.5 * ripple_pp(vbus))
        print("head " + row)
    print()
    print("ceiling  iq_cmd  " + "".join("%12s" % ("%.2fV" % v) for v in V_BUSES))
    for mnm in CEILINGS_MNM:
        iq = mnm / (1000.0 * KT)
        cells = ""
        for vbus in V_BUSES:
            peak = iq + 0.5 * ripple_pp(vbus)
            cells += "%9.3f %s" % (peak, "OK " if peak < I_TRIP else "TRIP")
        flag = "" if iq <= I_REF_MAX else "  (above FOC_I_MAX_A)"
        print("%5.0f mN.m %6.3f A %s%s" % (mnm, iq, cells, flag))
    print()
    print("target: peak < %.2f A with at least 0.10 A margin at 16.8 V (full 4S)." % I_TRIP)
    ok = all((m / (1000.0 * KT)) + 0.5 * ripple_pp(16.8) < I_TRIP - 0.10
             for m in (28.0,))
    print("ceiling 28 mN.m at 16.8 V leaves %.3f A margin -> %s"
          % (I_TRIP - ((28.0 / (1000.0 * KT)) + 0.5 * ripple_pp(16.8)),
             "PASS" if ok else "FAIL"))


if __name__ == "__main__":
    main()
