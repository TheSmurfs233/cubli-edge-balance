/**
  ******************************************************************************
  * @file    bal_test.c
  * @brief   L1 offline test for User/Alg/alg_bal.c (S4 acceptance, 15 cases).
  *
  *          Compiles the real firmware source on the PC, no HAL, no board.
  *          Gains match docs/44 (kp 1.31383 N.m/rad, kd 0.175517 N.m/(rad/s),
  *          kw 3.819719e-4 N.m/(rad/s), limit 34.7 mN.m).
  *          Run through test/l1/run.sh.
  ******************************************************************************
  */

#include <stdio.h>
#include "alg_bal.h"

#define DEG2RAD    0.01745329252f
#define RPM2RPS    0.10471975512f   /* 2*pi/60 */
#define KP         1.31383f
#define KD         0.175517f
#define KW         3.819719e-4f
#define MAX_NM     0.0347f         /* 34.7 mN.m */
#define TOL_MNM    0.01f           /* 0.01 mN.m */

static int s_checks;
static int s_failed;

static void chk(int ok, const char *what)
{
  s_checks++;
  if (ok == 0)
  {
    s_failed++;
    printf("FAIL  %s\n", what);
  }
  else
  {
    printf("ok    %s\n", what);
  }
}

static float absf(float v)
{
  return (v < 0.0f) ? -v : v;
}

static int near_mnm(float tau_nm, float want_mnm, float tol_mnm)
{
  return (int)(absf((tau_nm * 1000.0f) - want_mnm) <= tol_mnm);
}

static alg_bal_cfg_t cfg_make(float sign, float max_nm)
{
  alg_bal_cfg_t c;

  c.kp_nm_rad = KP;
  c.kd_nm_rps = KD;
  c.kw_nm_rps = KW;
  c.sign      = sign;
  c.max_nm    = max_nm;
  c.ff_nm     = 0.0f;        /* friction feed-forward off by default */
  c.ff_deadband_rps = 0.1f;
  c.trim_deg = 0.0f;         /* slow zero adaptation off by default */
  c.trim_clamp_deg = 0.0f;
  c.trim_gate_deg = 0.0f;
  c.trim_w_k = 0.0f;
  c.walk_step_deg = 0.0f;      /* 0 = fall back to 0.01 deg per call */
  c.walk_deadband_rpm = 0.0f;  /* 0 = no deadband */
  c.walk_phi_gate_deg = 0.0f;  /* 0 = fall back to trim_gate_deg */

  return c;
}

int main(void)
{
  alg_bal_cfg_t cfg;
  alg_bal_out_t out;
  const alg_bal_out_t *last;
  int i;

  printf("bal_test: 21 cases (kp %.3f kd %.4f kw %.1e max %.4f)\n",
         KP, KD, KW, MAX_NM);

  /* 1. init with a valid set: ready, zero output, zero steps */
  cfg = cfg_make(1.0f, MAX_NM);
  alg_bal_init(&cfg);
  last = alg_bal_get();
  chk((alg_bal_ready() == 1u) && (last->steps == 0u) &&
      (near_mnm(last->tau_nm, 0.0f, TOL_MNM) != 0), "1 init loads gains and clears state");

  /* 2. pure attitude: +1 deg -> kp * rad = 22.9 mN.m */
  alg_bal_step(1.0f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((near_mnm(out.tau_nm, KP * 1000.0f * DEG2RAD, TOL_MNM) != 0) &&
      (out.saturated == 0u) && (out.deadband == 0u), "2 attitude term is kp * phi");

  /* 3. pure attitude, negative side */
  alg_bal_step(-1.0f * DEG2RAD, 0.0f, 0.0f, &out);
  chk(near_mnm(out.tau_nm, -(KP * 1000.0f * DEG2RAD), TOL_MNM) != 0,
      "3 attitude term keeps the sign");

  /* 4. pure rate: 0.1 rad/s -> 9.70 mN.m */
  alg_bal_step(0.0f, 0.1f, 0.0f, &out);
  chk(near_mnm(out.tau_nm, KD * 0.1f * 1000.0f, TOL_MNM) != 0,
      "4 rate term is kd * dphi");

  /* 5. wheel speed term: 500 rpm -> 2.09 mN.m */
  alg_bal_step(0.0f, 0.0f, 500.0f * RPM2RPS, &out);
  chk(near_mnm(out.tau_nm, KW * 500.0f * RPM2RPS * 1000.0f, TOL_MNM) != 0,
      "5 wheel term is kw * omega");

  /* 6. mounting sign flips everything */
  cfg = cfg_make(-1.0f, MAX_NM);
  alg_bal_init(&cfg);
  alg_bal_step(1.0f * DEG2RAD, 0.0f, 0.0f, &out);
  chk(near_mnm(out.tau_nm, -(KP * 1000.0f * DEG2RAD), TOL_MNM) != 0,
      "6 mounting sign flips the command");

  /* 7. saturation on the positive side: 3 deg needs 38.1 mN.m > 34.7 */
  cfg = cfg_make(1.0f, MAX_NM);
  alg_bal_init(&cfg);
  alg_bal_step(3.0f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((near_mnm(out.tau_nm, MAX_NM * 1000.0f, TOL_MNM) != 0) &&
      (out.saturated == 1u), "7 positive saturation clamps to the limit");

  /* 8. saturation on the negative side, raw value kept for the log */
  alg_bal_step(-3.0f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((near_mnm(out.tau_nm, -(MAX_NM * 1000.0f), TOL_MNM) != 0) &&
      (near_mnm(out.tau_raw_nm, -(KP * 3.0f * DEG2RAD * 1000.0f), TOL_MNM) != 0),
      "8 negative saturation clamps but keeps the raw value");

  /* 9. deadband: 0.02 deg needs 0.46 mN.m < 1.0 -> rounded to zero */
  alg_bal_step(0.02f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((near_mnm(out.tau_nm, 0.0f, TOL_MNM) != 0) && (out.deadband == 1u),
      "9 sub-millinewton commands are rounded to zero");

  /* 10. just below the limit is passed through, not flagged */
  cfg = cfg_make(1.0f, MAX_NM);
  alg_bal_init(&cfg);
  alg_bal_step(0.20f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((out.saturated == 0u) && (out.deadband == 0u) &&
      (near_mnm(out.tau_nm, 4.59f, 0.01f) != 0), "10 small but useful commands pass through");

  /* 11. steps counter counts every evaluation */
  alg_bal_reset();
  alg_bal_step(0.0f, 0.0f, 0.0f, 0);
  alg_bal_step(0.0f, 0.0f, 0.0f, 0);
  alg_bal_step(0.0f, 0.0f, 0.0f, &out);
  chk(out.steps == 3u, "11 step counter increments, NULL destination allowed");

  /* 12. reset clears the counter but keeps the tuning */
  alg_bal_reset();
  alg_bal_step(1.0f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((out.steps == 1u) && (near_mnm(out.tau_nm, 22.93f, TOL_MNM) != 0),
      "12 reset clears state but keeps the gains");

  /* 13. a rejected tuning set leaves the law disabled and silent */
  cfg = cfg_make(1.0f, 0.0f);           /* max_nm == 0 -> reject */
  alg_bal_init(&cfg);
  alg_bal_step(3.0f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((alg_bal_ready() == 0u) && (near_mnm(out.tau_nm, 0.0f, TOL_MNM) != 0),
      "13 invalid tuning set is rejected and produces no torque");

  /* 14. NULL destination still updates the internal copy (kept below the
        limit on purpose: 22.9 + 0.2 + 0.8 mN.m) */
  cfg = cfg_make(1.0f, MAX_NM);
  alg_bal_init(&cfg);
  alg_bal_step(1.0f * DEG2RAD, 0.02f, 20.0f * RPM2RPS, 0);
  last = alg_bal_get();
  chk((near_mnm(last->tau_nm, ((KP * 1.0f * DEG2RAD) + (KD * 0.02f) +
                 (KW * 20.0f * RPM2RPS)) * 1000.0f, TOL_MNM) != 0) &&
      (absf(last->phi_rad - (1.0f * DEG2RAD)) <= 1.0e-6f) &&
      (absf(last->omega_rps - (20.0f * RPM2RPS)) <= 1.0e-6f),
      "14 three terms add up and the inputs are echoed");

  /* 15. golden values computed by the python design model
        (sim/s4_design.py, same gains): catches a silent formula drift
        between the C law and the model that produced the gains. */
  cfg = cfg_make(1.0f, MAX_NM);
  alg_bal_init(&cfg);
  alg_bal_step(0.60f * DEG2RAD, 0.02f, 200.0f * RPM2RPS, &out);
  chk(near_mnm(out.tau_nm, 25.2687f, 0.002f) != 0, "15a golden 0.6 deg 0.02 radps 200 rpm");
  alg_bal_step(0.20f * DEG2RAD, -0.05f, -300.0f * RPM2RPS, &out);
  chk(near_mnm(out.tau_nm, -16.1897f, 0.002f) != 0, "15b golden 0.2 deg -0.05 radps -300 rpm");
  alg_bal_step(-0.90f * DEG2RAD, 0.10f, 400.0f * RPM2RPS, &out);
  chk(near_mnm(out.tau_nm, 12.9141f, 0.002f) != 0, "15c golden -0.9 deg 0.10 radps 400 rpm");

  /* 16. friction feed-forward (docs/18 T20). The motor must pay the Coulomb
        torque itself, otherwise the LQR can only buy it with wheel speed.
        The term follows the WHEEL sign, not the tilt, and is not multiplied
        by cfg.sign (it is a physical torque, not a control decision). */
  cfg = cfg_make(1.0f, MAX_NM);
  cfg.ff_nm = 0.00205f;                 /* 2.05 mN.m measured in S3 */
  cfg.ff_deadband_rps = 0.1f;
  alg_bal_init(&cfg);
  alg_bal_step(0.0f, 0.0f, 100.0f * RPM2RPS, &out);
  chk((near_mnm(out.tau_ff_nm, 2.05f, TOL_MNM) != 0) &&
      (near_mnm(out.tau_nm, (KW * 100.0f * RPM2RPS * 1000.0f) + 2.05f, 0.02f) != 0),
      "16a ff is +tau_f when the wheel spins forward");
  alg_bal_step(0.0f, 0.0f, -100.0f * RPM2RPS, &out);
  chk((near_mnm(out.tau_ff_nm, -2.05f, TOL_MNM) != 0) &&
      (near_mnm(out.tau_nm, -(KW * 100.0f * RPM2RPS * 1000.0f) - 2.05f, 0.02f) != 0),
      "16b ff is -tau_f when the wheel spins backward");
  alg_bal_step(0.0f, 0.0f, 0.01f * RPM2RPS, &out);   /* inside the band */
  chk(near_mnm(out.tau_ff_nm, -2.05f, TOL_MNM) != 0,
      "16c ff keeps its last sign inside the deadband (no chatter)");
  cfg = cfg_make(-1.0f, MAX_NM);        /* flipped mounting sign */
  cfg.ff_nm = 0.00205f;
  cfg.ff_deadband_rps = 0.1f;
  alg_bal_init(&cfg);
  alg_bal_step(0.0f, 0.0f, 100.0f * RPM2RPS, &out);
  chk(near_mnm(out.tau_ff_nm, 2.05f, TOL_MNM) != 0,
      "16d ff is not flipped by cfg.sign");

  /* 17. slow zero adaptation (docs/45 4.17). Gravity pins the body at the true
        balance point, so the wheel speed is the zero error: omega_ss =
        (kp/kw)*(trim - phi_ss). The trim is walked against the wheel, and the
        walk stops by itself when the wheel stops. */
  cfg = cfg_make(1.0f, MAX_NM);
  cfg.trim_clamp_deg = 2.0f;
  cfg.trim_gate_deg  = 4.0f;
  cfg.trim_w_k       = 0.0f;            /* frozen */
  alg_bal_init(&cfg);
  for (i = 0; i < 200; i++)             /* 1 s at +500 rpm */
  {
    alg_bal_step(0.2f * DEG2RAD, 0.0f, 500.0f * RPM2RPS, &out);
    alg_bal_adapt(0.005f);
  }
  chk(absf(alg_bal_trim_deg()) < 1e-6f,
      "17a k = 0 leaves the trim alone even with the wheel turning");

  cfg.trim_w_k = 3.0e-4f;               /* the value in board_cfg.h */
  alg_bal_init(&cfg);
  for (i = 0; i < 200; i++)             /* 1 s at +500 rpm -> -k*500*1 = -0.15 */
  {
    alg_bal_step(0.2f * DEG2RAD, 0.0f, 500.0f * RPM2RPS, &out);
    alg_bal_adapt(0.005f);
  }
  chk((alg_bal_trim_deg() < -0.13f) && (alg_bal_trim_deg() > -0.17f),
      "17b a forward wheel walks the trim backward at k*wheel");

  alg_bal_init(&cfg);
  for (i = 0; i < 200; i++)             /* the mirror case */
  {
    alg_bal_step(-0.2f * DEG2RAD, 0.0f, -500.0f * RPM2RPS, &out);
    alg_bal_adapt(0.005f);
  }
  chk((alg_bal_trim_deg() > 0.13f) && (alg_bal_trim_deg() < 0.17f),
      "17c a backward wheel walks the trim forward");

  /* 18. a saturated law must stop the walk: bench 2026-11-06 showed the walk
   *     ramming the clamp while the torque sat at the ceiling, and the body
   *     falling right after.  Now the walk freezes as soon as the law saturates,
   *     so a wheel that never stops cannot drag the reference to the clamp. */
  alg_bal_init(&cfg);
  for (i = 0; i < 20000; i++)
  {
    alg_bal_step(0.0f, 0.0f, 900.0f * RPM2RPS, &out);
    alg_bal_adapt(0.005f);
  }
  chk((out.saturated != 0u) && (alg_bal_trim_deg() > -0.30f),
      "18 a saturated law freezes the walk short of the clamp");

  /* 18b. and the rate cap keeps one bogus reading from moving it fast: a
   *      single call may not move the reference by more than 0.01 deg. */
  {
    float before;
    alg_bal_init(&cfg);
    (void)alg_bal_step(0.0f, 0.0f, 200.0f * RPM2RPS, &out);
    before = alg_bal_trim_deg();
    alg_bal_adapt(1.0f);                  /* absurd dt, same as a frozen speed */
    chk((before - alg_bal_trim_deg()) < 0.011f,
        "18b the walk rate is capped at 0.01 deg per call");
  }

  /* 19. gate: outside trim_gate_deg the body is still being captured */
  cfg.trim_w_k = 3.0e-4f;
  alg_bal_init(&cfg);
  for (i = 0; i < 200; i++)
  {
    alg_bal_step(6.0f * DEG2RAD, 0.0f, 500.0f * RPM2RPS, &out);  /* > 4 deg */
    alg_bal_adapt(0.005f);
  }
  chk(absf(alg_bal_trim_deg()) < 1e-6f, "19 no walk outside the gate");

  /* 20. manual trim: clamped, reported, and used by the next step */
  cfg.trim_w_k = 0.0f;
  alg_bal_init(&cfg);
  chk(absf(alg_bal_trim_set(9.0f) - 2.0f) < 1e-3f,
      "20a manual trim is clamped to +2 deg");
  chk(absf(alg_bal_trim_set(-9.0f) + 2.0f) < 1e-3f,
      "20b manual trim is clamped to -2 deg");
  alg_bal_trim_set(0.5f);
  alg_bal_step(0.0f, 0.0f, 0.0f, &out);
  chk((absf(alg_bal_trim_deg() - 0.5f) < 1e-4f) &&
      (near_mnm(out.tau_nm, -KP * 0.5f * DEG2RAD * 1000.0f, 0.05f) != 0),
      "20c a manual trim offsets the law by kp * trim");

  /* 21. the gate looks at the loop error (phi - trim), not at the raw tilt:
   *     a body parked at a cable-induced offset has a large |phi| but a tiny
   *     error, and that is the case the walk exists to remove.  Bench
   *     2026-11-11: the absolute form froze the walk at |phi| = 2.6 deg with the
   *     wheel parked at 200 rpm and the trim sitting on its clamp. */
  cfg = cfg_make(1.0f, 1.0f);           /* 1 N.m: never saturates here */
  cfg.trim_clamp_deg = 3.0f;
  cfg.trim_gate_deg = 4.0f;
  cfg.walk_phi_gate_deg = 2.0f;
  cfg.trim_w_k = 2.0e-4f;
  cfg.walk_step_deg = 0.05f;
  cfg.walk_deadband_rpm = 40.0f;

  alg_bal_init(&cfg);
  (void)alg_bal_trim_set(-2.5f);
  for (i = 0; i < 200; i++)             /* phi == trim: parked on the offset */
  {
    alg_bal_step(-2.5f * DEG2RAD, 0.0f, 200.0f * RPM2RPS, &out);
    alg_bal_adapt(0.5f);
  }
  chk(alg_bal_trim_deg() < -2.7f,
      "21a a parked offset (phi == trim) still lets the walk move");

  alg_bal_init(&cfg);
  for (i = 0; i < 200; i++)             /* phi - trim = 6 deg: still capturing */
  {
    alg_bal_step(6.0f * DEG2RAD, 0.0f, 200.0f * RPM2RPS, &out);
    alg_bal_adapt(0.5f);
  }
  chk(absf(alg_bal_trim_deg()) < 1e-6f,
      "21b a large loop error still freezes the walk");

  printf("checks: %d  failed: %d\n", s_checks, s_failed);
  printf("RESULT: %s\n", (s_failed == 0) ? "PASS" : "FAIL");

  return (s_failed == 0) ? 0 : 1;
}
