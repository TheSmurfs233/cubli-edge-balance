/**
  ******************************************************************************
  * @file    bal_test.c
  * @brief   L1 offline test for User/Alg/alg_bal.c (S4 acceptance, 15 cases).
  *
  *          Compiles the real firmware source on the PC, no HAL, no board.
  *          Gains match docs/44 (kp 0.728 N.m/rad, kd 0.0970 N.m/(rad/s),
  *          kw 4.0e-5 N.m/(rad/s), limit 34.7 mN.m).
  *          Run through test/l1/run.sh.
  ******************************************************************************
  */

#include <stdio.h>
#include "alg_bal.h"

#define DEG2RAD    0.01745329252f
#define RPM2RPS    0.10471975512f   /* 2*pi/60 */
#define KP         0.728f
#define KD         0.0970f
#define KW         4.0e-5f
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

  return c;
}

int main(void)
{
  alg_bal_cfg_t cfg;
  alg_bal_out_t out;
  const alg_bal_out_t *last;

  printf("bal_test: 15 cases (kp %.3f kd %.4f kw %.1e max %.4f)\n",
         KP, KD, KW, MAX_NM);

  /* 1. init with a valid set: ready, zero output, zero steps */
  cfg = cfg_make(1.0f, MAX_NM);
  alg_bal_init(&cfg);
  last = alg_bal_get();
  chk((alg_bal_ready() == 1u) && (last->steps == 0u) &&
      (near_mnm(last->tau_nm, 0.0f, TOL_MNM) != 0), "1 init loads gains and clears state");

  /* 2. pure attitude: +1 deg -> kp * rad = 12.71 mN.m */
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

  /* 9. deadband: 0.05 deg needs 0.64 mN.m < 1.0 -> rounded to zero */
  alg_bal_step(0.05f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((near_mnm(out.tau_nm, 0.0f, TOL_MNM) != 0) && (out.deadband == 1u),
      "9 sub-millinewton commands are rounded to zero");

  /* 10. just below the limit is passed through, not flagged */
  cfg = cfg_make(1.0f, MAX_NM);
  alg_bal_init(&cfg);
  alg_bal_step(0.20f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((out.saturated == 0u) && (out.deadband == 0u) &&
      (near_mnm(out.tau_nm, 2.54f, 0.01f) != 0), "10 small but useful commands pass through");

  /* 11. steps counter counts every evaluation */
  alg_bal_reset();
  alg_bal_step(0.0f, 0.0f, 0.0f, 0);
  alg_bal_step(0.0f, 0.0f, 0.0f, 0);
  alg_bal_step(0.0f, 0.0f, 0.0f, &out);
  chk(out.steps == 3u, "11 step counter increments, NULL destination allowed");

  /* 12. reset clears the counter but keeps the tuning */
  alg_bal_reset();
  alg_bal_step(1.0f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((out.steps == 1u) && (near_mnm(out.tau_nm, 12.71f, TOL_MNM) != 0),
      "12 reset clears state but keeps the gains");

  /* 13. a rejected tuning set leaves the law disabled and silent */
  cfg = cfg_make(1.0f, 0.0f);           /* max_nm == 0 -> reject */
  alg_bal_init(&cfg);
  alg_bal_step(3.0f * DEG2RAD, 0.0f, 0.0f, &out);
  chk((alg_bal_ready() == 0u) && (near_mnm(out.tau_nm, 0.0f, TOL_MNM) != 0),
      "13 invalid tuning set is rejected and produces no torque");

  /* 14. NULL destination still updates the internal copy (kept below the
        limit on purpose: 25.4 + 1.9 + 0.1 mN.m) */
  cfg = cfg_make(1.0f, MAX_NM);
  alg_bal_init(&cfg);
  alg_bal_step(2.0f * DEG2RAD, 0.02f, 20.0f * RPM2RPS, 0);
  last = alg_bal_get();
  chk((near_mnm(last->tau_nm, ((KP * 2.0f * DEG2RAD) + (KD * 0.02f) +
                 (KW * 20.0f * RPM2RPS)) * 1000.0f, TOL_MNM) != 0) &&
      (absf(last->phi_rad - (2.0f * DEG2RAD)) <= 1.0e-6f) &&
      (absf(last->omega_rps - (20.0f * RPM2RPS)) <= 1.0e-6f),
      "14 three terms add up and the inputs are echoed");

  /* 15. golden values computed by the python design model
        (sim/s4_design.py, same gains): catches a silent formula drift
        between the C law and the model that produced the gains. */
  cfg = cfg_make(1.0f, MAX_NM);
  alg_bal_init(&cfg);
  alg_bal_step(1.50f * DEG2RAD, 0.05f, 300.0f * RPM2RPS, &out);
  chk(near_mnm(out.tau_nm, 25.1656f, 0.002f) != 0, "15a golden 1.5 deg 0.05 radps 300 rpm");
  alg_bal_step(0.80f * DEG2RAD, -0.12f, -450.0f * RPM2RPS, &out);
  chk(near_mnm(out.tau_nm, -3.3602f, 0.002f) != 0, "15b golden 0.8 deg -0.12 radps -450 rpm");
  alg_bal_step(-2.20f * DEG2RAD, 0.03f, 120.0f * RPM2RPS, &out);
  chk(near_mnm(out.tau_nm, -24.5405f, 0.002f) != 0, "15c golden -2.2 deg 0.03 radps 120 rpm");

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

  printf("checks: %d  failed: %d\n", s_checks, s_failed);
  printf("RESULT: %s\n", (s_failed == 0) ? "PASS" : "FAIL");

  return (s_failed == 0) ? 0 : 1;
}
