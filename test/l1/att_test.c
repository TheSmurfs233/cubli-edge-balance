/**
  ******************************************************************************
  * @file    att_test.c
  * @brief   L1 offline test for User/Mid/mid_att.c (S2 acceptance, 12 cases).
  *
  *          Compiles the real firmware source on the PC, no HAL, no board.
  *          Run through test/l1/run.sh.
  ******************************************************************************
  */

#include <stdio.h>
#include <math.h>
#include "mid_att.h"

#define DEG2RAD   0.01745329252f
#define TOL       1.0e-3f          /* deg */
#define NOMINAL_TS (1000)          /* us, first frame has no reference */

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

static int near(float got, float want, float tol)
{
  float d = got - want;
  if (d < 0.0f)
  {
    d = -d;
  }
  return (int)(d <= tol);
}

static att_config_t cfg_make(uint8_t axis, float sign, float offset, float lp_hz,
                             uint16_t timeout_ms, float range_deg)
{
  att_config_t c;

  c.axis       = axis;
  c.sign       = sign;
  c.offset_deg = offset;
  c.lp_hz      = lp_hz;
  c.timeout_ms = timeout_ms;
  c.range_deg  = range_deg;

  return c;
}

/* Build one 0x41 payload sample: pitch in degrees, rate in deg/s. */
static void ahrs_set(fdilink_ahrs_t *a, float pitch_deg, float rate_dps, int64_t ts_us)
{
  a->roll_speed  = 0.0f;
  a->pitch_speed = rate_dps * DEG2RAD;
  a->heading_speed = 0.0f;
  a->roll        = 0.0f;
  a->pitch       = pitch_deg * DEG2RAD;
  a->heading     = 0.0f;
  a->q[0] = 1.0f;
  a->q[1] = 0.0f;
  a->q[2] = 0.0f;
  a->q[3] = 0.0f;
  a->timestamp_us = ts_us;
  a->seq = 0u;
}

int main(void)
{
  att_config_t cfg;
  att_state_t  st;
  fdilink_ahrs_t a;
  int          i;

  printf("att_test: 12 cases\n");

  /* 1. init: everything zeroed, nothing valid yet */
  cfg = cfg_make(ATT_AXIS_PITCH, 1.0f, 0.0f, 0.0f, 100u, 60.0f);
  att_init(&cfg);
  st = *att_get();
  chk((near(st.theta_deg, 0.0f, TOL) != 0) && (near(st.theta_dot_dps, 0.0f, TOL) != 0) &&
      (st.valid == 0u) && (st.seq == 0u), "1 init zeros state");

  /* 2. positive slope: 10 deg raw, zero 2 deg -> 8 deg body pitch */
  cfg = cfg_make(ATT_AXIS_PITCH, 1.0f, 2.0f, 0.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 10.0f, 0.0f, NOMINAL_TS);
  att_update(&a, 5u, 0.0f);
  st = *att_get();
  chk((near(st.theta_deg, 8.0f, TOL) != 0) && (st.valid == 1u), "2 offset + positive sign");

  /* 3. negative sign flips the direction */
  cfg = cfg_make(ATT_AXIS_PITCH, -1.0f, 0.0f, 0.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 10.0f, 0.0f, NOMINAL_TS);
  att_update(&a, 5u, 0.0f);
  chk(near(att_get()->theta_deg, -10.0f, TOL) != 0, "3 negative sign");

  /* 4. rate sign follows the same sign as the angle */
  cfg = cfg_make(ATT_AXIS_PITCH, -1.0f, 0.0f, 0.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 0.0f, 25.0f, NOMINAL_TS);
  att_update(&a, 5u, 0.0f);
  chk(near(att_get()->theta_dot_dps, -25.0f, TOL) != 0, "4 rate sign");

  /* 5. outside the usable range: clamped and invalid */
  cfg = cfg_make(ATT_AXIS_PITCH, 1.0f, 0.0f, 0.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 89.0f, 0.0f, NOMINAL_TS);
  att_update(&a, 5u, 0.0f);
  st = *att_get();
  chk((near(st.theta_deg, 60.0f, TOL) != 0) && (st.valid == 0u), "5 out of range clamps and invalidates");

  /* 6. exactly at the range limit is still usable */
  cfg = cfg_make(ATT_AXIS_PITCH, 1.0f, 0.0f, 0.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 60.0f, 0.0f, NOMINAL_TS);
  att_update(&a, 5u, 0.0f);
  st = *att_get();
  chk((near(st.theta_deg, 60.0f, TOL) != 0) && (st.valid == 1u), "6 range limit is inclusive");

  /* 7. frame age: 100 ms ok, 101 ms not */
  cfg = cfg_make(ATT_AXIS_PITCH, 1.0f, 0.0f, 0.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 4.0f, 0.0f, NOMINAL_TS);
  att_update(&a, 100u, 0.0f);
  chk(att_get()->valid == 1u, "7a age == timeout is usable");
  ahrs_set(&a, 4.0f, 0.0f, NOMINAL_TS + 5000);
  att_update(&a, 101u, 0.0f);
  chk(att_get()->valid == 0u, "7b age > timeout is unusable");

  /* 8. uneven sample interval does not bias the angle (filter off) */
  cfg = cfg_make(ATT_AXIS_PITCH, 1.0f, 0.0f, 0.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 3.0f, 0.0f, NOMINAL_TS);
  att_update(&a, 5u, 0.0f);
  ahrs_set(&a, 9.0f, 0.0f, NOMINAL_TS + 4000);
  att_update(&a, 4u, 0.0f);
  ahrs_set(&a, -6.0f, 0.0f, NOMINAL_TS + 11000);
  att_update(&a, 7u, 0.0f);
  chk(near(att_get()->theta_deg, -6.0f, TOL) != 0, "8 uneven dt with filter off");

  /* 9. low pass: after 300 ms at 2 Hz the step is within 5 percent */
  cfg = cfg_make(ATT_AXIS_PITCH, 1.0f, 0.0f, 2.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 0.0f, 0.0f, NOMINAL_TS);
  att_update(&a, 5u, 0.0f);                     /* reference at 0 deg */
  for (i = 1; i <= 60; i++)
  {
    ahrs_set(&a, 10.0f, 0.0f, NOMINAL_TS + (int64_t)i * 5000);
    att_update(&a, 5u, 0.0f);
  }
  st = *att_get();
  chk(near(st.theta_deg, 10.0f, 0.5f) != 0, "9 low pass settles within 5 percent in 300 ms");

  /* 10. filter off: sample passes straight through, no lag */
  cfg = cfg_make(ATT_AXIS_PITCH, 1.0f, 0.0f, 0.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 0.0f, 0.0f, NOMINAL_TS);
  att_update(&a, 5u, 0.0f);
  ahrs_set(&a, 7.5f, 0.0f, NOMINAL_TS + 5000);
  att_update(&a, 5u, 0.0f);
  chk(near(att_get()->theta_deg, 7.5f, TOL) != 0, "10 filter off has no phase lag");

  /* 11. att_zero() captures the current reading as upright */
  cfg = cfg_make(ATT_AXIS_PITCH, 1.0f, 0.0f, 0.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 12.0f, 0.0f, NOMINAL_TS);
  att_update(&a, 5u, 0.0f);
  att_zero();
  st = *att_get();
  chk((near(st.theta_deg, 0.0f, TOL) != 0) && (near(att_offset_deg(), 12.0f, TOL) != 0),
      "11 att_zero captures the current raw value");

  /* 12. timestamp wrap keeps the interval sane (filter on: 60 samples of 5 ms) */
  cfg = cfg_make(ATT_AXIS_PITCH, 1.0f, 0.0f, 2.0f, 100u, 60.0f);
  att_init(&cfg);
  ahrs_set(&a, 0.0f, 0.0f, 0xFFFFFF00);
  att_update(&a, 5u, 0.0f);
  for (i = 1; i <= 60; i++)
  {
    ahrs_set(&a, 10.0f, 0.0f, 0xFFFFFF00 + (int64_t)i * 5000);
    att_update(&a, 5u, 0.0f);
  }
  st = *att_get();
  chk(near(st.theta_deg, 10.0f, 0.5f) != 0, "12 timestamp wrap keeps the filter running");

  printf("checks: %d  failed: %d\n", s_checks, s_failed);
  printf("RESULT: %s\n", (s_failed == 0) ? "PASS" : "FAIL");

  return (s_failed == 0) ? 0 : 1;
}