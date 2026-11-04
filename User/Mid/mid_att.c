/**
  ******************************************************************************
  * @file    mid_att.c
  * @brief   Attitude layer implementation (S2). Pure computation, no HAL.
  *
  *          Kept free of HAL and of board_cfg.h so this very same source file
  *          is compiled on a PC by test/l1/att_test.c: everything that differs
  *          between the lab and the board arrives through att_config_t.
  *
  *          NOTE: this file must stay pure ASCII.
  ******************************************************************************
  */

#include "mid_att.h"

/* ------------------------------------------------------------------------- */
/* state                                                                      */
/* ------------------------------------------------------------------------- */

static att_config_t s_cfg;
static att_state_t  s_st;
static uint8_t      s_ready;      /* first sample seen                      */
static uint8_t      s_have_ts;    /* s_last_ts_us is valid                  */
static int64_t      s_last_ts_us;
static float        s_raw_deg;    /* last raw reading, for att_zero()       */

/* ------------------------------------------------------------------------- */
/* helpers                                                                    */
/* ------------------------------------------------------------------------- */

static float lp_alpha(float lp_hz, uint32_t dt_us)
{
  float tau;
  float dt;

  if (lp_hz <= 0.0f)
  {
    return 1.0f;                  /* filter off: no phase lag at all */
  }

  tau = 1.0f / (2.0f * 3.14159265f * lp_hz);
  dt  = (float)dt_us * 1.0e-6f;

  return dt / (tau + dt);
}

static uint32_t sample_dt_us(int64_t ts_us)
{
  uint64_t diff;

  if (s_have_ts == 0u)
  {
    return 0u;                    /* first frame: snap, no filter */
  }

  /* The module time base is 32-bit microseconds and wraps about every 71
   * minutes; unsigned subtraction survives the wrap. */
  diff = (uint64_t)(uint32_t)ts_us - (uint64_t)(uint32_t)s_last_ts_us;

  if (diff < (uint64_t)ATT_DT_MIN_US)
  {
    return (uint32_t)0u;          /* duplicate / jittered: keep the last value */
  }
  if (diff > (uint64_t)ATT_DT_MAX_US)
  {
    return ATT_DT_MAX_US;         /* gap: do not let the filter jump */
  }

  return (uint32_t)diff;
}

/* ------------------------------------------------------------------------- */
/* public                                                                     */
/* ------------------------------------------------------------------------- */

void att_init(const att_config_t *cfg)
{
  if (cfg == 0)
  {
    return;
  }

  s_cfg            = *cfg;
  s_ready          = 0u;
  s_have_ts        = 0u;
  s_last_ts_us     = 0;
  s_raw_deg        = 0.0f;

  s_st.theta_deg     = 0.0f;
  s_st.theta_dot_dps = 0.0f;
  s_st.wheel_rpm     = 0.0f;
  s_st.age_ms        = 0xFFFFu;   /* nothing received yet */
  s_st.valid         = 0u;
  s_st.seq           = 0u;
}

void att_update(const fdilink_ahrs_t *ahrs, uint16_t age_ms, float wheel_rpm)
{
  float    raw_deg;
  float    rate_dps;
  float    target;
  float    alpha;
  float    limit;
  uint32_t dt_us;
  uint8_t  out_of_range;

  s_st.age_ms    = age_ms;
  s_st.wheel_rpm = wheel_rpm;
  s_st.valid     = 0u;

  if (ahrs == 0)
  {
    return;
  }
  if ((s_cfg.range_deg <= 0.0f) || (s_cfg.sign == 0.0f))
  {
    return;                       /* att_init() has not been called */
  }

  if (s_cfg.axis == ATT_AXIS_ROLL)
  {
    raw_deg  = ahrs->roll * ATT_RAD2DEG;
    rate_dps = ahrs->roll_speed * ATT_RAD2DEG;
  }
  else
  {
    raw_deg  = ahrs->pitch * ATT_RAD2DEG;
    rate_dps = ahrs->pitch_speed * ATT_RAD2DEG;
  }

  s_raw_deg = raw_deg;

  /* Body pitch: remove the mounting offset, then apply the direction sign. */
  target = s_cfg.sign * (raw_deg - s_cfg.offset_deg);

  out_of_range = 0u;
  limit        = s_cfg.range_deg + ATT_RANGE_EPS_DEG;
  if ((target > limit) || (target < -limit))
  {
    out_of_range = 1u;            /* reported as unusable, value is clamped */
  }
  if (target > s_cfg.range_deg)
  {
    target = s_cfg.range_deg;
  }
  if (target < -s_cfg.range_deg)
  {
    target = -s_cfg.range_deg;
  }

  dt_us        = sample_dt_us(ahrs->timestamp_us);
  s_last_ts_us = ahrs->timestamp_us;
  s_have_ts    = 1u;

  /* The caller may run far faster than the module sends frames, so the same
   * 0x41 frame can be offered several times. A repeated timestamp carries no
   * new information and must not move the filtered value. */
  if (s_ready == 0u)
  {
    s_st.theta_deg = target;      /* first sample: no filter transient */
    s_ready        = 1u;
  }
  else if (dt_us != 0u)
  {
    alpha = lp_alpha(s_cfg.lp_hz, dt_us);
    s_st.theta_deg += alpha * (target - s_st.theta_deg);
  }
  else
  {
    /* repeated frame: keep theta, only refresh age and validity */
  }

  s_st.theta_dot_dps = s_cfg.sign * rate_dps;
  s_st.seq++;

  if ((age_ms > s_cfg.timeout_ms) || (out_of_range != 0u))
  {
    s_st.valid = 0u;
  }
  else
  {
    s_st.valid = 1u;
  }
}

const att_state_t *att_get(void)
{
  return &s_st;
}

void att_zero(void)
{
  s_cfg.offset_deg = s_raw_deg;
  s_st.theta_deg   = 0.0f;
}

float att_offset_deg(void)
{
  return s_cfg.offset_deg;
}
