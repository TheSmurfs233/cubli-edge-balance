/**
  ******************************************************************************
  * @file    alg_bal.c
  * @brief   One-axis balance law for a reaction-wheel cube (S4).
  *
  *   tau = sign * (kp*phi + kd*dphi + kw*omega)      [N.m]
  *
  * Order of operations: sum, apply mounting sign, round tiny commands to
  * zero, clamp to the torque budget. The deadband sits before the limit on
  * purpose: a command of 0.4 mN.m is not worth the current-loop noise, and
  * zero is always inside the limit.
  ******************************************************************************
  */

#include "alg_bal.h"

/* ------------------------------------------------------------------------- */
/* module state                                                              */
/* ------------------------------------------------------------------------- */

static alg_bal_cfg_t s_cfg;
static alg_bal_out_t s_out;
static uint8_t       s_ready;
static float         s_ff_sgn;   /* last feed-forward direction, +-1 or 0 */
static float         s_trim_rad; /* zero trim in use, rad                 */
static float         s_phi_deg;  /* last true body angle, deg             */

/* ------------------------------------------------------------------------- */
/* internal helpers                                                          */
/* ------------------------------------------------------------------------- */

/**
  * @brief  Round a command whose magnitude is below the drive deadband.
  */
static float apply_deadband(float tau_nm, uint8_t *flag)
{
  const float min_nm = ALG_BAL_TAU_MIN_MNM / 1000.0f;

  if ((tau_nm > -min_nm) && (tau_nm < min_nm))
  {
    *flag = 1u;
    return 0.0f;
  }
  return tau_nm;
}

/**
  * @brief  Symmetric clamp. Reports saturation only when clipping happens.
  */
static float apply_limit(float tau_nm, float max_nm, uint8_t *flag)
{
  if (tau_nm > max_nm)
  {
    *flag = 1u;
    return max_nm;
  }
  if (tau_nm < -max_nm)
  {
    *flag = 1u;
    return -max_nm;
  }
  return tau_nm;
}

/* ------------------------------------------------------------------------- */
/* public API                                                                */
/* ------------------------------------------------------------------------- */

void alg_bal_init(const alg_bal_cfg_t *cfg)
{
  s_cfg.kp_nm_rad = 0.0f;
  s_cfg.kd_nm_rps = 0.0f;
  s_cfg.kw_nm_rps = 0.0f;
  s_cfg.sign      = 1.0f;
  s_cfg.max_nm    = 0.0f;
  s_cfg.ff_nm     = 0.0f;
  s_cfg.ff_deadband_rps = 0.0f;
  s_cfg.trim_deg        = 0.0f;
  s_cfg.trim_clamp_deg  = 0.0f;
  s_cfg.trim_gate_deg   = 0.0f;
  s_cfg.trim_w_k        = 0.0f;
  s_ready         = 0u;

  if (cfg != 0)
  {
    /* NaN fails every comparison, so these guards reject NaN as well. */
    if ((cfg->max_nm > 0.0f) &&
        (cfg->kp_nm_rad >= 0.0f) &&
        (cfg->kd_nm_rps >= 0.0f) &&
        ((cfg->sign == 1.0f) || (cfg->sign == -1.0f)))
    {
      s_cfg = *cfg;
      s_ready = 1u;
    }
  }

  alg_bal_reset();
}

void alg_bal_reset(void)
{
  s_out.tau_nm     = 0.0f;
  s_out.tau_raw_nm = 0.0f;
  s_out.tau_ff_nm  = 0.0f;
  s_out.phi_rad    = 0.0f;
  s_out.dphi_rps   = 0.0f;
  s_out.omega_rps  = 0.0f;
  s_out.trim_deg   = 0.0f;
  s_out.saturated  = 0u;
  s_out.deadband   = 0u;
  s_out.steps      = 0u;
  s_ff_sgn         = 0.0f;
  s_trim_rad       = s_cfg.trim_deg * 0.01745329252f;
  s_out.trim_deg   = s_cfg.trim_deg;
  s_phi_deg        = 0.0f;
}

static float s_trim_rad_prev;

void alg_bal_adapt(float dt_s)
{
  float lim;
  float gate;
  float rpm;
  float rel_deg;

  s_trim_rad_prev = s_trim_rad;

  if ((s_ready == 0u) || (dt_s <= 0.0f))
  {
    return;
  }
  if (s_cfg.trim_w_k <= 0.0f)
  {
    return;
  }

  gate = s_cfg.walk_phi_gate_deg;
  if (gate <= 0.0f)
  {
    gate = s_cfg.trim_gate_deg;
  }
  /* The guard is about the loop error, not about the absolute tilt: a machine
     standing steadily at a cable-induced offset has a large |phi| but a tiny
     |phi - trim|, and that is exactly the case the walk exists to remove.
     2026-11-11 bench: the absolute form froze the walk at |phi| = 2.6 deg (USB
     lead pulling) with the wheel parked at 200 rpm and trim at its clamp. */
  rel_deg = s_phi_deg - (s_trim_rad * 57.29577951f);
  if ((gate > 0.0f) && ((rel_deg > gate) || (rel_deg < -gate)))
  {
    /* Still capturing: the balance loop owns the machine, a trim change here
       would lean the body while it is being pulled upright. */
    return;
  }

  /* In steady state omega_ss = (kp/kw)*(trim - phi_ss): the wheel speed IS the
     zero error. Walking the trim against the wheel makes that error vanish, and
     the walk stops by itself when the wheel stops. */
  if (s_out.saturated != 0u)
  {
    /* A saturated law is already losing the fight; moving the reference while
       the torque sits at the ceiling just adds a second runaway term.  Bench
       2026-11-06: with this missing the walk reached its clamp in ~10 s and the
       body fell shortly after. */
    return;
  }

  rpm = s_out.omega_rps * 9.5492966f;

  /* No walk below the deadband: the wheel is already where we want it and the
   * sign of a 20 rpm reading is noise. */
  if ((rpm < s_cfg.walk_deadband_rpm) && (rpm > -s_cfg.walk_deadband_rpm))
  {
    return;
  }

  s_trim_rad -= (s_cfg.trim_w_k * rpm * 0.01745329252f) * dt_s;

  /* Hard rate cap: a bogus wheel reading must not be able to ram the reference
     into its clamp inside a second (0.01 deg per call at 200 Hz = 2 deg/s). */
  {
    float step = s_trim_rad - s_trim_rad_prev;
    float max_step = s_cfg.walk_step_deg * 0.01745329252f;
    if (max_step <= 0.0f)
    {
      max_step = 0.01f * 0.01745329252f;
    }
    if (step > max_step)
    {
      s_trim_rad = s_trim_rad_prev + max_step;
    }
    else if (step < -max_step)
    {
      s_trim_rad = s_trim_rad_prev - max_step;
    }
    s_trim_rad_prev = s_trim_rad;
  }

  lim = s_cfg.trim_clamp_deg * 0.01745329252f;
  if (lim > 0.0f)
  {
    if (s_trim_rad > lim)
    {
      s_trim_rad = lim;
    }
    else if (s_trim_rad < -lim)
    {
      s_trim_rad = -lim;
    }
  }
  s_out.trim_deg = s_trim_rad * 57.2957795f;
}

float alg_bal_trim_set(float deg)
{
  float lim = s_cfg.trim_clamp_deg;

  if (lim < 0.0f)
  {
    lim = 0.0f;
  }
  if (deg > lim)
  {
    deg = lim;
  }
  else if (deg < -lim)
  {
    deg = -lim;
  }
  s_trim_rad       = deg * 0.01745329252f;
  s_out.trim_deg   = deg;
  return deg;
}

float alg_bal_trim_wk_set(float k)
{
  if (!(k > 0.0f))
  {
    k = 0.0f;            /* negative and NaN both freeze the walk */
  }
  if (k > ALG_BAL_TRIM_WK_MAX)
  {
    k = ALG_BAL_TRIM_WK_MAX;
  }
  s_cfg.trim_w_k = k;
  return s_cfg.trim_w_k;
}

float alg_bal_trim_wk_get(void)
{
  return s_cfg.trim_w_k;
}

float alg_bal_trim_deg(void)
{
  return s_out.trim_deg;
}

void alg_bal_step(float phi_rad, float dphi_rps, float omega_rps,
                  alg_bal_out_t *out)
{
  float tau;
  float ff;
  uint8_t deadband = 0u;
  uint8_t saturated = 0u;

  tau = (s_cfg.kp_nm_rad * (phi_rad - s_trim_rad)) +
        (s_cfg.kd_nm_rps * dphi_rps) +
        (s_cfg.kw_nm_rps * omega_rps);

  /* Remembered for alg_bal_adapt(): the raw law output (before the mounting
     sign, before feed-forward) is what tells the slow trim which way to lean. */
  s_phi_deg = phi_rad * 57.2957795f;

  tau *= s_cfg.sign;

  s_out.tau_raw_nm = tau;

  /* Friction feed-forward. The motor has to supply the bearing/Coulomb torque
     itself; otherwise the LQR can only make that torque with kw * omega and the
     wheel parks at (tau_f - kp * theta0) / kw, which is a four digit rpm for a
     small zero error (docs/45 4.8). The sign follows the wheel with a hysteresis
     band so noise around zero cannot flip it every cycle. NOT multiplied by
     cfg.sign: it is a physical torque, not a control decision. */
  ff = 0.0f;
  if (s_cfg.ff_nm > 0.0f)
  {
    if (omega_rps > s_cfg.ff_deadband_rps)
    {
      s_ff_sgn = 1.0f;
    }
    else if (omega_rps < -s_cfg.ff_deadband_rps)
    {
      s_ff_sgn = -1.0f;
    }
    ff = s_ff_sgn * s_cfg.ff_nm;
    tau += ff;
  }
  s_out.tau_ff_nm = ff;

  tau = apply_deadband(tau, &deadband);
  if (deadband == 0u)
  {
    tau = apply_limit(tau, s_cfg.max_nm, &saturated);
  }

  s_out.tau_nm    = tau;
  s_out.phi_rad   = phi_rad;
  s_out.dphi_rps  = dphi_rps;
  s_out.omega_rps = omega_rps;
  s_out.trim_deg  = s_trim_rad * 57.2957795f;
  s_out.saturated = saturated;
  s_out.deadband  = deadband;
  s_out.steps++;

  if (out != 0)
  {
    *out = s_out;
  }
}

const alg_bal_out_t *alg_bal_get(void)
{
  return &s_out;
}

uint8_t alg_bal_ready(void)
{
  return s_ready;
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/
