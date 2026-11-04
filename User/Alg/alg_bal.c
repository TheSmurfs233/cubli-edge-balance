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
static float         s_law_mnm;  /* last law output before ff, mN.m       */
static float         s_u_lp_mnm; /* low-passed law output, mN.m           */
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
  s_cfg.trim_adapt_k    = 0.0f;
  s_cfg.trim_adapt_tau_s = 0.0f;
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
  s_law_mnm        = 0.0f;
  s_u_lp_mnm       = 0.0f;
  s_phi_deg        = 0.0f;
}

void alg_bal_adapt(float dt_s)
{
  float lim;
  float gate;

  if ((s_ready == 0u) || (dt_s <= 0.0f))
  {
    return;
  }
  if (s_cfg.trim_adapt_k <= 0.0f)
  {
    return;
  }

  /* Low-pass the law output: the DC part is the disturbance the loop is
     holding, the AC part is the balancing action and must not move the trim. */
  if (s_cfg.trim_adapt_tau_s > 0.0f)
  {
    s_u_lp_mnm += (s_law_mnm - s_u_lp_mnm) * (dt_s / s_cfg.trim_adapt_tau_s);
  }
  else
  {
    s_u_lp_mnm = s_law_mnm;
  }

  gate = s_cfg.trim_gate_deg;
  if ((gate > 0.0f) && ((s_phi_deg > gate) || (s_phi_deg < -gate)))
  {
    /* Outside the window the balance loop owns the machine; a trim change here
       would lean the body while it is still being captured. */
    return;
  }

  /* A positive command means the loop is holding the body against a forward
     push, so the zero moves BACKWARD and gravity takes the load over:
     mgh * sin(trim) = -(ext + tau_f). Advancing the trim lowers the command,
     which is what stops the wheel from winding up. */
  s_trim_rad -= (s_cfg.trim_adapt_k * s_u_lp_mnm * 0.01745329252f) * dt_s;

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
  s_law_mnm = tau * 1000.0f;
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
