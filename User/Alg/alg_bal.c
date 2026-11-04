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
  s_out.phi_rad    = 0.0f;
  s_out.dphi_rps   = 0.0f;
  s_out.omega_rps  = 0.0f;
  s_out.saturated  = 0u;
  s_out.deadband   = 0u;
  s_out.steps      = 0u;
}

void alg_bal_step(float phi_rad, float dphi_rps, float omega_rps,
                  alg_bal_out_t *out)
{
  float tau;
  uint8_t deadband = 0u;
  uint8_t saturated = 0u;

  tau = (s_cfg.kp_nm_rad * phi_rad) +
        (s_cfg.kd_nm_rps * dphi_rps) +
        (s_cfg.kw_nm_rps * omega_rps);
  tau *= s_cfg.sign;

  s_out.tau_raw_nm = tau;

  tau = apply_deadband(tau, &deadband);
  if (deadband == 0u)
  {
    tau = apply_limit(tau, s_cfg.max_nm, &saturated);
  }

  s_out.tau_nm    = tau;
  s_out.phi_rad   = phi_rad;
  s_out.dphi_rps  = dphi_rps;
  s_out.omega_rps = omega_rps;
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
