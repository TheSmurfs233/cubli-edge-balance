/**
  ******************************************************************************
  * @file    alg_bal.h
  * @brief   One-axis balance law for a reaction-wheel cube (S4).
  *
  *   tau[N.m] = kp * phi[rad] + kd * dphi[rad/s] + kw * omega[rad/s]
  *
  * All three gains are POSITIVE. +phi is the body leaning away from upright,
  * +omega is the wheel spinning in the direction that pushes the body further
  * over, so a positive command has to accelerate the wheel backwards and the
  * reaction torque on the body pushes it back. The sign of the physical
  * mounting (which way the wheel actually spins for a positive torque) is
  * applied once, here, by cfg.sign.
  *
  * Pure math: no HAL, no Bsp globals, no board_cfg.h. The same source is
  * compiled on the host (test/l1/bal_test.c) and on the target.
  ******************************************************************************
  */
#ifndef ALG_BAL_H
#define ALG_BAL_H

#include <stdint.h>

/** Torque below this is rounded to zero to avoid fighting the deadband of the
    drive (mN.m).  Kept outside the config struct so both callers agree. */
#define ALG_BAL_TAU_MIN_MNM 1.0f

/**
  * @brief  Tuning set. Every value is in SI units (N.m per rad, per rad/s).
  */
typedef struct
{
  float kp_nm_rad;   /**< attitude gain, N.m/rad                 */
  float kd_nm_rps;   /**< attitude rate gain, N.m/(rad/s)         */
  float kw_nm_rps;   /**< wheel speed gain, N.m/(rad/s)           */
  float sign;        /**< +1.0f or -1.0f, physical wheel direction */
  float max_nm;      /**< symmetric torque limit, > 0             */
} alg_bal_cfg_t;

/**
  * @brief  One evaluation result. Read-only to callers.
  */
typedef struct
{
  float tau_nm;      /**< command after sign, deadband and limit  */
  float tau_raw_nm;  /**< command after sign, before limit        */
  float phi_rad;     /**< input echo: body tilt away from upright */
  float dphi_rps;    /**< input echo: body tilt rate              */
  float omega_rps;   /**< input echo: wheel speed relative body   */
  uint8_t saturated; /**< 1 = limit hit this cycle                */
  uint8_t deadband;  /**< 1 = command was rounded to zero          */
  uint32_t steps;    /**< number of evaluations since init/reset  */
} alg_bal_out_t;

/** @brief Load a tuning set. A non-finite or <=0 max_nm disables the law. */
void alg_bal_init(const alg_bal_cfg_t *cfg);

/** @brief Drop the step counter and the last output, keep the tuning. */
void alg_bal_reset(void);

/**
  * @brief  Evaluate the law.
  * @param  phi_rad    body tilt away from the balance point, rad
  * @param  dphi_rps   body tilt rate, rad/s
  * @param  omega_rps  wheel speed relative to the body, rad/s
  * @param  out        destination, may be NULL to discard
  */
void alg_bal_step(float phi_rad, float dphi_rps, float omega_rps,
                  alg_bal_out_t *out);

/** @brief Last output (never NULL; zeroed before the first step). */
const alg_bal_out_t *alg_bal_get(void);

/** @brief 1 = a valid tuning set has been loaded. */
uint8_t alg_bal_ready(void);

#endif /* ALG_BAL_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
