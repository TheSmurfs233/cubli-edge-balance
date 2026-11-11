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
  float ff_nm;       /**< friction feed-forward magnitude, N.m; 0 = off */
  float ff_deadband_rps;
  float ff_viscous_nm_s;   /* extra viscous torque, N.m per rad/s */ /**< |wheel speed| below which the ff sign is
                              kept from the previous call, rad/s       */
  float trim_deg;        /**< initial zero trim, deg (positive = lean back) */
  float trim_clamp_deg;  /**< adaptation is clamped to +-this, deg     */
  float trim_gate_deg;   /**< adapt only while |phi| stays under this  */
  float trim_w_k;        /**< deg per rpm per second: the slow trim is
                              walked by the wheel speed; 0 = frozen     */
  float walk_step_deg;      /**< hard cap per alg_bal_adapt() call, deg   */
  float walk_deadband_rpm;  /**< no walk below this in either direction   */
  float walk_fast_rpm;      /* walk gain schedule knee, rpm             */
  float walk_fast_x;        /* gain multiplier above the knee           */
  float walk_filt_tau_s;    /**< low pass on omega, seconds (0 = off)   */
  float frame_dt_s;         /**< nominal step between calls, seconds    */
  float walk_phi_gate_deg;  /**< walk only while |phi| stays under this;
                                 <= 0 falls back to trim_gate_deg         */
} alg_bal_cfg_t;

/**
  * @brief  One evaluation result. Read-only to callers.
  */
typedef struct
{
  float tau_nm;      /**< command after sign, ff, deadband, limit */
  float tau_raw_nm;  /**< law output after sign, before ff/limit  */
  float tau_ff_nm;   /**< friction feed-forward applied, N.m      */
  float phi_rad;     /**< input echo: body tilt away from upright */
  float dphi_rps;    /**< input echo: body tilt rate              */
  float omega_rps;   /**< input echo: wheel speed relative body   */
  float trim_deg;    /**< zero trim in use this cycle, deg        */
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

/**
  * @brief  Advance the slow zero trim by dt (call once per new attitude frame).
  *
  * In steady state gravity pins the body at the true balance point and the
  * wheel parks at
  *
  *     omega_ss = (kp/kw) * (trim - phi_ss)
  *
  * so a zero error of z (= -phi_ss) is paid for entirely in wheel speed: on the
  * bench, -2.32 deg of phi held 1340 rpm. The only knob that changes it is the
  * trim, and the wheel speed is the error signal that says how far off it is:
  *
  *     trim -= trim_w_k * omega_rpm * dt        (deg per rpm per second)
  *
  * The sign is self-correcting: with the wheel turning the trim walks toward
  * phi_ss, and it stops walking exactly when the wheel stops. Adaptation is
  * gated by |phi| < trim_gate_deg and clamped to +-trim_clamp_deg.
  * trim_w_k = 0 freezes it (docs/45 4.12, 4.17).
  *
  * @param  dt_s  time since the previous call, s (>0)
  */
void alg_bal_adapt(float dt_s);

/** @brief Set the trim directly (runtime "bal trim <deg>"), clamped. Returns
    the value actually used, deg. */
float alg_bal_trim_set(float deg);

/** @brief Current trim, deg. */
float alg_bal_trim_deg(void);

/**
  * @brief  Set the wheel-walk gain at runtime ("bal walk <k>").
  *
  * Units are deg per rpm per second, same as trim_w_k in alg_bal_cfg_t. 0
  * freezes the walk. Negative and NaN collapse to 0, anything above
  * ALG_BAL_TRIM_WK_MAX is clamped, because the caller is a console.
  *
  * @return the value actually stored
  */
float alg_bal_trim_wk_set(float k);

/** @brief Current wheel-walk gain, deg per rpm per second. */
float alg_bal_trim_wk_get(void);

/** @brief Upper bound accepted by alg_bal_trim_wk_set (1e-2). */
#define ALG_BAL_TRIM_WK_MAX   1.0e-2f

/** @brief 1 = a valid tuning set has been loaded. */
uint8_t alg_bal_ready(void);

#endif /* ALG_BAL_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
