/**
  ******************************************************************************
  * @file    mid_att.h
  * @brief   Attitude layer: module Euler angles -> body pitch state (S2).
  *
  *          The cube and the IMU are bolted together, so the sensor's attitude
  *          and angular rate ARE the body's. What still has to be pinned down:
  *            - the frame        (constant rotation, expressed by API/macros)
  *            - the zero         (raw reading that means "upright")
  *            - validity         (frame age, usable range)
  *
  *          Source units: the FDILink 0x41 payload carries radians and rad/s
  *          (see mid_fdilink.h). This layer converts to degrees / deg-per-s so
  *          the control law, the console and the docs all speak one unit.
  *
  *          Design notes:
  *            - angular rate is passed through (gyro is position independent),
  *              it is never differentiated or integrated here;
  *            - linear acceleration is NOT used (offset-dependent: a x r,
  *              w x (w x r) terms);
  *            - the low pass is off by default: measure the raw signal first.
  *
  *          NOTE: this file must stay pure ASCII.
  ******************************************************************************
  */

#ifndef __MID_ATT_H
#define __MID_ATT_H

#include <stdint.h>
#include "mid_fdilink.h"

/* Source axis that maps onto the body pitch axis. */
#define ATT_AXIS_PITCH      0u
#define ATT_AXIS_ROLL       1u

/* Robustness bounds for the sample interval derived from the frame clock. */
#define ATT_DT_MIN_US       1000u
#define ATT_DT_MAX_US       100000u

/* Guard band for the range test. A reading that sits exactly on the limit must
 * stay usable, but converting deg -> rad -> deg in single precision costs a few
 * 1e-6 deg, which would otherwise flag the exact-limit sample as out of range. */
#define ATT_RANGE_EPS_DEG   0.01f

#define ATT_RAD2DEG         57.2957795f

typedef struct
{
  uint8_t   axis;         /* ATT_AXIS_PITCH or ATT_AXIS_ROLL                 */
  float     sign;         /* +1 or -1, direction calibration                 */
  float     offset_deg;   /* sensor reading (deg) that means upright         */
  float     lp_hz;        /* 0 = no filter (default while measuring)         */
  uint16_t  timeout_ms;   /* frame older than this -> valid = 0              */
  float     range_deg;    /* usable range; outside it -> valid = 0           */
} att_config_t;

typedef struct
{
  float     theta_deg;      /* body pitch, + = lean forward                  */
  float     theta_dot_dps;  /* body pitch rate, deg/s                        */
  float     wheel_rpm;      /* copied from the caller (mid_enc)              */
  uint16_t  age_ms;         /* age of the source frame                       */
  uint8_t   valid;          /* 1 = usable by a balance law                   */
  uint32_t  seq;            /* accepted samples since att_init               */
} att_state_t;

/* cfg must not be NULL. Call once before att_update(). */
void               att_init(const att_config_t *cfg);

/* Call once per received 0x41 frame, in the same tick as imu_poll(). */
void               att_update(const fdilink_ahrs_t *ahrs, uint16_t age_ms, float wheel_rpm);

/* Never returns NULL after att_init(). */
const att_state_t *att_get(void);

/* Treat the current reading as "upright". RAM only: the flash calibration
 * pages are for the encoder and must not be touched from here. */
void               att_zero(void);

/* Current zero in degrees, so the console can show what was captured. */
float              att_offset_deg(void);

#endif /* __MID_ATT_H */
