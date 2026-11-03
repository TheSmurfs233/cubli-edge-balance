/**
  ******************************************************************************
  * @file    mid_fdilink.h
  * @brief   FDILink V21.0823 protocol parser (pure logic, no HAL, no hardware access)
  *
  * Source: sheetdata/3.FDIsystem_communication_protocol_V21.0823.pdf
  *   Frame: FC | type | len | seq | CRC8 | CRC16_H | CRC16_L | payload | FD
  *          i.e. CRC16 sits inside the header, before the payload
  *          (manual 1.1: 1 1 1 1 1 2 1-255 1 bytes)
  *   CRC8 : table lookup over "FC + type + len + seq", init 0
  *   CRC16: payload only, CCITT polynomial 0x1021, init 0, high byte first in the frame
  *
  * This file is "bytes in, structs out"; it can be unit-tested on a PC with fake data.
  * Everything UART/interrupt related lives in bsp_imu.c.
  ******************************************************************************
  */

#ifndef MID_FDILINK_H
#define MID_FDILINK_H

#include <stdint.h>

/* ===== Frame constants ===== */
#define FDILINK_HEAD            0xFCu
#define FDILINK_TAIL            0xFDu

#define FDILINK_MSG_IMU         0x40u   /* calibrated IMU data, 56 B payload */
#define FDILINK_MSG_AHRS        0x41u   /* attitude solution, 48 B payload */
#define FDILINK_MSG_INS         0x42u   /* INS/GPS, 72 B payload (not parsed in this version) */
#define FDILINK_MSG_RAW_SENSOR  0x58u   /* raw sensor data (not parsed in this version) */
#define FDILINK_MSG_HEARTBEAT   0xF0u   /* 1 Hz heartbeat, 0 B payload */

#define FDILINK_IMU_LEN         56u
#define FDILINK_AHRS_LEN        48u

#define FDILINK_MAX_PAYLOAD     72u     /* largest payload accepted in this version (INS/GPS) */
#define FDILINK_RAW_MAX         80u     /* payload + 8 B of header and tail */

/* ===== Parse results ===== */

/* 0x40 MSG_IMU: gyro + accelerometer + magnetometer + temperature/pressure */
typedef struct
{
  float    gyro[3];               /* rad/s, body frame X/Y/Z angular rate */
  float    accel[3];              /* m/s^2, body frame X/Y/Z acceleration (gravity included) */
  float    mag[3];                /* mG, body frame X/Y/Z magnetic field */
  float    temperature;           /* degC */
  float    pressure;              /* Pa */
  float    pressure_temperature;  /* degC */
  int64_t  timestamp_us;          /* microseconds since module power-up */
  uint8_t  seq;                   /* frame sequence number */
} fdilink_imu_t;

/* 0x41 MSG_AHRS: attitude (Euler angles + quaternion) */
typedef struct
{
  float    roll_speed;            /* rad/s */
  float    pitch_speed;           /* rad/s */
  float    heading_speed;         /* rad/s */
  float    roll;                  /* rad */
  float    pitch;                 /* rad */
  float    heading;               /* rad */
  float    q[4];                  /* quaternion Q1..Q4 */
  int64_t  timestamp_us;
  uint8_t  seq;
} fdilink_ahrs_t;

/* Statistics: the first place to look when "data arrives but no frame decodes" */
typedef struct
{
  uint32_t bytes;          /* total bytes fed to the parser */
  uint32_t frames;         /* frames that passed every check */
  uint32_t frames_imu;     /* number of 0x40 frames */
  uint32_t frames_ahrs;    /* number of 0x41 frames */
  uint32_t frames_other;   /* other valid frames (heartbeat etc.) */
  uint32_t err_crc8;       /* header CRC8 failures */
  uint32_t err_crc16;      /* payload CRC16 failures */
  uint32_t err_tail;       /* tail byte is not 0xFD */
  uint32_t err_len;        /* length out of range */
  uint32_t dropped_byte;   /* bytes skipped while hunting for a header (link/baud suspect) */
  uint32_t seq_gap_imu;    /* accumulated IMU sequence gaps (lost frames) */
  uint32_t seq_gap_ahrs;
} fdilink_stat_t;

/* Parser context: allocated by the caller (global or static), no dynamic memory */
typedef struct
{
  uint8_t  state;                 /* current state machine state, see the enum in the .c file */
  uint8_t  type;
  uint8_t  len;
  uint8_t  seq;
  uint8_t  crc8_run;              /* running CRC8 over the header */
  uint16_t crc16_calc;            /* computed payload CRC16 */
  uint16_t crc16_rx;              /* CRC16 taken from the header (high byte first) */
  uint16_t idx;                   /* payload bytes received so far */
  uint8_t  payload[FDILINK_MAX_PAYLOAD];

  uint8_t  raw[FDILINK_RAW_MAX];  /* raw bytes of the frame currently being received */
  uint8_t  raw_len;

  uint8_t  last_frame[FDILINK_RAW_MAX];  /* raw bytes of the latest valid frame (imu raw) */
  uint8_t  last_frame_len;
  uint8_t  last_type;

  uint8_t  last_seq;          /* sequence of the previous frame (all types share one counter) */
  uint8_t  last_seq_valid;

  uint8_t  imu_valid;
  uint8_t  ahrs_valid;

  fdilink_imu_t   imu;
  fdilink_ahrs_t  ahrs;
  fdilink_stat_t  stat;
} fdilink_ctx_t;

/* ===== Interface ===== */

/* Clear the context (statistics included) */
void fdilink_init(fdilink_ctx_t *ctx);
/* Reset only the receive state machine (for resync); keep statistics and the latest frame */
void fdilink_reset(fdilink_ctx_t *ctx);

/* Feed an arbitrary byte stream; returns the number of frames decoded this call */
uint32_t fdilink_feed(fdilink_ctx_t *ctx, const uint8_t *data, uint32_t len);

const fdilink_imu_t  *fdilink_get_imu(const fdilink_ctx_t *ctx);
const fdilink_ahrs_t *fdilink_get_ahrs(const fdilink_ctx_t *ctx);
const fdilink_stat_t *fdilink_get_stat(const fdilink_ctx_t *ctx);

/* The two protocol checks, exposed for unit tests and documentation examples */
uint8_t  fdilink_crc8(const uint8_t *p, uint32_t len);
uint16_t fdilink_crc16(const uint8_t *p, uint32_t len);

/* Radians to degrees (kept here so the app layer does not repeat 57.29578f everywhere) */
float fdilink_rad2deg(float rad);

#endif /* MID_FDILINK_H */

/************************ (C) COPYRIGHT 2026 *********************END OF FILE****/
