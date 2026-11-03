/**
  ******************************************************************************
  * @file    mid_fdilink.c
  * @brief   FDILink V21.0823 protocol parser implementation (pure logic, no HAL)
  *
  * Parsing strategy:
  *   1. A strict byte-wise state machine; nothing beyond the current frame is
  *      buffered, so the RAM footprint is fixed;
  *   2. Any failed check returns to the header-hunting state at once; the bad byte
  *      is counted and the parser resynchronises. A wrong baud rate or plain ASCII
  *      on the wire only shows up as a growing dropped_byte counter, never as a
  *      hang or an out-of-bounds access;
  *   3. Both CRCs follow the protocol manual (CRC8 table, CRC16 bitwise CCITT).
  ******************************************************************************
  */

#include "mid_fdilink.h"

#include <string.h>

/* ===== State machine states ===== */
enum
{
  ST_HEAD = 0,
  ST_TYPE,
  ST_LEN,
  ST_SEQ,
  ST_CRC8,
  ST_PAYLOAD,
  ST_CRC16_H,
  ST_CRC16_L,
  ST_TAIL
};

/* Table from manual section 1.2 (FDILink - User guide, 2021, page 1-2). Do not change. */
static const uint8_t s_crc8_table[256] =
{
    0,  94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,  31,  65,
  157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,  62,  96, 130, 220,
   35, 125, 159, 193,  66,  28, 254, 160, 225, 191,  93,   3, 128, 222,  60,  98,
  190, 224,   2,  92, 223, 129,  99,  61, 124,  34, 192, 158,  29,  67, 161, 255,
   70,  24, 250, 164,  39, 121, 155, 197, 132, 218,  56, 102, 229, 187,  89,   7,
  219, 133, 103,  57, 186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154,
  101,  59, 217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36,
  248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5, 231, 185,
  140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,  47, 113, 147, 205,
   17,  79, 173, 243, 112,  46, 204, 146, 211, 141, 111,  49, 178, 236,  14,  80,
  175, 241,  19,  77, 206, 144, 114,  44, 109,  51, 209, 143,  12,  82, 176, 238,
   50, 108, 142, 208,  83,  13, 239, 177, 240, 174,  76,  18, 145, 207,  45, 115,
  202, 148, 118,  40, 171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139,
   87,   9, 235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22,
  233, 183,  85,  11, 136, 214,  52, 106,  43, 117, 151, 201,  74,  20, 246, 168,
  116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84, 215, 137, 107,  53
};

/* ===== Little-endian readers (module sends LE; host is LE too, explicit is safer) ===== */

static uint32_t rd_u32(const uint8_t *p)
{
  return ((uint32_t)p[0])
       | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16)
       | ((uint32_t)p[3] << 24);
}

static float rd_f32(const uint8_t *p)
{
  uint32_t v = rd_u32(p);
  float f;

  memcpy(&f, &v, sizeof(f));
  return f;
}

static int64_t rd_i64(const uint8_t *p)
{
  uint64_t lo = (uint64_t)rd_u32(p);
  uint64_t hi = (uint64_t)rd_u32(p + 4);
  uint64_t v  = lo | (hi << 32);

  return (int64_t)v;
}

/* ===== CRC ===== */

uint8_t fdilink_crc8(const uint8_t *p, uint32_t len)
{
  uint8_t crc = 0u;
  uint32_t i;

  for (i = 0u; i < len; i++)
  {
    crc = s_crc8_table[crc ^ p[i]];
  }
  return crc;
}

uint16_t fdilink_crc16(const uint8_t *p, uint32_t len)
{
  uint16_t crc = 0u;
  uint32_t i;
  uint8_t  b;

  /* CRC-16/CCITT (polynomial 0x1021, init 0); equivalent to the manual's table method */
  for (i = 0u; i < len; i++)
  {
    crc ^= (uint16_t)((uint16_t)p[i] << 8);
    for (b = 0u; b < 8u; b++)
    {
      if ((crc & 0x8000u) != 0u)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

float fdilink_rad2deg(float rad)
{
  return rad * 57.29578f;
}

/* ===== Life cycle ===== */

void fdilink_reset(fdilink_ctx_t *ctx)
{
  if (ctx == 0)
  {
    return;
  }
  ctx->state   = (uint8_t)ST_HEAD;
  ctx->idx     = 0u;
  ctx->raw_len = 0u;
  ctx->crc8_run = 0u;
  ctx->crc16_calc = 0u;
  ctx->crc16_rx = 0u;
}

void fdilink_init(fdilink_ctx_t *ctx)
{
  if (ctx == 0)
  {
    return;
  }
  (void)memset(ctx, 0, sizeof(*ctx));
  fdilink_reset(ctx);
}

const fdilink_imu_t *fdilink_get_imu(const fdilink_ctx_t *ctx)
{
  return (ctx == 0) ? 0 : &ctx->imu;
}

const fdilink_ahrs_t *fdilink_get_ahrs(const fdilink_ctx_t *ctx)
{
  return (ctx == 0) ? 0 : &ctx->ahrs;
}

const fdilink_stat_t *fdilink_get_stat(const fdilink_ctx_t *ctx)
{
  return (ctx == 0) ? 0 : &ctx->stat;
}

/* ===== Payload decoding ===== */

static void decode_imu(fdilink_ctx_t *ctx)
{
  const uint8_t *p = ctx->payload;
  fdilink_imu_t *im = &ctx->imu;

  im->gyro[0] = rd_f32(p + 0);
  im->gyro[1] = rd_f32(p + 4);
  im->gyro[2] = rd_f32(p + 8);
  im->accel[0] = rd_f32(p + 12);
  im->accel[1] = rd_f32(p + 16);
  im->accel[2] = rd_f32(p + 20);
  im->mag[0] = rd_f32(p + 24);
  im->mag[1] = rd_f32(p + 28);
  im->mag[2] = rd_f32(p + 32);
  im->temperature = rd_f32(p + 36);
  im->pressure = rd_f32(p + 40);
  im->pressure_temperature = rd_f32(p + 44);
  im->timestamp_us = rd_i64(p + 48);
  im->seq = ctx->seq;

  ctx->imu_valid = 1u;
}

static void decode_ahrs(fdilink_ctx_t *ctx)
{
  const uint8_t *p = ctx->payload;
  fdilink_ahrs_t *ah = &ctx->ahrs;

  ah->roll_speed = rd_f32(p + 0);
  ah->pitch_speed = rd_f32(p + 4);
  ah->heading_speed = rd_f32(p + 8);
  ah->roll = rd_f32(p + 12);
  ah->pitch = rd_f32(p + 16);
  ah->heading = rd_f32(p + 20);
  ah->q[0] = rd_f32(p + 24);
  ah->q[1] = rd_f32(p + 28);
  ah->q[2] = rd_f32(p + 32);
  ah->q[3] = rd_f32(p + 36);
  ah->timestamp_us = rd_i64(p + 40);
  ah->seq = ctx->seq;

  ctx->ahrs_valid = 1u;
}

/* The sequence is 8 bits and wraps; count how many frames were missed in between */
static uint32_t seq_gap(uint8_t prev, uint8_t now)
{
  return (uint32_t)(uint8_t)(now - prev - 1u);
}

static void dispatch_frame(fdilink_ctx_t *ctx)
{
  uint32_t gap = 0u;

  ctx->stat.frames++;
  ctx->last_type = ctx->type;

  /* The sequence increments once per transmitted data frame; 0x40/0x41/heartbeat share
   * one counter (protocol section 1.1, field C), so a gap can only be judged on that
   * single global sequence; comparing across message types would report false losses */
  if (ctx->last_seq_valid != 0u)
  {
    gap = seq_gap(ctx->last_seq, ctx->seq);
  }
  ctx->last_seq = ctx->seq;
  ctx->last_seq_valid = 1u;
  if (gap != 0u)
  {
    if (ctx->type == FDILINK_MSG_IMU)
    {
      ctx->stat.seq_gap_imu += gap;
    }
    else if (ctx->type == FDILINK_MSG_AHRS)
    {
      ctx->stat.seq_gap_ahrs += gap;
    }
  }

  /* Keep the raw bytes so the debug command "imu raw" can print them */
  if ((ctx->raw_len > 0u) && (ctx->raw_len <= FDILINK_RAW_MAX))
  {
    (void)memcpy(ctx->last_frame, ctx->raw, ctx->raw_len);
    ctx->last_frame_len = ctx->raw_len;
  }

  switch (ctx->type)
  {
    case FDILINK_MSG_IMU:
      if (ctx->len >= FDILINK_IMU_LEN)
      {
        ctx->stat.frames_imu++;
        decode_imu(ctx);
      }
      else
      {
        ctx->stat.err_len++;
      }
      break;

    case FDILINK_MSG_AHRS:
      if (ctx->len >= FDILINK_AHRS_LEN)
      {
        ctx->stat.frames_ahrs++;
        decode_ahrs(ctx);
      }
      else
      {
        ctx->stat.err_len++;
      }
      break;

    default:
      ctx->stat.frames_other++;
      break;
  }
}

/* ===== Byte state machine ===== */

static void resync(fdilink_ctx_t *ctx)
{
  ctx->state   = (uint8_t)ST_HEAD;
  ctx->idx     = 0u;
  ctx->raw_len = 0u;
}

static void raw_put(fdilink_ctx_t *ctx, uint8_t b)
{
  if (ctx->raw_len < FDILINK_RAW_MAX)
  {
    ctx->raw[ctx->raw_len] = b;
    ctx->raw_len++;
  }
}

/* Check CRC16 once the payload is complete (covers payload only; the CRC16 field sits */
static uint8_t crc16_verify(fdilink_ctx_t *ctx)
{
  ctx->crc16_calc = fdilink_crc16(ctx->payload, (uint32_t)ctx->len);
  return (uint8_t)((ctx->crc16_rx == ctx->crc16_calc) ? 1u : 0u);
}

uint32_t fdilink_feed(fdilink_ctx_t *ctx, const uint8_t *data, uint32_t len)
{
  uint32_t i;
  uint32_t frames0;
  uint8_t  b;

  if ((ctx == 0) || (data == 0))
  {
    return 0u;
  }

  frames0 = ctx->stat.frames;
  ctx->stat.bytes += len;

  for (i = 0u; i < len; i++)
  {
    b = data[i];

    switch (ctx->state)
    {
      case ST_HEAD:
        if (b == FDILINK_HEAD)
        {
          ctx->raw_len = 0u;
          raw_put(ctx, b);
          ctx->crc8_run = s_crc8_table[FDILINK_HEAD];
          ctx->state = (uint8_t)ST_TYPE;
        }
        else
        {
          ctx->stat.dropped_byte++;
        }
        break;

      case ST_TYPE:
        ctx->type = b;
        raw_put(ctx, b);
        ctx->crc8_run = s_crc8_table[ctx->crc8_run ^ b];
        ctx->state = (uint8_t)ST_LEN;
        break;

      case ST_LEN:
        ctx->len = b;
        raw_put(ctx, b);
        if ((uint32_t)b > FDILINK_MAX_PAYLOAD)
        {
          ctx->stat.err_len++;
          resync(ctx);
        }
        else
        {
          ctx->crc8_run = s_crc8_table[ctx->crc8_run ^ b];
          ctx->state = (uint8_t)ST_SEQ;
        }
        break;

      case ST_SEQ:
        ctx->seq = b;
        raw_put(ctx, b);
        ctx->crc8_run = s_crc8_table[ctx->crc8_run ^ b];
        ctx->state = (uint8_t)ST_CRC8;
        break;

      case ST_CRC8:
        raw_put(ctx, b);
        if (b != ctx->crc8_run)
        {
          ctx->stat.err_crc8++;
          resync(ctx);
        }
        else
        {
          ctx->idx = 0u;
          /* Header order (protocol section 1.1 table): FC, type, len, seq, CRC8, CRC16(hi,lo), payload, FD */
          ctx->state = (uint8_t)ST_CRC16_H;
        }
        break;

      case ST_CRC16_H:
        ctx->crc16_rx = (uint16_t)((uint16_t)b << 8);
        raw_put(ctx, b);
        ctx->state = (uint8_t)ST_CRC16_L;
        break;

      case ST_CRC16_L:
        ctx->crc16_rx = (uint16_t)(ctx->crc16_rx | (uint16_t)b);
        raw_put(ctx, b);
        if (ctx->len == 0u)
        {
          if (crc16_verify(ctx) == 0u)
          {
            ctx->stat.err_crc16++;
            resync(ctx);
          }
          else
          {
            ctx->state = (uint8_t)ST_TAIL;
          }
        }
        else
        {
          ctx->state = (uint8_t)ST_PAYLOAD;
        }
        break;

      case ST_PAYLOAD:
        ctx->payload[ctx->idx] = b;
        raw_put(ctx, b);
        ctx->idx++;
        if (ctx->idx >= (uint16_t)ctx->len)
        {
          if (crc16_verify(ctx) == 0u)
          {
            ctx->stat.err_crc16++;
            resync(ctx);
          }
          else
          {
            ctx->state = (uint8_t)ST_TAIL;
          }
        }
        break;

      case ST_TAIL:
        if (b == FDILINK_TAIL)
        {
          raw_put(ctx, b);
          dispatch_frame(ctx);
          resync(ctx);
        }
        else
        {
          ctx->stat.err_tail++;
          resync(ctx);
        }
        break;

      default:
        resync(ctx);
        break;
    }
  }

  return ctx->stat.frames - frames0;
}

/************************ (C) COPYRIGHT 2026 *********************END OF FILE****/
