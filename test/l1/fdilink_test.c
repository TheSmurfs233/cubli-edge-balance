/* L1 offline test for mid_fdilink.c: synthetic frames in, structs out.
 * Build with the firmware parser source unchanged (no HAL, no hardware). */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "mid_fdilink.h"

static int g_fail = 0;
static int g_checks = 0;

static void ck(const char *name, int ok, const char *detail)
{
    g_checks++;
    if (!ok) { g_fail++; }
    printf("  %s  %s%s%s\n", ok ? "PASS" : "FAIL", name,
           detail[0] ? "  <- " : "", detail);
}

static void wr_f32(uint8_t *p, float f) { memcpy(p, &f, 4); }
static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF); p[3] = (uint8_t)(v >> 24);
}

/* Frame: FC | type | len | seq | CRC8 | CRC16_H | CRC16_L | payload | FD */
static uint16_t build(uint8_t *out, uint8_t type, uint8_t seq,
                      const uint8_t *payload, uint8_t len)
{
    uint8_t hdr[4];
    uint16_t c16;
    uint8_t c;

    hdr[0] = FDILINK_HEAD; hdr[1] = type; hdr[2] = len; hdr[3] = seq;
    c = fdilink_crc8(hdr, 4u);
    c16 = fdilink_crc16(payload, len);

    out[0] = hdr[0]; out[1] = hdr[1]; out[2] = hdr[2]; out[3] = hdr[3];
    out[4] = c;
    out[5] = (uint8_t)(c16 >> 8);      /* high byte first */
    out[6] = (uint8_t)(c16 & 0xFF);
    memcpy(&out[7], payload, len);
    out[7 + len] = FDILINK_TAIL;
    return (uint16_t)(8u + len);
}

/* Feed byte by byte: the UART hands bytes over one at a time in the real driver */
static void feed_bytes(fdilink_ctx_t *ctx, const uint8_t *buf, uint16_t n)
{
    uint16_t i;
    for (i = 0; i < n; i++) { (void)fdilink_feed(ctx, &buf[i], 1u); }
}

int main(void)
{
    fdilink_ctx_t ctx;
    const fdilink_imu_t *im;
    const fdilink_ahrs_t *ah;
    const fdilink_stat_t *st;
    uint8_t p[72];
    uint8_t buf[80];
    uint16_t n;
    int i;

    /* ---- 1. 0x40 IMU frame ---- */
    memset(p, 0, sizeof(p));
    for (i = 0; i < 3; i++) { wr_f32(&p[i * 4], 0.5f * (float)(i + 1)); }        /* gyro  */
    for (i = 0; i < 3; i++) { wr_f32(&p[12 + i * 4], 9.7f + (float)i); }         /* accel */
    for (i = 0; i < 3; i++) { wr_f32(&p[24 + i * 4], 100.0f * (float)i); }       /* mag   */
    wr_f32(&p[36], 25.5f);                                                       /* temp  */
    wr_f32(&p[40], 101325.0f);                                                   /* press */
    wr_f32(&p[44], 26.5f);                                                       /* pt    */
    wr_u32(&p[48], 0x11223344u);                                                 /* ts lo */
    wr_u32(&p[52], 0x00000001u);                                                 /* ts hi */
    fdilink_init(&ctx);
    n = build(buf, FDILINK_MSG_IMU, 7u, p, FDILINK_IMU_LEN);
    feed_bytes(&ctx, buf, n);
    st = fdilink_get_stat(&ctx);
    ck("0x40: one frame, no error",
       (st->frames == 1u) && (st->frames_imu == 1u) && (st->err_crc8 == 0u) &&
       (st->err_crc16 == 0u) && (st->err_tail == 0u), "");
    im = fdilink_get_imu(&ctx);
    ck("0x40: get_imu returns data", im != NULL, "");
    if (im != NULL) {
        ck("0x40: gyro/accel/temperature decoded",
           (fabsf(im->gyro[2] - 1.5f) < 1e-6f) &&
           (fabsf(im->accel[2] - 11.7f) < 1e-5f) &&
           (fabsf(im->temperature - 25.5f) < 1e-6f), "");
        ck("0x40: timestamp = 0x0000000111223344",
           im->timestamp_us == (int64_t)0x0000000111223344LL, "");
        ck("0x40: seq = 7", im->seq == 7u, "");
    }

    /* ---- 2. 0x41 AHRS frame ---- */
    memset(p, 0, sizeof(p));
    wr_f32(&p[0], 0.01f);  wr_f32(&p[4], 0.02f);  wr_f32(&p[8], 0.03f);   /* speeds  */
    wr_f32(&p[12], 12.5f); wr_f32(&p[16], -3.25f); wr_f32(&p[20], 91.0f); /* r/p/h   */
    wr_f32(&p[24], 1.0f);  wr_f32(&p[28], 0.0f);  wr_f32(&p[32], 0.0f); wr_f32(&p[36], 0.0f);
    wr_u32(&p[40], 0xAABBCCDDu);
    wr_u32(&p[44], 0u);
    n = build(buf, FDILINK_MSG_AHRS, 9u, p, FDILINK_AHRS_LEN);
    feed_bytes(&ctx, buf, n);
    st = fdilink_get_stat(&ctx);
    ck("0x41: counters split imu/ahrs",
       (st->frames == 2u) && (st->frames_imu == 1u) && (st->frames_ahrs == 1u), "");
    ah = fdilink_get_ahrs(&ctx);
    ck("0x41: get_ahrs returns data", ah != NULL, "");
    if (ah != NULL) {
        ck("0x41: euler decoded",
           (fabsf(ah->roll - 12.5f) < 1e-6f) && (fabsf(ah->pitch + 3.25f) < 1e-6f) &&
           (fabsf(ah->heading - 91.0f) < 1e-6f), "");
    }

    /* ---- 3. corruption: CRC8, CRC16, tail ---- */
    fdilink_init(&ctx);
    n = build(buf, FDILINK_MSG_AHRS, 1u, p, FDILINK_AHRS_LEN);
    buf[4] ^= 0xFFu;                       /* header CRC8 */
    feed_bytes(&ctx, buf, n);
    st = fdilink_get_stat(&ctx);
    ck("corrupt: CRC8 rejected", (st->frames == 0u) && (st->err_crc8 >= 1u), "");

    fdilink_init(&ctx);
    n = build(buf, FDILINK_MSG_AHRS, 1u, p, FDILINK_AHRS_LEN);
    buf[7] ^= 0x01u;                       /* first payload byte */
    feed_bytes(&ctx, buf, n);
    st = fdilink_get_stat(&ctx);
    ck("corrupt: CRC16 rejected", (st->frames == 0u) && (st->err_crc16 >= 1u), "");

    fdilink_init(&ctx);
    n = build(buf, FDILINK_MSG_AHRS, 1u, p, FDILINK_AHRS_LEN);
    buf[n - 1] = 0x00u;                    /* tail */
    feed_bytes(&ctx, buf, n);
    st = fdilink_get_stat(&ctx);
    ck("corrupt: bad tail rejected", (st->frames == 0u) && (st->err_tail >= 1u), "");

    /* ---- 4. garbage before a good frame: parser must resync ---- */
    fdilink_init(&ctx);
    {
        uint8_t junk[5] = {0x00u, 0xFCu, 0x12u, 0xFFu, 0xFD};
        feed_bytes(&ctx, junk, 5u);
    }
    n = build(buf, FDILINK_MSG_AHRS, 2u, p, FDILINK_AHRS_LEN);
    feed_bytes(&ctx, buf, n);
    st = fdilink_get_stat(&ctx);
    ck("resync: good frame after garbage is parsed",
       (st->frames == 1u) && (st->dropped_byte >= 1u), "");

    /* ---- 5. byte-at-a-time vs one-shot give identical results ---- */
    {
        fdilink_ctx_t a, b;
        const fdilink_imu_t *ma, *mb;
        int same = 0;

        memset(p, 0, sizeof(p));
        wr_f32(&p[0], 1.25f);
        n = build(buf, FDILINK_MSG_IMU, 3u, p, FDILINK_IMU_LEN);
        fdilink_init(&a); feed_bytes(&a, buf, n);
        fdilink_init(&b); (void)fdilink_feed(&b, buf, (uint32_t)n);   /* whole buffer */
        ma = fdilink_get_imu(&a);
        mb = fdilink_get_imu(&b);
        if ((ma != NULL) && (mb != NULL)) {
            same = (memcmp(ma, mb, sizeof(*ma)) == 0);
        }
        ck("feed: one-shot chunk == byte stream", same, "");
    }

    /* ---- 6. sequence gap detection ---- */
    fdilink_init(&ctx);
    n = build(buf, FDILINK_MSG_IMU, 10u, p, FDILINK_IMU_LEN);
    feed_bytes(&ctx, buf, n);
    n = build(buf, FDILINK_MSG_IMU, 13u, p, FDILINK_IMU_LEN);
    feed_bytes(&ctx, buf, n);
    st = fdilink_get_stat(&ctx);
    ck("seq: gap of 3 frames counted", st->seq_gap_imu == 2u, "");

    /* ---- 7. rad2deg helper ---- */
    ck("rad2deg: pi -> 180", fabsf(fdilink_rad2deg(3.14159265f) - 180.0f) < 1e-3f, "");

    printf("\nchecks: %d, failed: %d\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}
