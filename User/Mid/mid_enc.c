/**
  ******************************************************************************
  * @file    mid_enc.c
  * @brief   MT6701 angle decoding (see mid_enc.h).
  *
  * Layer: Mid.
  *
  * SSI frame, 24 bits MSB first:
  *   bits 23..10  D[13:0]   angle, 0..16383 -> 0..360 deg
  *   bits  9..6   Mg[3:0]   magnetic status (0 = normal)
  *   bits  5..0   CRC[5:0]  x^6 + x + 1 over the 18 bits above
  *
  * Electrical angle = (mech - zero) * pole_pairs * direction, wrapped to
  * [0, 2pi). The zero offset is captured with "enc zero" and stored in its own
  * flash page so it does not disturb the current-calibration record.
  ******************************************************************************
  */

#include "mid_enc.h"

/* A slipped frame re-cuts the same word, so its angle may not move more than
 * this many counts (200 counts = 4.4 deg mechanical). */
#define ENC_SLIP_TOL   8u
#include <math.h>
#include "alg_pll.h"
#include "board_cfg.h"
#include "bsp_flash.h"
#include "bsp_prof.h"
#include "bsp_ssi.h"

#define ENC_TWO_PI      6.283185307179586f
#define ENC_RAD_PER_LSB (ENC_TWO_PI / 16384.0f)
#define ENC_RAD_TO_DEG  (180.0f / 3.14159265358979f)

typedef struct {
    uint32_t magic;
    uint32_t version;
    float    zero_rad;
    uint32_t dir;
    uint16_t crc;            /* CRC16-CCITT over the bytes above */
    uint16_t pad;
} enc_record_t;

/* Encoder subsystem switch: a sensorless build runs with no encoder
 * at all ("enc off"). */
static uint8_t s_enc_enabled = 1u;

static mid_enc_t s_enc;
/* 1 = a frame with a plausible angle and a good CRC has been seen since
 * boot. Used to tell "no encoder fitted" (all-zero frames) from "the
 * encoder died" so the former can fall back instead of faulting. */
static uint8_t   s_raw_seen_ok;
static uint8_t   s_bad_run;      /* consecutive CRC failures */
static uint8_t   s_late_run;     /* calls in a row with a frame in flight */
static uint8_t   s_last[4];      /* raw bytes of the last collected frame */
static uint8_t   s_win;          /* bit offset the last frame decoded at   */
static uint32_t  s_t_enc;        /* timestamp of the last accepted frame   */

/* Consecutive CRC failures that switch from abort() to resync(). One or two bad
 * frames are noise; a run means the reader is misaligned and only a long clock
 * pause brings the MT6701 back (18#L-16, docs/00 #050). */
#define ENC_BAD_RUN_RESYNC 5u
/* A frame still in flight after this many control calls is not slow, it is
 * wedged: a DMA request lost to an abort leaves CNDTR mid-count and TCIF never
 * fires. Measured 2026-11-07: s_busy stayed set until the liveness guard
 * faulted 5 ms later, because the busy path returned without ever reaching
 * the recovery in bsp_ssi_start(). */
#define ENC_LATE_RESYNC    20u

/* How many raw LSB (16384 per turn) the wheel can cover per microsecond, times
 * ten so the limit stays integer maths: 7 is 0.7 LSB/us, i.e. 2560 rpm. The
 * age MUST be scaled in microseconds. It used to be
 *     lim = SLACK + ((now - t_last) / 1000u) * 700u
 * which divides a cycle count by 1000 (that is kHz, not ms) and, with the
 * encoder read every 100 us, rounded to zero: only the 20-count slack was
 * left. Everything above ~730 rpm then looked implausible, the angle froze,
 * the FOC vector starved, the speed loop saturated and the ripple tripped the
 * over-current guard (measured 2026-11-07: "spd 300" crawled to 108 rpm and
 * tripped at 1.31 A). */
#define ENC_LSB_PER_US_X10 10u
/* Age stops growing here, so a long stall cannot widen the window. */
#define ENC_AGE_CAP_US     20000u
/* Counts of slack on top of the travel estimate (0.12 %% of a turn). */
#define ENC_ALIGN_SLACK    20u
static float    s_zero_rad;   /* zero_deg converted once */

/* Speed estimator state (see ENC_SPD_WINDOW_MS) */
static uint32_t  s_pll_clears;      /* mid_enc_pll_clear() calls  */
static alg_pll_t s_pll;          /* S8.1 speed observer */
static uint8_t   s_spd_src;      /* 0 window, 1 PLL */
static float     s_win_rpm;      /* window estimator, for comparison */
static uint32_t  s_t_pll;        /* PLL sample timestamp */
static uint8_t  s_have_prev;
static uint32_t s_t_prev;
static float    s_prev_mech;
static float    s_prev_elec;
static float    s_acc_mech;
static float    s_acc_elec;
static uint32_t s_acc_cyc;

/* ------------------------------------------------------------------ */
/* CRC16-CCITT (0x1021, init 0xFFFF), same as the calibration record   */
/* ------------------------------------------------------------------ */
static uint16_t enc_crc16(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t       crc = 0xFFFFu;
    uint32_t       i;
    uint8_t        b;

    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (b = 0u; b < 8u; b++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* CRC-6 is fed three 6-bit chunks; because the polynomial degree equals the
 * chunk width, one 64-entry table does the whole 18-bit frame in 3 lookups
 * (the bitwise loop cost ~2.4us per call, 30% of the whole encoder cost).
 * The table was cross-checked against 15 real frames captured from the chip. */
static const uint8_t s_crc6_tab[64] = {
    0x00u, 0x03u, 0x06u, 0x05u, 0x0Cu, 0x0Fu, 0x0Au, 0x09u,
    0x18u, 0x1Bu, 0x1Eu, 0x1Du, 0x14u, 0x17u, 0x12u, 0x11u,
    0x30u, 0x33u, 0x36u, 0x35u, 0x3Cu, 0x3Fu, 0x3Au, 0x39u,
    0x28u, 0x2Bu, 0x2Eu, 0x2Du, 0x24u, 0x27u, 0x22u, 0x21u,
    0x23u, 0x20u, 0x25u, 0x26u, 0x2Fu, 0x2Cu, 0x29u, 0x2Au,
    0x3Bu, 0x38u, 0x3Du, 0x3Eu, 0x37u, 0x34u, 0x31u, 0x32u,
    0x13u, 0x10u, 0x15u, 0x16u, 0x1Fu, 0x1Cu, 0x19u, 0x1Au,
    0x0Bu, 0x08u, 0x0Du, 0x0Eu, 0x07u, 0x04u, 0x01u, 0x02u,
};

uint8_t mid_enc_crc6(uint32_t bits18)
{
    uint8_t crc;

    crc = s_crc6_tab[(bits18 >> 12) & 0x3Fu];
    crc = s_crc6_tab[crc ^ ((bits18 >> 6) & 0x3Fu)];
    crc = s_crc6_tab[crc ^ (bits18 & 0x3Fu)];
    return crc;
}


uint8_t mid_enc_late_run(void)
{
    return s_late_run;
}

uint8_t mid_enc_bad_run(void)
{
    return s_bad_run;
}

void mid_enc_last_frame(uint8_t out[4])
{
    if (out != 0) {
        out[0] = s_last[0];
        out[1] = s_last[1];
        out[2] = s_last[2];
        out[3] = s_last[3];
    }
}


/* How many of the four captured bytes actually carried data.
 *
 * The MT6701 drives its 24-bit word and then releases the line, so the first
 * 0xFF byte marks where it stopped. 3 means two data bytes + idle - the
 * symptom of a bit counter parked inside the word (docs/00 #053).
 * */
uint8_t mid_enc_data_bytes(void)
{
    uint8_t i;

    for (i = 0u; i < BSP_SSI_FRAME_LEN; i++) {
        if (s_last[i] == 0xFFu) {
            return i;
        }
    }
    return (uint8_t)BSP_SSI_FRAME_LEN;
}


uint8_t mid_enc_win(void)
{
    return s_win;
}

/* ------------------------------------------------------------------ */
/* Zero / direction persistence                                        */
/* ------------------------------------------------------------------ */
static void enc_save(void)
{
    enc_record_t rec;

    rec.magic    = ENC_FLASH_MAGIC;
    rec.version  = ENC_FLASH_VERSION;
    rec.zero_rad = s_enc.zero_deg / ENC_RAD_TO_DEG;
    rec.dir      = (uint32_t)s_enc.dir;
    rec.pad      = 0u;
    rec.crc      = enc_crc16(&rec, (uint32_t)(sizeof(rec) - sizeof(rec.crc)
                                              - sizeof(rec.pad)));

    (void)bsp_flash_erase_page(ENC_FLASH_ADDR);
    (void)bsp_flash_program(ENC_FLASH_ADDR, &rec, (uint32_t)sizeof(rec));
}

static void enc_load(void)
{
    enc_record_t rec;

    bsp_flash_read(ENC_FLASH_ADDR, &rec, (uint32_t)sizeof(rec));

    if ((rec.magic == ENC_FLASH_MAGIC) &&
        (rec.version == ENC_FLASH_VERSION) &&
        (rec.crc == enc_crc16(&rec, (uint32_t)(sizeof(rec) - sizeof(rec.crc)
                                               - sizeof(rec.pad)))) &&
        (rec.zero_rad >= 0.0f) && (rec.zero_rad < ENC_TWO_PI) &&
        (rec.dir <= 1u)) {
        s_enc.zero_deg = rec.zero_rad * ENC_RAD_TO_DEG;
        s_zero_rad     = rec.zero_rad;
        s_enc.dir      = (uint8_t)rec.dir;
        s_enc.src      = ENC_SRC_FLASH;
    } else {
        s_enc.zero_deg = 0.0f;
        s_enc.dir      = ENC_DIR_DEFAULT;
        s_enc.src      = ENC_SRC_NONE;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
void mid_enc_init(void)
{
    s_enc.raw      = 0u;
    s_enc.mg       = 0u;
    s_enc.crc_ok   = 0u;
    s_enc.valid    = 0u;
    s_enc.mech_deg = 0.0f;
    s_enc.elec_deg = 0.0f;
    s_enc.frames   = 0u;
    s_enc.crc_errs = 0u;
    s_enc.io_errs  = 0u;
    s_enc.late     = 0u;
    s_enc.ovr      = 0u;
    s_enc.spd_hz   = 0.0f;
    s_enc.spd_rpm  = 0.0f;

    s_have_prev = 0u;
    s_acc_mech  = 0.0f;
    s_acc_elec  = 0.0f;
    s_acc_cyc   = 0u;

    bsp_ssi_init();
    enc_load();


    /* S8.1: speed observer. The angle fed in is the electrical one, so the
     * PLL output is already in electrical rad/s. */
    alg_pll_init(&s_pll, PLL_WN_HZ, PLL_ZETA,
                 ENC_TWO_PI * (SPD_RPM_MAX / 60.0f) * (float)ENC_POLE_PAIRS);
    s_spd_src = ENC_SPD_DEFAULT;
    s_win_rpm = 0.0f;
}

/* Pick the 24-bit window out of the four bytes that passes CRC6.
 *
 * The MT6701 shifts out a fixed 24-bit word and has no frame sync, so a
 * transfer cut short leaves the reader one byte (or a few bits) off and then
 * every frame fails CRC forever: docs/00 #052 measured 100 % CRC error until
 * the wheel was turned by hand. Reading one byte more makes the real frame one
 * of nine windows, so a CRC pass per window recovers the phase instead of
 * waiting for luck (18#L-16).
 *
 * The gate matters as much as the search: with nine windows a CRC6 false hit
 * is not rare, and taking one would move the angle to a random value. Phase 0
 * is the documented contract (CRC alone), but a window found by searching has
 * to be plausible too - at 20 kHz the wheel cannot move far between frames.
 * */
/* Byte offset of the last good frame (0/8/16/24), for diagnostics. */

static uint8_t enc_decode(const uint8_t f[BSP_SSI_FRAME_LEN], uint32_t lim,
                          uint32_t *w_out)
{
    uint8_t  i;
    uint8_t  idle = 0xFFu;
    uint8_t  hits = 0u;
    uint32_t w;

    /* The MT6701 shifts out 24 data bits and then holds the line high for the
     * remaining clocks, so a four-byte burst always contains exactly one 0xFF
     * byte: the idle tail. The 24 data bits start right after it in stream
     * order. Anchoring on that byte is deterministic; the old "try all 32
     * rotations and trust CRC6" search was not, because with 32 candidates a
     * corrupt burst produced a CRC-passing window roughly 3 times out of 4 and
     * whichever one won moved the electrical angle (measured 2026-11-07: 30 %
     * of the bursts slipped by a byte while the wheel turned). */
    for (i = 0u; i < BSP_SSI_FRAME_LEN; i++) {
        uint32_t cand;
        int32_t  jd;
        uint32_t d2;

        if (f[i] != 0xFFu) {
            continue;
        }
        hits++;
        /* A data byte can itself be 0xFF, so every 0xFF has to be tried as
         * the boundary: CRC6 and the travel gate decide which one is real. */
        cand = ((uint32_t)f[(i + 1u) & 3u] << 16) |
               ((uint32_t)f[(i + 2u) & 3u] << 8)  |
                (uint32_t)f[(i + 3u) & 3u];
        if (mid_enc_crc6(cand >> 6) != (cand & 0x3Fu)) {
            continue;
        }
        if (s_enc.valid != 0u) {
            jd = (int32_t)((cand >> 10) & 0x3FFFu) - (int32_t)s_enc.raw;
            if (jd >  8192) { jd -= 16384; }
            if (jd < -8192) { jd += 16384; }
            d2 = (uint32_t)((jd < 0) ? -jd : jd);
            if (d2 > lim) {
                continue;                /* CRC luck on a corrupt burst */
            }
        }
        idle = i;
        w    = cand;
        break;
    }
    (void)hits;
    if (idle == 0xFFu) {
        return 0u;                       /* no frame boundary with a good CRC */
    }

    /* Byte offset of the frame start, in the units the ssi command and the
     * host tests use: 0 = aligned, 8/16/24 = one/two/three bytes late. */
    s_win = (uint8_t)(((1u + idle) & 3u) * 8u);
    *w_out = w;
    return 1u;
}


void mid_enc_update(void)
{
    uint8_t  f[BSP_SSI_FRAME_LEN];
    uint32_t w;
    uint16_t d;
    uint8_t  mg;
    uint8_t  rc;
    uint32_t lim;
    float    mech;
    float    elec;

    rc = bsp_ssi_poll(f);
    if (s_enc_enabled == 0u) {
        return;                  /* no encoder: nothing to read */
    }

    if (rc == 0u) {
        /* Nothing ready. Two very different cases:
         *  - a transfer is still armed  -> the DMA missed its slot (late)
         *  - nothing was armed at all    -> the bridge is off and this call is
         *    the one that arms the next frame, which is the normal 1 kHz path
         * Counting both as "late" made the stopped state look broken
         * (late == frames), so only the first case is counted. */
        if (bsp_ssi_busy() != 0u) {
            /* A transfer is still shifting. Never re-arm on top of it: doing
             * that aborted the frame mid-shift, which is what left the MT6701
             * stuck at bit 8 (docs/00 #052, #053). Just wait for the DMA. */
            s_enc.late++;
            if (++s_late_run >= ENC_LATE_RESYNC) {
                s_late_run = 0u;
                bsp_ssi_force_resync();     /* wedged, not slow */
            }
            return;
        }
        bsp_ssi_start();
        return;
    }

    s_late_run = 0u;
    s_enc.frames++;
    s_last[0] = f[0];
    s_last[1] = f[1];
    s_last[2] = f[2];
    s_last[3] = f[3];

    /* A lost byte must never reach the control loop: keep the previous angle
     * (one period old, 0.09 deg at 5 Hz) instead of a wrong one. */
    if (rc == 2u) {
        s_enc.ovr++;
        bsp_ssi_abort();        /* a lost byte shifts every later frame */
        bsp_ssi_start();
        return;
    }

    /* Slack of 20 counts on top of the physical travel: 20 counts is 0.12 %%
     * of a turn, so a CRC6 false hit (1 in 64, times 32 rotations) almost
     * never lands that close to the previous angle. The old 100-count
     * slack let about one frame in a thousand through to a wrong angle. */
    {
        uint32_t age_us;

        age_us = (uint32_t)((bsp_prof_now() - s_t_enc) / PHASE_CYC_PER_US);
        if (age_us > ENC_AGE_CAP_US) {
            age_us = ENC_AGE_CAP_US;
        }
        lim = ENC_ALIGN_SLACK + (age_us * ENC_LSB_PER_US_X10) / 10u;
    }
    if (enc_decode(f, lim, &w) == 0u) {
        /* Do not just skip the frame: a corrupt read usually means the reader
         * slipped by a byte, and without a resync every frame after it is
         * misaligned too (18#L-16). Aborting flushes the receive path so the
         * next frame starts on a byte boundary again.
         *
         * abort() alone was enough while the slip happened once, but a frame
         * dropped mid-shift leaves the MT6701 still counting clock edges, so
         * the next frame starts inside the old one, fails CRC, is aborted
         * again, and the reader stays locked out until a long clock silence
         * (measured 2026-11-07: 100 % CRC error for minutes, cleared only by
         * turning the wheel half a revolution). A run of failures is that
         * state, so pause the clock instead of aborting yet again. */
        s_enc.crc_ok = 0u;
        s_enc.crc_errs++;
        /* Do NOT touch s_enc.raw here. It is the reference the plausibility
         * gate measures the next frame against; a corrupt word would move the
         * reference to a random angle, so every later good frame looks like a
         * CRC-lucky jump and gets rejected too. That latched the reader out
         * completely and the FOC steered on a frozen angle until the phase
         * currents tripped the overcurrent guard (measured 2026-11-07). The
         * raw bytes of the rejected frame stay readable through
         * mid_enc_last_frame() for diagnosis. */
        if (bsp_ssi_busy() != 0u) {
            /* Another caller already armed the next frame. Aborting or
             * resyncing now would cut it mid-shift and make the next read
             * worse, so leave the bus alone and let that frame finish. */
            return;
        }
        /* Count the run so "ssi" can show it, but do not touch the bus: a
         * byte-level slip is recovered by the rotation decoder on the very
         * next frame, and parking the bus costs 2.5 ms of stale angle (50
         * control frames), which starves the FOC vector and makes the motor
         * buzz instead of producing torque (measured 2026-11-07). */
        s_bad_run++;
        bsp_ssi_start();        /* keep arming: the next frame may decode */
        return;
    }
    s_bad_run = 0u;

    d  = (uint16_t)((w >> 10) & 0x3FFFu);
    mg = (uint8_t)((w >> 6) & 0x0Fu);

    s_t_enc = bsp_prof_now();
    s_enc.raw    = d;
    s_enc.mg     = mg;
    s_enc.crc_ok = 1u;
    if (d != 0u) {
        s_raw_seen_ok = 1u;      /* a real encoder is answering */
    }

    /* Angle: raw -> mechanical degrees, then remove the zero reference. */
    mech = (float)d * ENC_RAD_PER_LSB;                 /* 0 .. 2pi */
    mech -= s_zero_rad;
    while (mech < 0.0f)       { mech += ENC_TWO_PI; }
    while (mech >= ENC_TWO_PI) { mech -= ENC_TWO_PI; }

    /* Electrical angle: x pole pairs, direction reverses the sign. */
    elec = mech * (float)ENC_POLE_PAIRS;
    while (elec >= ENC_TWO_PI) { elec -= ENC_TWO_PI; }
    if (s_enc.dir != 0u) {
        elec = ENC_TWO_PI - elec;
        if (elec >= ENC_TWO_PI) { elec = 0.0f; }
    }

    s_enc.mech_deg = mech * ENC_RAD_TO_DEG;
    s_enc.elec_deg = elec * ENC_RAD_TO_DEG;

    /* ---- S8.1: PLL observer, every frame ---- */
    {
        uint32_t nowp = bsp_prof_now();

        if (s_t_pll != 0u) {
            uint32_t dtp = nowp - s_t_pll;

            /* A gap (after "enc zero" or a stalled loop) would make the
             * loop see a huge delta: restart instead of following it. */
            if ((dtp > 0u) && (dtp < (2u * SYS_CORE_CLK_HZ / 1000u))) {
                float w_rad = alg_pll_step(&s_pll, elec,
                                           (float)dtp / (float)SYS_CORE_CLK_HZ);

                if (s_spd_src == 1u) {
                    float f_elec = w_rad / ENC_TWO_PI;

                    s_enc.spd_hz  = f_elec;
                    s_enc.spd_rpm = f_elec * (60.0f / (float)ENC_POLE_PAIRS);
                }
            } else {
                alg_pll_reset(&s_pll, elec);
            }
        } else {
            alg_pll_reset(&s_pll, elec);
        }
        s_t_pll = nowp;
    }

    /* ---- speed: angle delta over a fixed time window ---- */
    {
        uint32_t now = bsp_prof_now();

        if (s_have_prev != 0u) {
            float    dm = mech - s_prev_mech;
            float    de = elec - s_prev_elec;
            uint32_t dt = now - s_t_prev;

            /* shortest path across the 2pi wrap */
            if (dm > 3.14159265f)  { dm -= ENC_TWO_PI; }
            if (dm < -3.14159265f) { dm += ENC_TWO_PI; }
            if (de > 3.14159265f)  { de -= ENC_TWO_PI; }
            if (de < -3.14159265f) { de += ENC_TWO_PI; }

            if (dt == 0u) {
                /* Two updates at the same instant: keep the reference so
                 * the next call covers this movement (and so a window never
                 * mixes deltas that have no elapsed time behind them). */
            } else if (dt > (2u * SYS_CORE_CLK_HZ / 1000u)) {
                /* A real gap (after "enc zero" or a stalled loop) would
                 * alias the wrap: restart the window instead. Clearing
                 * s_win_rpm matters: leaving the old value behind made the
                 * lost-lock guard read a frozen speed forever. */
                s_acc_mech = 0.0f;
                s_acc_elec = 0.0f;
                s_acc_cyc  = 0u;
                s_win_rpm  = 0.0f;
                s_t_prev    = now;
                s_prev_mech = mech;
                s_prev_elec = elec;
            } else {
                s_acc_mech += dm;
                s_acc_elec += de;
                s_acc_cyc  += dt;
                s_t_prev    = now;
                s_prev_mech = mech;
                s_prev_elec = elec;
            }
            if (s_acc_cyc >= (SYS_CORE_CLK_HZ / 1000u) * ENC_SPD_WINDOW_MS) {
                float dt_s = (float)s_acc_cyc / (float)SYS_CORE_CLK_HZ;
                s_win_rpm = ((s_acc_mech / dt_s) * (60.0f / ENC_TWO_PI));

                /* The window estimator only drives the outputs when it is
                 * the selected source; the PLL keeps them otherwise. */
                if (s_spd_src == 0u) {
                    s_enc.spd_hz  = (s_acc_elec / dt_s) / ENC_TWO_PI;
                    s_enc.spd_rpm = s_win_rpm;
                }
                s_acc_mech = 0.0f;
                s_acc_elec = 0.0f;
                s_acc_cyc  = 0u;
            }
        }

        s_have_prev = 1u;
    }

    /* Arm the next transfer now: the bridge is a 20 kHz task and a poll that
     * only re-arms on "not ready" would read every other frame (10 kHz). */
    bsp_ssi_start();

    s_enc.valid = 1u;
}

void mid_enc_set_enabled(uint8_t en)
{
    s_enc_enabled = (en != 0u) ? 1u : 0u;
    if (s_enc_enabled == 0u) {
        s_enc.valid = 0u;        /* a dead link must never look valid */
    }
}

uint8_t mid_enc_enabled(void) { return s_enc_enabled; }

uint8_t mid_enc_seen_ok(void) { return s_raw_seen_ok; }

void mid_enc_set_spd_src(uint8_t src)
{
    s_spd_src = src;
    if (src == 1u) {
        alg_pll_reset(&s_pll, s_enc.elec_deg * (ENC_TWO_PI / 360.0f));
    }
}

uint8_t mid_enc_spd_src(void)
{
    return s_spd_src;
}

float mid_enc_pll_rpm(void)
{
    return (s_pll.omega / ENC_TWO_PI) * (60.0f / (float)ENC_POLE_PAIRS);
}

uint8_t mid_enc_pll_locked(void)
{
    return s_pll.locked;
}

void mid_enc_set_pll_bw(float wn_hz, float zeta)
{
    alg_pll_set_bw(&s_pll, wn_hz, zeta);
}

/* Drop the PLL state: called when the bridge is disabled so a free-running
 * speed estimate cannot survive the transition (measured: the reported
 * speed stayed at the 2000 rpm clamp for ~6 s after "stop").
 * Deliberately NOT a motion detector: that interacted with the gap path
 * of the PLL and fired during normal acceleration. */
void mid_enc_pll_clear(void)
{
    s_pll_clears++;
    s_pll.omega = 0.0f;
    alg_pll_reset(&s_pll, s_enc.elec_deg * (ENC_TWO_PI / 360.0f));
    s_enc.spd_hz  = 0.0f;
    s_enc.spd_rpm = 0.0f;
    s_win_rpm     = 0.0f;
    s_acc_mech = 0.0f;
    s_acc_elec = 0.0f;
    s_acc_cyc  = 0u;
}

uint32_t mid_enc_pll_clears(void)
{
    return s_pll_clears;
}

float mid_enc_win_rpm(void)
{
    return s_win_rpm;
}

const mid_enc_t *mid_enc_info(void)
{
    return &s_enc;   /* single writer, no shadow copy */
}

void mid_enc_clear_errs(void)
{
    /* Only the error counters: "frames" keeps advancing so the enc
     * command can still show an error rate after a clear. */
    s_enc.crc_errs = 0u;
    s_enc.ovr      = 0u;
    s_enc.late     = 0u;
    s_enc.io_errs  = 0u;
}

uint8_t mid_enc_zero(void)
{
    uint8_t  f[BSP_SSI_FRAME_LEN];
    uint32_t w = 0u;
    uint16_t d;

    if (bsp_ssi_read(f) != 0u) {
        return 1u;
    }

    /* The read returns FOUR bytes, so a 3-byte local here overflowed the stack
     * by one byte and the captured zero depended on whatever the compiler
     * happened to place next. Decode it exactly like mid_enc_update() does:
     * anchor on the idle byte and let the travel gate off (no history). */
    if (enc_decode(f, 0xFFFFFFFFu, &w) == 0u) {
        return 1u;                      /* nothing decodeable: keep the old zero */
    }
    d = (uint16_t)((w >> 10) & 0x3FFFu);

    s_enc.zero_deg = (float)d * ENC_RAD_PER_LSB * ENC_RAD_TO_DEG;
    s_zero_rad     = (float)d * ENC_RAD_PER_LSB;
    if (s_enc.zero_deg >= 360.0f) {
        s_enc.zero_deg -= 360.0f;
    }
    enc_save();
    s_have_prev = 0u;            /* angles jumped: restart the window */

    return 0u;
}

uint8_t mid_enc_zero_set(float deg)
{
    while (deg < 0.0f)    { deg += 360.0f; }
    while (deg >= 360.0f) { deg -= 360.0f; }

    s_enc.zero_deg = deg;
    s_zero_rad     = deg / ENC_RAD_TO_DEG;
    enc_save();
    s_have_prev    = 0u;             /* angles jumped: restart the speed window */
    return 0u;
}

uint8_t mid_enc_set_dir(uint8_t dir)
{
    if (dir > 1u) {
        return 1u;
    }
    s_enc.dir   = dir;
    s_have_prev = 0u;            /* electrical angle flipped */
    enc_save();
    return 0u;
}
