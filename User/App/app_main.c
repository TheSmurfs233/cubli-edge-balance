/**
  ******************************************************************************
  * @file    app_main.c
  * @brief   Main loop and task scheduling.
  *
  * while(1) {
  *     bsp_key_poll()      key debounce (called every loop, >10kHz)
  *     key_action()        key event -> LED toggle
  *     app_cmd_poll()      serial command parsing (non-blocking)
  *     bsp_uart_tx_pump()  push DMA transmission
  *     task_1ms()          uptime / loop rate statistics
  *     task_wave()         VOFA+ waveform upload
  * }
  *
  * 2026-10-25 S1: LED/KEY/UART/state machine skeleton
  ******************************************************************************
  */

#include "app_main.h"
#include "app_cmd.h"
#include "board_cfg.h"
#include "alg_svpwm.h"
#include "alg_vf.h"
#include "alg_foc.h"
#include "alg_obs.h"
#include "alg_if.h"
#include "alg_spd.h"
#include "alg_bal.h"
#include "bsp_adc.h"
#include "bsp_vbus.h"
#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "bsp_prof.h"
#include "bsp_pwm.h"
#include "bsp_ssi.h"
#include "bsp_sys.h"
#include "bsp_uart.h"
#include "bsp_crash.h"
#include "bsp_wp.h"
#include "bsp_stack.h"
#include "mid_att.h"
#include "mid_current.h"
#include "mid_enc.h"
#include "mid_log.h"
#include "mid_trig.h"
#include <math.h>
#include "mid_vofa.h"

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */
static app_status_t s_st;
static uint32_t     s_loop_cnt;
static uint32_t     s_t_1ms;
static uint32_t     s_t_1k;   /* 1 kHz task timestamp */
static uint32_t     s_t_wave;
static uint32_t     s_t_enc;
static uint16_t     s_wave_period_ms; /* upload period in ms */
static uint32_t     s_adc_seen;       /* ADC frame counter already consumed */

/* ---- open-loop V/f (S4) ---- */
static alg_vf_t      s_vf;
static uint8_t       s_obs_on;    /* 1 = observer running (shadow) */

static void app_sfoc_release(void);
static uint8_t       s_coast;     /* #168: no angle source, rotor still
                                   * turning -> zero current, no short  */
static uint32_t      s_coast_ticks;
/* S10.5 hand-over */
static uint8_t   s_sfoc;      /* 1 = sensorless: angle and speed from the observer */
static uint8_t   s_ho_state;   /* 0 idle, 1 watching, 2 blending, 3 done */
static uint16_t  s_ho_cnt;
static float     s_ho_w;       /* blend weight 0..1 */
static uint16_t  s_lost_ms;      /* loss-of-lock debounce, 20 kHz frames */
static uint16_t  s_rail_ms;      /* PLL rail dwell, 20 kHz frames        */
static float     s_if_rpm_tgt;  /* rpm requested by sfoc go */
static float     s_ho_dth;     /* |theta_obs - theta_if|, rad */
static float     s_ho_ds;      /* signed theta_obs - theta_if, rad */
static float     s_ho_c0 = 1.0f; /* cos(s_ho_dth) at blend start (S10.8) */

static alg_if_t      s_if;       /* S10.4 open-loop ramp */
static alg_obs_t     s_obs;      /* S10.1 shadow observer */
static app_vf_info_t s_vf_info;

/* ---- FOC current loop (S6) ---- */
static alg_foc_t      s_foc;
static app_foc_info_t s_foc_info;

/* #167: id/iq/vd/vq are written by the FOC step. When the bridge is off (stop,
 * fault, or no angle source at all) nothing refreshes them, so the waveform
 * kept showing "iq 0.07 A" on a drive whose duties were 50/50/50. */
static void foc_info_clear_meas(void)
{
    s_foc_info.id = 0.0f;
    s_foc_info.iq = 0.0f;
    s_foc_info.vd = 0.0f;
    s_foc_info.vq = 0.0f;
    s_foc_info.m  = 0.0f;
}
static uint8_t        s_foc_trip_cnt;

/* S7.6 protections. */
static app_prot_t       s_prot;
static app_fault_info_t s_last_fault;
static app_fault_info_t s_fault;
static uint32_t s_prot_start_ms;   /* start of the grace window        */
static uint32_t s_prot_ov_ms;      /* overspeed debounce               */
static uint32_t s_prot_stall_ms;   /* stall debounce                   */
static float    s_spd_lp;        /* low-passed speed for the stall judge */
static uint32_t s_prot_uv_ms;      /* under-voltage debounce           */
static uint32_t s_prot_ovv_ms;     /* over-voltage debounce            */
static uint16_t s_prot_enc_cnt;    /* consecutive bad encoder frames   */
static uint32_t s_prot_bad_seen;   /* last encoder good-frame counter  */
static uint16_t s_prot_enc_zero_cnt;/* consecutive raw==0 frames        */
static uint16_t s_prot_enc_crc_cnt; /* consecutive CRC-bad encoder frames */
/* S7.3 speed loop. */
static alg_spd_t s_spd;
static uint8_t   s_spd_div;
static uint32_t  s_spd_kick_t0;
/* S7.5 slewed speed-loop current command, A. The PI output is stepped once
 * per 1 ms; without the slew the current reference jumps from 0 to the
 * 1.20 A clamp and the current loop overshoot (~10%) crosses the 1.28 A
 * trip. Reset wherever the loop is (re)seeded so the slew never fights a
 * preload. */
static float     s_spd_iq;

/* Ripple diagnostic: min/max of iq and speed over a short window. */
static app_rip_t  s_rip;
static uint32_t   s_rip_t0;
static uint8_t    s_rip_on;  /* start of the kick window */
static float     s_spd_target;    /* commanded, before the ramp  */
static float     s_torque_mnm;    /* last accepted torque, mN.m   */
static float     s_spd_ramp_hz;   /* rpm per second              */
static att_config_t s_att_cfg;    /* S2: body pitch layer        */

/* S7.4 speed step measurement. */
static app_sstep_info_t s_sstep;
static uint8_t   s_sstep_phase;   /* 0 idle, 1 settle, 2 record  */
static uint32_t  s_sstep_t0;
static uint16_t  s_sstep_n;
static uint16_t  s_sstep_t10;
static uint16_t  s_sstep_t90;
static uint16_t  s_sstep_settle;
static uint8_t   s_sstep_got10;
static uint8_t   s_sstep_got90;
static uint8_t   s_sstep_in_band;
static float     s_sstep_peak;
static float     s_sstep_tail_min;
static float     s_sstep_tail_max;
static float     s_sstep_tail_sum;
static uint16_t  s_sstep_tail_n;

/* S6.4: automatic current step measurement. */
static app_step_info_t s_step;
static uint8_t         s_step_phase;    /* 0 idle, 1 settle, 2 record  */
static uint32_t        s_step_t0;
static uint32_t        s_step_n;
static uint32_t        s_step_t10;
static uint32_t        s_step_t90;
static uint8_t         s_step_got10;
static uint8_t         s_step_got90;
static float           s_step_peak;
static float           s_step_tail_min;
static float           s_step_tail_max;
static float           s_step_tail_sum;
static uint32_t        s_step_tail_n;
static uint16_t      s_i_fold_cnt;
static uint16_t      s_i_trip_cnt;
static uint8_t       s_trig_status;     /* mid_trig_init() result */
static uint8_t       s_enc_div;         /* ENC_DIV prescaler */

/* ALIGN (S5 zero calibration) */
static app_align_t   s_align;
static uint32_t      s_align_t0;        /* tick at start                */
static float         s_align_err_sum;   /* accum. electrical error (rad)  */
static uint32_t      s_align_err_n;     /* samples accumulated            */
static float         s_align_fwd_sum;   /* forward sweep only (rad)       */
static uint32_t      s_align_fwd_n;
static float         s_align_rev_sum;   /* reverse sweep only (rad)       */
static uint32_t      s_align_rev_n;
static uint32_t      s_align_frames0;
static uint8_t       s_align_ol;      /* 1 = open-loop pre-position */   /* encoder frame count at start   */

/* ---- where each control step lands inside the 50us PWM period ---- */
static app_phase_info_t s_phase;

static void phase_mark_isr(uint16_t cnt)
{
    if (bsp_prof_enabled() == 0u) { return; }

    if (s_phase.isr.valid == 0u) {
        s_phase.isr.cnt_min = cnt;
        s_phase.isr.cnt_max = cnt;
        s_phase.isr.valid   = 1u;
    } else {
        if (cnt < s_phase.isr.cnt_min) { s_phase.isr.cnt_min = cnt; }
        if (cnt > s_phase.isr.cnt_max) { s_phase.isr.cnt_max = cnt; }
    }
    s_phase.isr.cnt = cnt;
}

/* Microseconds since t_ref (the ADC interrupt that produced this frame),
 * kept in DWT cycles: converting to us here would cost a 64-bit division per
 * mark (__aeabi_uldivmod, ~1us) and inflate the very number being measured.
 * The command handler converts once when printing. */
#define PHASE_MAX_CYC   340000u        /* 2ms at 170MHz: anything larger is bogus */

static void phase_mark(app_phase_t *p, uint32_t t_ref)
{
    uint32_t cyc;

    /* The probe follows the profiler switch: "prof off" leaves the control
     * chain completely uninstrumented, which is how S6's real-time budget
     * should be evaluated. */
    if (bsp_prof_enabled() == 0u) {
        return;
    }

    cyc = bsp_prof_now() - t_ref;

    if (cyc > PHASE_MAX_CYC) {
        return;                        /* the ADC interrupted again meanwhile */
    }
    if (p->valid == 0u) {
        p->min_cyc = cyc;
        p->max_cyc = cyc;
        p->valid   = 1u;
    } else {
        if (cyc < p->min_cyc) { p->min_cyc = cyc; }
        if (cyc > p->max_cyc) { p->max_cyc = cyc; }
    }
    p->last_cyc = cyc;
}

const app_phase_info_t *app_phase_info(void)
{
    return &s_phase;
}

void app_phase_reset(void)
{
    s_phase.isr.valid  = 0u;
    s_phase.isr.cnt    = 0u;
    s_phase.isr.cnt_min = 0u;
    s_phase.isr.cnt_max = 0u;
    s_phase.pick.valid = 0u;
    s_phase.comp.valid = 0u;
    s_phase.wr.valid   = 0u;
    s_phase.pick.last_cyc = s_phase.pick.min_cyc = s_phase.pick.max_cyc = 0u;
    s_phase.comp.last_cyc = s_phase.comp.min_cyc = s_phase.comp.max_cyc = 0u;
    s_phase.wr.last_cyc   = s_phase.wr.min_cyc   = s_phase.wr.max_cyc   = 0u;
}

/* ------------------------------------------------------------------ */
/* Name helpers                                                        */
/* ------------------------------------------------------------------ */
const char *app_state_name(uint8_t st)
{
    switch (st) {
    case APP_ST_INIT:  return "INIT";
    case APP_ST_IDLE:  return "IDLE";
    case APP_ST_RUN:   return "RUN";
    case APP_ST_ALIGN: return "ALIGN";
    case APP_ST_FAULT: return "FAULT";
    default:           return "?";
    }
}

const char *app_mode_name(uint8_t m)
{
    switch (m) {
    case CTRL_IDLE:     return "IDLE";
    case CTRL_OPENLOOP: return "OPENLOOP";
    case CTRL_FOC:      return "FOC";
    default:            return "?";
    }
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* Attitude layer (S2)                                                 */
/* ------------------------------------------------------------------ */

void app_att_update(void)
{
    fdilink_ahrs_t a;
    uint32_t       age;

    if (imu_get_ahrs(&a) == 0u)
    {
        return;                  /* no 0x41 frame yet: keep the last state */
    }

    age = imu_frame_age_ms();
    if (age > 60000u)
    {
        age = 60000u;            /* keep the uint16 cast meaningful */
    }

    att_update(&a, (uint16_t)age, mid_enc_info()->spd_rpm);
}

static void att_cfg_load(void)
{
    s_att_cfg.axis       = (uint8_t)ATT_AXIS;
    s_att_cfg.sign       = ATT_SIGN;
    s_att_cfg.offset_deg = ATT_OFFSET_DEG;
    s_att_cfg.lp_hz      = ATT_LP_HZ;
    s_att_cfg.timeout_ms = (uint16_t)ATT_TIMEOUT_MS;
    s_att_cfg.range_deg  = ATT_RANGE_DEG;
}

/* ------------------------------------------------------------------ */
/* Balance law, shadow mode (S4)                                       */
/*                                                                     */
/* The law is a pure function in Alg/; this block only packs the three */
/* states, keeps the last result for the 'bal' command, and never      */
/* touches the torque interface. Shadow mode is the acceptance tool:   */
/* if the sign is wrong, the printed command is wrong, but nothing     */
/* moves.                                                              */
/* ------------------------------------------------------------------ */

#define BAL_DEG2RAD     0.01745329252f
#define BAL_RPM2RPS     0.10471975512f

/* S6 threshold consistency is checked in bal_cfg_load(): ARMCC V5 and gcc both
 * reject float operands in an integer constant expression, so the usual
 * negative-array trick cannot express "0.4 deg < 12 deg". */

static alg_bal_cfg_t  s_bal_cfg;
static app_bal_info_t s_bal;
/* Volatile aliases: reading these forces a reload from memory, so the
 * register the compiler would otherwise keep across a call cannot turn
 * an access into a wild pointer (see app_bal_update()). */
static app_bal_info_t *volatile s_bal_p = &s_bal;
static uint8_t        s_bal_on;
static uint32_t       s_bal_seq_prev;   /* attitude frame seen by the last update */
static uint8_t        s_bal_auto = 1u;  /* whether the wheel may walk the trim  */
static uint32_t       s_enc_frames_prev;/* encoder frames at the last check     */
static uint32_t       s_enc_frame_ms;   /* tick of the last encoder frame       */
static uint8_t        s_learn_done;     /* one-shot zero learn already taken    */

static void bal_cfg_load(void)
{
    /* BAL_SCALE is the first-flight derating: the nominal gains stay in
     * board_cfg.h untouched so a test can raise the scale without losing the
     * values that came out of the LQR solve. */
    s_bal_cfg.kp_nm_rad = BAL_KP_NM_RAD * BAL_SCALE;
    s_bal_cfg.kd_nm_rps = BAL_KD_NM_RPS * BAL_SCALE;
    s_bal_cfg.kw_nm_rps = BAL_KW_NM_RPS * BAL_SCALE;
    s_bal_cfg.sign      = BAL_SIGN;
    s_bal_cfg.max_nm    = BAL_MAX_MNM / 1000.0f;
    s_bal_cfg.ff_nm     = BAL_FF_MNM / 1000.0f;
    s_bal_cfg.ff_deadband_rps = BAL_FF_DEADBAND_RPM * BAL_RPM2RPS;
    s_bal_cfg.trim_deg = BAL_TRIM_DEG;
    s_bal_cfg.trim_clamp_deg = BAL_TRIM_MAX_DEG;
    s_bal_cfg.trim_gate_deg = BAL_TRIM_GATE_DEG;
    s_bal_cfg.trim_w_k = BAL_TRIM_W_K;

    /* Threshold order is a safety property: ARM must sit inside TRIP, the
     * balance ceiling must stay below the torque interface, and the stale
     * window must be short compared with the run limit. A wrong edit here
     * would still compile, so say it out loud at boot. */
    if ((BAL_ARM_DEG >= BAL_TRIP_DEG) ||
        (BAL_ARM_DPS >= BAL_TRIP_DPS) ||
        (BAL_ARM_RPM >= BAL_TRIP_RPM) ||
        (BAL_MAX_MNM >= TORQUE_MAX_MNM) ||
        ((BAL_MAX_MS > 0u) &&
         ((uint32_t)BAL_TRIP_STALE_MS > ((uint32_t)BAL_MAX_MS / 2u))))
    {
        log_printf("WARN bal: BAL_* threshold order looks wrong, check board_cfg.h");
    }
}

/* Declared here, above every user, so the volatile alias is visible to
 * app_bal_update(); the S6 code below reaches it through that alias. */
static app_bal_run_t s_run;
static app_bal_run_t *volatile s_run_p = &s_run;

/* A pointer that fails bsp_sys_ram_ptr_ok() is replaced by this zeroed
 * state: valid == 0 makes every user take the "no usable frame" path
 * instead of dereferencing the wild address. */
static const att_state_t s_att_safe;
static uint32_t s_att_ptr_bad;

/* 2026-11-07 16:14 crash: the faulting store was VSTR s0,[r0,#0x10] with
 * r0 taken from [0x20000154] = s_bal_p, which held 0x2002B970 instead of
 * 0x20003170. r4 itself was correct, so the damage was a *write* into .data,
 * not a lost register. Re-derive both aliases from the objects themselves on
 * every use: the addresses then come from the literal pool in flash, and a
 * corrupted word can only cost one skipped write instead of a bus fault. */
static uint32_t s_ptr_fix;

/* The 17:30 crash put the fault at bal_p+0x26, the LDR that re-reads the alias
 * right after the repair branch, with r4 corrupted from 0x20000140 to
 * 0x20100940 while r5/r6/r0 were still correct. The only call between the two
 * reads on that branch is log_printf, so the repair path must stop logging:
 * counting and remembering the bad value costs four stores and no register. */
static volatile uint32_t s_ptr_bad_val;
static volatile uint32_t s_ptr_bad_which;
static volatile uint32_t s_ptr_bad_ms;

static void ptr_record(uint32_t which, uint32_t bad)
{
    s_ptr_bad_val = bad;
    s_ptr_bad_which = which;
    s_ptr_bad_ms = HAL_GetTick();
}

static void bal_ptr_guard(void)
{
    app_bal_info_t *bp = s_bal_p;

    if (bp != &s_bal)
    {
        ++s_ptr_fix;
        ptr_record(1u, (uint32_t)(uintptr_t)bp);
        s_bal_p = &s_bal;
    }

    {
        app_bal_run_t *rp = s_run_p;

        if (rp != &s_run)
        {
            ++s_ptr_fix;
            ptr_record(2u, (uint32_t)(uintptr_t)rp);
            s_run_p = &s_run;
        }
    }
}

uint32_t app_guard_ptr_fix_count(void)
{
    return s_ptr_fix;
}

uint32_t app_guard_ptr_bad_val(void)
{
    return s_ptr_bad_val;
}

uint32_t app_guard_ptr_bad_which(void)
{
    return s_ptr_bad_which;
}

uint32_t app_guard_ptr_bad_ms(void)
{
    return s_ptr_bad_ms;
}

/* The guard above runs once per call, and the 17:00 crash proved that is not
 * enough: s_bal_p was read three times in one straight line, the first two
 * reads good, the third one corrupt. Every use now goes through an accessor
 * that re-loads the alias and repairs it on the spot, so the window between
 * the check and the dereference shrinks to a couple of instructions and a
 * stray write costs one skipped field instead of a bus fault. */
static app_bal_info_t *bal_p(void)
{
    app_bal_info_t *p = s_bal_p;

    if (p != &s_bal)
    {
        ++s_ptr_fix;
        ptr_record(1u, (uint32_t)(uintptr_t)p);
        s_bal_p = &s_bal;
        p = &s_bal;
    }
    return p;
}

static app_bal_run_t *run_p(void)
{
    app_bal_run_t *p = s_run_p;

    if (p != &s_run)
    {
        ++s_ptr_fix;
        ptr_record(2u, (uint32_t)(uintptr_t)p);
        s_run_p = &s_run;
        p = &s_run;
    }
    return p;
}

void app_wp_arm(void)
{
    bsp_wp_arm((uint32_t)(uintptr_t)&s_bal_p, (uint32_t)(uintptr_t)&s_bal);
}

void app_guard_att_ptr_bad(uint32_t v)
{
    ++s_att_ptr_bad;
    if (s_att_ptr_bad <= 3u)
    {
        log_printf("[GUARD] att ptr bad: %x", (unsigned)v);
    }
}

uint32_t app_guard_att_ptr_count(void)
{
    return s_att_ptr_bad;
}

void app_bal_update(void)
{
    const att_state_t *a;
    alg_bal_out_t      o;
    float              phi_rad;
    float              dphi_rad;
    float              omega_rad;

    if (s_bal_on == 0u)
    {
        return;
    }

    bsp_sys_ctx = BSP_CTX_BAL;
    bal_ptr_guard();
    a = att_get();
    if (bsp_sys_ram_ptr_ok(a) == 0u)
    {
        app_guard_att_ptr_bad((uint32_t)(uintptr_t)a);
        a = &s_att_safe;
    }
    bal_p()->att_valid = a->valid;
    bal_p()->phi_deg   = a->theta_deg;
    bal_p()->dphi_dps  = a->theta_dot_dps;
    bal_p()->omega_rpm = a->wheel_rpm;

    if (a->valid == 0u)
    {
        /* No usable frame: report zero rather than a stale command. */
        bal_p()->valid     = 0u;
        bal_p()->saturated = 0u;
        bal_p()->deadband  = 0u;
        bal_p()->tau_mnm   = 0.0f;
        bal_p()->tau_kp_mnm = 0.0f;
        bal_p()->tau_kd_mnm = 0.0f;
        bal_p()->tau_kw_mnm = 0.0f;
        return;
    }

    phi_rad   = a->theta_deg * BAL_DEG2RAD;
    dphi_rad  = a->theta_dot_dps * BAL_DEG2RAD;
    omega_rad = a->wheel_rpm * BAL_RPM2RPS;

    /* 2026-11-07: every bench crash faults on the first read after this call
     * that uses the callee-saved r4, and r4 then holds what used to be &bal_p()->
     * r4 has to survive a call under AAPCS, so either the callee or an interrupt
     * taken inside it breaks that promise. Sample r4 either side of the call so
     * a violation becomes a counter instead of a bus fault, and reach the state
     * through the volatile aliases below so the register is not needed. */
    {
        uint32_t r4_before = bsp_sys_get_r4();

        alg_bal_step(phi_rad, dphi_rad, omega_rad, &o);

        if (bsp_sys_get_r4() != r4_before)
        {
            app_guard_r4_bad(r4_before, bsp_sys_get_r4());
        }
    }

    /* The slow zero trim runs once per attitude frame (5 ms), not once per main
     * loop pass: the low-pass that feeds it is a 0.5 s filter and a 20 kHz
     * call rate would make it depend on how busy the loop is.
     *
     * 2026-11-05: only while the loop is actually balancing. The trim has no
     * negative feedback when the body is held still (flat on the desk), so an
     * ungated adaptor wound it to the clamp and then shoved the cube over. */
    if ((s_bal_seq_prev != a->seq) && (s_bal_auto != 0u) &&
        (run_p()->engaged != 0u) &&
        (run_p()->state == BAL_ST_RUN))
    {
        s_bal_seq_prev = a->seq;
        /* The 2026-11-06 runaway happened because a frozen wheel reading was
         * trusted: the walk then integrated a constant and parked the trim on
         * its clamp. Only walk on a reading that is valid and still moving. */
        if (app_bal_walk_ready() != 0u)
        {
            alg_bal_adapt(BAL_FRAME_DT_S);
        }
    }

    bal_p()->valid       = 1u;
    bal_p()->tau_mnm     = o.tau_nm * 1000.0f;
    bal_p()->saturated   = o.saturated;
    bal_p()->deadband    = o.deadband;
    bal_p()->tau_ff_mnm  = o.tau_ff_nm * 1000.0f;
    bal_p()->trim_deg    = alg_bal_trim_deg();
    bal_p()->tau_kp_mnm  = s_bal_cfg.sign * s_bal_cfg.kp_nm_rad * phi_rad * 1000.0f;
    bal_p()->tau_kd_mnm  = s_bal_cfg.sign * s_bal_cfg.kd_nm_rps * dphi_rad * 1000.0f;
    bal_p()->tau_kw_mnm  = s_bal_cfg.sign * s_bal_cfg.kw_nm_rps * omega_rad * 1000.0f;
    bal_p()->n++;
}

uint8_t app_bal_set(uint8_t on)
{
    if (on == 0u)
    {
        s_bal_on      = 0u;
        bal_p()->on      = 0u;
        bal_p()->valid   = 0u;
        bal_p()->tau_mnm = 0.0f;
        bal_p()->n       = 0u;
        alg_bal_reset();
        return 0u;
    }

    if (alg_bal_ready() == 0u)
    {
        return 4u;
    }
    if (att_get()->valid == 0u)
    {
        return 2u;
    }

    s_bal_on = 1u;
    bal_p()->on = 1u;
    bal_p()->n  = 0u;
    alg_bal_reset();
    return 0u;
}

const app_bal_info_t *app_bal_info(void)
{
    return &s_bal;
}

float app_bal_trim_set(float deg)
{
    /* Manual override: a human just told the loop where the zero is, so stop
     * walking it. "bal auto on" gives it back. */
    float r = alg_bal_trim_set(deg);

    s_bal_auto = 0u;
    bal_p()->trim_deg = r;
    return r;
}

void app_bal_trim_learn(float deg)
{
    /* Used by the one-shot learn at ENGAGE + BAL_LEARN_MS: it sets the trim
     * without disarming the wheel-driven walk. */
    bal_p()->trim_deg = alg_bal_trim_set(deg);
}

void app_bal_auto_set(uint8_t on)
{
    s_bal_auto = (on != 0u) ? 1u : 0u;
}

float app_bal_walk_set(float k)
{
    return alg_bal_trim_wk_set(k);
}

float app_bal_walk_get(void)
{
    return alg_bal_trim_wk_get();
}

uint8_t app_bal_walk_ready(void)
{
    const mid_enc_t *e   = mid_enc_info();
    uint32_t         now = HAL_GetTick();

    if (e->frames != s_enc_frames_prev)
    {
        s_enc_frames_prev = e->frames;
        s_enc_frame_ms    = now;
    }
    if (e->valid == 0u)
    {
        return 0u;                       /* no angle has ever been decoded */
    }
    if ((uint32_t)(now - s_enc_frame_ms) > (uint32_t)BAL_WALK_STALE_MS)
    {
        return 0u;                       /* the reading stopped advancing  */
    }
    return 1u;
}

uint8_t app_bal_auto_get(void)
{
    return s_bal_auto;
}

uint8_t app_bal_snap(void)
{
    /* One fixed-point step towards "wheel at rest". At steady state the law
     * settles on
     *     omega_ss = (kp / kw) * (trim - phi_ss)
     * so the trim that centres the body on the true balance point is
     *     trim_new = trim - omega / (kp/kw in rpm per degree)
     * With the scaled gains that divisor is 573 rpm/deg, and the bench measured
     * 610, so one step lands within ~7 percent and two or three steps are enough.
     * Refused outside a running loop: moving the reference with the drive off
     * would only arm the next start with a lie. */
    float kp, kw, rpm_per_deg, trim, w, next;
    const app_bal_run_t *r = app_balance_info();

    if ((r == 0) || (r->state != BAL_ST_RUN) || (r->engaged == 0u))
    {
        return 2u;
    }
    if ((att_get() == 0) || (att_get()->valid == 0u))
    {
        return 3u;
    }
    kp = s_bal_cfg.kp_nm_rad;
    kw = s_bal_cfg.kw_nm_rps;
    if ((kp <= 0.0f) || (kw <= 0.0f))
    {
        return 4u;
    }
    w = bal_p()->omega_rpm;
    if (fabsf(w) > 1200.0f)
    {
        /* Above this the reading is more likely wrong than the machine is fast;
         * a step computed from it could throw the body. */
        return 5u;
    }
    rpm_per_deg = (kp / kw) / 6.0f;   /* (kp/kw) * (pi/180) * (60/2pi) */
    trim = alg_bal_trim_deg();
    next = trim - (w / rpm_per_deg);
    (void)app_bal_trim_set(next);
    return 0u;
}

float app_bal_trim_get(void)
{
    return alg_bal_trim_deg();
}

/* ------------------------------------------------------------------ */
/* Closed-loop orchestration (S6)                                      */
/*                                                                     */
/* Hard priority: TRIP > DISARMED > ARMED > RUN. Nothing here computes */
/* the law; it only decides whether the result may reach the motor,    */
/* how fast it may move, and when to cut it.                           */
/* ------------------------------------------------------------------ */

static uint32_t      s_run_t0;
static uint32_t      s_last_frames;
static float         s_tau_cmd;
static uint16_t      s_held_frames;  /* frames with a big torque but a static body */
static float         s_held_phi;     /* angle when the current held run began      */
static uint8_t       s_engage_wait;  /* armed, waiting for the operator to let go */
static uint8_t       s_engage_cnt;   /* consecutive frames above the rate gate  */

const char *app_balance_state_name(uint8_t st)
{
    switch (st)
    {
    case BAL_ST_DISARMED: return "disarmed";
    case BAL_ST_ARMED:    return "armed";
    case BAL_ST_RUN:      return "run";
    case BAL_ST_TRIP:     return "trip";
    default:              return "?";
    }
}

const char *app_balance_trip_name(uint8_t trip)
{
    switch (trip)
    {
    case BAL_TRIP_NONE:    return "none";
    case BAL_TRIP_TILT:    return "tilt";
    case BAL_TRIP_RATE:    return "rate";
    case BAL_TRIP_WHEEL:   return "wheel";
    case BAL_TRIP_STALE:   return "stale";
    case BAL_TRIP_FAULT:   return "fault";
    case BAL_TRIP_TIMEOUT: return "timeout";
    case BAL_TRIP_DRIVE:   return "drive";
    case BAL_TRIP_HELD:    return "held";
    default:               return "?";
    }
}

const app_bal_run_t *app_balance_info(void)
{
    return &s_run;
}

/* Latch a trip: torque first, power stage second. If the order were reversed,
 * the PWM-stop edge would still be preceded by a frame of stale iq. */
static void bal_trip(uint8_t why)
{
    s_run.state           = (uint8_t)BAL_ST_TRIP;
    s_run.trip            = why;
    s_tau_cmd             = 0.0f;
    s_run.tau_applied_mnm = 0.0f;
    app_torque_off();
    app_stop();
}

uint8_t app_balance_arm(uint8_t on)
{
    const att_state_t *a;

    if (on == 0u)
    {
        s_run.state           = (uint8_t)BAL_ST_DISARMED;
        s_run.trip            = (uint8_t)BAL_TRIP_NONE;
        s_tau_cmd             = 0.0f;
        s_engage_wait         = 0u;
        s_engage_cnt          = 0u;
        s_run.engaged         = 0u;
        s_run.tau_applied_mnm = 0.0f;
        app_torque_off();
        return 0u;
    }

    if ((s_run.state == BAL_ST_ARMED) || (s_run.state == BAL_ST_RUN))
    {
        return 6u;
    }

    a = att_get();
    if (a->valid == 0u)                              { return 1u; }
    if (fabsf(a->theta_deg) > BAL_ARM_DEG)           { return 2u; }
    if (fabsf(a->theta_dot_dps) > BAL_ARM_DPS)       { return 3u; }
    if (fabsf(a->wheel_rpm) > BAL_ARM_RPM)           { return 4u; }

    /* Drive readiness. app_torque_set(0) cannot answer this: a zero command is
     * short-circuited to "off" and always succeeds, so look at the state. */
    if (s_st.state != APP_ST_RUN)
    {
        s_run.drive_ret = 2u;
        return 5u;
    }
    if (s_st.mode != CTRL_FOC)
    {
        s_run.drive_ret = 4u;
        return 5u;
    }
    if (s_spd.on != 0u)
    {
        s_run.drive_ret = 6u;
        return 5u;
    }
    s_run.drive_ret = 0u;

    (void)app_bal_set(1u);       /* the orchestration needs the law running */

    s_run.state          = (uint8_t)BAL_ST_ARMED;
    s_run.trip           = (uint8_t)BAL_TRIP_NONE;
    s_run.run_ms         = 0u;
    s_run.frames         = 0u;
    s_run.sat_frames     = 0u;
    s_run.tau_peak_mnm   = 0.0f;
    s_run.phi_peak_deg   = 0.0f;
    s_run.omega_peak_rpm = 0.0f;
    s_run.rate_peak_dps  = 0.0f;
    s_tau_cmd            = 0.0f;
    s_held_frames        = 0u;
    s_engage_wait        = 1u;
    s_engage_cnt         = 0u;
    s_run.engaged        = 0u;
    s_run.tau_applied_mnm = 0.0f;
    s_run_t0             = HAL_GetTick();
    s_last_frames        = imu_ctx()->stat.frames;
    s_learn_done         = 0u;
    return 0u;
}

uint8_t app_balance_start_now(void)
{
    uint8_t rc;

    /* One button must be enough: bring the bridge up first. "mode 2" + "start"
     * only prepare the FOC loop (50% zero vector, current-zero calibration);
     * no torque flows until the balance law commands one. */
    if (s_st.mode != CTRL_FOC)
    {
        rc = app_set_mode((uint8_t)CTRL_FOC);
        if (rc != 0u)
        {
            s_run.drive_ret = rc;
            return 7u;                 /* could not select the FOC mode */
        }
    }
    if (s_st.state != APP_ST_RUN)
    {
        rc = app_start();
        if (rc != 0u)
        {
            s_run.drive_ret = rc;
            return 7u;                 /* could not start the bridge */
        }
    }

    /* The operator is holding the cube where it is supposed to balance, so make
     * that pose the zero. A guessed ATT_OFFSET_DEG sat 0.9 deg away from the
     * real equilibrium once, which is ~5 mN.m of constant torque and a wheel
     * that winds up to the trip line (sim/s6_disturb.py: 2 deg -> 1239 rpm). */
    att_zero();

    rc = app_balance_arm(1u);
    if (rc != 0u)
    {
        return rc;                     /* attitude/window/drive refused it */
    }

    /* Engage now instead of waiting for the release edge: the button was pressed
     * on purpose, and the ramp in app_balance_update() keeps the first commands
     * small so a slightly-off release pose cannot saturate the wheel. */
    s_engage_wait = 0u;
    s_engage_cnt  = 0u;
    s_run.engaged = 1u;
    s_run.state   = (uint8_t)BAL_ST_ARMED;
    s_run_t0      = HAL_GetTick();
    s_run.run_ms  = 0u;
    s_tau_cmd     = 0.0f;
    s_held_frames = 0u;
    s_learn_done  = 0u;
    return 0u;
}

void app_balance_update(void)
{
    const att_state_t *a;
    float   tau;
    uint8_t rc;

    if ((s_run.state == BAL_ST_DISARMED) || (s_run.state == BAL_ST_TRIP))
    {
        return;
    }

    a = att_get();
    s_run.run_ms = HAL_GetTick() - s_run_t0;

    /* Latching interlocks run on every loop, not only on a new frame: a dead
     * IMU must cut the torque immediately, not in up to 5 ms plus a timeout. */
    if (app_fault_info()->code != (uint8_t)APP_FAULT_NONE)
    {
        bal_trip((uint8_t)BAL_TRIP_FAULT);
        return;
    }
    if ((a->valid == 0u) || (imu_frame_age_ms() > (uint32_t)BAL_TRIP_STALE_MS))
    {
        bal_trip((uint8_t)BAL_TRIP_STALE);
        return;
    }
    if (fabsf(a->theta_deg) > BAL_TRIP_DEG)
    {
        bal_trip((uint8_t)BAL_TRIP_TILT);
        return;
    }
    if (fabsf(a->theta_dot_dps) > BAL_TRIP_DPS)
    {
        bal_trip((uint8_t)BAL_TRIP_RATE);
        return;
    }
    if (fabsf(a->wheel_rpm) > BAL_TRIP_RPM)
    {
        bal_trip((uint8_t)BAL_TRIP_WHEEL);
        return;
    }
    if ((BAL_MAX_MS > 0u) && (s_run.run_ms > (uint32_t)BAL_MAX_MS))
    {
        bal_trip((uint8_t)BAL_TRIP_TIMEOUT);
        return;
    }

    /* 5 ms zero-order hold: one command per attitude frame. Applying the same
     * frame 375 times would turn kd into a constant offset (an implicit
     * friction term) and is exactly what the L1 model does not contain.
     * The counter is the IMU frame counter, not att_state.seq: mid_att bumps
     * its own seq on every call, so the main loop would look like fresh data. */
    {
        uint32_t frames = imu_ctx()->stat.frames;
        if (frames == s_last_frames)
        {
            return;
        }
        s_last_frames = frames;
    }

    app_bal_update();                  /* recompute with this frame */
    tau = bal_p()->tau_mnm;

    /* Engage-on-release. While waiting we command nothing: fighting the hand
     * would only spin the wheel (measured 403 rpm in 265 ms). The first frame
     * that shows motion means the operator let go, so start from zero there and
     * restart the run window - the time spent waiting is not runway. */
    if (s_engage_wait != 0u)
    {
        if (fabsf(a->theta_dot_dps) >= BAL_ENGAGE_DPS)
        {
            if (s_engage_cnt < 0xFFu) { s_engage_cnt++; }
        }
        else
        {
            s_engage_cnt = 0u;             /* a wobble never lasts 3 frames */
        }

        if (s_engage_cnt >= BAL_ENGAGE_FRAMES)
        {
            s_engage_wait = 0u;
            s_run.engaged = 1u;
            s_run_t0      = HAL_GetTick();
            s_tau_cmd     = 0.0f;
        }
        else
        {
            s_run.state           = (uint8_t)BAL_ST_ARMED;
            s_run.tau_applied_mnm = 0.0f;
            return;
        }
    }
    /* One-shot zero learn. After BAL_LEARN_MS the loop has settled onto the real
     * balance point, so the angle it is holding IS the zero error (measured
     * 2026-11-06: phi -2.32 deg <-> wheel 1340 rpm). Take it once and let the
     * wheel-driven walk handle the rest; skip it when the loop already sits
     * inside a fraction of a degree of zero. */
    if (s_learn_done == 0u)
    {
        if (s_run.run_ms >= (uint32_t)BAL_LEARN_MS)
        {
            s_learn_done = 1u;
            if ((fabsf(a->theta_deg) > 0.15f) && (fabsf(a->theta_deg) < BAL_TRIM_GATE_DEG))
            {
                app_bal_trim_learn(a->theta_deg);
                log_printf("[BAL] zero learned %.2f deg (wheel %.0f rpm)",
                           (double)a->theta_deg, (double)a->wheel_rpm);
            }
        }
    }

    /* Held-cube detector. A free cube cannot refuse a torque: a large command
     * with a static body means the cube rests on something, and the wheel is the
     * only thing that will move. 28 mN.m for 0.35 s already saturates the wheel
     * window, so trip on it quickly and label it. */
    if ((s_run.run_ms >= (uint32_t)BAL_HELD_GRACE_MS) &&
        (fabsf(tau) >= BAL_HELD_MNM) && (fabsf(a->theta_deg) >= BAL_HELD_DEG) &&
        (fabsf(a->theta_dot_dps) <= BAL_HELD_DPS))
    {
        /* A slow oscillation also crosses zero rate. A hand does not move the
         * angle at all, so restart the count whenever the body has moved since
         * the run began (2026-11-11: a 2 s wobble tripped this detector). */
        if (s_held_frames == 0u)
        {
            s_held_phi = a->theta_deg;
        }
        else if (fabsf(a->theta_deg - s_held_phi) > BAL_HELD_PHI_DEG)
        {
            s_held_frames = 0u;
            s_held_phi    = a->theta_deg;
        }
        if (s_held_frames < 0xFFFFu) { s_held_frames++; }
        if (((uint32_t)s_held_frames * 5u) >= (uint32_t)BAL_HELD_MS)
        {
            bal_trip((uint8_t)BAL_TRIP_HELD);
            return;
        }
    }
    else
    {
        s_held_frames = 0u;
    }

    if (tau > (s_tau_cmd + BAL_SLEW_MNM_STEP)) { tau = s_tau_cmd + BAL_SLEW_MNM_STEP; }
    if (tau < (s_tau_cmd - BAL_SLEW_MNM_STEP)) { tau = s_tau_cmd - BAL_SLEW_MNM_STEP; }

    /* Engage ramp. A button start engages while the hand is still on the cube,
     * so the pose can be a fraction of a degree off zero -> kp alone asks for
     * 20+ mN.m (22.9 mN.m/deg). Growing the ceiling over BAL_ENGAGE_RAMP_MS
     * keeps that transient from turning into a full-scale wheel spin. */
    {
        uint32_t t_eng   = (uint32_t)(HAL_GetTick() - s_run_t0);
        float    ceiling = BAL_MAX_MNM;

        if (t_eng < (uint32_t)BAL_ENGAGE_RAMP_MS)
        {
            ceiling = BAL_MAX_MNM * ((float)t_eng / (float)BAL_ENGAGE_RAMP_MS);
            if (ceiling < BAL_SLEW_MNM_STEP) { ceiling = BAL_SLEW_MNM_STEP; }
        }
        if (tau > ceiling)  { tau = ceiling; }
        if (tau < -ceiling) { tau = -ceiling; }
    }

    rc = app_torque_set(tau);
    s_run.drive_ret = rc;
    if (rc != 0u)
    {
        bal_trip((uint8_t)BAL_TRIP_DRIVE);
        return;
    }

    s_tau_cmd             = tau;
    s_run.state           = (uint8_t)BAL_ST_RUN;
    s_run.tau_applied_mnm = tau;
    s_run.frames++;
    if (bal_p()->saturated != 0u)                    { s_run.sat_frames++; }
    if (fabsf(tau) > s_run.tau_peak_mnm)          { s_run.tau_peak_mnm = fabsf(tau); }
    if (fabsf(a->theta_deg) > s_run.phi_peak_deg) { s_run.phi_peak_deg = fabsf(a->theta_deg); }
    if (fabsf(a->wheel_rpm) > s_run.omega_peak_rpm) { s_run.omega_peak_rpm = fabsf(a->wheel_rpm); }
    if (fabsf(a->theta_dot_dps) > s_run.rate_peak_dps) { s_run.rate_peak_dps = fabsf(a->theta_dot_dps); }
}

/* Init                                                                */
/* ------------------------------------------------------------------ */
void app_init(void)
{
    /* Publish the backup domain first: the fault record from the previous run
     * lives there and the banner prints it a few lines further down. */
    bsp_crash_init();
    app_wp_arm();
    bsp_stack_paint();   /* record how deep the stack ever gets */

    /* S10.1: shadow observer. Starts disabled; "obs on" arms it. */
    alg_obs_init(&s_obs, MOTOR_R_OHM, MOTOR_L_H, OBS_LPF_HZ,
                  OBS_PLL_WN_HZ, OBS_W_MAX_RADS);
    s_obs_on = 0u;

    /* S10.4: open-loop I/f ramp (also configured on every start). */
    alg_if_init(&s_if, IF_ACC_HZ_S, IF_I_AMP_A, IF_F_MIN_HZ);
    alg_if_set_align_time(&s_if, IF_ALIGN_MS / 1000.0f);
    alg_if_set_ramp_time(&s_if, IF_I_RAMP_MS / 1000.0f);   /* S10.8 soft start */
    alg_if_set_xfade(&s_if, IF_XFADE_MS / 1000.0f);

    /* S10.2: calibration measured on hardware, see board_cfg.h. */
    s_obs.comp      = OBS_COMP_MODE;
    s_obs.delay_s   = OBS_DELAY_US * 0.000001f;
    s_obs.theta_off = OBS_TRIM_DEG * 0.0174532925199433f;
    s_st.uptime_ms    = 0u;
    s_st.state        = APP_ST_INIT;
    s_st.mode         = CTRL_IDLE;
    s_st.key_cnt      = 0u;
    s_st.loop_khz     = 0u;
    s_st.wave_on      = 0u;
    s_st.wave_rate_hz = DBG_WAVE_RATE_HZ;
    s_st.wave_frames  = 0u;
    s_st.wave_drop    = 0u;
    s_st.rx_drop      = 0u;
    s_st.tx_pending   = 0u;

    s_loop_cnt       = 0u;
    s_t_1ms          = HAL_GetTick();
    s_t_1k           = s_t_1ms;
    s_t_wave         = s_t_1ms;
    s_t_enc          = s_t_1ms;
    s_wave_period_ms = (uint16_t)(1000u / s_st.wave_rate_hz);
    s_adc_seen       = 0u;

    /* FOC defaults; the "pi" command changes them at runtime. */
    alg_foc_init(&s_foc, FOC_VDC_NOMINAL, FOC_M_MAX,
                 FOC_KP, FOC_KI, FOC_V_LIM);
    alg_foc_set_ff(&s_foc, FOC_K_EMF, FOC_L_H, FOC_FF_GAIN);
    s_foc.ff_lo = FOC_FF_LO_W;
    s_foc.ff_hi = FOC_FF_HI_W;

    s_prot.oc_a          = FOC_I_TRIP_A;
    s_prot.overspeed_rpm = PROT_OVERSPEED_RPM;
    s_prot.overspeed_ms  = PROT_OVERSPEED_MS;
    s_prot.stall_rpm     = PROT_STALL_RPM;
    s_prot.stall_iq_a    = PROT_STALL_IQ_A;
    s_prot.stall_ms      = PROT_STALL_MS;
    s_prot.uv_v          = VBUS_UV_V;
    s_prot.ov_v          = VBUS_OV_V;
    s_prot.vbus_ms       = PROT_VBUS_MS;

    s_fault.code      = (uint8_t)APP_FAULT_NONE;
    s_fault.value     = 0.0f;
    s_fault.uptime_ms = 0u;
    { uint8_t i; for (i = 0u; i < (uint8_t)APP_FAULT_COUNT; i++) {
        s_fault.count[i] = 0u; } }

    bsp_vbus_init();

    s_spd_ramp_hz = SPD_RAMP_RPM_S;
    alg_spd_init(&s_spd, SPD_KP, SPD_KI, FOC_I_MAX_A,
                 SPD_FF_BASE, SPD_FF_SLOPE, SPD_KICK, SPD_KICK_MS);
    /* Must come AFTER init: init resets gain_hi_scale to 1.0 (no
     * scheduling), which silently disabled the feature earlier. */
    alg_spd_set_sched(&s_spd, SPD_GAIN_LO_RPM, SPD_GAIN_HI_RPM, SPD_GAIN_HI_SCALE);
    alg_spd_set_lpf(&s_spd, SPD_LPF_LO_RPM, SPD_LPF_HI_RPM, SPD_LPF_LO_K, SPD_LPF_HI_K);
    s_spd.ff_acc = SPD_FF_ACC;
    /* #167: clamp the acceleration feedforward to the ramp rate it is meant
     * to compensate (see alg_spd_step). */
    s_spd.acc_max = s_spd_ramp_hz * 0.104719755f;
    s_spd_div = 0u;
    s_foc.ff_lo = FOC_FF_LO_W;
    s_foc.ff_hi = FOC_FF_HI_W;

    alg_vf_init(&s_vf);
    s_vf_info.on       = 0u;
    s_vf_info.dir      = 0u;
    s_vf_info.f_hz     = 0.0f;
    s_vf_info.f_target = 0.0f;
    s_vf_info.m        = 0.0f;
    s_vf_info.i_max    = 0.0f;
    s_i_fold_cnt       = 0u;
    s_i_trip_cnt       = 0u;
    app_phase_reset();

    /* ---- peripherals ---- */
    bsp_sys_init();          /* ART: flash prefetch + I-cache, must be first */
    /* Eager FP context saving. With lazy stacking on, FPCCR.LSPACT stays set
     * after exception entries that do not need the FP context, and FPCAR then
     * points into the interrupted frame: the next FP instruction dumps s0-s15
     * on top of live locals. Every bench crash showed exactly that signature
     * (LSPACT=1, FPCAR = pre-fault SP, saved registers holding float bit
     * patterns). Cost of eager saving is ~30 cycles and 0x68 bytes per entry. */
    bsp_sys_fpu_eager(1u);
    bsp_prof_init();         /* DWT cycle counter, must be first */
    s_trig_status = mid_trig_init();   /* LUT: always OK; CORDIC: self-test */
    mid_enc_init();                    /* MT6701: zero/dir from flash */
    bsp_gpio_init();
    bsp_uart_init();
    imu_init();              /* N100 Mini on USART1: arm RX DMA, no TX yet */
    att_cfg_load();          /* S2: frame axis, zero, range, timeout     */
    att_init(&s_att_cfg);
    bal_cfg_load();          /* S4: gains, mounting sign, ceiling        */
    alg_bal_init(&s_bal_cfg);
    bsp_pwm_init();          /* counter runs, six outputs stay off (MOE=0) */
    mid_current_init();      /* offsets to nominal, needs calibration */
    bsp_adc_init();          /* arm the injected group (triggered by TIM1_CC4) */
    /* Restore the zero-drift calibration from flash: with a valid record there
     * is nothing to calibrate and "start" goes straight to work. */
    (void)mid_current_load();
    app_cmd_init();

    /* ---- boot banner ---- */
    log_printf("");
    log_printf("=============================================");
    log_printf(" %s", BOARD_NAME);
    log_printf(" build = %s %s #%u", __DATE__, __TIME__, (unsigned)FW_BUILD_ID);
    log_printf(" core = %u MHz, FOC = %u Hz (Ts=%u us)",
               (unsigned)(SYS_CORE_CLK_HZ / 1000000u),
               (unsigned)FOC_FREQ_HZ,
               (unsigned)FOC_TS_US);
    log_printf(" pwm  = %u Hz (ARR=%u, DTG=%u)",
               (unsigned)PWM_FREQ_HZ_CALC, (unsigned)PWM_ARR, (unsigned)PWM_DTG_REG);
    log_printf(" trig = %s%s", mid_trig_name(),
               (s_trig_status != 0u) ? " (SELF TEST FAILED)" : "");
    log_printf(" enc  = %s, off %.1f deg, dir %u%s",
               (mid_enc_info()->src == ENC_SRC_FLASH) ? "flash" : "default",
               (double)mid_enc_info()->zero_deg,
               (unsigned)mid_enc_info()->dir,
               (bsp_ssi_cfg_error() != 0u) ? " (DMA CH MISMATCH, SPI DISABLED)" : "");
    log_printf(" cur  = Rs=%u mOhm x%u, %u uA/LSB, +-%u mA",
               (unsigned)(RSHUNT_OHM * 1000.0f + 0.5f), (unsigned)INA240_GAIN,
               (unsigned)(CUR_A_PER_LSB * 1000000.0f + 0.5f),
               (unsigned)(CUR_RANGE_A * 1000.0f + 0.5f));
    {
        /* Power-on self-check result: where the zero-drift offsets came from.
         * With a valid record nothing is calibrated later; without one the
         * first "start" calibrates and writes the record back. */
        const current_sample_t *cs = mid_current_get();

        if (cs->ready != 0u) {
            log_printf(" cal  = flash ok, off %u/%u/%u, gen=%u",
                       (unsigned)cs->offset[0], (unsigned)cs->offset[1],
                       (unsigned)cs->offset[2], (unsigned)cs->gen);
        } else {
            log_printf(" cal  = none, will calibrate at first start");
        }
    }
    log_printf(" uart = %u baud (USART2 -> CH340N), VOFA+ = FireWater%s",
               (unsigned)DBG_UART_BAUD,
               (bsp_uart_cfg_error() != 0u) ? " (CFG MISMATCH)" : "");
    log_printf(" imu  = USART1 %u baud, %s",
               (unsigned)IMU_UART_BAUD,
               (imu_stream_ok() != 0u) ? "streaming" : "no frame yet");
    log_printf(" att  = %s axis, sign %s, zero %.2f deg, range %.0f deg, timeout %u ms",
               (ATT_AXIS == ATT_AXIS_ROLL) ? "roll" : "pitch",
               (ATT_SIGN < 0.0f) ? "-1" : "+1",
               (double)ATT_OFFSET_DEG, (double)ATT_RANGE_DEG,
               (unsigned)ATT_TIMEOUT_MS);

    /* Fault record left by the previous run, plus the reason this one started.
     * Both are wiped afterwards so the next reset reads clean. */
    bsp_crash_report();

    log_printf("---------------------------------------------");
    log_printf(" KEY -> LED toggle");
    log_printf(" type 'help' for the command list");
    log_printf("=============================================");

    /* CubeMX regeneration silently restores TIM1.DeadTime from the .ioc; the
     * effective dead time is identical (60ns from the EG3033 either way) but a
     * mismatch means tim.c and board_cfg.h disagree, so say it out loud. */
    if (bsp_pwm_get_dtg() != (uint16_t)PWM_DTG_REG) {
        log_printf("ERR bdtr dtg=%u expected=%u (tim.c / CubeMX)",
                   (unsigned)bsp_pwm_get_dtg(), (unsigned)PWM_DTG_REG);
    }

    s_st.state = APP_ST_IDLE;
}

/* ------------------------------------------------------------------ */
/* ALIGN: pull the rotor onto the phase A axis to find the electrical  */
/* zero. A fixed voltage vector at theta = 0 makes the stator field    */
/* point along phase A; the permanent magnet turns to follow it.       */
/* ------------------------------------------------------------------ */
#define ALIGN_TWO_PI   6.283185307179586f

static void align_finish(uint8_t fail)
{
    bsp_pwm_disable();                   /* six outputs low before anything else */
    bsp_pwm_set_all_duty(PWM_START_DUTY);

    s_align.active = 0u;
    s_align.done   = 1u;
    s_align.fail   = fail;
    s_st.state     = (fail == 2u) ? APP_ST_FAULT : APP_ST_IDLE;

    if (fail == 0u) {
        log_printf("OK align done, zero %.2f deg, err %.2f (fwd %.2f rev %.2f)",
                   (double)s_align.zero_deg, (double)s_align.err_deg,
                   (double)s_align.err_fwd, (double)s_align.err_rev);
    } else if (fail == 1u) {
        log_printf("ERR align timeout, rotor did not settle");
    } else if (fail == 2u) {
        log_printf("ERR align overcurrent, trip");
    } else {
        log_printf("ERR align encoder or flash");
    }
}

static void task_align(void)
{
    const mid_enc_t        *e = mid_enc_info();
    const current_sample_t *cs;
    uint32_t now = HAL_GetTick();
    uint32_t el  = (uint32_t)(now - s_align_t0);
    uint32_t t_fwd_end = (uint32_t)ALIGN_RAMP_MS + (uint32_t)ALIGN_SWEEP_MS;
    uint32_t t_rev_end = t_fwd_end + (uint32_t)ALIGN_SWEEP_MS;
    float    m;
    float    theta = 0.0f;
    float    duty[3];
    float    imax;
    float    t;
    float    revs = ALIGN_SWEEP_HZ * ((float)ALIGN_SWEEP_MS / 1000.0f);
    uint8_t  accumulate = 0u;
    uint8_t  i;

    if (s_align.active == 0u) {
        return;
    }

    /* Current guard first: the vector is the thing that can hurt. */
    cs = mid_current_get();
    imax = fabsf(cs->ia);
    t = fabsf(cs->ib); if (t > imax) { imax = t; }
    t = fabsf(cs->ic); if (t > imax) { imax = t; }
    if (imax > ALIGN_I_MAX_A) {
        align_finish(2u);
        return;
    }
    /* ---- S10.0c: open-loop pre-position (no encoder involved) ----
     * Hold theta = 0 while the magnitude ramps: the rotor is pulled onto
     * that axis and stays there. Nothing to measure, so this runs without
     * any encoder frame and without the sweep. ---- */
    if (s_align_ol != 0u) {
        if (el < (uint32_t)ALIGN_OL_MS) {
            m = ALIGN_M_START +
                ((ALIGN_M_MAX - ALIGN_M_START) * (float)el / (float)ALIGN_OL_MS);

            alg_svpwm(0.0f, m, duty);
            for (i = 0u; i < 3u; i++) {
                if (duty[i] < 0.0f) { duty[i] = 0.0f; }
                if (duty[i] > 1.0f) { duty[i] = 1.0f; }
            }
            bsp_pwm_set_duty(duty[0], duty[1], duty[2]);
            s_align.m = m;
            return;
        }

        /* The rotor now sits on the theta = 0 axis, which is all an
         * encoder-less drive needs: the forced frame is in phase by
         * construction.
         *
         * Deliberately NOT writing the encoder zero here: measured on
         * hardware (#128) the aligned position is ~5 deg electrical short
         * of the vector (cogging/friction equilibrium), while the S5 sweep
         * zero is accurate to +-5.3 deg. Storing the aligned position moved
         * zero_deg 131.91 -> 234.01 and would have made the encoder path
         * worse. Report the offset instead - the I/f hand-over needs it. */
        s_align_ol = 0u;
        if (mid_enc_info()->valid != 0u) {
            float off = mid_enc_info()->elec_deg;

            log_printf("OK align ol done, forced frame offset %.1f deg (elec)",
                       (double)off);
        } else {
            log_printf("OK align ol done, rotor at theta=0 (no encoder)");
        }
        align_finish(0u);
        return;
    }

    /* ---- magnitude ramp ---- */
    if (el < (uint32_t)ALIGN_RAMP_MS) {
        m = ALIGN_M_START +
            ((ALIGN_M_MAX - ALIGN_M_START) * (float)el / (float)ALIGN_RAMP_MS);
        theta = 0.0f;
    }
    /* ---- forward sweep: 0 -> revs, accumulate after the transient ---- */
    else if (el < t_fwd_end) {
        float ts = (float)(el - (uint32_t)ALIGN_RAMP_MS) / 1000.0f;

        m = ALIGN_M_MAX;
        theta = ALIGN_TWO_PI * ALIGN_SWEEP_HZ * ts;
        if ((uint32_t)(el - (uint32_t)ALIGN_RAMP_MS) > (uint32_t)ALIGN_SETTLE_MS) {
            accumulate = 1u;
        }
    }
    /* ---- reverse sweep: revs -> 0 ---- */
    else if (el < t_rev_end) {
        float ts = (float)(el - t_fwd_end) / 1000.0f;

        m = ALIGN_M_MAX;
        theta = ALIGN_TWO_PI * (revs - (ALIGN_SWEEP_HZ * ts));
        if (theta < 0.0f) {
            theta += ALIGN_TWO_PI;
        }
        if ((uint32_t)(el - t_fwd_end) > (uint32_t)ALIGN_SETTLE_MS) {
            accumulate = 1u;
        }
    }
    /* ---- done: apply the averaged offset ---- */
    else {
        m = ALIGN_M_MAX;
        theta = 0.0f;
    }

    while (theta >= ALIGN_TWO_PI) { theta -= ALIGN_TWO_PI; }
    while (theta < 0.0f)          { theta += ALIGN_TWO_PI; }

    alg_svpwm(theta, m, duty);
    for (i = 0u; i < 3u; i++) {
        if (duty[i] < 0.0f) { duty[i] = 0.0f; }
        if (duty[i] > 1.0f) { duty[i] = 1.0f; }
    }
    bsp_pwm_set_duty(duty[0], duty[1], duty[2]);
    s_align.m = m;

    /* Abort when no NEW encoder frame arrives (frames stops advancing): the
     * "valid" flag only says that a frame arrived once, not that it still does. */
    if (e->frames == s_align_frames0) {
        if (el > 300u) {
            align_finish(3u);
        }
        return;
    }

    /* ---- accumulate the tracking error ---- */
    if (accumulate != 0u) {
        float enc_rad = e->elec_deg * 0.0174532925199433f;
        float err = enc_rad - theta;

        while (err >  3.14159265f) { err -= ALIGN_TWO_PI; }
        while (err < -3.14159265f) { err += ALIGN_TWO_PI; }

        s_align_err_sum += err;
        s_align_err_n++;
        if (el < t_fwd_end) {
            s_align_fwd_sum += err;
            s_align_fwd_n++;
        } else {
            s_align_rev_sum += err;
            s_align_rev_n++;
        }
    }

    if (el >= t_rev_end) {
        if (s_align_err_n == 0u) {
            align_finish(1u);
            return;
        }

        /* mean electrical error -> mechanical correction */
        {
            float err_elec_deg = (s_align_err_sum / (float)s_align_err_n)
                                 * 57.29577951308232f;
            float corr = err_elec_deg / (float)ENC_POLE_PAIRS;
            float nz   = mid_enc_info()->zero_deg + corr;

            s_align.err_deg = err_elec_deg;
            s_align.n       = s_align_err_n;
            s_align.err_fwd = (s_align_fwd_n != 0u)
                              ? (s_align_fwd_sum / (float)s_align_fwd_n)
                                * 57.29577951308232f : 0.0f;
            s_align.err_rev = (s_align_rev_n != 0u)
                              ? (s_align_rev_sum / (float)s_align_rev_n)
                                * 57.29577951308232f : 0.0f;

            (void)mid_enc_zero_set(nz);
            s_align.zero_deg = mid_enc_info()->zero_deg;
            align_finish(0u);
        }
        return;
    }

    if (el > ALIGN_TIMEOUT_MS) {
        align_finish(1u);
    }
}
uint8_t app_align_start(void)
{
    /* Order matters: starting an alignment already moves the state away
     * from RUN, so checking the state first would make "already aligning"
     * (3) unreachable. */
    if (s_align.active != 0u) {
        return 3u;
    }
    if (s_st.state != APP_ST_RUN) {
        return 2u;                       /* start the bridge first */
    }

    /* The vector itself is written by task_align every frame (it ramps), so
     * nothing to do here beyond arming the state. */

    s_align_ol         = 0u;
    s_align.active     = 1u;
    s_align.done       = 0u;
    s_align.fail       = 0u;
    s_align.zero_deg   = mid_enc_info()->zero_deg;
    s_align_t0         = HAL_GetTick();
    s_align_err_sum    = 0.0f;
    s_align_err_n      = 0u;
    s_align_fwd_sum    = 0.0f;
    s_align_fwd_n      = 0u;
    s_align_rev_sum    = 0.0f;
    s_align_rev_n      = 0u;
    s_align_frames0    = mid_enc_info()->frames;
    s_align.err_deg    = 0.0f;
    s_align.n          = 0u;

    s_st.state = APP_ST_ALIGN;
    return 0u;
}

uint8_t app_align_ol_start(void)
{
    /* Order matters: starting an alignment already moves the state away
     * from RUN, so checking the state first would make "already aligning"
     * (3) unreachable. */
    if (s_align.active != 0u) {
        return 3u;
    }
    if (s_st.state != APP_ST_RUN) {
        return 2u;                       /* start the bridge first */
    }

    /* Arm the same state as the sweep alignment, plus the open-loop flag:
     * task_align then holds theta = 0 and skips the sweep and every
     * encoder check (S10.0c). */
    s_align_ol         = 1u;
    s_align.active     = 1u;
    s_align.done       = 0u;
    s_align.fail       = 0u;
    s_align.zero_deg   = mid_enc_info()->zero_deg;
    s_align.err_deg    = 0.0f;
    s_align.n          = 0u;
    s_align.m          = 0.0f;
    s_align_t0         = HAL_GetTick();
    s_align_frames0    = mid_enc_info()->frames;

    s_st.state = APP_ST_ALIGN;
    return 0u;
}

const app_align_t *app_align_info(void)
{
    return &s_align;
}

/* ------------------------------------------------------------------ */
/* State control                                                       */
/* ------------------------------------------------------------------ */
uint8_t app_start(void)
{
    if (s_st.state == APP_ST_FAULT) {
        return 1u;                       /* FAULT blocks start */
    }
    if (s_st.mode == CTRL_IDLE) {
        return 2u;                       /* no control mode selected */
    }
    if (s_st.mode == CTRL_FOC) {
        /* Only the STATE is cleared here. The gains are set once in
         * app_init(), so "pi <kp> <ki>" survives start/stop: that is the
         * order a person tunes in (set the gains, then start). */
        alg_foc_reset(&s_foc);
        s_foc_info.id_ref  = 0.0f;
        s_foc_info.iq_ref  = 0.0f;
        s_foc_info.ready   = 0u;
        s_foc_trip_cnt     = 0u;
    }

    /* The ADC trigger (TIM1_CC4) only reaches the ADC while MOE=1, so the
     * zero-drift calibration can only run with the bridge enabled. Start at
     * the 50% zero vector: no phase-to-phase voltage, hence no current. The
     * 256 calibration frames take 12.8ms and complete in the background. */
    bsp_pwm_set_all_duty(PWM_START_DUTY);
    bsp_pwm_enable();                    /* preload duty, CCxE, then MOE */

    /* S7.6: protections ignore the first milliseconds. */
    s_prot_start_ms = HAL_GetTick();
    s_prot_enc_cnt  = 0u;
    s_prot_enc_zero_cnt = 0u;
    s_prot_enc_crc_cnt  = 0u;
    s_prot_bad_seen = mid_enc_info()->frames;
    s_coast         = 0u;                /* #168: start from the zero vector */
    s_coast_ticks   = 0u;
    if (mid_current_get()->ready == 0u) {
        mid_current_restart_cal();
    }
    s_st.state = APP_ST_RUN;
    return 0u;
}

uint8_t app_stop(void)
{
    /* S10: a free-running PLL must not survive the transition. */
    mid_enc_pll_clear();
    s_sfoc     = 0u;
    s_ho_state = 0u;
    s_spd_lp   = 0.0f;
    s_torque_mnm = 0.0f;

    alg_vf_stop(&s_vf);                  /* drop the V/f ramp first */
    alg_foc_reset(&s_foc);
    app_spd_off();
    s_step_phase       = 0u;             /* cancel any running step */
    s_sstep_phase      = 0u;
    /* Acknowledge the latched fault, keep the per-type counters. */
    s_fault.code       = (uint8_t)APP_FAULT_NONE;
    s_fault.value      = 0.0f;
    s_foc_info.id_ref  = 0.0f;
    s_foc_info.iq_ref  = 0.0f;
    foc_info_clear_meas();               /* #167: no stale display */
    /* #172: a stop or a fault must also give up the angle source. They used
     * to survive, so the next "start" resumed sensorless on a stale observer
     * angle: measured iq_ref 0 while the current loop pinned vq at the rail
     * and drew 0.9 A with the shaft still - the motor looked dead until
     * another stop/start pair. */
    s_sfoc         = 0u;
    s_ho_state     = 0u;
    s_ho_w         = 0.0f;
    s_ho_cnt       = 0u;
    s_ho_c0        = 1.0f;
    s_lost_ms      = 0u;
    s_rail_ms      = 0u;
    s_if_rpm_tgt   = 0.0f;
    s_coast        = 0u;
    s_coast_ticks  = 0u;
    alg_if_stop(&s_if);
    alg_obs_reset(&s_obs, 0.0f);
    s_foc_info.ready   = 0u;
    s_foc_trip_cnt     = 0u;
    s_vf_info.on       = 0u;
    s_vf_info.f_hz     = 0.0f;
    s_vf_info.f_target = 0.0f;
    s_vf_info.m        = 0.0f;
    s_i_fold_cnt       = 0u;
    s_i_trip_cnt       = 0u;

    bsp_pwm_disable();                   /* MOE=0 first: six outputs low */
    bsp_pwm_set_all_duty(PWM_START_DUTY);
    s_align.active = 0u;                 /* abort any alignment */
    s_st.state = APP_ST_IDLE;            /* also clears FAULT */
    return 0u;
}

uint8_t app_set_mode(uint8_t m)
{
    if (m > (uint8_t)CTRL_FOC) { return 1u; }
    if ((s_st.state == APP_ST_RUN) || (s_st.state == APP_ST_ALIGN)) {
        return 2u;                       /* not while running or aligning */
    }
    s_st.mode  = m;
    s_st.state = APP_ST_IDLE;
    return 0u;
}

/* ------------------------------------------------------------------ */
/* Open-loop V/f (S4)                                                  */
/* ------------------------------------------------------------------ */
uint8_t app_vf_set(float hz)
{
    if (s_st.state != APP_ST_RUN) {
        return 2u;                       /* bridge not running */
    }
    if (mid_current_get()->ready == 0u) {
        return 3u;                       /* zero drift still being measured */
    }
    if (hz <= 0.0f) {
        return app_vf_off();             /* "0 Hz" means stop modulating */
    }
    alg_vf_start(&s_vf, hz);
    s_i_fold_cnt = 0u;
    s_i_trip_cnt = 0u;

    /* Publish immediately: "pwm" is refused while vf is on, and that check
     * must not wait for the next ADC frame. */
    s_vf_info.on       = 1u;
    s_vf_info.dir      = s_vf.dir;
    s_vf_info.f_target = s_vf.f_target;
    s_vf_info.f_hz     = s_vf.f_hz;
    s_vf_info.m        = 0.0f;
    return 0u;
}

uint8_t app_vf_off(void)
{
    alg_vf_stop(&s_vf);
    s_vf_info.on       = 0u;
    s_vf_info.f_hz     = 0.0f;
    s_vf_info.f_target = 0.0f;
    s_vf_info.m        = 0.0f;
    s_vf_info.i_max    = 0.0f;
    if (s_st.state == APP_ST_RUN) {
        bsp_pwm_set_all_duty(PWM_START_DUTY);
    }
    return 0u;
}

/* ------------------------------------------------------------------ */
/* FOC current loop control (S6)                                       */
/* ------------------------------------------------------------------ */
void app_fault_raise(uint8_t code, float value)
{
    /* Verify the bridge really goes quiet before anything else. */
    bsp_pwm_disable();
    bsp_pwm_set_all_duty(PWM_START_DUTY);

    app_spd_off();
    /* #172: a stop or a fault must also give up the angle source. They used
     * to survive, so the next "start" resumed sensorless on a stale observer
     * angle: measured iq_ref 0 while the current loop pinned vq at the rail
     * and drew 0.9 A with the shaft still - the motor looked dead until
     * another stop/start pair. */
    s_sfoc         = 0u;
    s_ho_state     = 0u;
    s_ho_w         = 0.0f;
    s_ho_cnt       = 0u;
    s_ho_c0        = 1.0f;
    s_lost_ms      = 0u;
    s_rail_ms      = 0u;
    s_if_rpm_tgt   = 0.0f;
    s_coast        = 0u;
    s_coast_ticks  = 0u;
    alg_if_stop(&s_if);
    alg_obs_reset(&s_obs, 0.0f);
    s_foc_info.id_ref  = 0.0f;
    s_foc_info.iq_ref  = 0.0f;
    foc_info_clear_meas();               /* #167: no stale display */
    s_step_phase       = 0u;
    alg_foc_reset(&s_foc);

    /* S10: the bridge is about to go quiet, so the speed estimate must not
     * keep free-running (see mid_enc_pll_clear). */
    mid_enc_pll_clear();

    s_fault.code      = code;
    s_fault.value     = value;
    s_fault.uptime_ms = HAL_GetTick();

    /* "stop" clears s_fault, which used to erase the only evidence of what
     * went wrong. Keep a copy that survives until the next fault. */
    s_last_fault = s_fault;
    if (code < (uint8_t)APP_FAULT_COUNT) {
        s_fault.count[code]++;
    }

    s_st.state = APP_ST_FAULT;

    switch (code) {
    case (uint8_t)APP_FAULT_OVERCURRENT:
        log_printf("ERR fault overcurrent I=%d mA", (int)(value * 1000.0f));
        break;
    case (uint8_t)APP_FAULT_OVERSPEED:
        log_printf("ERR fault overspeed %d rpm", (int)value);
        break;
    case (uint8_t)APP_FAULT_STALL:
        log_printf("ERR fault stall, iq %d mA at %d rpm",
                   (int)(value * 1000.0f), (int)mid_enc_info()->spd_rpm);
        break;
    case (uint8_t)APP_FAULT_UNDERVOLT:
        log_printf("ERR fault undervoltage %d mV", (int)(value * 1000.0f));
        break;
    case (uint8_t)APP_FAULT_OVERVOLT:
        log_printf("ERR fault overvoltage %d mV", (int)(value * 1000.0f));
        break;
    default:
        log_printf("ERR fault encoder invalid");
        break;
    }
    log_printf(" type stop to reset");
}

const app_fault_info_t *app_last_fault(void)
{
    return &s_last_fault;
}

const app_fault_info_t *app_fault_info(void)
{
    return &s_fault;
}

const app_prot_t *app_prot_info(void)
{
    return &s_prot;
}

uint8_t app_prot_set(uint8_t which, float value)
{
    switch (which) {
    case 0u: if ((value < 100.0f)  || (value > 6000.0f)) { return 1u; }
             s_prot.overspeed_rpm = value; break;
    case 1u: if ((value < 1.0f)    || (value > 1000.0f)) { return 1u; }
             s_prot.stall_rpm = value; break;
    case 2u: if ((value < 0.05f)   || (value > 1.2f)) { return 1u; }
             s_prot.stall_iq_a = value; break;
    case 3u: if ((value < 5.0f)    || (value > 20.0f)) { return 1u; }
             s_prot.uv_v = value; break;
    case 4u: if ((value < 5.0f)    || (value > 24.0f)) { return 1u; }
             s_prot.ov_v = value; break;
    case 5u: /* the shunt saturates at +-1.32 A, so the trip must stay below
              * that; the lower bound keeps it above the noise floor. */
             if ((value < 0.20f) || (value > 1.30f)) { return 1u; }
             s_prot.oc_a = value; break;
    default: return 1u;
    }
    return 0u;
}

uint8_t app_foc_set(float id, float iq)
{
    float mag;

    if (s_st.state != APP_ST_RUN) {
        return 2u;                       /* send start first */
    }
    if (s_st.mode != CTRL_FOC) {
        return 4u;                       /* mode 2 required */
    }
    if (mid_current_get()->ready == 0u) {
        return 3u;                       /* zero drift still being measured */
    }
    if (s_spd.on != 0u) {
        /* The speed loop rewrites iq_ref every 1 ms, so accepting this would
         * look like it worked and then do nothing. */
        return 6u;
    }

    mag = sqrtf((id * id) + (iq * iq));
    if (mag > FOC_I_MAX_A) {
        return 5u;                       /* caller asked for too much current */
    }

    s_foc_info.id_ref = id;
    s_foc_info.iq_ref = iq;
    return 0u;
}

uint8_t app_foc_set_pi(float kp, float ki)
{
    if ((kp < FOC_KP_MIN) || (kp > FOC_KP_MAX)) { return 1u; }
    if ((ki < FOC_KI_MIN) || (ki > FOC_KI_MAX)) { return 1u; }
    if (s_st.mode != CTRL_FOC) { return 4u; }

    s_foc.pi_d.kp = kp;
    s_foc.pi_q.kp = kp;
    s_foc.pi_d.ki = ki;
    s_foc.pi_q.ki = ki;
    alg_pi_reset(&s_foc.pi_d);          /* no stale wind-up with new gains */
    alg_pi_reset(&s_foc.pi_q);
    return 0u;
}

void app_foc_set_ff(float k_emf, float l_h, float gain)
{
    alg_foc_set_ff(&s_foc, k_emf, l_h, gain);
}

void app_foc_get_ff(float *k_emf, float *l_h, float *gain)
{
    if (k_emf != 0) { *k_emf = s_foc.k_emf; }
    if (l_h   != 0) { *l_h   = s_foc.l_h; }
    if (gain  != 0) { *gain  = s_foc.ff_gain; }
}

void app_foc_get_pi(float *kp, float *ki)
{
    if (kp != 0) { *kp = s_foc.pi_d.kp; }
    if (ki != 0) { *ki = s_foc.pi_d.ki; }
}

/* ---------------- torque interface (S3) ---------------- */
uint8_t app_torque_set(float mnm)
{
    float   iq;
    uint8_t rc;

    if ((mnm > TORQUE_MAX_MNM) || (mnm < -TORQUE_MAX_MNM)) {
        return 5u;
    }
    if ((mnm < TORQUE_MIN_MNM) && (mnm > -TORQUE_MIN_MNM)) {
        app_torque_off();
        return 0u;
    }

    iq = mnm / (1000.0f * MOTOR_KT_NM_A);
    if ((iq > FOC_I_MAX_A) || (iq < -FOC_I_MAX_A)) {
        return 5u;                   /* Kt changed under our feet */
    }

    rc = app_foc_set(0.0f, iq);
    if (rc == 0u) {
        s_torque_mnm = mnm;
    }
    return rc;
}

void app_torque_off(void)
{
    s_torque_mnm = 0.0f;
    if ((s_st.state == APP_ST_RUN) && (s_st.mode == CTRL_FOC) && (s_spd.on == 0u)) {
        s_foc_info.id_ref = 0.0f;
        s_foc_info.iq_ref = 0.0f;
    }
}

float app_torque_get(void)
{
    return s_torque_mnm;
}

uint8_t app_spd_set(float rpm)
{
    if (s_st.state != APP_ST_RUN) { return 2u; }
    if (s_st.mode != CTRL_FOC)    { return 4u; }
    if ((rpm > SPD_RPM_MAX) || (rpm < -SPD_RPM_MAX)) { return 5u; }

    if ((rpm < 0.5f) && (rpm > -0.5f)) {
        app_spd_off();
        return 0u;
    }

    /* "spd" is the ENCODER speed loop. Two ways it used to lie:
     *  - with no encoder the frame hold returns before the loop acts,
     *    so it printed OK and the motor did not move (measured 20:32
     *    on #164: stop, start, "spd 200" -> nothing);
     *  - during the I/f ramp the forced frame overwrites idq on every
     *    frame, so the speed output was discarded too.
     * Say so instead of accepting the command. "spd 0" still releases. */
    if ((s_sfoc == 0u) && (s_if.run != 0u)) { return 7u; }
    if ((s_sfoc == 0u) && (mid_enc_enabled() == 0u)) { return 6u; }

    /* The reference itself is ramped in the 1 kHz task; the kick window
     * starts now so the rotor breaks free before the ramp asks for speed. */
    s_spd_target   = rpm;
    s_torque_mnm   = 0.0f;      /* the speed loop owns iq_ref now */
    s_spd.on       = 1u;
    s_spd.kick_on  = 1u;
    s_spd_kick_t0  = HAL_GetTick();
    s_spd_div     = (uint8_t)SPD_DIV;   /* run a step on the next frame */
    s_spd_iq      = 0.0f;               /* slew starts from a clean zero */
    alg_pi_reset(&s_spd.pi);
    s_foc_info.spd_ref    = rpm;   /* display only; the loop uses the ramp */
    s_foc_info.spd_target = rpm;
    return 0u;
}

void app_spd_off(void)
{
    s_spd.on       = 0u;
    s_spd.spd_ref  = 0.0f;
    s_spd_target   = 0.0f;
    s_spd.out     = 0.0f;
    s_spd_iq      = 0.0f;
    alg_pi_reset(&s_spd.pi);
    s_foc_info.spd_ref = 0.0f;
    s_foc_info.spd_target = 0.0f;
    s_foc_info.id_ref  = 0.0f;
    s_foc_info.iq_ref  = 0.0f;
}

uint8_t app_spd_set_ff(float base_a, float slope_a_per_rpm)
{
    if ((base_a < 0.0f) || (base_a > 0.5f)) {
        return 1u;
    }
    if ((slope_a_per_rpm < 0.0f) || (slope_a_per_rpm > 5e-4f)) {
        return 1u;
    }
    s_spd.ff_base  = base_a;
    s_spd.ff_slope = slope_a_per_rpm;
    return 0u;
}

void app_spd_get_ff(float *base_a, float *slope_a_per_rpm)
{
    if (base_a != 0) {
        *base_a = s_spd.ff_base;
    }
    if (slope_a_per_rpm != 0) {
        *slope_a_per_rpm = s_spd.ff_slope;
    }
}

uint8_t app_spd_set_pi(float kp, float ki)
{
    if ((kp < 0.0f) || (kp > 5.0f))   { return 1u; }
    if ((ki < 0.0f) || (ki > 50.0f))  { return 1u; }

    s_spd.pi.kp = kp;
    s_spd.pi.ki = ki;
    alg_pi_reset(&s_spd.pi);
    return 0u;
}

void app_spd_get_pi(float *kp, float *ki)
{
    if (kp != 0) { *kp = s_spd.pi.kp; }
    if (ki != 0) { *ki = s_spd.pi.ki; }
}

void app_rip_start(void)
{
    s_rip.iq_min  =  1e9f;
    s_rip.iq_max  = -1e9f;
    s_rip.spd_min =  1e9f;
    s_rip.spd_max = -1e9f;
    s_rip.n       = 0u;
    s_rip_t0      = HAL_GetTick();
    s_rip_on      = 1u;
}

const app_rip_t *app_rip_info(void)
{
    return &s_rip;
}

uint8_t app_sstep_start(float rpm)
{
    if (s_st.state != APP_ST_RUN) { return 2u; }
    if (s_st.mode != CTRL_FOC)    { return 4u; }
    if ((rpm > SPD_RPM_MAX) || (rpm < -SPD_RPM_MAX)) { return 5u; }
    if ((rpm < 50.0f) && (rpm > -50.0f)) { return 5u; }

    s_sstep.target = rpm;
    s_sstep.valid  = 0u;
    s_sstep_phase  = 1u;
    s_sstep_t0     = HAL_GetTick();
    app_spd_off();
    return 0u;
}

const app_sstep_info_t *app_sstep_info(void)
{
    return &s_sstep;
}

uint8_t app_step_start(float amp)
{
    if (s_st.state != APP_ST_RUN) { return 2u; }
    if (s_st.mode != CTRL_FOC)    { return 4u; }
    if ((amp > FOC_I_MAX_A) || (amp < -FOC_I_MAX_A)) { return 5u; }

    s_step.amp   = amp;
    s_step.valid = 0u;
    s_step_phase = 1u;
    s_step_t0    = HAL_GetTick();
    s_foc_info.id_ref = 0.0f;
    s_foc_info.iq_ref = 0.0f;
    return 0u;
}

const app_step_info_t *app_step_info(void)
{
    return &s_step;
}

const app_foc_info_t *app_foc_info(void)
{
    return &s_foc_info;
}

void app_vf_set_acc(float hz_per_s)
{
    if (hz_per_s < VF_ACC_MIN_HZ_PER_S) { hz_per_s = VF_ACC_MIN_HZ_PER_S; }
    if (hz_per_s > VF_ACC_MAX_HZ_PER_S) { hz_per_s = VF_ACC_MAX_HZ_PER_S; }
    s_vf.acc_hz_s = hz_per_s;
}

uint8_t app_vf_set_dir(uint8_t dir)
{
    if (dir > 1u) { return 1u; }
    s_vf.dir = dir;
    s_vf_info.dir = dir;
    return 0u;
}

const app_vf_info_t *app_vf_info(void)
{
    return &s_vf_info;
}

/* One V/f step per ADC frame (20kHz). ts is exact, so the angle integration
 * has no jitter; S6 will move this into the ADC interrupt.
 * t_pick = DWT timestamp taken as the main loop picked the frame up, so the
 * "comp" and "wr" marks measure stage durations, not absolute offsets. */
/* ------------------------------------------------------------------ */
/* FOC current loop (S6)                                               */
/* ------------------------------------------------------------------ */
static void task_foc(uint32_t t_pick)
{
    bsp_sys_ctx = BSP_CTX_FOC;
    const current_sample_t *cs = mid_current_get();
    const mid_enc_t        *e  = mid_enc_info();
    float duty[3];
    float imax;
    float t;
    float id_ref;
    float iq_ref;
    float mag;
    float k;
    uint32_t t_comp;
    uint8_t  i;
    uint8_t  coast = 0u;             /* #168: coasting, no angle source */

    /* No encoder frame yet: hold the zero vector rather than invent an angle.
     * Exceptions: the I/f ramp (S10.4) and the sensorless mode (S10.5) supply
     * their own angle, so they must work with no encoder fitted at all.
     *
     * S10.10 (#168): "no angle source but the rotor still turns" used to land
     * here too, and the zero vector is a three-phase SHORT: measured 1.316 A
     * of back-EMF current at 1500 rpm (trip 1.28 A) and a hard brake. So when
     * back-EMF is present, keep the loop closed at zero current on the
     * observer angle; only drop to the zero vector once the rotor has
     * stopped. |e| is the criterion, never the observer's omega. */
    if ((e->valid == 0u) && (s_if.run == 0u) && (s_sfoc == 0u)) {
        float em2 = (s_obs.e_alpha * s_obs.e_alpha)
                  + (s_obs.e_beta  * s_obs.e_beta);

        if ((s_coast == 0u) && (s_obs_on != 0u) && (em2 > COAST_E_MIN2)) {
            s_coast       = 1u;
            s_coast_ticks = 0u;
        }
        if (s_coast != 0u) {
            s_coast_ticks++;
            if ((em2 < COAST_E_MIN2) || (s_coast_ticks > COAST_TICKS_MAX)) {
                s_coast = 0u;
            }
        }
        if (s_coast == 0u) {
            bsp_pwm_set_all_duty(PWM_START_DUTY);
            foc_info_clear_meas();       /* #167: zero vector, zero display */
            return;
        }

        /* Coast: cancel the back-EMF with zero current, do not short it. */
        coast      = 1u;
        id_ref     = 0.0f;
        iq_ref     = 0.0f;
        s_foc_info.id_ref  = 0.0f;
        s_foc_info.iq_ref  = 0.0f;
        s_foc_info.spd_rpm = s_obs.omega_e * 9.54929658551372f
                           / (float)ENC_POLE_PAIRS;
    }

    /* ---- encoder liveness: only meaningful when the encoder is the angle
     * source. In sensorless mode it must be skipped: with no encoder fitted
     * the frame counter never advances and this would fault at once. ---- */
    /* ---- encoder liveness: count frames without a NEW good one.
     * Using the bad-frame counter was wrong: while the encoder is healthy
     * that counter never changes, so the "stuck" timer fired 20 frames
     * after start (measured: false encoder fault at 1 ms). ---- */
    if ((s_if.run == 0u) && (s_sfoc == 0u) &&
        (mid_enc_enabled() != 0u)) {
        if ((e->frames != s_prot_bad_seen) || (bsp_ssi_quiet() != 0u)) {
            /* A resync parks the bus for 2.5 ms on purpose: 50 control
             * frames at 20 kHz of intended silence is not a stalled encoder. */
            s_prot_bad_seen = e->frames;
            s_prot_enc_cnt  = 0u;
        } else if (s_prot_enc_cnt < 0xFFFFu) {
            s_prot_enc_cnt++;
            if (s_prot_enc_cnt >= PROT_ENC_FRAMES) {
                if (mid_enc_seen_ok() == 0u) {
                    /* Never decoded a real frame since boot: no encoder on
                     * the bus. Drop out of encoder mode instead of faulting
                     * so "sfoc go" is still reachable. A real mid-run
                     * failure still faults. */
                    mid_enc_set_enabled(0u);
                    s_prot_enc_cnt      = 0u;
                    s_prot_enc_zero_cnt = 0u;
                    log_printf("warn no encoder frames, encoder off "
                               "(enc on to re-enable)");
                    return;
                }
                log_printf("fault: encoder stuck, %u calls without a new frame",
                           (unsigned)s_prot_enc_cnt);
                app_fault_raise((uint8_t)APP_FAULT_ENCODER, 0.0f);
                return;
            }
        }

        /* A disconnected MT6701 drives MISO to a constant level, so every
         * bit reads 0 and the CRC of an all-zero frame is itself zero: the
         * CRC check passes and the angle looks like a valid 0. Measured on
         * hardware with the cable unplugged: raw 0, crc ok, no fault.
         * A real sensor sitting exactly at raw 0 for PROT_ENC_ZERO_FRAMES
         * frames in a row is far less likely than a dead bus. */
        if (e->raw == 0u) {
            if (s_prot_enc_zero_cnt < 0xFFFFu) {
                s_prot_enc_zero_cnt++;
            }
            if (s_prot_enc_zero_cnt >= (uint16_t)PROT_ENC_ZERO_FRAMES) {
                if (mid_enc_seen_ok() == 0u) {
                    /* All-zero frames and never a good one: the bus is
                     * idle (no encoder fitted). Measured 20:22 on #163:
                     * "start" faulted 10 ms in, before "sfoc go" could
                     * switch to sensorless. */
                    mid_enc_set_enabled(0u);
                    s_prot_enc_cnt      = 0u;
                    s_prot_enc_zero_cnt = 0u;
                    log_printf("warn no encoder frames, encoder off "
                               "(enc on to re-enable)");
                    return;
                }
                log_printf("fault: encoder reading zero, %u frames",
                           (unsigned)s_prot_enc_zero_cnt);
                app_fault_raise((uint8_t)APP_FAULT_ENCODER, 0.0f);
                return;
            }
        } else {
            s_prot_enc_zero_cnt = 0u;
        }

        /* ---- corrupt-frame guard: frames keep arriving but the CRC says the
         * sensor is not telling the truth (18#L-16, 2026-11-05: the MT6701
         * latched a magnetic status after the wheel spun up and drove the
         * frame tail high, crc BAD on ~90 % of frames while mid_enc kept
         * re-using the last good angle). Steering the FOC on a frozen angle
         * turns the commanded iq into a wrong-frame vector and the phase
         * currents run away into the overcurrent trip, so fault first. ---- */
        if (e->crc_ok == 0u) {
            if (s_prot_enc_crc_cnt < 0xFFFFu) {
                s_prot_enc_crc_cnt++;
            }
            if (s_prot_enc_crc_cnt >= (uint16_t)PROT_ENC_BAD_FRAMES) {
                log_printf("fault: encoder crc, %u bad frames in a row, raw %u win %u",
                           (unsigned)s_prot_enc_crc_cnt,
                           (unsigned)e->raw, (unsigned)mid_enc_win());
                app_fault_raise((uint8_t)APP_FAULT_ENCODER, (float)s_prot_enc_crc_cnt);
                return;
            }
        } else {
            s_prot_enc_crc_cnt = 0u;
        }
    }

    /* ---- current guard (S7 will replace this with a hardware trip) ---- */
    imax = fabsf(cs->ia);
    t = fabsf(cs->ib); if (t > imax) { imax = t; }
    t = fabsf(cs->ic); if (t > imax) { imax = t; }

    if (imax >= s_prot.oc_a) {
        if (++s_foc_trip_cnt >= FOC_TRIP_FRAMES) {
            s_foc_trip_cnt = 0u;
            s_foc_info.faults++;
            app_fault_raise((uint8_t)APP_FAULT_OVERCURRENT, imax);
            return;
        }
    } else {
        s_foc_trip_cnt = 0u;
    }

    /* ---- S6.4 step generator: overrides the references while running ---- */
    if (s_step_phase != 0u) {
        uint32_t el = HAL_GetTick() - s_step_t0;

        if (s_step_phase == 1u) {        /* settle at zero current */
            s_foc_info.id_ref = 0.0f;
            s_foc_info.iq_ref = 0.0f;
            if (el >= (uint32_t)STEP_SETTLE_MS) {
                s_step_phase  = 2u;
                s_step_t0     = HAL_GetTick();
                s_step_n      = 0u;
                s_step_t10    = 0u;
                s_step_t90    = 0u;
                s_step_got10  = 0u;
                s_step_got90  = 0u;
                s_step_peak   = 0.0f;
                s_step_tail_min =  1e9f;
                s_step_tail_max = -1e9f;
                s_step_tail_sum = 0.0f;
                s_step_tail_n   = 0u;
                s_foc_info.iq_ref = s_step.amp;
            }
        }
        else if (s_step_phase == 2u) {   /* record the response */
            /* Recompute the elapsed time: s_step_t0 was just reset when this
             * phase started, and reusing the value computed at the top of the
             * block made the window look 100 ms old, so the report fired on
             * the first frame and the measurement was always empty. */
            uint32_t er = HAL_GetTick() - s_step_t0;

            s_foc_info.id_ref = 0.0f;
            s_foc_info.iq_ref = s_step.amp;

            /* measured iq from the previous step is the sample for this frame */
            s_step_n++;
            if (s_step_n > 4u) {         /* skip the one-frame delay */
                /* Magnitudes, so "step -0.30" measures the same way. */
                float y    = fabsf(s_foc.iq);
                float amp  = fabsf(s_step.amp);
                float y10  = 0.1f * amp;
                float y90  = 0.9f * amp;

                if ((s_step_got10 == 0u) && (y >= y10)) {
                    s_step_got10 = 1u;
                    s_step_t10   = s_step_n;
                }
                if ((s_step_got90 == 0u) && (y >= y90)) {
                    s_step_got90 = 1u;
                    s_step_t90   = s_step_n;
                }
                if (y > s_step_peak) { s_step_peak = y; }

                /* tail window = last 5 ms, used for error and ripple */
                if (er >= ((uint32_t)STEP_RECORD_MS - 5u)) {
                    if (y < s_step_tail_min) { s_step_tail_min = y; }
                    if (y > s_step_tail_max) { s_step_tail_max = y; }
                    s_step_tail_sum += y;
                    s_step_tail_n++;
                }
            }

            if (er >= (uint32_t)STEP_RECORD_MS) {
                s_step_phase = 0u;
                s_step.valid = 1u;
                s_step.rise_ms = -1.0f;      /* -1 = not measurable */
                if ((s_step_got10 != 0u) && (s_step_got90 != 0u) &&
                    (s_step_t90 >= s_step_t10)) {
                    s_step.rise_ms = (float)(s_step_t90 - s_step_t10) * 0.05f;
                }
                s_step.overshoot = 0.0f;
                if (fabsf(s_step.amp) > 0.05f) {
                    s_step.overshoot =
                        ((s_step_peak - fabsf(s_step.amp)) / fabsf(s_step.amp))
                        * 100.0f;
                    if (s_step.overshoot < 0.0f) { s_step.overshoot = 0.0f; }
                }
                s_step.err = 0.0f;
                s_step.ss  = 0.0f;
                if (s_step_tail_n != 0u) {
                    float avg = s_step_tail_sum / (float)s_step_tail_n;
                    s_step.ss = avg;
                    if (fabsf(s_step.amp) > 0.05f) {
                        s_step.err = ((fabsf(avg) - fabsf(s_step.amp))
                                      / fabsf(s_step.amp)) * 100.0f;
                        if (s_step.err < 0.0f) { s_step.err = -s_step.err; }
                    }
                    s_step.ripple = s_step_tail_max - s_step_tail_min;
                }
                s_step.peak = s_step_peak;
                s_foc_info.iq_ref = 0.0f;
                s_foc_info.id_ref = 0.0f;
                /* rise == 0 means the current crossed 10% and 90% inside a
                 * single 50 us sample: at 0.3 A the slew is voltage limited
                 * (di/dt = V/L ~ 11 A/ms), so there is no ramp to resolve. */
                if (s_step.rise_ms < 0.0f) {
                    log_printf("OK step %.2f A, rise n/a, over %.1f%%, err %.1f%%, rip %.0f mA",
                               (double)s_step.amp, (double)s_step.overshoot,
                               (double)s_step.err,
                               (double)(s_step.ripple * 1000.0f));
                } else {
                    log_printf("OK step %.2f A, rise %.2f ms, over %.1f%%, err %.1f%%, rip %.0f mA",
                               (double)s_step.amp, (double)s_step.rise_ms,
                               (double)s_step.overshoot, (double)s_step.err,
                               (double)(s_step.ripple * 1000.0f));
                }
                log_printf(" ss %.3f A, peak %.3f A, tail n=%u",
                           (double)s_step.ss, (double)s_step.peak,
                           (unsigned)s_step_tail_n);
            }
        }
    }

    /* ---- S10.6: loss of lock --------------------------------------
     * A locked observer cannot report a speed with no back-EMF. When it
     * does (measured: eq 0.0 V while claiming 400 rpm) the drive is in a
     * false equilibrium: the rotor is held still by a frozen angle while
     * the speed loop keeps pushing. Detect and drop out. */
    if ((s_sfoc != 0u) && (s_ho_state == 3u)) {   /* only once handed over */
        float spd_obs = s_obs.omega_e * 9.54929658551372f / (float)ENC_POLE_PAIRS;
        float eq_a = sqrtf((s_obs.e_alpha * s_obs.e_alpha) +
                           (s_obs.e_beta  * s_obs.e_beta));   /* |e|, not eq */
        float sp_a = (spd_obs > 0.0f) ? spd_obs : -spd_obs;
        float w_a  = (s_obs.omega_e > 0.0f) ? s_obs.omega_e : -s_obs.omega_e;
        float eq_exp = w_a * MOTOR_KE_V_RAD_S;   /* what a locked observer owes */

        /* Ratio, not an absolute level: eq and omega grow together during
         * the ramp, so a fixed threshold aborted a healthy start. */
        if ((eq_exp > 0.05f) && (sp_a > HO_LOST_MIN_RPM) &&
            (eq_a < (HO_LOST_EQ_RATIO * eq_exp))) {
            if (s_lost_ms < 0xFFFFu) {
                s_lost_ms++;
            }
            if (s_lost_ms >= (uint16_t)(HO_LOST_MS * (FOC_FREQ_HZ / 1000u))) {
                s_lost_ms = 0u;
                if (s_spd.on != 0u) {
                    app_sfoc_abort();
                } else {
                    /* S10.15: after "sfoc go 0" the loop is off and the
                     * rotor is only coasting. Losing the back-EMF here
                     * means it came to rest, which is what was asked for -
                     * release quietly instead of crying "lost lock". */
                    app_sfoc_release();
                    log_printf("OK rotor stopped, sensorless released");
                }
            }
        } else {
            s_lost_ms = 0u;
        }
    } else {
        s_lost_ms = 0u;
    }

    /* ---- S10.13: PLL speed rail ----
     * The observer PLL clamps at OBS_W_MAX_RADS. A clamped PLL is not an
     * observer any more: the angle free-runs at the rail rate and the
     * drive turns into an open-loop V/f push, so both the reported speed
     * and any protection built on it are meaningless. Drop out instead. */
    if ((s_sfoc != 0u) && (s_ho_state == 3u)) {
        float w_rail = (s_obs.omega_e > 0.0f) ? s_obs.omega_e
                                              : -s_obs.omega_e;

        if (w_rail >= (OBS_RAIL_RATIO * OBS_W_MAX_RADS)) {
            if (s_rail_ms < 0xFFFFu) {
                s_rail_ms++;
            }
            if (s_rail_ms >= (uint16_t)(HO_RAIL_MS * (FOC_FREQ_HZ / 1000u))) {
                float rpm_rail = s_obs.omega_e * 9.54929658551372f
                               / (float)ENC_POLE_PAIRS;

                s_rail_ms = 0u;
                app_sfoc_release();
                log_printf("ERR obs speed railed (%d rpm), drive released",
                           (int)rpm_rail);
            }
        } else {
            s_rail_ms = 0u;
        }
    } else {
        s_rail_ms = 0u;
    }

    /* ---- S7.3 speed loop: 1 kHz, drives iq_ref (d stays at zero) ---- */
    if (s_spd.on != 0u) {
        s_spd_div++;
        if (s_spd_div >= (uint8_t)SPD_DIV) {
            s_spd_div = 0u;
            float spd_fb;

            /* S10.5b: in sensorless mode the observer's speed is the feedback;
             * the encoder is only there to verify it. */
            spd_fb = (s_sfoc != 0u)
                   ? (s_obs.omega_e * 9.54929658551372f / (float)ENC_POLE_PAIRS)
                   : e->spd_rpm;

            (void)alg_spd_step(&s_spd, spd_fb, SPD_TS_S);
            /* S10.13: in reverse the observer frame hands torque the other
             * way round (measured: +0.6 A on the forced frame drags the
             * rotor to -300 rpm). Mirror the command so that "push harder"
             * still means grow the speed the loop is being asked for. */
            /* S7.5: slew the reference so the current loop sees a ramp, not
             * a step. See SPD_IQ_SLEW_A in board_cfg.h for the measurement. */
            {
                float want = s_spd.out;
                float d    = want - s_spd_iq;
                if (d >  SPD_IQ_SLEW_A) { d =  SPD_IQ_SLEW_A; }
                if (d < -SPD_IQ_SLEW_A) { d = -SPD_IQ_SLEW_A; }
                s_spd_iq += d;
                s_foc_info.iq_ref = s_spd_iq;
            }
            if ((s_sfoc != 0u) && (s_obs.omega_e < 0.0f)) {
                s_foc_info.iq_ref = -s_foc_info.iq_ref;
            }
            s_foc_info.id_ref = 0.0f;
            s_foc_info.spd_rpm = spd_fb;
        }
        if (s_sfoc == 0u) {
            s_foc_info.spd_rpm = e->spd_rpm;
        }
    }

    /* ---- reference vector limit: never ask for more than the ADC range ---- */
    id_ref = s_foc_info.id_ref;
    iq_ref = s_foc_info.iq_ref;
    mag = sqrtf((id_ref * id_ref) + (iq_ref * iq_ref));
    if (mag > FOC_I_MAX_A) {
        k = FOC_I_MAX_A / mag;
        id_ref *= k;
        iq_ref *= k;
    }

    /* ---- S10.4: open-loop I/f ----
     * The forced frame supplies both the angle and the current vector; the
     * speed loop must stay out of the way (no meaningful feedback yet).
     * Hard priority: once the observer owns the drive (s_sfoc) the ramp must
     * not touch the current references any more - this block runs AFTER the
     * speed loop and would silently win (measured 06:00: the I/f resurrected
     * itself and pinned the drive at its 600 rpm drag ceiling). */
    if ((s_if.run != 0u) && (s_sfoc == 0u)) {
        alg_if_step(&s_if, FOC_TS);

        /* ---- S10.5: hand-over evaluation ----
         * Only once the drag phase is running (the rotor is actually turning). */
        if ((s_ho_state == 1u) && (s_if.phase == 1u)) {
            float de = s_obs.theta_out - s_if.theta;

            float rpm_if;

            while (de >  3.14159265f) { de -= 6.283185307179586f; }
            while (de < -3.14159265f) { de += 6.283185307179586f; }
            s_ho_ds  = de;                         /* signed: reverse < 0  */
            s_ho_dth = (de > 0.0f) ? de : -de;     /* magnitude, diagnostics */

            /* Judge the observer, not its agreement with the forced frame:
             * the drag always runs at a load angle (measured ~74 deg), so the
             * two angles are never close. */
            rpm_if = s_if.f_hz * (60.0f / (float)ENC_POLE_PAIRS);

            /* Do NOT gate on pll.locked: it is |err| < 0.15 rad on the
             * instantaneous error, which ripples with the current at 20 kHz
             * and flips 0/1 (measured: 3/3 hand-overs stuck in "watching"
             * with the motor already at 399 rpm). Use the consistency ratio
             * instead: a healthy observer reports an eq close to what its own
             * speed implies. */
            {
                /* |e| = sqrt(ea^2 + eb^2): invariant to the angle error, unlike
                 * eq (which collapses at 90 deg and made this criterion flaky). */
                float e_mag  = sqrtf((s_obs.e_alpha * s_obs.e_alpha) +
                                     (s_obs.e_beta  * s_obs.e_beta));
                /* Expected back-EMF at the DRAG speed, not at the
                 * observer's own estimate: validating the observer against
                 * itself fails whenever its omega drifts up during the
                 * forced run (measured 13:21: one start handed over, the
                 * next sat in "watching" at 600 rpm). */
                float rpm_abs = (rpm_if > 0.0f) ? rpm_if : -rpm_if;
                float w_exp   = rpm_abs * 0.104719755120f * (float)ENC_POLE_PAIRS;
                float eq_ex2  = w_exp * MOTOR_KE_V_RAD_S;

                if ((e_mag > OBS_E_MIN_V) &&
                    (fabsf(rpm_if) > HO_MIN_RPM) &&
                    ((eq_ex2 < 0.05f) || (e_mag > (0.50f * eq_ex2)))) {
                    if (s_ho_cnt < 0xFFFFu) {
                        s_ho_cnt++;
                    }
                    if (s_ho_cnt >= (uint16_t)HO_HOLD_CNT) {
                        s_ho_state = 2u;
                        s_ho_w     = 0.0f;
                        /* Torque factor of the drag: the forced q axis sits
                         * s_ho_dth away from the real q axis, so the drag only
                         * converts cos(dth) of its current into torque. */
                        /* #170: SIGNED. cos(de) is what the drag really
                         * makes on the rotor q axis: positive forward,
                         * NEGATIVE in reverse. The old code used |de| and
                         * clamped to >= 0.05, i.e. it pre-loaded the wrong
                         * torque sign every time the motor ran backwards. */
                        s_ho_c0    = cosf(s_ho_ds);
                        if (fabsf(s_ho_c0) < 0.05f) {
                            s_ho_c0 = (s_ho_c0 >= 0.0f) ? 0.05f : -0.05f;
                        }
                        if (s_ho_c0 >  1.00f) { s_ho_c0 =  1.00f; }
                        if (s_ho_c0 < -1.00f) { s_ho_c0 = -1.00f; }
                    }
                } else {
                    s_ho_cnt = 0u;
                }
            }
        } else if (s_ho_state == 2u) {
            s_ho_w += FOC_TS / HO_BLEND_S;
            if (s_ho_w >= 1.0f) {
                float rpm_tgt;
                float rpm_now;
                float iq0;

                s_ho_w     = 1.0f;
                s_ho_state = 3u;

                /* S10.5b: from here the observer owns both angle and speed.
                 * Start the speed loop at the frequency the drag was running,
                 * then drop the forced ramp so nothing fights it. */
                rpm_now = s_if.f_hz * (60.0f / (float)ENC_POLE_PAIRS);
                rpm_tgt = (s_if_rpm_tgt != 0.0f) ? s_if_rpm_tgt : rpm_now;
                if (rpm_tgt >  SPD_RPM_MAX) { rpm_tgt =  SPD_RPM_MAX; }
                if (rpm_tgt < -SPD_RPM_MAX) { rpm_tgt = -SPD_RPM_MAX; }

                /* Torque the drag was delivering. Captured BEFORE alg_if_stop,
                 * which zeroes i_now and f_hz. */
                iq0 = s_if.i_now * s_ho_c0;   /* signed: reverse < 0 (#170) */
                if (iq0 >  FOC_I_MAX_A) { iq0 =  FOC_I_MAX_A; }
                if (iq0 < -FOC_I_MAX_A) { iq0 = -FOC_I_MAX_A; }

                s_sfoc  = 1u;
                alg_if_stop(&s_if);
                (void)app_spd_set(rpm_tgt);

                /* S10.8 bumpless transfer: the rotor is ALREADY at rpm_now.
                 * spd_ref was zero (app_spd_off cleared it), so the ramp
                 * started from 0 and the PI saw a -rpm_now error: measured
                 * -1.2 A (clamped) for ~0.5 s right at the hand-over = the
                 * second visible kick. Seed the reference, the integrator (to
                 * the torque the drag was making) and the reference history
                 * (without it the acceleration feedforward sees a
                 * (rpm_now - 0)/ts step: ~32000 rad/s^2 * 3.11e-3 A = ~100 A). */
                s_spd.spd_ref = rpm_now;
                s_spd_iq      = iq0;   /* slew continues from the drag torque */
                alg_spd_preload(&s_spd, iq0);
            }
        }
        /* Torque, not alignment: the current must sit on the q axis of the
         * forced frame. A d-axis vector only pulls the rotor INTO the frame
         * (that is what align ol does) and produces no torque, so the rotor
         * would just sit there. Measured with id = i_amp on #138: frequency
         * ramped to 35 Hz, current 0.40 A, encoder 0 rpm. */
        if (s_if.phase == 0u) {
            /* Pre-align: d-axis, torque free (this is what "align ol" does). */
            id_ref = s_if.i_now;
            iq_ref = 0.0f;
        } else {
            /* Drag: the vector crossfades d -> q over IF_XFADE_MS so the
             * torque ramps up instead of hopping 90 deg in one frame. */
            alg_if_apply(&s_if, &id_ref, &iq_ref);
        }

        /* S10.8: while the frame rotates toward the observer the current
         * vector is dth*(1-w) away from the real q axis, so the same current
         * makes MORE torque (cos goes from 0.28 to 1.0 over the blend). Scale
         * by cos(dth)/cos(dth*(1-w)) to keep the torque flat; at w = 1 it
         * lands exactly on i_now*cos(dth), the value the speed loop is
         * preloaded with, so torque is continuous across the hand-over. */
        if ((s_ho_state == 2u) && (s_if.phase == 1u)) {
            float rem = s_ho_ds * (1.0f - s_ho_w);
            float cw  = cosf(rem);

            if (cw > 0.20f) {
                float sc = s_ho_c0 / cw;

                id_ref *= sc;
                iq_ref *= sc;
            }
        }
        s_foc_info.id_ref = id_ref;
        s_foc_info.iq_ref = iq_ref;
        s_foc_info.spd_rpm = s_if.f_hz * (60.0f / (float)ENC_POLE_PAIRS);
    }

    /* ---- S10.5: which angle drives the Park transform? ----
     *   not in I/f        -> encoder
     *   I/f, before blend -> forced frame
     *   I/f, blending     -> forced + w * (obs - forced)
     *                        (blend the DIFFERENCE: blending the wrapped
     *                         angles directly jumps at the 2*pi seam)
     *   I/f, after blend  -> observer */
    {
        float th_use;
        float we_use;

        if ((coast != 0u) || (s_sfoc != 0u)) {
            /* True sensorless, or coasting after a release with no encoder:
             * the observer supplies the angle. */
            th_use = s_obs.theta_out;
            we_use = s_obs.omega_e;
        } else if (s_if.run == 0u) {
            th_use = e->elec_deg * 0.0174532925199433f;
            we_use = e->spd_hz * 6.283185307179586f;
        } else {
            float de = s_obs.theta_out - s_if.theta;

            while (de >  3.14159265f) { de -= 6.283185307179586f; }
            while (de < -3.14159265f) { de += 6.283185307179586f; }

            th_use = s_if.theta + (s_ho_w * de);
            while (th_use >= 6.283185307179586f) { th_use -= 6.283185307179586f; }
            while (th_use <  0.0f)                { th_use += 6.283185307179586f; }

            we_use = (6.283185307179586f * s_if.f_hz * (1.0f - s_ho_w)) +
                     (s_obs.omega_e * s_ho_w);
        }

        /* S7.2: electrical angular speed for the feedforward, signed. */
        (void)alg_foc_step(&s_foc, cs->ia, cs->ib, cs->ic,
                           th_use, we_use,
                           id_ref, iq_ref, FOC_TS, duty);
    }

    /* ---- S10.1: shadow observer (measures only, never drives) ---- */
    if (s_obs_on != 0u) {
        alg_obs_step(&s_obs, s_foc.v_alpha, s_foc.v_beta,
                     s_foc.i_alpha, s_foc.i_beta, FOC_TS);
    }

    for (i = 0u; i < 3u; i++) {
        if (duty[i] < 0.0f) { duty[i] = 0.0f; }
        if (duty[i] > 1.0f) { duty[i] = 1.0f; }
    }

    t_comp = bsp_prof_now();
    phase_mark(&s_phase.comp, t_pick);
    bsp_pwm_set_duty(duty[0], duty[1], duty[2]);
    phase_mark(&s_phase.wr, t_comp);
    bsp_prof_add(PROF_CONV, t_pick);

    s_foc_info.id    = s_foc.id;
    s_foc_info.iq    = s_foc.iq;
    s_foc_info.vd    = s_foc.vd;
    s_foc_info.vq    = s_foc.vq;
    s_foc_info.m     = s_foc.m;
    s_foc_info.ready = 1u;
}

static void task_vf(uint32_t t_pick)
{
    const current_sample_t *cs;
    float theta;
    float m;
    float duty[3];
    float imax;
    float t;

    if (s_vf.on == 0u) {
        return;
    }

    theta = alg_vf_step(&s_vf, VF_TS_S);

    /* ---- current guard: there is no hardware protection until S7 ---- */
    cs = mid_current_get();
    imax = fabsf(cs->ia);
    t = fabsf(cs->ib); if (t > imax) { imax = t; }
    t = fabsf(cs->ic); if (t > imax) { imax = t; }

    if (imax >= VF_I_TRIP_A) {
        if (++s_i_trip_cnt >= VF_I_TRIP_FRAMES) {
            alg_vf_stop(&s_vf);
            bsp_pwm_disable();           /* MOE=0 immediately */
            bsp_pwm_set_all_duty(PWM_START_DUTY);
            s_st.state           = APP_ST_FAULT;
            s_vf_info.on         = 0u;
            s_vf_info.m          = 0.0f;
            s_i_trip_cnt         = 0u;
            log_printf("ERR overcurrent trip I=%d mA, type stop to reset",
                       (int)(imax * 1000.0f));
            return;
        }
    } else {
        s_i_trip_cnt = 0u;
    }

    if (imax > VF_I_FOLD_A) {
        if (++s_i_fold_cnt >= VF_I_FOLD_FRAMES) {
            s_i_fold_cnt = 0u;
            s_vf.m_scale *= 0.98f;
            if (s_vf.m_scale < VF_M_SCALE_MIN) { s_vf.m_scale = VF_M_SCALE_MIN; }
        }
    } else {
        s_i_fold_cnt = 0u;
        if (imax < VF_I_FOLD_BACK_A) {
            s_vf.m_scale *= 1.001f;
            if (s_vf.m_scale > 1.0f) { s_vf.m_scale = 1.0f; }
        }
    }

    m = alg_vf_mod_index(&s_vf);
    alg_svpwm(theta, m, duty);
    {
        uint32_t t_comp = bsp_prof_now();

        phase_mark(&s_phase.comp, t_pick);      /* V/f + SVPWM cost */
        bsp_pwm_set_duty(duty[0], duty[1], duty[2]);
        phase_mark(&s_phase.wr, t_comp);        /* CCR write cost */
    }

    s_vf_info.on       = 1u;
    s_vf_info.dir      = s_vf.dir;
    s_vf_info.f_hz     = s_vf.f_hz;
    s_vf_info.f_target = s_vf.f_target;
    s_vf_info.m        = m;
    s_vf_info.i_max    = imax;
}

void app_wave_set(uint8_t on, uint16_t rate_hz)
{
    /* FireWater CSV: 21 channels ~ 137B/frame, so 500Hz is ~68kB/s
     * (69% of a 1Mbaud link) - that already overflows the DAPLink VCP, use
     * 250Hz. HAL_GetTick only resolves 1ms, so 1000Hz is out of reach. */
    if (rate_hz < 1u)                { rate_hz = 1u; }
    if (rate_hz > DBG_WAVE_RATE_MAX) { rate_hz = DBG_WAVE_RATE_MAX; }
    s_st.wave_rate_hz = rate_hz;
    s_st.wave_on      = (on != 0u) ? 1u : 0u;
    s_wave_period_ms  = (uint16_t)(1000u / rate_hz);
    if (s_wave_period_ms == 0u) { s_wave_period_ms = 1u; }
    s_t_wave          = HAL_GetTick();
}

void app_counters_clear(void)
{
    s_st.wave_frames = 0u;
    s_st.wave_drop   = 0u;
    s_st.rx_drop     = 0u;

    /* Also reset the encoder error counters: "clr" is documented as
     * "clear counters", and without this the CRC count could not be
     * measured over a window (it kept the historical total). */
    mid_enc_clear_errs();
}

/* ------------------------------------------------------------------ */
/* S10.1 back-EMF observer (shadow mode)                              */
/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* S10.4 open-loop I/f                                                */
/* ------------------------------------------------------------------ */
uint8_t app_if_start(float rpm)
{
    float f_hz;
    float rpm_drag;

    if (s_st.state != APP_ST_RUN) {
        return 2u;
    }

    /* Keep the caller's request for the closed loop, but only drag the rotor
     * up to IF_RPM_MAX: the hand-over happens at 250 rpm, so pulling the I/f
     * further is pointless. The clamp used to leak into the speed-loop target
     * (measured 05:14: 'sfoc go 1400' became target 600). */
    s_if_rpm_tgt = rpm;

    rpm_drag = rpm;
    if (rpm_drag >  IF_RPM_MAX) { rpm_drag =  IF_RPM_MAX; }
    if (rpm_drag < -IF_RPM_MAX) { rpm_drag = -IF_RPM_MAX; }

    /* Mechanical rpm -> electrical Hz. */
    f_hz = (rpm_drag / 60.0f) * (float)ENC_POLE_PAIRS;

    app_spd_off();                  /* no speed loop during a forced run */

    if (s_if.run != 0u) {
        /* Already dragging: only move the target. Resetting theta and the
         * frequency here yanks the forced frame back to 0 and throws the
         * rotor out of sync at once (measured: 200 rpm -> 0 rpm). */
        alg_if_set_target(&s_if, f_hz);
    } else {
        /* Do NOT re-init here: that used to wipe the amplitude set by
         * "if i" back to the default, which made a parameter sweep look
         * like "the amplitude does not matter". */
        alg_if_start(&s_if);
        alg_if_set_target(&s_if, f_hz);
    }
    return 0u;
}

void app_if_stop(void)
{
    alg_if_stop(&s_if);
    s_foc_info.id_ref = 0.0f;
    s_foc_info.iq_ref = 0.0f;
}

void app_if_set_amp(float a)
{
    if ((a > 0.05f) && (a <= FOC_I_MAX_A)) {
        s_if.i_amp = a;
    }
}

void app_if_set_acc(float hz_s)
{
    if ((hz_s >= 2.0f) && (hz_s <= 300.0f)) {
        s_if.acc_hz_s = hz_s;
    }
}

/* ---- S10.5 hand-over ---- */
/* Drop out of sensorless safely: release the current, keep the reason
 * in the caller. */
static void app_sfoc_release(void)
{
    s_sfoc     = 0u;
    s_ho_state = 0u;
    app_spd_off();
    app_if_stop();
}

/* S10.13: the detector works on |e|, so the message quotes |e| as well -
 * printing the projected eq made a healthy 90 deg offset look like zero
 * back-EMF and cost an afternoon. */
void app_sfoc_abort(void)
{
    float e_abs = sqrtf((s_obs.e_alpha * s_obs.e_alpha) +
                        (s_obs.e_beta  * s_obs.e_beta));
    float w_abs = (s_obs.omega_e > 0.0f) ? s_obs.omega_e : -s_obs.omega_e;

    app_sfoc_release();
    log_printf("ERR obs lost lock (|e| %.2f V, expect %.2f V), drive released",
               (double)e_abs, (double)(w_abs * MOTOR_KE_V_RAD_S));
}

uint8_t app_sfoc_on(void)   { return s_sfoc; }

uint8_t app_ho_state(void) { return s_ho_state; }
float   app_ho_w(void)     { return s_ho_w; }
float   app_ho_dth(void)   { return s_ho_dth; }
float   app_ho_ds(void)    { return s_ho_ds; }
void    app_ho_reset(void)
{
    s_sfoc     = 0u;
    s_ho_state = 0u;
    s_ho_cnt   = 0u;
    s_ho_w     = 0.0f;
}
void    app_ho_arm(void)
{
    s_sfoc     = 0u;          /* leave sensorless mode while re-arming */
    s_ho_state = 1u;
    s_ho_cnt   = 0u;
    s_ho_w     = 0.0f;
}

uint8_t app_if_phase(void) { return s_if.phase; }
uint16_t app_ho_cnt(void)   { return s_ho_cnt; }

uint8_t app_if_on(void)      { return s_if.run; }
float   app_if_theta(void)   { return s_if.theta; }
float   app_if_freq_hz(void) { return s_if.f_hz; }
float   app_if_amp(void)     { return s_if.i_now; }

uint8_t app_obs_set_on(uint8_t on)
{
    s_obs_on = (on != 0u) ? 1u : 0u;
    if (s_obs_on != 0u) {
        /* Start from the encoder angle when it is valid, otherwise 0:
         * the PLL converges either way, this only avoids a long swing. */
        float th0 = (mid_enc_info()->valid != 0u)
                  ? (mid_enc_info()->elec_deg * 0.0174532925199433f) : 0.0f;

        alg_obs_reset(&s_obs, th0);
    }
    return s_obs_on;
}

uint8_t app_obs_get_on(void)
{
    return s_obs_on;
}

uint8_t app_obs_set_bw(float wn_hz)
{
    if ((wn_hz < 5.0f) || (wn_hz > 500.0f)) {
        return 1u;
    }
    alg_pll_set_bw(&s_obs.pll, wn_hz, 0.707f);
    return 0u;
}

uint8_t app_obs_set_lpf(float hz)
{
    if ((hz < 0.0f) || (hz > 8000.0f)) {
        return 1u;
    }
    s_obs.lpf_hz = hz;
    s_obs.lpf_k  = (hz > 0.0f) ? (1.0f / (6.283185307179586f * hz)) : 0.0f;
    return 0u;
}

void app_obs_set_comp(uint8_t mode)
{
    s_obs.comp = (mode <= 2u) ? mode : 0u;
}

void app_obs_set_trim(float deg)
{
    s_obs.theta_off = deg * 0.0174532925199433f;
}

void app_obs_set_delay(float us)
{
    if ((us >= 0.0f) && (us <= 500.0f)) {
        s_obs.delay_s = us * 0.000001f;
    }
}

void app_obs_set_rl(float r_ohm, float l_h)
{
    if (r_ohm > 0.0f) { s_obs.r_ohm = r_ohm; }
    if (l_h   > 0.0f) { s_obs.l_h   = l_h;   }
}

void app_obs_reset(float theta)
{
    alg_obs_reset(&s_obs, theta);
}

const void *app_obs_info(void)
{
    return (const void *)&s_obs;
}

const app_status_t *app_status(void)
{
    return &s_st;
}

/* ------------------------------------------------------------------ */
/* Key action                                                          */
/*                                                                     */
/* Press + release : take the held pose as zero, start the drive and    */
/*                   engage BAL_KEY_START_DELAY_MS after the release.   */
/* Long press      : the same, but engage immediately (bench use).      */
/* Press while armed: disarm (the safety valve that needs no console).  */
/* ------------------------------------------------------------------ */
/* Cheap runtime invariant: the FPU must stay enabled.
 *
 * A VFP instruction with CP10/CP11 switched off raises UsageFault/UNDEFINSTR,
 * which is exactly the signature of the 2026-11-07 bench crash (PC on a
 * VMUL.F32). If a stray write ever clears CPACR we want to see it happen
 * instead of finding the corpse afterwards, so this runs every 256 loops,
 * logs the first few events and restores the enable bits. */
static uint32_t s_cpacr_bad;
static uint32_t s_sp_bad;
static uint32_t s_lspact_seen;
static uint32_t s_ring_bad;
static uint32_t s_state_bad;
static uint32_t s_r4_bad;

static void guard_check(void)
{
    if (bsp_stack_sp_check() == 0u)
    {
        ++s_sp_bad;
        if (s_sp_bad <= 3u)
        {
            log_printf("[GUARD] sp out of window: %x", (unsigned)bsp_stack_sp());
        }
    }
    uint32_t cpacr;
    uint32_t ring;
    uint32_t bad;

    if ((s_loop_cnt & 0xFFu) != 0u)
    {
        return;
    }

    /* Ring descriptors and the balance state are the two places a stray write
     * would turn into a wild pointer later. Check them while they are still
     * describable: a corrupted ringbuf_t (.buf) makes the next ring_push()
     * store into an address nobody intended, which is exactly how the bench
     * crashes presented (BFAR just above RAM). */
    /* Every 256 loops: is the FPU still owing a context save while normal code
     * runs? LSPACT set here means a stale FPCAR will be used for the deferred
     * save, i.e. float registers land on top of somebody's stack frame. */
    if ((bsp_sys_fpccr() & 0x00000001u) != 0u)
    {
        ++s_lspact_seen;
    }

    ring = bsp_uart_ring_health();
    if ((ring != 0u) && (s_ring_bad == 0u))
    {
        log_printf("[GUARD] ring struct corrupt: %x", (unsigned)ring);
    }
    s_ring_bad = ring;

    bad = 0u;
    if ((bal_p()->phi_deg != bal_p()->phi_deg) || (fabsf(bal_p()->phi_deg) > 200.0f)) { bad |= 1u; }
    if (fabsf(bal_p()->tau_mnm) > 500.0f)                                        { bad |= 2u; }
    if (s_run.state > (uint8_t)BAL_ST_TRIP)                                   { bad |= 4u; }
    if (s_run.trip > (uint8_t)BAL_TRIP_HELD)                                { bad |= 8u; }
    if ((bad != 0u) && (s_state_bad == 0u))
    {
        log_printf("[GUARD] balance state corrupt: %x, phi %.2f, tau %.2f, state %u, trip %u",
                   (unsigned)bad, (double)bal_p()->phi_deg, (double)bal_p()->tau_mnm,
                   (unsigned)s_run.state, (unsigned)s_run.trip);
    }
    s_state_bad = bad;

    cpacr = (uint32_t)0x00F00000u;   /* host build: pretend it is fine */
#ifdef SCB
    cpacr = SCB->CPACR;
#endif
    if ((cpacr & 0x00F00000u) == 0x00F00000u)
    {
        return;
    }
    ++s_cpacr_bad;
#ifdef SCB
    SCB->CPACR = cpacr | 0x00F00000u;
    __DSB();
    __ISB();
#endif
    if (s_cpacr_bad <= 3u)
    {
        log_printf("[GUARD] CPACR lost the FPU: %x, restored", (unsigned)cpacr);
    }
}

void app_guard_r4_bad(uint32_t before, uint32_t after)
{
    ++s_r4_bad;
    if (s_r4_bad <= 3u)
    {
        log_printf("[GUARD] r4 clobbered across alg_bal_step: %x -> %x",
                   (unsigned)before, (unsigned)after);
    }
}

uint32_t app_guard_r4_count(void)
{
    return s_r4_bad;
}

uint32_t app_guard_ring_bad(void)
{
    return s_ring_bad;
}

uint32_t app_guard_state_bad(void)
{
    return s_state_bad;
}

uint32_t app_guard_cpacr_bad(void)
{
    return s_cpacr_bad;
}

uint32_t app_guard_sp_bad(void)
{
    return s_sp_bad;
}

uint32_t app_guard_lspact(void)
{
    return s_lspact_seen;
}

static void key_action(uint32_t now)
{
    static uint32_t s_key_t_down = 0u;
    static uint8_t  s_key_wait_release = 0u;   /* pressed, waiting for release */
    static uint8_t  s_key_fired = 0u;          /* the long press already acted */
    static uint32_t s_key_start_at = 0u;       /* 0 = nothing scheduled        */

    /* Scheduled start: counted from the release, so the operator has the whole
     * delay to settle the cube and get the hand away before torque appears. */
    if ((s_key_start_at != 0u) && ((int32_t)(now - s_key_start_at) >= 0))
    {
        uint8_t rc;

        s_key_start_at = 0u;
        rc = app_balance_start_now();
        if (rc == 0u)
        {
            bsp_led_write(1u);
            log_printf("[KEY] start -> balance ENGAGED (zero taken)");
        }
        else
        {
            bsp_led_write(0u);
            log_printf("[KEY] start -> balance refused (rc %u)", (unsigned)rc);
        }
    }

    if (bsp_key_is_down() != 0u)
    {
        if (s_key_t_down == 0u)
        {
            s_key_t_down = now;
        }

        /* The press edge is consumed while still held, so the countdown starts
         * from the release and not from whenever the debounce noticed it. */
        if (bsp_key_take_press())
        {
            /* app_balance_info() takes &s_run from the literal pool, so this
             * path cannot be poisoned by a write into the pointer alias. */
            if (app_balance_info()->state != (uint8_t)BAL_ST_DISARMED)
            {
                (void)app_balance_arm(0u);
                bsp_led_write(0u);
                s_key_wait_release = 0u;
                s_key_start_at = 0u;
                log_printf("[KEY] pressed -> balance DISARMED");
            }
            else
            {
                s_key_start_at = 0u;           /* a new press restarts the plan */
                s_key_wait_release = 1u;
                bsp_led_toggle();
                s_st.key_cnt = bsp_key_press_count();
                log_printf("[KEY] pressed #%u -> release to start, %u ms",
                           (unsigned)s_st.key_cnt,
                           (unsigned)BAL_KEY_START_DELAY_MS);
            }
        }

        if ((s_key_fired == 0u) &&
            ((uint32_t)(now - s_key_t_down) >= (uint32_t)BAL_KEY_HOLD_MS))
        {
            uint8_t rc;

            s_key_fired = 1u;
            s_key_wait_release = 0u;
            s_key_start_at = 0u;               /* long press overrides          */
            rc = app_balance_start_now();
            if (rc == 0u)
            {
                bsp_led_write(1u);
                log_printf("[KEY] hold %u ms -> balance ENGAGED (zero taken)",
                           (unsigned)BAL_KEY_HOLD_MS);
            }
            else
            {
                log_printf("[KEY] hold %u ms -> balance refused (rc %u)",
                           (unsigned)BAL_KEY_HOLD_MS, (unsigned)rc);
            }
        }
        return;
    }

    s_key_t_down = 0u;
    if (s_key_fired != 0u)
    {
        s_key_fired = 0u;
        (void)bsp_key_take_press();
        return;                                /* the long press already acted */
    }

    if (s_key_wait_release != 0u)
    {
        s_key_wait_release = 0u;
        s_key_start_at = now + (uint32_t)BAL_KEY_START_DELAY_MS;
        log_printf("[KEY] released -> engaging in %u ms, hand off the cube",
                   (unsigned)BAL_KEY_START_DELAY_MS);
    }
}

/* ------------------------------------------------------------------ */
/* Encoder poll while stopped (1kHz)                                   */
/*                                                                     */
/* The ADC trigger is gated by MOE, so no frames arrive while the      */
/* bridge is off and the 20kHz path in task_adc never runs. Poll the   */
/* encoder here instead so "enc" and "enc zero" work at standstill.    */
/* ------------------------------------------------------------------ */
static void task_enc_slow(uint32_t now)
{
    if (s_st.state == APP_ST_RUN) {
        return;                      /* task_adc already reads at 20kHz */
    }
    if ((uint32_t)(now - s_t_enc) < ENC_POLL_MS) {
        return;
    }
    s_t_enc = now;
    {
        uint32_t t0 = bsp_prof_now();

        mid_enc_update();
        bsp_prof_add(PROF_ENC, t0);
    }
}

/* ------------------------------------------------------------------ */
/* 1ms task: uptime and loop rate statistics                           */
/* ------------------------------------------------------------------ */
/* S7.4: speed step measurement, 1 kHz. Mirrors the current-loop "step". */
static void sstep_task(uint32_t now)
{
    float spd = mid_enc_info()->spd_rpm;
    float tgt = s_sstep.target;
    uint32_t el;
    float band;

    if (s_sstep_phase == 0u) {
        return;
    }

    if (s_sstep_phase == 1u) {          /* settle at zero speed */
        uint32_t st = (uint32_t)(now - s_sstep_t0);

        app_spd_off();
        /* Wait until the rotor has really stopped: only checking the
         * elapsed time left the previous run coasting, and a step that
         * starts from 1000 rpm looks like 67% overshoot. */
        if (((st >= (uint32_t)SSTEP_SETTLE_MS) &&
             (mid_enc_info()->spd_rpm < 20.0f) &&
             (mid_enc_info()->spd_rpm > -20.0f)) ||
            (st >= (uint32_t)SSTEP_SETTLE_MAX_MS)) {
            s_sstep_phase  = 2u;
            s_sstep_t0     = now;
            s_sstep_n      = 0u;
            s_sstep_t10    = 0u;
            s_sstep_t90    = 0u;
            s_sstep_settle = 0u;
            s_sstep_got10  = 0u;
            s_sstep_got90  = 0u;
            s_sstep_in_band = 0u;
            s_sstep_peak   = 0.0f;
            s_sstep_tail_min =  1e9f;
            s_sstep_tail_max = -1e9f;
            s_sstep_tail_sum = 0.0f;
            s_sstep_tail_n   = 0u;
            (void)app_spd_set(tgt);        /* the actual step */
        }
    } else if (s_sstep_phase == 2u) {   /* record the response */
        el = (uint32_t)(now - s_sstep_t0);
        s_sstep_n++;

        if (s_sstep_n > 2u) {
            float sgn = (tgt > 0.0f) ? 1.0f : -1.0f;
            float y   = spd * sgn;         /* signed towards the target */
            float a   = tgt * sgn;

            if ((s_sstep_got10 == 0u) && (y >= (0.1f * a))) {
                s_sstep_got10 = 1u;
                s_sstep_t10   = s_sstep_n;
            }
            if ((s_sstep_got90 == 0u) && (y >= (0.9f * a))) {
                s_sstep_got90 = 1u;
                s_sstep_t90   = s_sstep_n;
            }
            if (y > s_sstep_peak) { s_sstep_peak = y; }

            /* settle: inside +-5% of the target and staying there */
            band = 0.05f * a;
            if ((y > (a - band)) && (y < (a + band))) {
                if (s_sstep_in_band == 0u) {
                    s_sstep_in_band = 1u;
                    s_sstep_settle  = s_sstep_n;
                }
            } else {
                s_sstep_in_band = 0u;
                s_sstep_settle  = 0u;
            }

            /* tail window: last 100 ms */
            if ((uint32_t)el >= ((uint32_t)SSTEP_RECORD_MS - 100u)) {
                if (y < s_sstep_tail_min) { s_sstep_tail_min = y; }
                if (y > s_sstep_tail_max) { s_sstep_tail_max = y; }
                s_sstep_tail_sum += y;
                s_sstep_tail_n++;
            }
        }

        if (el >= (uint32_t)SSTEP_RECORD_MS) {
            float a = (tgt > 0.0f) ? tgt : -tgt;

            s_sstep_phase = 0u;
            s_sstep.valid = 1u;
            s_sstep.rise_ms = -1.0f;
            if ((s_sstep_got10 != 0u) && (s_sstep_got90 != 0u) &&
                (s_sstep_t90 >= s_sstep_t10)) {
                s_sstep.rise_ms = (float)(s_sstep_t90 - s_sstep_t10);
            }
            /* settle time from the step to entering the band, 1 kHz */
            s_sstep.settle_ms = (float)s_sstep_settle;

            s_sstep.overshoot = 0.0f;
            if (a > 5.0f) {
                s_sstep.overshoot = ((s_sstep_peak - a) / a) * 100.0f;
                if (s_sstep.overshoot < 0.0f) { s_sstep.overshoot = 0.0f; }
            }

            s_sstep.err  = 0.0f;
            s_sstep.ss   = 0.0f;
            s_sstep.ripple = 0.0f;
            if (s_sstep_tail_n != 0u) {
                float avg = s_sstep_tail_sum / (float)s_sstep_tail_n;
                s_sstep.ss = avg;
                if (a > 5.0f) {
                    s_sstep.err = ((avg - a) / a) * 100.0f;
                    if (s_sstep.err < 0.0f) { s_sstep.err = -s_sstep.err; }
                }
                s_sstep.ripple = s_sstep_tail_max - s_sstep_tail_min;
            }
            s_sstep.peak = s_sstep_peak;

            app_spd_off();
            log_printf("OK sstep %d rpm, rise %d ms, over %.1f%%, err %.1f%%",
                       (int)tgt, (int)s_sstep.rise_ms,
                       (double)s_sstep.overshoot, (double)s_sstep.err);
            log_printf(" settle %d ms, ss %d rpm, peak %d rpm, rip %d rpm",
                       (int)s_sstep.settle_ms, (int)s_sstep.ss,
                       (int)s_sstep.peak, (int)s_sstep.ripple);
        }
    }
}

/* S7.6: speed and bus-voltage protections, evaluated once per millisecond. */
/* Speed source for the protections: the encoder is the ground truth when it
 * is alive; in true encoder-less operation fall back to the observer (else the
 * stall judge would see 0 rpm and trip immediately). */
static float prot_speed_rpm(void)
{
    const mid_enc_t *e = mid_enc_info();

    if ((mid_enc_enabled() != 0u) && (e->valid != 0u)) {
        return e->spd_rpm;
    }
    if (s_sfoc != 0u) {
        return s_obs.omega_e * 9.54929658551372f / (float)ENC_POLE_PAIRS;
    }
    if (s_if.run != 0u) {
        /* I/f drag: the commanded frequency is the best estimate we have,
         * otherwise the stall judge would see 0 rpm and trip. */
        return s_if.f_hz * (60.0f / (float)ENC_POLE_PAIRS);
    }
    return 0.0f;
}

static void prot_check_1ms(uint32_t now)
{
    const mid_enc_t *e = mid_enc_info();
    float spd;
    float vbus;

    if (s_st.state != APP_ST_RUN) {
        s_prot_ov_ms = 0u;
        s_prot_stall_ms = 0u;
        s_prot_uv_ms = 0u;
        s_prot_ovv_ms = 0u;
        return;
    }

    /* Grace window: the first milliseconds after start are not representative
     * (the bus sags while the gate drivers come up, the speed estimate is
     * still empty). */
    s_spd_lp += (PROT_STALL_LP_K * (prot_speed_rpm() - s_spd_lp));

    if ((uint32_t)(now - s_prot_start_ms) < (uint32_t)PROT_GRACE_MS) {
        s_prot_ov_ms = 0u;
        s_prot_stall_ms = 0u;
        s_prot_uv_ms = 0u;
        s_prot_ovv_ms = 0u;
        return;
    }

    spd  = prot_speed_rpm();
    vbus = bsp_vbus_volts();

    /* (the stall judge low-pass lives just above, before the grace check) */

    /* ---- over-speed: any mode ---- */
    if ((spd > s_prot.overspeed_rpm) || (spd < -s_prot.overspeed_rpm)) {
        if (s_prot_ov_ms == 0u) {
            s_prot_ov_ms = now;
        } else if ((uint32_t)(now - s_prot_ov_ms) >= (uint32_t)s_prot.overspeed_ms) {
            app_fault_raise((uint8_t)APP_FAULT_OVERSPEED, spd);
            return;
        }
    } else {
        s_prot_ov_ms = 0u;
    }

    /* ---- stall: only meaningful while the speed loop owns the torque. In
     * torque mode holding the shaft still is a legitimate experiment. ---- */
    if ((s_spd.on != 0u) &&
        (s_spd_lp < s_prot.stall_rpm) && (s_spd_lp > -s_prot.stall_rpm) &&
        ((s_foc_info.iq_ref > s_prot.stall_iq_a) ||
         (s_foc_info.iq_ref < -s_prot.stall_iq_a))) {
        if (s_prot_stall_ms == 0u) {
            s_prot_stall_ms = now;
        } else if ((uint32_t)(now - s_prot_stall_ms) >= (uint32_t)s_prot.stall_ms) {
            app_fault_raise((uint8_t)APP_FAULT_STALL, s_foc_info.iq_ref);
            return;
        }
    } else {
        s_prot_stall_ms = 0u;
    }

    /* ---- bus voltage ---- */
    if (vbus < s_prot.uv_v) {
        if (s_prot_uv_ms == 0u) {
            s_prot_uv_ms = now;
        } else if ((uint32_t)(now - s_prot_uv_ms) >= (uint32_t)s_prot.vbus_ms) {
            app_fault_raise((uint8_t)APP_FAULT_UNDERVOLT, vbus);
            return;
        }
    } else {
        s_prot_uv_ms = 0u;
    }

    if (vbus > s_prot.ov_v) {
        if (s_prot_ovv_ms == 0u) {
            s_prot_ovv_ms = now;
        } else if ((uint32_t)(now - s_prot_ovv_ms) >= (uint32_t)s_prot.vbus_ms) {
            app_fault_raise((uint8_t)APP_FAULT_OVERVOLT, vbus);
            return;
        }
    } else {
        s_prot_ovv_ms = 0u;
    }
}

static void task_1ms(uint32_t now)
{
    uint32_t dt = (uint32_t)(now - s_t_1ms);
    uint32_t d1 = (uint32_t)(now - s_t_1k);

    /* ---- true 1 kHz work -------------------------------------------
     * NOTE: the dt >= 1000 block below is the once-per-SECOND stats
     * block, not a 1 ms one. Putting this work there made sstep advance
     * once per second (metrics empty) and made the bus-voltage filter
     * run at the loop rate. A dedicated timestamp is the simple fix. */
    if (d1 >= 1u) {
        s_t_1k = now;

        if (bsp_vbus_update() != 0u) {
            alg_foc_set_vdc(&s_foc, bsp_vbus_volts());
        }

        /* Ripple diagnostic, sampled at 1 kHz. */
        if (s_rip_on != 0u) {
            if ((uint32_t)(now - s_rip_t0) < (uint32_t)RIP_WINDOW_MS) {
                float iq = s_foc_info.iq;
                float sp = s_foc_info.spd_rpm;

                if (iq < s_rip.iq_min)  { s_rip.iq_min  = iq; }
                if (iq > s_rip.iq_max)  { s_rip.iq_max  = iq; }
                if (sp < s_rip.spd_min) { s_rip.spd_min = sp; }
                if (sp > s_rip.spd_max) { s_rip.spd_max = sp; }
                s_rip.n++;
            } else {
                s_rip_on = 0u;      /* done, keep the result */
            }
        }

        prot_check_1ms(now);
        sstep_task(now);

        /* S7.3: the start-up kick lasts SPD_KICK_MS after a target change only. */
        if ((s_spd.kick_on != 0u) &&
            ((uint32_t)(now - s_spd_kick_t0) >= (uint32_t)SPD_KICK_MS)) {
            s_spd.kick_on = 0u;
        }

        /* Acceleration limit: walk spd_ref towards the commanded target at
         * s_spd_ramp_hz rpm/s. 1 kHz, so one step is ramp/1000 rpm. */
        if (s_spd.on != 0u) {
            float step_rpm = s_spd_ramp_hz * 0.001f;
            float d = s_spd_target - s_spd.spd_ref;

            if (d > step_rpm) {
                s_spd.spd_ref += step_rpm;
            } else if (d < -step_rpm) {
                s_spd.spd_ref -= step_rpm;
            } else {
                s_spd.spd_ref = s_spd_target;
            }
        }
    }

    if (dt >= 1000u) {
        /* s_loop_cnt counts iterations over dt milliseconds.
         *   loops per second  = s_loop_cnt * 1000 / dt
         *   kHz               = loops per second / 1000 = s_loop_cnt / dt
         * (previous version forgot the second division and reported Hz as kHz) */
        s_st.loop_khz = (dt != 0u) ? (uint32_t)(s_loop_cnt / dt) : 0u;
        s_loop_cnt    = 0u;
        s_t_1ms       = now;
        s_st.rx_drop  = bsp_uart_rx_drop_count();
    }
}

/* ------------------------------------------------------------------ */
/* ADC task: consume the newest injected frame (20kHz)                 */
/* ------------------------------------------------------------------ */
static void task_adc(void)
{
    uint32_t n = bsp_adc_frames();
    uint32_t t0;
    uint32_t t_ref;                  /* ISR timestamp, snapshotted once */
    uint32_t t_enc;
    uint16_t raw[3];

    if (n == s_adc_seen) { return; }
    s_adc_seen = n;

    t_ref = bsp_adc_isr_cyc();

    t0 = bsp_prof_now();
    phase_mark(&s_phase.pick, t_ref);    /* how late the loop noticed the frame */
    bsp_adc_get_raw(raw);
    mid_current_update(raw);
    bsp_prof_add(PROF_CONV, t0);

    phase_mark_isr(bsp_adc_isr_cnt());

    if (s_st.mode == CTRL_FOC) {
        /* The current loop needs the angle for Park, so the encoder frame is
         * collected first. The SPI RX DMA was started in the ADC ISR, so the
         * data is normally already there (measured: 0 late frames at 20 kHz). */
        t_enc = bsp_prof_now();
        mid_enc_update();
        bsp_prof_add(PROF_ENC, t_enc);
    } else {
        /* S4: the V/f ramp runs once per ADC frame, right after the currents
         * are fresh. */
        task_vf(t0);
    }

    if ((s_st.mode == CTRL_FOC) && (s_st.state == APP_ST_RUN)) {
        task_foc(t0);
    }

    /* S5: in V/f the encoder is read AFTER the control chain, so its transfer
     * can never delay the duty update. FOC already collected it above. */
    if (s_st.mode == CTRL_FOC) {
        /* already read */
    } else if (ENC_DIV <= 1u) {
        t_enc = bsp_prof_now();
        mid_enc_update();
        bsp_prof_add(PROF_ENC, t_enc);
    } else {
        s_enc_div++;
        if (s_enc_div >= ENC_DIV) {
            s_enc_div = 0u;
            t_enc = bsp_prof_now();
            mid_enc_update();
            bsp_prof_add(PROF_ENC, t_enc);
        }
    }

    /* S5: supervised forced alignment (fixed vector already written) */
    if (s_st.state == APP_ST_ALIGN) {
        task_align();
    }
}

/* ------------------------------------------------------------------ */
/* VOFA+ waveform task                                                 */
/* ------------------------------------------------------------------ */
static void task_wave(uint32_t now)
{
    float    ch[WAVE_CH];
    float    a, b, c;
    uint32_t t0;
    const current_sample_t *cs;

    if (!s_st.wave_on) { return; }

    /* "now - last >= period" also absorbs HAL_GetTick jitter */
    if ((uint32_t)(now - s_t_wave) < (uint32_t)s_wave_period_ms) { return; }
    s_t_wave = now;

    t0 = bsp_prof_now();

    bsp_pwm_get_duty(&a, &b, &c);
    cs = mid_current_get();

    ch[0] = (float)now / 1000.0f;      /* uptime, s            */
    ch[1] = a;                         /* duty A               */
    ch[2] = b;                         /* duty B               */
    ch[3] = c;                         /* duty C               */
    ch[4] = cs->ia;                    /* phase currents, A    */
    ch[5] = cs->ib;
    ch[6] = cs->ic;
    ch[7] = mid_enc_info()->mech_deg;
    {
        /* ch8/ch9 follow the SAME priority as the control loop
         * (observer -> I/f -> encoder), so they stay meaningful with no
         * encoder fitted ("enc off"). ch7 is the raw mechanical angle and
         * therefore encoder-only. */
        float th_deg = 0.0f;
        float hz_use = 0.0f;

        /* Encoder first: while it is fitted, ch8/ch9 stay the raw encoder
         * reference so they can be compared against the observer (ch19) and
         * its difference (ch20). Only when the encoder is gone do they fall
         * back to whatever drives the loop, so they are never dead. */
        if ((mid_enc_enabled() != 0u) && (mid_enc_info()->valid != 0u)) {
            th_deg = mid_enc_info()->elec_deg;
            hz_use = mid_enc_info()->spd_hz;
        } else if (s_sfoc != 0u) {
            th_deg = s_obs.theta_out * 57.29577951308232f;
            hz_use = s_obs.omega_e / 6.283185307179586f;
        } else if (s_if.run != 0u) {
            th_deg = s_if.theta * 57.29577951308232f;
            hz_use = s_if.f_hz;
        } else if (s_coast != 0u) {
            /* #168 coast: the observer still tracks the spinning rotor with
             * the bridge in zero-current mode, so show it here instead of a
             * dead 0 Hz. */
            th_deg = s_obs.theta_out * 57.29577951308232f;
            hz_use = s_obs.omega_e / 6.283185307179586f;
        }

        ch[8] = th_deg;
        ch[9] = hz_use;   /* electrical Hz of whatever drives the loop */
    }
    /* S6: FOC currents and references. Plotting the measured and the
     * commanded value together is what makes a current step visible. */
    ch[10] = s_foc_info.id;              /* measured id, A   */
    ch[11] = s_foc_info.iq;              /* measured iq, A   */
    ch[12] = s_foc_info.id_ref;          /* commanded id, A  */
    ch[13] = s_foc_info.iq_ref;          /* commanded iq, A  */
    ch[14] = s_foc_info.vd;              /* V                */
    ch[15] = s_foc_info.vq;              /* V                */
    ch[16] = bsp_vbus_volts();           /* DC bus voltage, V */
    ch[17] = mid_enc_win_rpm();          /* window estimator, rpm */
    ch[18] = mid_enc_pll_rpm();          /* PLL observer, rpm     */
    /* S10.1: back-EMF observer against the encoder. ch19 = estimated
     * electrical angle, ch20 = wrapped difference to the encoder (deg).
     * If ch20 stays small the observer is good enough to take over. */
    ch[19] = s_obs.theta_out * 57.29577951308232f;
    {
        float de = s_obs.theta_out - (mid_enc_info()->elec_deg * 0.0174532925199433f);

        while (de >  3.14159265f) { de -= 6.283185307179586f; }
        while (de < -3.14159265f) { de += 6.283185307179586f; }
        ch[20] = de * 57.29577951308232f;
    }

    if (vofa_send(ch, (uint8_t)WAVE_CH) != 0u) {
        s_st.wave_frames++;
    } else {
        s_st.wave_drop++;
    }

    bsp_prof_add(PROF_WAVE, t0);
}

/* ------------------------------------------------------------------ */
/* Main loop                                                           */
/* ------------------------------------------------------------------ */
void app_loop(void)
{
    bsp_sys_ctx = BSP_CTX_LOOP;
    uint32_t now    = HAL_GetTick();
    uint32_t t_loop = bsp_prof_now();
    uint32_t t;

    s_st.uptime_ms = now;

    bsp_key_poll(now);
    key_action(now);

    t = bsp_prof_now();
    app_cmd_poll();
    bsp_prof_add(PROF_CMD, t);

    t = bsp_prof_now();
    bsp_uart_tx_pump();
    bsp_prof_add(PROF_TXPUMP, t);

    t = bsp_prof_now();
    imu_poll();              /* drain USART1 ring, feed the FDILink parser */
    bsp_prof_add(PROF_IMU, t);

    t = bsp_prof_now();
    app_att_update();        /* newest 0x41 frame -> body pitch (S2) */
    bsp_prof_add(PROF_ATT, t);

    t = bsp_prof_now();
    app_bal_update();        /* S4 shadow: compute only, never drives */
    bsp_prof_add(PROF_BAL, t);

    t = bsp_prof_now();
    app_balance_update();    /* S6: interlocked, only the ARMED/RUN path drives */
    bsp_prof_add(PROF_BALRUN, t);

    task_adc();
    task_enc_slow(now);
    task_1ms(now);
    task_wave(now);

    s_loop_cnt++;
    guard_check();
    bsp_prof_add(PROF_LOOP, t_loop);
}
