/**
  ******************************************************************************
  * @file    app_main.h
  * @brief   Application layer: state machine skeleton + main loop scheduling.
  *
  * S1 scope:
  *   - KEY -> LED toggle
  *   - state machine skeleton (IDLE / RUN / FAULT) switchable by command
  *   - serial debug (text commands + VOFA+ waveform)
  * Motor control (SVPWM / current loop) is added in S2..S6 by hooking into
  * the placeholders marked [S2] / [S3] ...
  ******************************************************************************
  */

#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Run state */
typedef enum {
    APP_ST_INIT = 0,
    APP_ST_IDLE,
    APP_ST_RUN,
    APP_ST_ALIGN,       /* forced alignment: fixed vector, rotor pulled to theta=0 */
    APP_ST_FAULT
} app_state_t;

/* ALIGN (electrical zero calibration) outcome */
typedef struct {
    uint8_t  active;        /* running right now                      */
    uint8_t  done;          /* finished at least once                  */
    uint8_t  fail;          /* 0 = ok, 1 = timeout, 2 = overcurrent,
                             * 3 = encoder or flash problem            */
    float    zero_deg;      /* captured zero when fail == 0            */
    float    m;             /* current modulation index during align   */
    float    err_deg;       /* measured electrical offset (deg)        */
    uint32_t n;             /* error samples accumulated               */
    float    err_fwd;       /* mean error, forward sweep (deg elec)    */
    float    err_rev;       /* mean error, reverse sweep (deg elec)    */
} app_align_t;

/* Start the alignment; requires the bridge running (start first).
 * Returns 0 on success, 2 = not running, 3 = already aligning. */
uint8_t app_align_start(void);
/* Open-loop pre-position: hold a fixed vector at theta=0 for
 * ALIGN_OL_MS and stop. Needs no encoder (S10.0c).
 * Returns 0 = OK, 2 = not running, 3 = already aligning. */
uint8_t app_align_ol_start(void);
/* ---- S10.1 back-EMF observer (shadow mode) ---- */
struct alg_obs_s;   /* opaque; see alg_obs.h */
uint8_t  app_obs_set_on(uint8_t on);   /* 0 = off, 1 = running       */
uint8_t  app_obs_get_on(void);
uint8_t  app_obs_set_bw(float wn_hz);  /* 0 = OK, 1 = rejected       */
void     app_obs_set_rl(float r_ohm, float l_h);
uint8_t  app_obs_set_lpf(float hz);    /* 0 = low pass off          */
void     app_obs_set_comp(uint8_t mode); /* 0 off,1 adv,2 ret  */
void     app_obs_set_trim(float deg);   /* static angle trim */
void     app_obs_set_delay(float us);  /* extra delay for the comp  */
void     app_obs_reset(float theta);
const void *app_obs_info(void);        /* cast to const alg_obs_t*   */

/* ---- S10.4 open-loop I/f startup ---- */
/* Force the current vector at <rpm> mechanical (0 = hold position).
 * Returns 0 = OK, 2 = not running (start the bridge first). */
uint8_t  app_if_start(float rpm);
void     app_if_stop(void);
uint8_t  app_if_on(void);
uint8_t  app_if_phase(void);
uint16_t app_ho_cnt(void);
float    app_if_theta(void);
float    app_if_freq_hz(void);
float    app_if_amp(void);

/* ---- S10.5 hand-over ---- */
void     app_ho_reset(void);      /* back to idle */
void     app_ho_arm(void);
uint8_t  app_sfoc_on(void);
void     app_sfoc_abort(void);  /* drop out of sensorless safely */     /* 1 = angle+speed from the observer */        /* start watching for the hand-over */
uint8_t  app_ho_state(void);      /* 0 idle, 1 watching, 2 blend, 3 done */
float    app_ho_w(void);          /* blend weight 0..1 */
float    app_ho_dth(void);        /* |theta_obs - theta_if|, rad */
float    app_ho_ds(void);         /* signed theta_obs - theta_if, rad */
void     app_if_set_amp(float a);      /* 0.05 .. I_MAX */
void     app_if_set_acc(float hz_s);   /* 2 .. 300 Hz/s  */
const app_align_t *app_align_info(void);

/* Control mode */
typedef enum {
    CTRL_IDLE = 0,      /* no PWM output */
    CTRL_OPENLOOP,      /* V/F open loop (S4) */
    CTRL_FOC            /* closed current loop (S6) */
} ctrl_mode_t;

typedef struct {
    uint32_t uptime_ms;
    uint8_t  state;
    uint8_t  mode;
    uint32_t key_cnt;
    uint32_t loop_khz;
    uint8_t  wave_on;
    uint16_t wave_rate_hz;
    uint32_t wave_frames;
    uint32_t wave_drop;
    uint32_t rx_drop;
    uint32_t tx_pending;
} app_status_t;

/* Open-loop V/f snapshot (S4), filled once per ADC frame while modulating. */
typedef struct {
    uint8_t on;          /* 1 = V/f is driving the bridge */
    uint8_t dir;         /* 0/1: 1 = reversed phase sequence */
    float   f_hz;        /* present electrical frequency */
    float   f_target;    /* ramp target */
    float   m;           /* modulation index actually applied */
    float   i_max;       /* largest of |ia|,|ib|,|ic| this frame */
} app_vf_info_t;

/* Control-chain phase probe. The ADC interrupt is the reference: "isr" records
 * where in the PWM period it fired (TIM1 CNT), the other three record how many
 * microseconds later each step ran (DWT). */
typedef struct {
    uint16_t cnt;
    uint16_t cnt_min;
    uint16_t cnt_max;
    uint8_t  valid;
} app_phase_isr_t;

typedef struct {
    uint32_t last_cyc;   /* DWT cycles since the ADC interrupt */
    uint32_t min_cyc;
    uint32_t max_cyc;
    uint8_t  valid;
} app_phase_t;

typedef struct {
    app_phase_isr_t isr;     /* ADC interrupt entry (the sample instant) */
    app_phase_t     pick;    /* ISR -> main loop picked the frame up    */
    app_phase_t     comp;    /* pick -> duties computed (V/f + SVPWM)   */
    app_phase_t     wr;      /* duties computed -> CCR1..3 written      */
} app_phase_info_t;

void                app_init(void);
void                app_loop(void);

const app_status_t *app_status(void);

/* 0 = OK, non-zero = rejection reason */
uint8_t app_start(void);
uint8_t app_stop(void);
uint8_t app_set_mode(uint8_t m);
void    app_wave_set(uint8_t on, uint16_t rate_hz);
void    app_counters_clear(void);

/* Open-loop V/f. set() starts modulating towards hz.
 * Returns: 0 = OK, 2 = not running (send start first), 3 = still calibrating. */
uint8_t app_vf_set(float hz);
uint8_t app_vf_off(void);
void    app_vf_set_acc(float hz_per_s);
uint8_t app_vf_set_dir(uint8_t dir);
const app_vf_info_t *app_vf_info(void);

/* ---------------- FOC current loop (S6) ---------------- */
typedef struct {
    float id_ref;
    float iq_ref;
    float id;
    float iq;
    float vd;
    float vq;
    float m;          /* modulation index used                */
    uint32_t faults;  /* frames aborted by the current guard  */
    float    spd_ref; /* ramped speed reference, rpm              */
    float    spd_target; /* commanded target, rpm                 */
    float    spd_rpm; /* measured speed used by the speed loop     */
    uint8_t  ready;   /* at least one FOC step has run        */
} app_foc_info_t;

/* Set the d/q current references (A). Requires RUN in CTRL_FOC.
 * Returns 0 = OK, 2 = not running, 4 = wrong mode, 5 = |I| out of range. */
uint8_t app_foc_set(float id, float iq);

/* Runtime PI tuning (takes effect immediately, integrators are reset).
 * Returns 0 = OK, 1 = value out of range, 4 = wrong mode. */
uint8_t app_foc_set_pi(float kp, float ki);

/* S7.2 feedforward: k_emf in V/(elec rad/s), l_h in H, gain 0..1. */
void app_foc_set_ff(float k_emf, float l_h, float gain);
void app_foc_get_ff(float *k_emf, float *l_h, float *gain);
void    app_foc_get_pi(float *kp, float *ki);

typedef struct {
    float   amp;          /* commanded iq step, A            */
    float   rise_ms;      /* 10% -> 90%                   */
    float   overshoot;    /* percent of the step             */
    float   err;          /* steady state error, percent     */
    float   ripple;       /* peak to peak in the tail, A     */
    float   ss;           /* measured steady state value, A   */
    float   peak;         /* highest measured value, A        */
    uint8_t valid;        /* results ready                   */
} app_step_info_t;

/* Run an automatic iq step (needs RUN + CTRL_FOC). The metric is computed
 * in firmware at 20 kHz, so it does not depend on the waveform rate.
 * Returns 0 = OK, 2 = not running, 4 = wrong mode, 5 = |I| out of range. */
/* ---------------- speed loop (S7.3) ---------------- */
/* Target in rpm, signed. 0 = speed loop off. */
uint8_t app_spd_set(float rpm);
void    app_spd_off(void);

/* ---------------- torque interface (S3) ---------------- */
/* Hold a wheel torque in mN.m; the q-axis current loop is the actuator.
 * Returns 0 = OK, 2 = not running, 3 = zero drift still measuring,
 * 4 = CTRL_FOC required, 5 = above TORQUE_MAX_MNM (or the current limit),
 * 6 = the speed loop is running and owns iq_ref. */
uint8_t app_torque_set(float mnm);
void    app_torque_off(void);
float   app_torque_get(void);
uint8_t app_spd_set_pi(float kp, float ki);
void    app_spd_get_pi(float *kp, float *ki);

typedef struct {
    float   target;       /* commanded speed, rpm            */
    float   rise_ms;      /* 10% -> 90%                      */
    float   overshoot;    /* percent of the step             */
    float   err;          /* steady state error, percent     */
    float   settle_ms;    /* time to enter +-5% and stay     */
    float   ripple;       /* peak to peak in the tail, rpm   */
    float   ss;           /* measured steady state, rpm      */
    float   peak;         /* highest measured, rpm           */
    uint8_t valid;
} app_sstep_info_t;

/* Automated speed step (needs RUN + CTRL_FOC).
 * Returns 0 = OK, 2 = not running, 4 = wrong mode, 5 = too fast. */
uint8_t app_sstep_start(float rpm);
const app_sstep_info_t *app_sstep_info(void);

/* Friction feedforward: base current + slope per rpm (option C, S7.3).
 * Returns 0 = OK, 1 = out of range. */
uint8_t app_spd_set_ff(float base_a, float slope_a_per_rpm);
void    app_spd_get_ff(float *base_a, float *slope_a_per_rpm);

uint8_t app_step_start(float amp);
const app_step_info_t *app_step_info(void);
/* ---------------- protections (S7.6) ---------------- */
typedef enum {
    APP_FAULT_NONE = 0,
    APP_FAULT_OVERCURRENT = 1,
    APP_FAULT_OVERSPEED  = 2,
    APP_FAULT_STALL      = 3,
    APP_FAULT_UNDERVOLT  = 4,
    APP_FAULT_OVERVOLT   = 5,
    APP_FAULT_ENCODER    = 6,
    APP_FAULT_COUNT      = 7
} app_fault_t;

typedef struct {
    float    oc_a;          /* over-current trip, A */
    float    overspeed_rpm;
    uint16_t overspeed_ms;
    float    stall_rpm;
    float    stall_iq_a;
    uint16_t stall_ms;
    float    uv_v;
    float    ov_v;
    uint16_t vbus_ms;
} app_prot_t;

typedef struct {
    uint8_t  code;        /* last fault, APP_FAULT_NONE when clean   */
    float    value;       /* the measurement that tripped it         */
    uint32_t uptime_ms;   /* when it happened                        */
    uint16_t count[APP_FAULT_COUNT];
} app_fault_info_t;

/* Raise a fault: bridge off, speed loop released, references cleared. */
void app_fault_raise(uint8_t code, float value);
const app_fault_info_t *app_fault_info(void);
const app_prot_t       *app_prot_info(void);

typedef struct {
    float iq_min, iq_max;     /* measured iq over the window, A   */
    float spd_min, spd_max;   /* measured speed, rpm              */
    uint16_t n;               /* samples collected                */
} app_rip_t;

/* Start (or restart) a ripple measurement. */
void app_rip_start(void);
const app_rip_t *app_rip_info(void);
uint8_t app_prot_set(uint8_t which, float value);

const app_foc_info_t *app_foc_info(void);

/* S2: refresh the attitude layer from the newest 0x41 frame. Safe to call
 * from the main loop and from a blocking command: a repeated frame is a
 * no-op. */
void app_att_update(void);

/* ---------------- balance law, shadow mode (S4) ---------------- */
/* While "on" the law is evaluated at the attitude rate but the result is
 * ONLY stored and printed: this layer never calls app_torque_set(). The first
 * closed-loop run is a separate, explicitly requested step (docs/44 5.6). */
typedef struct {
    uint8_t  on;          /* 1 = evaluating                     */
    uint8_t  valid;       /* 1 = a fresh attitude sample was used */
    uint8_t  saturated;   /* 1 = the command hit BAL_MAX_MNM      */
    uint8_t  deadband;    /* 1 = the command was rounded to zero  */
    uint8_t  att_valid;   /* attitude layer's own flag            */
    float    phi_deg;     /* tilt fed to the law (body frame)     */
    float    dphi_dps;    /* tilt rate, deg/s                     */
    float    omega_rpm;   /* wheel speed, rpm                     */
    float    tau_mnm;     /* command that WOULD be sent           */
    float    tau_kp_mnm;  /* term breakdown, mN.m                 */
    float    tau_kd_mnm;
    float    tau_kw_mnm;
    float    tau_ff_mnm;  /* friction feed-forward applied        */
    float    trim_deg;    /* runtime zero trim in use             */
    uint32_t n;           /* evaluations since "bal on"           */
} app_bal_info_t;

/* 0 = OK, 2 = no attitude frame yet, 4 = balance tuning rejected. */
uint8_t app_bal_set(uint8_t on);
const app_bal_info_t *app_bal_info(void);
/* Runtime zero trim, clamped to +-BAL_TRIM_MAX_DEG, RAM only. Returns the value
 * actually stored so the caller can print it. */
float app_bal_trim_set(float deg);
float app_bal_trim_get(void);
void  app_bal_trim_learn(float deg);
void  app_bal_auto_set(uint8_t on);
uint8_t app_bal_auto_get(void);
/* One fixed-point step that centres the body so the wheel can stop:
 *     trim -= omega_rpm / (kp/kw in rpm per degree)
 * Returns 0 OK, 2 not running, 3 no attitude, 4 bad gains, 5 wheel reading out
 * of the trusted band (|omega| > 1200 rpm). Manual override: closes the walk. */
uint8_t app_bal_snap(void);

/* Number of times the FPU enable bits in CPACR had to be restored. */
uint32_t app_guard_cpacr_bad(void);
/* Evaluate once. Called from app_loop(); also from blocking commands so the
 * sample they print is the newest one. */
void app_bal_update(void);

/* ---------------- closed-loop orchestration (S6) ---------------- */
/* The law stays a pure function; this layer decides when it is allowed to
 * push, how fast the command may move, and when to cut the torque. It is the
 * only place that calls app_torque_set() on behalf of the balance loop. */
typedef enum {
    BAL_ST_DISARMED = 0,  /* idle, must not drive                       */
    BAL_ST_ARMED,         /* conditions met, waiting for a new frame    */
    BAL_ST_RUN,           /* pushing torque                             */
    BAL_ST_TRIP           /* stopped by an interlock, needs "bal run off" */
} bal_run_state_t;

typedef enum {
    BAL_TRIP_NONE = 0,
    BAL_TRIP_TILT,        /* |theta| past BAL_TRIP_DEG                   */
    BAL_TRIP_RATE,        /* |theta_dot| past BAL_TRIP_DPS               */
    BAL_TRIP_WHEEL,       /* |omega| past BAL_TRIP_RPM                   */
    BAL_TRIP_STALE,       /* attitude frame too old                      */
    BAL_TRIP_FAULT,       /* any drive fault                             */
    BAL_TRIP_TIMEOUT,     /* BAL_MAX_MS elapsed                          */
    BAL_TRIP_DRIVE,       /* app_torque_set() refused                    */
    BAL_TRIP_HELD         /* big torque asked, body not moving: held     */
} bal_trip_t;

typedef struct {
    uint8_t  state;        /* bal_run_state_t                          */
    uint8_t  trip;         /* bal_trip_t, valid when state == TRIP     */
    uint8_t  drive_ret;    /* last app_torque_set() return code        */
    uint8_t  engaged;      /* 0 = armed but waiting for release        */
    float    tau_applied_mnm;
    float    tau_peak_mnm; /* max |command| this run                   */
    float    phi_peak_deg; /* max |theta| this run                     */
    float    omega_peak_rpm;
    uint32_t run_ms;       /* since ARM                                */
    uint32_t frames;       /* frames that produced a new command       */
    uint32_t sat_frames;   /* frames where the law saturated           */
} app_bal_run_t;

/* 0 = OK; 1 = attitude invalid; 2 = tilt outside ARM window;
 * 3 = tilt rate too high; 4 = wheel spinning too fast;
 * 5 = drive not ready (see app_bal_run()->drive_ret); 6 = already armed. */
uint8_t app_balance_arm(uint8_t on);

/* Button start: take the pose being held as zero, arm, and engage at once.
 * Returns the same codes as app_balance_arm() (0 = engaged). */
uint8_t app_balance_start_now(void);

void    app_balance_update(void);
const app_bal_run_t *app_balance_info(void);
const char *app_balance_state_name(uint8_t st);
const char *app_balance_trip_name(uint8_t trip);

const app_phase_info_t *app_phase_info(void);
void                    app_phase_reset(void);

const char *app_state_name(uint8_t st);
const char *app_mode_name(uint8_t m);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MAIN_H */
