/**
  ******************************************************************************
  * @file    app_cmd.c
  * @brief   Host command parsing and dispatch.
  *
  * All output is plain ASCII on purpose: serial terminals, VOFA+ and Keil
  * ARMCC V5 all handle ASCII identically. Chinese text lives in /docs only.
  ******************************************************************************
  */

#include "app_cmd.h"
#include <math.h>
#include "app_main.h"
#include "alg_obs.h"
#include "board_cfg.h"
#include "bsp_adc.h"
#include "bsp_cordic.h"
#include "bsp_crash.h"
#include "bsp_img.h"
#include "bsp_wp.h"
#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "bsp_prof.h"
#include "bsp_vbus.h"
#include "bsp_pwm.h"
#include "bsp_ssi.h"
#include "bsp_uart.h"
#include "bsp_stack.h"
#include "bsp_sys.h"
#include "mid_att.h"
#include "mid_current.h"
#include "mid_enc.h"
#include "mid_log.h"
#include "mid_trig.h"
#include "mid_vofa.h"

#include <string.h>

#define CMD_LINE_MAX    64u

static char    s_line[CMD_LINE_MAX];
static uint8_t s_line_len;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static uint8_t split_args(char *s, char **tok, uint8_t max_tok)
{
    uint8_t n = 0u;

    while ((*s != '\0') && (n < max_tok)) {
        while ((*s == ' ') || (*s == '\t')) { s++; }
        if (*s == '\0') { break; }
        tok[n++] = s;
        while ((*s != '\0') && (*s != ' ') && (*s != '\t')) { s++; }
        if (*s != '\0') { *s++ = '\0'; }
    }
    return n;
}

static uint8_t parse_u32(const char *s, uint32_t *out)
{
    uint32_t v = 0u;
    uint8_t  digits = 0u;

    if (s == NULL) { return 0u; }
    while ((*s >= '0') && (*s <= '9')) {
        v = v * 10u + (uint32_t)(*s - '0');
        s++;
        digits++;
        if (digits > 9u) { return 0u; }
    }
    if ((digits == 0u) || (*s != '\0')) { return 0u; }
    *out = v;
    return 1u;
}

/* Duty argument: "0.5", ".5", "1", "0.25". Values above 1 are accepted and
 * clamped by bsp_pwm (the applied value is echoed back). */
static uint8_t parse_duty(const char *s, float *out)
{
    float   v = 0.0f;
    float   scale = 0.1f;
    uint8_t digits = 0u;

    if (s == NULL) { return 0u; }
    while ((*s >= '0') && (*s <= '9')) {
        v = v * 10.0f + (float)(*s - '0');
        s++;
        digits++;
        /* 5 digits: the largest runtime value is ki = 50000. Duties and rates
         * stay far below that, so widening the limit changes nothing there. */
        if (digits > 5u) { return 0u; }
    }
    if (*s == '.') {
        s++;
        while ((*s >= '0') && (*s <= '9')) {
            v += (float)(*s - '0') * scale;
            scale *= 0.1f;
            s++;
            digits++;
            if (digits > 6u) { return 0u; }
        }
    }
    if ((digits == 0u) || (*s != '\0')) { return 0u; }
    *out = v;
    return 1u;
}

/* Signed variant for the FOC current references (the q axis goes negative
 * when the motor is braked or driven backwards). */
static uint8_t parse_num(const char *s, float *out)
{
    uint8_t neg = 0u;
    float   v;

    if ((s != 0) && (*s == (char)0x2D)) {   /* "-" */
        neg = 1u;
        s++;
    }
    if (!parse_duty(s, &v)) {
        return 0u;
    }
    *out = (neg != 0u) ? -v : v;
    return 1u;
}

/* ------------------------------------------------------------------ */
/* Information screens                                                 */
/* ------------------------------------------------------------------ */
static void print_help(void)
{
    log_printf("---- commands ----");
    log_printf(" help           show this list");
    log_printf(" ver            version and board info");
    log_printf(" stat           runtime status snapshot");
    log_printf(" uart           debug link state: ring, gState, retries");
    log_printf(" crash [clear]  fault record kept in the backup registers");
    log_printf(" crc [save]     flash image CRC (save = store as baseline)");
    log_printf(" wp [on|off|test] DWT write watchpoint on the balance alias");
    log_printf(" fpu            FP lazy-stacking state; 'fpu eager on|off' switches it");
    log_printf(" start          start (select a mode first)");
    log_printf(" stop           stop and shut the bridge down");
    log_printf(" mode <0|1|2>   0=IDLE 1=OPENLOOP 2=FOC");
    log_printf(" pwm <d>        all three duties 0..1 (needs start)");
    log_printf(" pwm <a> <b> <c> three independent duties");
    log_printf(" cur            raw codes and phase currents");
    log_printf(" vf <hz>        open-loop V/f target, electrical Hz (needs start)");
    log_printf(" id <A>         FOC d-axis current reference (mode 2)");
    log_printf(" iq <A>         FOC q-axis current reference (mode 2)");
    log_printf(" foc            show the FOC currents and voltages");
    log_printf(" pi <kp> <ki>   FOC PI gains (runtime), pi def restores");
    log_printf(" step <A>       measure an iq step at 20 kHz, prints the result");
    log_printf(" ff <0..1>      back-EMF + decoupling feedforward gain");
    log_printf(" vbus           measured DC bus voltage and its status");
    log_printf(" rip            peak-to-peak iq and speed over 500 ms");
    log_printf(" fault          last fault and per-type counters");
    log_printf(" prot [os|st|siq|uv|ov <v>]  protection thresholds");
    log_printf(" spd <rpm>      speed loop target, spd off releases iq");
    log_printf(" torque <mN.m>  hold a wheel torque 0..34 mN.m, torque off releases");
    log_printf(" spi <kp> <ki>  speed PI gains, spi shows them");
    log_printf(" sstep <rpm>    measure a speed step at 1 kHz, prints the result");
    log_printf(" sff [<base> <slope>]  friction feedforward, slope is A per 1000 rpm");
    log_printf(" pll [on|off|wn <hz>|zeta <v>]  speed observer");
    log_printf(" vf off         stop modulating, back to the 50%% zero vector");
    log_printf(" acc <hz/s>     frequency ramp rate 1..200 Hz/s");
    log_printf(" dir <0|1>      phase sequence, 1 reverses the rotation");
    log_printf(" cal [erase]    zero-drift calibration (needs start), erase = wipe");
    log_printf(" prof [reset|on|off]  per-stage timing profile (DWT)");
    log_printf(" imu            IMU link health and the last attitude frame");
    log_printf(" att [zero|stat <s>]  body pitch, rate, zero and noise window");
    log_printf(" bal [on|off|log <s>] balance law, shadow mode (never drives)");
    log_printf(" bal trim <deg> move the zero so the wheel stops winding up");
    log_printf(" bal snap       one step: trim -= wheel / 573 rpm-per-deg");
    log_printf(" bal auto [on|off] let the wheel speed walk the zero (default on)");
    log_printf(" bal walk <k>   walk gain, deg per rpm per s (0 freezes, max 0.01)");
    log_printf(" bal run [on|off] closed loop with interlocks -- moves the motor");
    log_printf(" imu rate <hz> [save]  set the 0x41 AHRS packet rate (10..200)");
    log_printf(" imu cfg <text> forward one ASCII config command (e.g. imu cfg #fmsg)");
    log_printf(" phase [reset]  where each step lands in the 50us PWM period");
    log_printf(" trig           benchmark the sin/cos back end (LUT or CORDIC)");
    log_printf(" cordic         dump raw CORDIC readback for 4 angles");
    log_printf(" enc [zero [deg]|dir|raw]  angle, zero capture/restore, direction");
    log_printf(" ssi            last MT6701 frame bytes, CRC and resync counters");
    log_printf(" align [ol]     pull the rotor to a known angle (ol = no encoder needed)");
    log_printf(" obs [on|off|bw|r|l|lpf|comp]  back-EMF observer, shadow mode");
    log_printf(" if <rpm|off|i|acc>  open-loop I/f start (S10.4)");
    log_printf(" sfoc <go|off>  I/f then hand over to the observer (S10.5)");
    log_printf(" led <on|off|toggle>   drive the PF1 LED");
    log_printf(" wave <on [hz]|off>  waveform upload (21ch needs <=250)");
    log_printf(" rate <hz>      waveform rate 1..500 (FireWater)");
    log_printf(" clr            clear counters");
    log_printf(" echo <text>    loopback test");
    log_printf("------------------");
}

static void print_version(void)
{
    log_printf("board = %s", BOARD_NAME);
    log_printf("build = %s %s #%u", __DATE__, __TIME__, (unsigned)FW_BUILD_ID);
    log_printf("core  = %u Hz", (unsigned)SYS_CORE_CLK_HZ);
    log_printf("foc   = %u Hz, Ts = %u us",
               (unsigned)FOC_FREQ_HZ, (unsigned)FOC_TS_US);
    log_printf("pwm   = %u Hz, ARR = %u, DTG = %u",
               (unsigned)PWM_FREQ_HZ_CALC, (unsigned)PWM_ARR, (unsigned)PWM_DTG_REG);
    log_printf("cur   = Rs = %u mOhm, gain = %u, %u uA/LSB, range +-%u mA",
               (unsigned)(RSHUNT_OHM * 1000.0f + 0.5f), (unsigned)INA240_GAIN,
               (unsigned)(CUR_A_PER_LSB * 1000000.0f + 0.5f),
               (unsigned)(CUR_RANGE_A * 1000.0f + 0.5f));
    log_printf("vbus  = div = %u, %u uV/LSB, max = %u mV",
               (unsigned)(VBUS_DIV_RATIO + 0.5f),
               (unsigned)(VBUS_V_PER_LSB * 1000000.0f + 0.5f),
               (unsigned)(VBUS_MAX_MEASURABLE_V * 1000.0f + 0.5f));
    log_printf("uart  = %u baud (USART2 PA2/PA3, CH340N), VOFA+ = FireWater",
               (unsigned)DBG_UART_BAUD);
}

/* Where the zero-drift offsets came from (see CUR_SRC_* in mid_current.h) */
static const char *cal_src_str(uint8_t src)
{
    if (src == CUR_SRC_FLASH) { return "flash"; }
    if (src == CUR_SRC_LIVE)  { return "live"; }
    return "none";
}

static void print_stat(void)
{
    const app_status_t *st = app_status();

    log_printf("---- status ----");
    log_printf(" uptime   = %u ms", (unsigned)st->uptime_ms);
    log_printf(" state    = %s", app_state_name(st->state));
    log_printf(" mode     = %s", app_mode_name(st->mode));
    {
        float a, b, c;
        bsp_pwm_get_duty(&a, &b, &c);
        log_printf(" pwm      = %s, dtg=%u, duty=%.2f/%.2f/%.2f",
                   bsp_pwm_is_enabled() ? "ON" : "OFF",
                   (unsigned)bsp_pwm_get_dtg(), a, b, c);
    }
    {
        const current_sample_t *cs = mid_current_get();
        log_printf(" adc      = frames=%u ovr=%u cal=%u sat=%u src=%s gen=%u st=%u",
                   (unsigned)bsp_adc_frames(), (unsigned)bsp_adc_ovr_count(),
                   (unsigned)cs->ready, (unsigned)cs->saturated,
                   cal_src_str(cs->src), (unsigned)cs->gen,
                   (unsigned)bsp_adc_status());
    }
    log_printf(" led      = %s", bsp_led_is_on() ? "ON" : "OFF");
    log_printf(" key      = down=%u total=%u",
               (unsigned)bsp_key_is_down(), (unsigned)st->key_cnt);
    {
        const app_vf_info_t *v = app_vf_info();
        log_printf(" vf       = %s %.1f/%.1f Hz, m=%.2f, I=%.2f A, dir=%u",
                   v->on ? "ON" : "OFF", v->f_hz, v->f_target, v->m,
                   v->i_max, (unsigned)v->dir);
    }
    log_printf(" loop     = %u kHz", (unsigned)st->loop_khz);
    log_printf(" wave     = %s @ %u Hz, frames=%u drop=%u",
               st->wave_on ? "ON" : "OFF",
               (unsigned)st->wave_rate_hz,
               (unsigned)st->wave_frames,
               (unsigned)st->wave_drop);
    log_printf(" uart     = rx_drop=%u tx_pending=%u tx_stuck=%u",
               (unsigned)bsp_uart_rx_drop_count(),
               (unsigned)bsp_uart_tx_pending(),
               (unsigned)bsp_uart_tx_stuck_count());
    log_printf(" guard    = cpacr repairs %u, r4clob %u, ptrbad %u, ptrfix %u",
               (unsigned)app_guard_cpacr_bad(), (unsigned)app_guard_r4_count(),
               (unsigned)app_guard_att_ptr_count(),
               (unsigned)app_guard_ptr_fix_count());
    log_printf(" ptrs     = ptrfix %u, last bad %x (alias %u) at %u ms",
               (unsigned)app_guard_ptr_fix_count(),
               (unsigned)app_guard_ptr_bad_val(),
               (unsigned)app_guard_ptr_bad_which(),
               (unsigned)app_guard_ptr_bad_ms());

    log_printf(" wp       = armed %u, addr %x, hits %u, bad %u",
               (unsigned)bsp_wp_armed(), (unsigned)bsp_wp_addr(),
               (unsigned)bsp_wp_hits(), (unsigned)bsp_wp_bad());
    log_printf(" fpu      = fpccr %x, fpcar %x, lspact seen %u, save %s",
               (unsigned)bsp_sys_fpccr(), (unsigned)bsp_sys_fpcar(),
               (unsigned)app_guard_lspact(),
               (bsp_sys_fpu_eager_get() != 0u) ? "eager" : "lazy");
    log_printf(" enc      = badrun %u, resync %u (busy %u), ovr %u",
               (unsigned)mid_enc_bad_run(), (unsigned)bsp_ssi_resync_count(),
               (unsigned)bsp_ssi_resync_busy_count(),
               (unsigned)bsp_ssi_ovr_count());
    log_printf(" sp       = %x..%x, bad %u",
               (unsigned)bsp_stack_sp_lo(), (unsigned)bsp_stack_sp_hi(),
               (unsigned)app_guard_sp_bad());
    log_printf(" stack    = peak %u B of %u B",
               (unsigned)bsp_stack_peak_bytes(),
               (unsigned)bsp_stack_size_bytes());
    log_printf("----------------");
}

/* Debug link internals.  The interesting one is gState: if it stays 2
 * (HAL_UART_STATE_BUSY_TX) while tx_pending does not move, the HAL handle is
 * wedged and every later Transmit_DMA returns busy -- that is what made the
 * console go mute while the MCU kept running (18#T21). */
static void print_uart(void)
{
    log_printf("---- uart (USART2 debug) ----");
    log_printf(" baud   = %u", (unsigned)DBG_UART_BAUD);
    log_printf(" tx     = pending %u B, free %u B, idle %s",
               (unsigned)bsp_uart_tx_pending(), (unsigned)bsp_uart_tx_space(),
               (bsp_uart_tx_idle() != 0u) ? "yes" : "no");
    log_printf(" handle = gState %u, ErrorCode 0x%08X",
               (unsigned)bsp_uart_handle_state(), (unsigned)bsp_uart_handle_error());
    log_printf(" counts = stuck %u, busy %u, rx_drop %u",
               (unsigned)bsp_uart_tx_stuck_count(),
               (unsigned)bsp_uart_tx_busy_count(),
               (unsigned)bsp_uart_rx_drop_count());
    log_printf("----------------");
}

/* IMU status: link health first, then the last attitude.
 * The balance state machine will gate on imu_stream_ok(), so "age" is the
 * single number to look at when the cube refuses to enter balance. */
static void print_imu(void)
{
    const fdilink_ctx_t *ctx = imu_ctx();
    const fdilink_stat_t *st = &ctx->stat;
    fdilink_ahrs_t ahrs;

    log_printf("---- imu (USART1, FDILink) ----");
    log_printf(" link  = %s, %u baud, age %u ms (timeout %u)",
               (imu_stream_ok() != 0u) ? "OK" : "NO FRAME",
               (unsigned)IMU_UART_BAUD, (unsigned)imu_frame_age_ms(),
               (unsigned)IMU_FRAME_TIMEOUT_MS);
    log_printf(" rx    = %u B, drop=%u err=%u tx_stuck=%u",
               (unsigned)imu_rx_bytes(), (unsigned)imu_rx_drop(),
               (unsigned)imu_rx_error(), (unsigned)imu_tx_stuck());
    log_printf(" frame = %u ok (%u imu, %u ahrs, %u other)",
               (unsigned)st->frames, (unsigned)st->frames_imu,
               (unsigned)st->frames_ahrs, (unsigned)st->frames_other);
    log_printf(" error = crc8=%u crc16=%u tail=%u len=%u drop_byte=%u gap=%u/%u",
               (unsigned)st->err_crc8, (unsigned)st->err_crc16,
               (unsigned)st->err_tail, (unsigned)st->err_len,
               (unsigned)st->dropped_byte,
               (unsigned)st->seq_gap_imu, (unsigned)st->seq_gap_ahrs);
    if (imu_get_ahrs(&ahrs) != 0u) {
        log_printf(" ahrs  = roll %.2f pitch %.2f heading %.2f deg",
                   (double)fdilink_rad2deg(ahrs.roll),
                   (double)fdilink_rad2deg(ahrs.pitch),
                   (double)fdilink_rad2deg(ahrs.heading));
    } else {
        log_printf(" ahrs  = no 0x41 frame yet");
    }
    log_printf("----------------");
}

/* ---- IMU packet rate --------------------------------------------------
 * The module ships with 0x40 (raw) and 0x41 (AHRS) at 100 Hz each.  The
 * balance loop wants the attitude packet as fast as the link allows, so this
 * rewrites the 0x41 rate through the ASCII config channel.  Without "save"
 * the change is volatile and a power cycle restores 100 Hz. */

/* Keep feeding the parser while the module answers: replies land in the
 * ASCII ring only through imu_poll().  The debug UART is pumped from the main
 * loop, which is blocked here, so it needs an explicit pump as well - without
 * it the log ring overflows during a long config sequence and the diagnostic
 * lines are dropped. */
static void imu_wait(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - t0) < ms) {
        imu_poll();
        bsp_uart_tx_pump();
        HAL_Delay(1u);
    }
}

static uint8_t rep_has(const uint8_t *p, uint32_t n, const char *needle)
{
    uint32_t nl = (uint32_t)strlen(needle);
    uint32_t i;
    uint32_t k;

    if ((nl == 0u) || (n < nl)) { return 0u; }
    for (i = 0u; (i + nl) <= n; i++) {
        for (k = 0u; k < nl; k++) {
            if (p[i + k] != (uint8_t)needle[k]) { break; }
        }
        if (k == nl) { return 1u; }
    }
    return 0u;
}

static void print_imu_reply(const uint8_t *p, uint32_t n)
{
    char     s[160];
    uint32_t i;
    uint8_t  printable = 1u;

    for (i = 0u; i < n; i++) {
        if ((p[i] < 0x20u) && (p[i] != (uint8_t)'\r') && (p[i] != (uint8_t)'\n')) {
            printable = 0u;
        }
    }
    if (printable == 0u) {
        log_printf("imu < %u B (binary)", (unsigned)n);
        return;
    }
    for (i = 0u; (i < n) && (i < ((uint32_t)sizeof(s) - 1u)); i++) {
        char c = (char)p[i];
        s[i] = ((c == '\r') || (c == '\n')) ? ' ' : c;
    }
    s[i] = '\0';
    log_printf("imu < %s", s);
}

/* Last reply of imu_step(), kept so a caller can look for an echo after the
 * fact (e.g. "MSG_AHRS[41]  200.0Hz" proves the rate really changed). */
static uint8_t  s_rep[192];
static uint32_t s_rep_len;

static uint8_t imu_step(const char *cmd)
{
    log_printf("imu > %s", cmd);
    s_rep_len = imu_cmd(cmd, s_rep, (uint32_t)sizeof(s_rep), IMU_CMD_TIMEOUT_MS);
    if (s_rep_len == 0u) {
        log_printf("imu < no reply in %u ms", (unsigned)IMU_CMD_TIMEOUT_MS);
        return 0u;
    }
    print_imu_reply(s_rep, s_rep_len);
    return rep_has(s_rep, s_rep_len, "#OK");
}

/* Decimal formatting without printf: ARMCC's snprintf pulls in a lot of code
 * and the command strings here are short and fixed in shape. */
static void u32_str(uint32_t v, char *dst)
{
    char    tmp[12];
    uint8_t i = 0u;
    uint8_t k;

    if (v == 0u) { dst[0] = '0'; dst[1] = '\0'; return; }
    while ((v > 0u) && (i < (uint8_t)sizeof(tmp))) {
        tmp[i] = (char)('0' + (v % 10u));
        v /= 10u;
        i++;
    }
    for (k = 0u; k < i; k++) { dst[k] = tmp[i - 1u - k]; }
    dst[i] = '\0';
}

/* Total parsed frames; the only honest "is the stream back" test. A frame age
 * of a few hundred ms old still looks like "streaming" right after config mode,
 * so the age alone reports a false success. */
static uint32_t imu_frames(void)
{
    const fdilink_ctx_t *ctx = imu_ctx();

    return ctx->stat.frames;
}

/* Wake the stream and wait for real frames to arrive. After #freboot the module
 * needs ~2.3 s before it listens again, so a #fdeconfig sent immediately after
 * the reboot lands in that window and is lost; hence the periodic re-send. */
static uint8_t imu_wait_stream(uint32_t tries, uint32_t step_ms, uint32_t nudge_every)
{
    uint32_t frames0 = imu_frames();
    uint32_t i;

    for (i = 0u; i < tries; i++) {
        if (imu_frames() > (frames0 + 2u)) { return 1u; }
        if ((nudge_every != 0u) && (i > 0u) && ((i % nudge_every) == 0u)) {
            log_printf("imu > #fdeconfig (stream still down, nudge it)");
            (void)imu_send("#fdeconfig");
        }
        imu_wait(step_ms);
    }
    return (uint8_t)((imu_frames() > (frames0 + 2u)) ? 1u : 0u);
}

/* imu cfg <text>: forward one ASCII command verbatim and print the reply.
 * Needed for every command that has no dedicated entry point (e.g. #fmsg
 * with no arguments lists the current packet rates). */
static void cmd_imu_cfg(const char *text)
{
    uint8_t  reply[192];
    uint32_t n;

    log_printf("imu > %s", text);
    n = imu_cmd(text, reply, (uint32_t)sizeof(reply), IMU_CMD_TIMEOUT_MS);
    if (n == 0u) {
        log_printf("imu < no reply in %u ms", (unsigned)IMU_CMD_TIMEOUT_MS);
        return;
    }
    print_imu_reply(reply, n);
}

/* Wait until the binary stream actually stops. #fconfig is acknowledged with a
 * splice of "*#OK" and the frames still in flight, so the reply itself cannot
 * be judged; the honest signal is that no new frame arrives any more. Only then
 * does the module parse the next ASCII command - sending #fmsg 500 ms after
 * #fconfig got a bare "*#OK" back and changed nothing. */
static uint8_t imu_wait_cfg_quiet(void)
{
    uint32_t i;
    uint32_t last;
    uint32_t now_frames;
    uint32_t same = 0u;

    last = imu_frames();
    for (i = 0u; i < 25u; i++) {          /* up to ~3.7 s */
        imu_wait(150u);
        now_frames = imu_frames();
        if (now_frames == last) {
            same++;
            if (same >= 3u) { return 1u; } /* 450 ms without a frame */
        } else {
            same = 0u;
            last = now_frames;
        }
    }
    return 0u;
}

/* A rate reply is good only when the MSG_AHRS line carries a real value. Do
 * not test for the text "0.0Hz": it is a substring of "100.0Hz" and
 * "200.0Hz", which made every good echo look like a rejected one. Parse the
 * field instead: after "MSG_AHRS[41]" the value must start with 1..9, so an
 * out-of-range setting ("0.0Hz", e.g. 300 Hz) is the only rejected shape. */
static uint8_t imu_echo_ok(void)
{
    uint32_t i;
    uint32_t j;

    for (i = 0u; (i + 8u) <= s_rep_len; i++) {
        if (rep_has(&s_rep[i], 8u, "MSG_AHRS") != 0u) { break; }
    }
    if ((i + 8u) > s_rep_len) { return 0u; }
    j = i + 8u;
    while ((j < s_rep_len) && (s_rep[j] != (uint8_t)']')) { j++; }
    if (j >= s_rep_len) { return 0u; }
    j++;
    while ((j < s_rep_len) && (s_rep[j] == (uint8_t)' ')) { j++; }
    if (j >= s_rep_len) { return 0u; }
    return ((s_rep[j] >= (uint8_t)'1') && (s_rep[j] <= (uint8_t)'9')) ? 1u : 0u;
}

/* The module needs LINE SILENCE between #fconfig and the next command: 500 ms
 * after the *#OK a #fmsg is answered with a bare "*#OK" and ignored, and so is
 * a second and third one - every early command seems to restart the quiet
 * period. With ~2 s of nothing on the wire the rate table comes back and the
 * change sticks (verified against the module on 2026-11-04). */
static void imu_wait_line_quiet(void)
{
    imu_wait(2000u);
}

static void cmd_imu_rate(const char *hz_arg, uint8_t do_save)
{
    char     cmd[32];
    uint8_t  reply[192];
    uint32_t hz;
    uint32_t n;

    if (parse_u32(hz_arg, &hz) == 0u) {
        log_printf("ERR usage=imu rate <10..200> [save]");
        return;
    }
    /* 200 Hz is the module's own ceiling: #fmsg 41 300 is answered with
     * "MSG_AHRS[41] 0.0Hz" and quietly ignored (measured 2026-11-04). */
    if ((hz < 10u) || (hz > 200u)) {
        log_printf("ERR usage=imu rate <10..200> [save]");
        return;
    }
    log_printf("imu rate: 0x41 (AHRS) -> %u Hz, 0x40 (raw) untouched", (unsigned)hz);

    /* Enter config mode, then wait for the stream to actually stop: the reply
     * to #fconfig is normally mixed with frames still in flight, and the next
     * ASCII command is swallowed until the module is really in config mode. */
    (void)imu_step("#fconfig");
    if (imu_wait_cfg_quiet() == 0u) {
        log_printf("imu rate: the stream never stopped, module not responding");
        imu_exit_config_mode();
        return;
    }
    imu_wait(300u);
    imu_wait_line_quiet();
    log_printf("imu rate: module is in config mode");

    strcpy(cmd, "#fmsg 41 ");
    u32_str(hz, &cmd[9]);
    /* A bare "*#OK" is not proof: only the echo of the new rate is. */
    if ((imu_step(cmd) != 0u) && (imu_echo_ok() != 0u)) {
        log_printf("imu rate: module echoed the new rate");
    } else {
        log_printf("imu rate: WARNING #fmsg gave no rate echo, retrying once");
        imu_wait_line_quiet();
        if ((imu_step(cmd) != 0u) && (imu_echo_ok() != 0u)) {
            log_printf("imu rate: module echoed the new rate on retry");
        } else {
            log_printf("imu rate: WARNING rate not confirmed by the module");
        }
    }

    /* Read the rate table back: definitive proof of the pending setting. */
    imu_wait_line_quiet();
    if (imu_step("#fmsg") != 0u) {
        log_printf("imu rate: table read back above");
    }

    if (do_save != 0u) {
        imu_wait_line_quiet();
        (void)imu_step("#fsave");
        /* #fsave writes flash.  A #freboot only 400 ms later was accepted with
         * *#OK but the new rate was gone after the reboot; with 2500 ms of
         * quiet before the reboot the readback table and the rebooted stream
         * both came back at the new rate.  So: do not rush the reboot. */
        imu_wait(2500u);
        log_printf("imu > #freboot");
        n = imu_cmd("#freboot", reply, (uint32_t)sizeof(reply), 800u);
        if (n > 0u) {
            print_imu_reply(reply, n);
        }
        /* Only answer "y" when the prompt was actually seen: an unneeded y
         * returns *#ERROR, a missing one leaves the module stuck. */
        if ((n > 0u) && (rep_has(reply, n, "(y/n)") != 0u)) {
            log_printf("imu > y");
            (void)imu_send("y");
            imu_wait(400u);
            (void)imu_send("y");
            imu_wait(800u);
        } else if (n == 0u) {
            log_printf("imu > y (no reply, assume y/n prompt)");
            (void)imu_send("y");
            imu_wait(400u);
        } else {
            imu_wait(400u);
        }
        /* Wake the stream before waiting for frames: #fdeconfig also clears
         * the local ASCII flag, so binary frames are parsed again. Then retry
         * every 2 s for up to 30 s (the reboot itself takes ~2.3 s). */
        log_printf("imu > #fdeconfig (wake stream)");
        (void)imu_send("#fdeconfig");
        imu_wait(300u);
        if (imu_wait_stream(60u, 500u, 4u) == 0u) {
            log_printf("imu rate: stream did not resume in 30 s");
        }
    } else {
        log_printf("imu > #fdeconfig");
        (void)imu_send("#fdeconfig");
        imu_wait(300u);
        if (imu_wait_stream(20u, 250u, 4u) == 0u) {
            log_printf("imu rate: stream did not resume in 5 s");
        }
    }

    imu_exit_config_mode();
    if (imu_stream_ok() != 0u) {
        log_printf("imu rate: cfg off, frames = %u; check 'imu' for the new rate",
                   (unsigned)imu_frames());
    } else {
        log_printf("imu rate: cfg off but no frame yet, frames = %u",
                   (unsigned)imu_frames());
    }
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* att: body pitch layer (S2)                                          */
/* ------------------------------------------------------------------ */

static void print_att(void)
{
    const att_state_t *a = att_get();

    const char *axis = (ATT_AXIS == ATT_AXIS_ROLL) ? "roll " : "pitch";

    log_printf(" %s = %s%.2f deg, rate %s%.2f deg/s", axis,
               (a->theta_deg < 0.0f) ? "-" : "+", (double)fabsf(a->theta_deg),
               (a->theta_dot_dps < 0.0f) ? "-" : "+", (double)fabsf(a->theta_dot_dps));
    log_printf(" wheel  = %.0f rpm, frames = %u", (double)a->wheel_rpm, (unsigned)a->seq);
    log_printf(" link   = %s, age %u ms, zero %.2f deg",
               (a->valid != 0u) ? "OK" : "INVALID", (unsigned)a->age_ms,
               (double)att_offset_deg());
    log_printf(" limits = +-%.0f deg, timeout %u ms",
               (double)ATT_RANGE_DEG, (unsigned)ATT_TIMEOUT_MS);
}

/* Sample the attitude state for a fixed window and report spread and drift.
 * Blocking on purpose: on the bench this is the fastest way to answer
 * "how much does it wander" (A-1/A-6 in docs/42). The IMU ring is pumped
 * from imu_wait(), so 200 Hz frames keep arriving while we sit here. */
static void cmd_att_stat(const char *sec_arg)
{
    uint32_t sec;
    uint32_t t_end;
    uint32_t n = 0u;
    uint32_t bad = 0u;
    float    mn;
    float    mx;
    float    sum = 0.0f;
    float    first = 0.0f;
    float    last = 0.0f;
    float    v;

    if ((parse_u32(sec_arg, &sec) == 0u) || (sec == 0u) || (sec > 60u)) {
        log_printf("ERR usage=att stat <1..60>");
        return;
    }

    mn = 0.0f;
    mx = 0.0f;
    t_end = HAL_GetTick() + (sec * 1000u);

    while ((int32_t)(t_end - HAL_GetTick()) > 0)
    {
        imu_wait(5u);            /* keeps imu_poll() + the TX pump running */
        app_att_update();

        v = att_get()->theta_deg;
        if (att_get()->valid == 0u) {
            bad++;
        }
        if (n == 0u) {
            mn    = v;
            mx    = v;
            first = v;
        }
        if (v < mn) { mn = v; }
        if (v > mx) { mx = v; }
        sum += v;
        last = v;
        n++;
    }

    if (n == 0u) {
        log_printf("att stat: no sample");
        return;
    }

    log_printf("att stat %u s: n=%u, invalid=%u", (unsigned)sec, (unsigned)n, (unsigned)bad);
    log_printf(" min=%.2f max=%.2f pp=%.2f mean=%.2f deg",
               (double)mn, (double)mx, (double)(mx - mn), (double)(sum / (float)n));
    /* No "+" flag in the mini printf: build the sign by hand. */
    log_printf(" drift = %s%.2f deg over the window (first %.2f -> last %.2f)",
               ((last - first) < 0.0f) ? "-" : "+", (double)fabsf(last - first),
               (double)first, (double)last);
}

/* ------------------------------------------------------------------ */
/* bal: balance law shadow mode (S4)                                   */
/* ------------------------------------------------------------------ */

static void print_bal(void)
{
    const app_bal_info_t *b = app_bal_info();

    log_printf(" bal    = %s (shadow: computes, never drives)",
               (b->on != 0u) ? "on" : "off");
    log_printf(" state  = %s, attitude %s, n=%u",
               (b->valid != 0u) ? "VALID" : "stale",
               (b->att_valid != 0u) ? "OK" : "INVALID", (unsigned)b->n);
    log_printf(" input  = phi %s%.2f deg, rate %s%.2f deg/s, wheel %.0f rpm",
               (b->phi_deg < 0.0f) ? "-" : "+", (double)fabsf(b->phi_deg),
               (b->dphi_dps < 0.0f) ? "-" : "+", (double)fabsf(b->dphi_dps),
               (double)b->omega_rpm);
    log_printf(" tau    = %s%.2f mN.m (kp %s%.2f + kd %s%.2f + kw %s%.2f, ff %s%.2f)",
               (b->tau_mnm < 0.0f) ? "-" : "+", (double)fabsf(b->tau_mnm),
               (b->tau_kp_mnm < 0.0f) ? "-" : "+", (double)fabsf(b->tau_kp_mnm),
               (b->tau_kd_mnm < 0.0f) ? "-" : "+", (double)fabsf(b->tau_kd_mnm),
               (b->tau_kw_mnm < 0.0f) ? "-" : "+", (double)fabsf(b->tau_kw_mnm),
               (b->tau_ff_mnm < 0.0f) ? "-" : "+", (double)fabsf(b->tau_ff_mnm));
    log_printf(" flags  = %s, %s, ceiling %.0f mN.m, sign %s1",
               (b->saturated != 0u) ? "SATURATED" : "in range",
               (b->deadband != 0u) ? "deadband" : "active",
               (double)BAL_MAX_MNM,
               (BAL_SIGN < 0.0f) ? "-" : "+");
    log_printf(" zero   = trim %s%.2f deg (max %.1f), friction ff %.2f mN.m",
               (b->trim_deg < 0.0f) ? "-" : "+", (double)fabsf(b->trim_deg),
               (double)BAL_TRIM_MAX_DEG, (double)BAL_FF_MNM);
    log_printf(" auto   = %s, wheel walk %.0f e-6 deg per rpm per s, gate %.2f deg",
               (app_bal_auto_get() != 0u) ? "on" : "off (manual trim)",
               (double)(app_bal_walk_get() * 1000000.0f), (double)BAL_WALK_PHI_GATE_DEG);
}

/* Print one row per sample while the user rocks the cube by hand.
 * 20 ms rows = the 50 Hz the serial link can carry without hiding the timing. */
static void cmd_bal_log(const char *sec_arg)
{
    uint32_t sec;
    uint32_t t_end;
    uint32_t t0;
    const app_bal_info_t *b;

    if ((parse_u32(sec_arg, &sec) == 0u) || (sec == 0u) || (sec > 30u)) {
        log_printf("ERR usage=bal log <1..30>");
        return;
    }
    if (app_bal_info()->on == 0u) {
        log_printf("ERR bal log: send \"bal on\" first");
        return;
    }

    log_printf(" t,phi_deg,rate_dps,wheel_rpm,tau_mnm,sat,dead");
    t0    = HAL_GetTick();
    t_end = t0 + (sec * 1000u);

    while ((int32_t)(t_end - HAL_GetTick()) > 0)
    {
        imu_wait(20u);           /* pumps imu_poll() and the TX ring */
        app_att_update();
        app_bal_update();

        b = app_bal_info();
        log_printf(" %.2f,%.2f,%.2f,%.0f,%.2f,%u,%u",
                   (double)((HAL_GetTick() - t0) / 1000.0f),
                   (double)b->phi_deg, (double)b->dphi_dps, (double)b->omega_rpm,
                   (double)b->tau_mnm, (unsigned)b->saturated, (unsigned)b->deadband);
    }
}

/* One line summary of the closed-loop orchestrator. Kept separate from
 * print_bal() because the shadow law and the interlock state are two
 * different things: the law can be perfectly valid while the run is
 * disarmed. */
static void print_bal_run(void)
{
    const app_bal_run_t    *r = app_balance_info();
    const app_bal_info_t   *b = app_bal_info();
    const char *sign;

    log_printf("---- bal run (S6) ----");
    log_printf(" state  = %s, trip %s, drive %u%s",
               app_balance_state_name(r->state),
               app_balance_trip_name(r->trip), (unsigned)r->drive_ret,
               ((r->state == (uint8_t)BAL_ST_ARMED) && (r->engaged == 0u))
                   ? ", waiting for release" : "");
    sign = (r->tau_applied_mnm < 0.0f) ? "-" : "+";
    log_printf(" out    = %s%.2f mN.m applied, law %s%.2f mN.m, peak %s%.2f mN.m",
               sign, (double)fabsf(r->tau_applied_mnm),
               (b->tau_mnm < 0.0f) ? "-" : "+", (double)fabsf(b->tau_mnm),
               (r->tau_peak_mnm < 0.0f) ? "-" : "+", (double)fabsf(r->tau_peak_mnm));
    log_printf(" window = %u ms, frames %u, saturated %u",
               (unsigned)r->run_ms, (unsigned)r->frames, (unsigned)r->sat_frames);
    /* walk_calls is the trim-walk tick count. It must be about 2 per second of
     * window (BAL_WALK_DIV = 100 attitude frames of 5 ms). A four digit number
     * means the tick still runs off the main loop instead of the IMU stream. */
    log_printf(" walk   = %u calls (%u ms window, %.1f per s)",
               (unsigned)r->walk_calls, (unsigned)r->run_ms,
               (r->run_ms > 0u) ? ((double)r->walk_calls * 1000.0 / (double)r->run_ms) : 0.0);
    log_printf(" peaks  = phi %.2f deg, rate %.1f deg/s, wheel %.0f rpm",
               (double)r->phi_peak_deg, (double)r->rate_peak_dps,
               (double)r->omega_peak_rpm);
    log_printf(" limits = arm %.1f deg / %.0f dps / %.0f rpm, trip %.0f deg / %.0f dps / %.0f rpm",
               (double)BAL_ARM_DEG, (double)BAL_ARM_DPS, (double)BAL_ARM_RPM,
               (double)BAL_TRIP_DEG, (double)BAL_TRIP_DPS, (double)BAL_TRIP_RPM);
    log_printf(" scale  = %.2f x nominal, ceiling %.0f mN.m, slew %.0f mN.m per frame",
               (double)BAL_SCALE, (double)BAL_MAX_MNM, (double)BAL_SLEW_MNM_STEP);
    if (BAL_MAX_MS > 0u) {
        log_printf(" max    = %u ms", (unsigned)BAL_MAX_MS);
    } else {
        log_printf(" max    = no limit (runs until a trip or 'bal run off')");
    }
}

/* Arm the closed-loop. Deliberately chatty: this is the one command that can
 * put 33 mN.m into a hand-held cube, so every refusal names its reason. */
static void cmd_bal_run_on(void)
{
    uint8_t r;

    if (app_balance_info()->state == BAL_ST_TRIP) {
        log_printf("ERR bal run: tripped (%s), send 'start' then retry",
                   app_balance_trip_name(app_balance_info()->trip));
        return;
    }
    if (app_status()->state != APP_ST_RUN) {
        log_printf("ERR bal run: send 'mode 2' and 'start' first");
        return;
    }
    if (app_bal_info()->on == 0u) {
        log_printf("WARN bal law was off, turning shadow on for this run");
    }

    r = app_balance_arm(1u);
    if (r == 0u) {
        if (BAL_MAX_MS > 0u) {
            log_printf("OK bal run armed, max %u ms -- hand on the power switch, cube on the soft pad",
                       (unsigned)BAL_MAX_MS);
        } else {
            log_printf("OK bal run armed, no time limit -- hand on the power switch, cube on the soft pad");
        }
    } else if (r == 1u) {
        log_printf("ERR bal run: no valid attitude frame");
    } else if (r == 2u) {
        /* The arm check reads att_get()->theta_deg, so report the same number:
         * app_bal_info()->phi_deg is only refreshed by app_bal_update(), which
         * right after an "att zero" still holds the pre-zero value (a confusing
         * "tilt 0.00 deg outside +-2.50" was printed from exactly this). */
        log_printf("ERR bal run: tilt %.2f deg outside +-%.1f deg arm window",
                   (double)att_get()->theta_deg, (double)BAL_ARM_DEG);
    } else if (r == 3u) {
        log_printf("ERR bal run: tilt rate %.1f deg/s above %.0f, let it settle",
                   (double)fabsf(app_bal_info()->dphi_dps), (double)BAL_ARM_DPS);
    } else if (r == 4u) {
        log_printf("ERR bal run: wheel %.0f rpm above %.0f, wait or brake it",
                   (double)fabsf(app_bal_info()->omega_rpm), (double)BAL_ARM_RPM);
    } else if (r == 5u) {
        log_printf("ERR bal run: drive not ready (mode %s, spd loop %s)",
                   app_mode_name(app_status()->mode),
                   (app_balance_info()->drive_ret == 6u) ? "on" : "off");
    } else {
        log_printf("ERR bal run: already %s", app_balance_state_name(app_balance_info()->state));
    }
}

/* Dispatch                                                            */
/* ------------------------------------------------------------------ */
static void print_torque(void)
{
    const app_foc_info_t *f = app_foc_info();
    const mid_enc_t *e = mid_enc_info();
    float cmd = app_torque_get();

    log_printf("---- torque (S3) ----");
    log_printf(" cmd   = %.2f mN.m -> iq %.3f A (Kt %.4f N.m/A)",
               (double)cmd, (double)(cmd / (1000.0f * MOTOR_KT_NM_A)),
               (double)MOTOR_KT_NM_A);
    log_printf(" ref   = iq %.3f A, id %.3f A", (double)f->iq_ref, (double)f->id_ref);
    log_printf(" meas  = iq %.3f A, id %.3f A", (double)f->iq, (double)f->id);
    log_printf(" wheel = %.2f rpm, state %s", (double)e->spd_rpm,
               app_state_name(app_status()->state));
    log_printf(" limit = +-%.1f mN.m, release below %.1f mN.m",
               (double)TORQUE_MAX_MNM, (double)TORQUE_MIN_MNM);
}

static void cmd_exec(char *line)
{
    char   *tok[8];

    bsp_sys_ctx = BSP_CTX_CMD;
    uint8_t n;
    uint32_t val;

    n = split_args(line, tok, 8u);
    if (n == 0u) { return; }

    static float   s_pll_wn   = PLL_WN_HZ;
static float   s_pll_zeta = PLL_ZETA;

/* ---------------- help ---------------- */
    if (strcmp(tok[0], "help") == 0) {
        print_help();
    }
    /* ---------------- ver ----------------- */
    else if (strcmp(tok[0], "ver") == 0) {
        print_version();
    }
    /* ---------------- stat ---------------- */
    else if (strcmp(tok[0], "wp") == 0) {
            if ((n > 1) && (strcmp(tok[1], "test") == 0)) {
                if (bsp_wp_self_test() != 0u) {
                    log_printf("OK wp test, the comparator fired (hits %u)",
                               (unsigned)bsp_wp_hits());
                } else {
                    log_printf("ERR wp test, no hit (armed %u, addr %x)",
                               (unsigned)bsp_wp_armed(), (unsigned)bsp_wp_addr());
                }
            } else if ((n > 1) && (strcmp(tok[1], "on") == 0)) {
                app_wp_arm();
                log_printf("OK wp armed on %x", (unsigned)bsp_wp_addr());
            } else if ((n > 1) && (strcmp(tok[1], "off") == 0)) {
                bsp_wp_disarm();
                log_printf("OK wp off");
            } else if ((n > 1) && (strcmp(tok[1], "clear") == 0)) {
                bsp_wp_clear();
                log_printf("OK wp cleared");
            } else {
                (void)bsp_wp_print();
            }
        }
        else if (strcmp(tok[0], "crash") == 0) {
            if ((n > 1) && (strcmp(tok[1], "clear") == 0)) {
                bsp_crash_clear();
                log_printf("OK crash record cleared");
            } else {
                (void)bsp_crash_print_saved();
            }
        }
        else if (strcmp(tok[0], "crc") == 0) {
            uint32_t now = bsp_img_crc();
            uint32_t base = 0u;

            if ((n > 1) && (strcmp(tok[1], "save") == 0)) {
                bsp_crash_save_crc(now);
                log_printf("OK crc baseline saved: %08x", (unsigned)now);
            } else if (bsp_crash_load_crc(&base) == 0u) {
                log_printf("crc = %08x now, no baseline yet (crc save)", (unsigned)now);
            } else if (base == now) {
                log_printf("crc = %08x now, baseline %08x, MATCH",
                           (unsigned)now, (unsigned)base);
            } else {
                log_printf("crc = %08x now, baseline %08x, CHANGED (flash differs)",
                           (unsigned)now, (unsigned)base);
            }
        }
        else if (strcmp(tok[0], "fpu") == 0) {
            if ((n >= 3) && (strcmp(tok[1], "eager") == 0)) {
                if (strcmp(tok[2], "on") == 0) {
                    bsp_sys_fpu_eager(1);
                    log_printf("OK fpu save = eager (ASPEN/LSPEN cleared)");
                } else if (strcmp(tok[2], "off") == 0) {
                    bsp_sys_fpu_eager(0);
                    log_printf("OK fpu save = lazy (ASPEN/LSPEN set)");
                } else {
                    log_printf("ERR usage=fpu eager <on|off>");
                }
            } else {
                log_printf("fpu = save %s, fpccr %x, fpcar %x, lspact seen %u",
                           (bsp_sys_fpu_eager_get() != 0u) ? "eager" : "lazy",
                           (unsigned)bsp_sys_fpccr(), (unsigned)bsp_sys_fpcar(),
                           (unsigned)app_guard_lspact());
                log_printf(" usage: fpu eager on|off  (lazy = default, eager = save FP context on every entry)");
            }
        }
        else if (strcmp(tok[0], "uart") == 0) {
        print_uart();
    }
    else if (strcmp(tok[0], "stat") == 0) {
        print_stat();
    }
    /* ---------------- imu ----------------- */
    else if (strcmp(tok[0], "imu") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "rate") == 0)) {
            if (n < 3u) {
                log_printf("ERR usage=imu rate <10..200> [save]");
            } else {
                cmd_imu_rate(tok[2], (uint8_t)((n >= 4u) && (strcmp(tok[3], "save") == 0)));
            }
        } else if ((n >= 3u) && (strcmp(tok[1], "cfg") == 0)) {
            char    txt[48];
            uint8_t i;
            uint8_t fits = 1u;

            strcpy(txt, tok[2]);
            for (i = 3u; i < n; i++) {
                if ((strlen(txt) + strlen(tok[i]) + 2u) >= sizeof(txt)) { fits = 0u; break; }
                strcat(txt, " ");
                strcat(txt, tok[i]);
            }
            if (fits == 0u) {
                log_printf("ERR imu cfg: command too long");
            } else {
                cmd_imu_cfg(txt);
            }
        } else {
            print_imu();
        }
    }
    /* ---------------- att ----------------- */
    else if (strcmp(tok[0], "att") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "zero") == 0)) {
            att_zero();
            log_printf("OK att zero: upright is now %.2f deg raw", (double)att_offset_deg());
        } else if ((n >= 2u) && (strcmp(tok[1], "stat") == 0)) {
            cmd_att_stat((n >= 3u) ? tok[2] : "5");
        } else {
            print_att();
        }
    }
    /* ---------------- bal ----------------- */
    else if (strcmp(tok[0], "bal") == 0) {
        if ((n >= 3u) && (strcmp(tok[1], "run") == 0) && (strcmp(tok[2], "on") == 0)) {
            cmd_bal_run_on();
        } else if ((n >= 3u) && (strcmp(tok[1], "run") == 0) && (strcmp(tok[2], "off") == 0)) {
            (void)app_balance_arm(0u);
            log_printf("OK bal run off, state=%s", app_balance_state_name(app_balance_info()->state));
        } else if ((n >= 2u) && (strcmp(tok[1], "run") == 0)) {
            print_bal_run();
        } else if ((n >= 2u) && (strcmp(tok[1], "on") == 0)) {
            uint8_t r = app_bal_set(1u);
            if (r == 0u)      { log_printf("OK bal on (shadow: computes, does not drive)"); }
            else if (r == 2u) { log_printf("ERR bal: no attitude frame yet"); }
            else              { log_printf("ERR bal: tuning rejected, check BAL_* in board_cfg.h"); }
        } else if ((n >= 2u) && (strcmp(tok[1], "off") == 0)) {
            (void)app_bal_set(0u);
            log_printf("OK bal off");
        } else if ((n >= 2u) && (strcmp(tok[1], "log") == 0)) {
            cmd_bal_log((n >= 3u) ? tok[2] : "3");
        } else if ((n >= 2u) && (strcmp(tok[1], "auto") == 0)) {
            if ((n >= 3u) && (strcmp(tok[2], "on") == 0)) {
                app_bal_auto_set(1u);
                log_printf("OK bal auto on: the wheel speed walks the zero trim");
            } else if ((n >= 3u) && (strcmp(tok[2], "off") == 0)) {
                app_bal_auto_set(0u);
                log_printf("OK bal auto off: trim is frozen, use \"bal trim <deg>\"");
            } else if (n < 3u) {
                log_printf(" auto   = %s, usage=bal auto <on|off>",
                           (app_bal_auto_get() != 0u) ? "on" : "off");
            } else {
                log_printf("ERR usage=bal auto <on|off>");
            }
        } else if ((n >= 2u) && (strcmp(tok[1], "walk") == 0)) {
            float k = 0.0f;
            if ((n >= 3u) && (parse_num(tok[2], &k) != 0u)) {
                k = app_bal_walk_set(k);
                log_printf("OK bal walk %d e-6 deg per rpm per s%s",
                           (int)((k * 1000000.0f) + 0.5f),
                           (k <= 0.0f) ? " (frozen)" : "");
            } else if (n < 3u) {
                log_printf(" walk   = %d e-6 deg per rpm per s (max 10000), usage=bal walk <k>",
                           (int)((app_bal_walk_get() * 1000000.0f) + 0.5f));
            } else {
                log_printf("ERR usage=bal walk <k>");
            }
        } else if ((n >= 2u) && (strcmp(tok[1], "snap") == 0)) {
            uint8_t rc = app_bal_snap();

            if (rc == 0u) {
                double w = (double)app_bal_info()->omega_rpm;
                const char *sgn = (w < 0.0) ? "-" : "+";

                /* log_printf has no + flag: print the sign ourselves. */
                log_printf("OK bal snap: trim %.2f deg, wheel %s%.0f rpm",
                           (double)app_bal_trim_get(), sgn, (w < 0.0) ? -w : w);
            } else if (rc == 2u) {
                log_printf("ERR bal snap: no running loop, use the button or \"bal run on\"");
            } else if (rc == 3u) {
                log_printf("ERR bal snap: no valid attitude frame");
            } else if (rc == 5u) {
                log_printf("ERR bal snap: wheel %+.0f rpm is outside the trusted +-1200 band",
                           (double)app_bal_info()->omega_rpm);
            } else {
                log_printf("ERR bal snap (rc %u)", (unsigned)rc);
            }
        } else if ((n >= 2u) && (strcmp(tok[1], "trim") == 0)) {
            float deg = 0.0f;
            if ((n >= 3u) && (parse_num(tok[2], &deg) != 0u)) {
                deg = app_bal_trim_set(deg);
                log_printf("OK bal trim %.2f deg (auto trim off, angle used = theta - trim)",
                           (double)deg);
            } else if (n < 3u) {
                log_printf(" trim   = %.2f deg (max %.1f), usage=bal trim <deg>",
                           (double)app_bal_trim_get(), (double)BAL_TRIM_MAX_DEG);
            } else {
                log_printf("ERR usage=bal trim <deg>");
            }
        } else {
            print_bal();
        }
    }
    /* ---------------- start --------------- */
    else if (strcmp(tok[0], "start") == 0) {
        uint8_t r = app_start();
        if (r == 0u)      { log_printf("OK start, mode=%s", app_mode_name(app_status()->mode)); }
        else if (r == 1u) { log_printf("ERR state=FAULT, clear the fault first"); }
        else if (r == 3u) { log_printf("ERR start rejected"); }
        else              { log_printf("ERR mode=IDLE, send 'mode 1' or 'mode 2' first"); }
    }
    /* ---------------- stop ---------------- */
    else if (strcmp(tok[0], "stop") == 0) {
        (void)app_stop();
        log_printf("OK stop, PWM off");
    }
    /* ---------------- mode ---------------- */
    else if (strcmp(tok[0], "mode") == 0) {
        if (n < 2u)                        { log_printf("ERR usage=mode <0|1|2>"); }
        else if (!parse_u32(tok[1], &val)) { log_printf("ERR bad arg"); }
        else {
            uint8_t r = app_set_mode((uint8_t)val);
            if (r == 0u)      { log_printf("OK mode=%s", app_mode_name(app_status()->mode)); }
            else if (r == 2u) { log_printf("ERR stop the motor first"); }
            else              { log_printf("ERR mode must be 0/1/2"); }
        }
    }
    /* ---------------- pwm ----------------- */
    else if (strcmp(tok[0], "pwm") == 0) {
        float a, b, c;

        if ((n == 2u) || (n == 4u)) {
            if (app_status()->state != APP_ST_RUN) {
                log_printf("ERR start first");
            } else if (app_vf_info()->on != 0u) {
                log_printf("ERR vf active, send 'vf off' first");
            } else if (mid_current_get()->ready == 0u) {
                log_printf("ERR calibrating, wait");
            } else if (!parse_duty(tok[1], &a)) {
                log_printf("ERR bad duty, use 0..1");
            } else if ((n == 4u) && ((!parse_duty(tok[2], &b)) || (!parse_duty(tok[3], &c)))) {
                log_printf("ERR bad duty, use 0..1");
            } else {
                if (n == 2u) { b = a; c = a; }
                bsp_pwm_set_duty(a, b, c);
                bsp_pwm_get_duty(&a, &b, &c);       /* echo what was applied */
                log_printf("OK duty = %.2f/%.2f/%.2f", a, b, c);
            }
        } else if (n == 1u) {
            bsp_pwm_get_duty(&a, &b, &c);
            log_printf("pwm = %s, dtg=%u, duty=%.2f/%.2f/%.2f",
                       bsp_pwm_is_enabled() ? "ON" : "OFF",
                       (unsigned)bsp_pwm_get_dtg(), a, b, c);
        } else {
            log_printf("ERR usage=pwm <d> | pwm <a> <b> <c>");
        }
    }
    /* ---------------- id / iq (S6) ----------------- */
    else if ((strcmp(tok[0], "id") == 0) || (strcmp(tok[0], "iq") == 0)) {
        const app_foc_info_t *f = app_foc_info();

        if (n == 2u) {
            float amp;

            if (!parse_num(tok[1], &amp)) {
                log_printf("ERR usage=id|iq <A>, range -1.20 .. 1.20");
            } else {
                float id = f->id_ref;
                float iq = f->iq_ref;
                uint8_t rc;

                if (tok[0][1] == (char)0x64) {   /* "id": second char 'd' */
                    id = amp;
                } else {
                    iq = amp;
                }

                rc = app_foc_set(id, iq);
                switch (rc) {
                case 0u:
                    log_printf("OK id=%.2f A, iq=%.2f A", (double)id, (double)iq);
                    break;
                case 2u:
                    log_printf("ERR start first");
                    break;
                case 4u:
                    log_printf("ERR mode 2 first (FOC)");
                    break;
                case 5u:
                    log_printf("ERR current too high, |I| <= %.2f A",
                               (double)FOC_I_MAX_A);
                    break;
                case 6u:
                    log_printf("ERR speed loop on, spd off first");
                    break;
                default:
                    log_printf("ERR calibrating, wait");
                    break;
                }
            }
        } else {
            log_printf("id=%.2f iq=%.2f A, ref %.2f/%.2f, m=%.3f",
                       (double)f->id, (double)f->iq,
                       (double)f->id_ref, (double)f->iq_ref, (double)f->m);
        }
    }
    /* ---------------- pi (S6.4) ----------------- */
    else if (strcmp(tok[0], "pi") == 0) {
        float kp;
        float ki;

        app_foc_get_pi(&kp, &ki);

        if ((n == 2u) && (strcmp(tok[1], "def") == 0)) {
            switch (app_foc_set_pi(FOC_KP, FOC_KI)) {
            case 0u:
                log_printf("OK pi = %.2f/%.0f (default)", (double)FOC_KP, (double)FOC_KI);
                break;
            case 4u:
                log_printf("ERR mode 2 first (FOC)");
                break;
            default:
                log_printf("ERR pi rejected");
                break;
            }
        } else if (n == 3u) {
            float nkp;
            float nki;

            if (!parse_num(tok[1], &nkp) || !parse_num(tok[2], &nki)) {
                log_printf("ERR usage=pi <kp> <ki> | pi def | pi");
            } else {
                switch (app_foc_set_pi(nkp, nki)) {
                case 0u:
                    log_printf("OK pi = %.2f/%.0f", (double)nkp, (double)nki);
                    break;
                case 1u:
                    log_printf("ERR pi out of range, kp %.2f..%.2f",
                               (double)FOC_KP_MIN, (double)FOC_KP_MAX);
                    break;
                case 4u:
                    log_printf("ERR mode 2 first (FOC)");
                    break;
                default:
                    log_printf("ERR pi rejected");
                    break;
                }
            }
        } else {
            log_printf("pi = kp %.2f V/A, ki %.0f V/(A s)", (double)kp, (double)ki);
            log_printf(" min kp %.2f, max kp %.2f, ki %.0f..%.0f",
                       (double)FOC_KP_MIN, (double)FOC_KP_MAX,
                       (double)FOC_KI_MIN, (double)FOC_KI_MAX);
        }
    }
    /* ---------------- spd (S7.3) ----------------- */
    else if (strcmp(tok[0], "spd") == 0) {
        const app_foc_info_t *f = app_foc_info();

        if ((n == 2u) && (strcmp(tok[1], "off") == 0)) {
            app_spd_off();
            log_printf("OK spd off, iq released");
        } else if (n == 2u) {
            float rpm;

            if (!parse_num(tok[1], &rpm)) {
                log_printf("ERR usage=spd <rpm> | spd off | spd");
            } else {
                switch (app_spd_set(rpm)) {
                case 0u:
                    /* Name the feedback source: inside sensorless mode the
                     * speed loop uses the observer, and the reply used to
                     * look identical to the encoder case. */
                    log_printf("OK spd target = %d rpm (%s)", (int)rpm,
                               (app_sfoc_on() != 0u) ? "observer"
                                                     : "encoder");
                    break;
                case 2u:
                    log_printf("ERR start first");
                    break;
                case 4u:
                    log_printf("ERR mode 2 first (FOC)");
                    break;
                case 6u:
                    log_printf("ERR no encoder: use 'sfoc go <rpm>' "
                               "or 'enc on'");
                    break;
                case 7u:
                    log_printf("ERR I/f running, wait for hand-over "
                               "(or use 'sfoc go')");
                    break;
                default:
                    log_printf("ERR speed too high, |rpm| <= %d", (int)SPD_RPM_MAX);
                    break;
                }
            }
        } else {
            log_printf("spd ref %d rpm (target %d), meas %d rpm, iq %.2f A",
                       (int)f->spd_ref, (int)f->spd_target,
                       (int)f->spd_rpm, (double)f->iq_ref);
            log_printf(" speed loop %s, spd off returns to idle",
                       (f->spd_ref != 0.0f) ? "ON" : "OFF");
        }
    }
    /* ---------------- pll (S8.1) ---------------- */
    else if (strcmp(tok[0], "pll") == 0) {
        if (n == 2u) {
            if (strcmp(tok[1], "on") == 0) {
                mid_enc_set_spd_src(1u);
                log_printf("OK pll speed source");
            } else if (strcmp(tok[1], "off") == 0) {
                mid_enc_set_spd_src(0u);
                log_printf("OK window speed source");
            } else {
                log_printf("ERR usage=pll on|off|wn <hz>|zeta <v>");
            }
        } else if (n == 3u) {
            float v;

            if (!parse_num(tok[2], &v)) {
                log_printf("ERR usage=pll wn <hz>|zeta <v>");
            } else if (strcmp(tok[1], "wn") == 0) {
                if ((v < 10.0f) || (v > 400.0f)) {
                    log_printf("ERR wn range 10..400 Hz");
                } else {
                    mid_enc_set_pll_bw(v, s_pll_zeta);
                    s_pll_wn = v;
                    log_printf("OK pll wn = %.1f Hz", (double)v);
                }
            } else if (strcmp(tok[1], "zeta") == 0) {
                if ((v < 0.3f) || (v > 2.0f)) {
                    log_printf("ERR zeta range 0.3..2.0");
                } else {
                    mid_enc_set_pll_bw(s_pll_wn, v);
                    s_pll_zeta = v;
                    log_printf("OK pll zeta = %.3f", (double)v);
                }
            } else {
                log_printf("ERR usage=pll wn <hz>|zeta <v>");
            }
        } else {
            log_printf("pll src %s, wn %.1f Hz, zeta %.3f, locked %u",
                       (mid_enc_spd_src() != 0u) ? "pll" : "window",
                       (double)s_pll_wn, (double)s_pll_zeta,
                       (unsigned)mid_enc_pll_locked());
            log_printf(" pll %d rpm, window %d rpm",
                       (int)mid_enc_pll_rpm(), (int)mid_enc_win_rpm());
        }
    }

    /* ---------------- sff (option C) ------------- */
    else if (strcmp(tok[0], "sff") == 0) {
        float b = 0.0f;
        float k = 0.0f;

        app_spd_get_ff(&b, &k);
        if (n == 3u) {
            float nb;
            float nk;

            if (!parse_num(tok[1], &nb) || !parse_num(tok[2], &nk)) {
                log_printf("ERR usage=sff <base A> <slope A/krpm>");
            } else if (app_spd_set_ff(nb, nk * 0.001f) != 0u) {
                log_printf("ERR sff out of range");
            } else {
                log_printf("OK sff base %.3f A, slope %.3f A/krpm",
                           (double)nb, (double)nk);
            }
        } else {
            log_printf("sff base %.3f A, slope %.3f A/krpm",
                       (double)b, (double)(k * 1000.0f));
        }
    }

    /* ---------------- torque (S3) ---------------- */
    else if (strcmp(tok[0], "torque") == 0) {
        float mnm;

        if ((n == 2u) && (strcmp(tok[1], "off") == 0)) {
            app_torque_off();
            log_printf("OK torque released");
        } else if ((n == 1u) || (n > 2u)) {
            print_torque();
        } else if (!parse_num(tok[1], &mnm)) {
            log_printf("ERR usage=torque <mN.m> | torque off");
        } else {
            switch (app_torque_set(mnm)) {
            case 0u:
                if (app_torque_get() == 0.0f) {
                    log_printf("OK torque released (below TORQUE_MIN_MNM)");
                } else {
                    log_printf("OK torque %.2f mN.m (iq %.3f A)",
                               (double)mnm,
                               (double)(mnm / (1000.0f * MOTOR_KT_NM_A)));
                }
                break;
            case 2u:
                log_printf("ERR start first");
                break;
            case 3u:
                log_printf("ERR calibrating");
                break;
            case 4u:
                log_printf("ERR mode 2 first (FOC)");
                break;
            case 5u:
                log_printf("ERR torque out of range (+-%.1f mN.m)",
                           (double)TORQUE_MAX_MNM);
                break;
            case 6u:
                log_printf("ERR speed loop is running, spd off first");
                break;
            default:
                log_printf("ERR torque failed");
                break;
            }
        }
    }

    /* ---------------- sstep (S7.4) --------------- */
    else if (strcmp(tok[0], "sstep") == 0) {
        float rpm;

        if (n != 2u) {
            log_printf("ERR usage=sstep <rpm>, e.g. sstep 500");
        } else if (!parse_num(tok[1], &rpm)) {
            log_printf("ERR usage=sstep <rpm>, e.g. sstep 500");
        } else {
            switch (app_sstep_start(rpm)) {
            case 0u:
                log_printf("OK sstep %d rpm, wait 1 s for the result", (int)rpm);
                break;
            case 2u:
                log_printf("ERR start first");
                break;
            case 4u:
                log_printf("ERR mode 2 first (FOC)");
                break;
            default:
                log_printf("ERR speed out of range (50..%d rpm)",
                           (int)SPD_RPM_MAX);
                break;
            }
        }
    }

    /* ---------------- spi (S7.3) ----------------- */
    else if (strcmp(tok[0], "spi") == 0) {
        float kp;
        float ki;

        app_spd_get_pi(&kp, &ki);

        if (n == 3u) {
            float nkp;
            float nki;

            if (!parse_num(tok[1], &nkp) || !parse_num(tok[2], &nki)) {
                log_printf("ERR usage=spi <kp> <ki> | spi");
            } else {
                switch (app_spd_set_pi(nkp, nki)) {
                case 0u:
                    log_printf("OK spi = %.3f/%.3f", (double)nkp, (double)nki);
                    break;
                case 1u:
                    log_printf("ERR spi out of range, kp 0..5, ki 0..50");
                    break;
                default:
                    log_printf("ERR spi rejected");
                    break;
                }
            }
        } else {
            /* fmt_float2 always prints two decimals, so 0.016 A/rpm would
             * show as 0.02: print the gains in mA to keep them readable. */
            log_printf("spi = kp %.2f mA/rpm, ki %.2f mA/(rpm s)",
                       (double)(kp * 1000.0f), (double)(ki * 1000.0f));
        }
    }

    /* ---------------- fault (S7.6) ----------------- */
    else if (strcmp(tok[0], "fault") == 0) {
        const app_fault_info_t *f = app_fault_info();
        static const char *name[7] = {
            "none", "overcurrent", "overspeed", "stall",
            "undervoltage", "overvoltage", "encoder"
        };
        uint8_t i;

        if (f->code == 0u) {
            log_printf("fault none, all protections idle");
        } else {
            log_printf("fault %s, value %.3f, at %u ms",
                       name[(f->code < 7u) ? f->code : 0u],
                       (double)f->value, (unsigned)f->uptime_ms);
        }
        log_printf(" counts oc %u os %u st %u uv %u ov %u enc %u",
                   (unsigned)f->count[1], (unsigned)f->count[2],
                   (unsigned)f->count[3], (unsigned)f->count[4],
                   (unsigned)f->count[5], (unsigned)f->count[6]);
        {
            const app_fault_info_t *l = app_last_fault();

            if (l->code != 0u) {
                log_printf(" last   = %s, value %.3f, at %u ms",
                           name[(l->code < 7u) ? l->code : 0u],
                           (double)l->value, (unsigned)l->uptime_ms);
            }
        }
        (void)i;
    }
    /* ---------------- prot (S7.6) ----------------- */
    else if (strcmp(tok[0], "prot") == 0) {
        const app_prot_t *p = app_prot_info();

        if (n == 3u) {
            float v;
            uint8_t which = 0xFFu;

            if      (strcmp(tok[1], "os") == 0) { which = 0u; }
            else if (strcmp(tok[1], "st") == 0) { which = 1u; }
            else if (strcmp(tok[1], "siq") == 0) { which = 2u; }
            else if (strcmp(tok[1], "uv") == 0) { which = 3u; }
            else if (strcmp(tok[1], "ov") == 0) { which = 4u; }
            else if (strcmp(tok[1], "oc") == 0) { which = 5u; }

            if (!parse_num(tok[2], &v) || (which == 0xFFu)) {
                log_printf("ERR usage=prot os|st|siq|oc|uv|ov <value>");
            } else if (app_prot_set(which, v) != 0u) {
                log_printf("ERR prot value out of range");
            } else {
                log_printf("OK prot %s = %.2f", tok[1], (double)v);
            }
        } else {
            log_printf("prot os %d rpm/%d ms, stall %d rpm %d mA/%d ms",
                       (int)p->overspeed_rpm, (int)p->overspeed_ms,
                       (int)p->stall_rpm, (int)(p->stall_iq_a * 1000.0f),
                       (int)p->stall_ms);
            log_printf(" oc %.2f A, uv %.2f V, ov %.2f V, %d ms, grace %d ms",
                       (double)p->oc_a,
                       (double)p->uv_v, (double)p->ov_v,
                       (int)p->vbus_ms, (int)PROT_GRACE_MS);
        }
    }

    /* ---------------- rip (S7.3 diag) ------------- */
    else if (strcmp(tok[0], "rip") == 0) {
        const app_rip_t *r = app_rip_info();

        /* Print what the previous window collected, then restart it:
         * starting first would print the -1e9 sentinels. */
        if (r->n == 0u) {
            log_printf("rip: collecting, ask again in %d ms", (int)RIP_WINDOW_MS);
        } else {
            /* The mini printf has no "+" flag: a literal %+.2f came out as
             * text on hardware. Use plain %.2f and let the sign show. */
            log_printf("rip %d ms: iq %.2f to %.2f A (%.2f p-p)",
                       (int)RIP_WINDOW_MS, (double)r->iq_min, (double)r->iq_max,
                       (double)(r->iq_max - r->iq_min));
            log_printf(" spd %d~%d rpm (%d p-p), %u samples",
                       (int)r->spd_min, (int)r->spd_max,
                       (int)(r->spd_max - r->spd_min), (unsigned)r->n);
        }
        app_rip_start();
    }

    /* ---------------- vbus (S7.5) ----------------- */
    else if (strcmp(tok[0], "vbus") == 0) {
        log_printf("vbus = %.2f V (raw %.2f V, code %u), %s",
                   (double)bsp_vbus_volts(), (double)bsp_vbus_raw_volts(),
                   (unsigned)bsp_vbus_raw(),
                   (bsp_vbus_valid() != 0u) ? "ok" : "BAD");
        log_printf(" filter 1 kHz, %u errors, OV %.2f V, UV %.2f V",
                   (unsigned)bsp_vbus_errs(), (double)VBUS_OV_V,
                   (double)VBUS_UV_V);
    }

    /* ---------------- ff (S7.2) ----------------- */
    else if (strcmp(tok[0], "ff") == 0) {
        float k_emf;
        float l_h;
        float gain;

        app_foc_get_ff(&k_emf, &l_h, &gain);

        if (n == 2u) {
            float g;

            if (!parse_duty(tok[1], &g)) {
                log_printf("ERR usage=ff <0..1>, ff shows the values");
            } else if (g > 1.0f) {
                log_printf("ERR usage=ff <0..1>, ff shows the values");
            } else {
                app_foc_set_ff(k_emf, l_h, g);
                log_printf("OK ff gain = %.2f", (double)g);
            }
        } else {
            /* fmt_float2 always prints two decimals, so scale to readable units */
            log_printf("ff gain %.2f, k_emf %.2f mV/(rad/s), L %.2f uH",
                       (double)gain, (double)(k_emf * 1000.0f),
                       (double)(l_h * 1e6f));
            log_printf(" ff 0 = no feedforward, ff 1 = full");
        }
    }

    /* ---------------- step (S6.4) ----------------- */
    else if (strcmp(tok[0], "step") == 0) {
        float amp;

        if (n != 2u) {
            log_printf("ERR usage=step <A>, e.g. step 0.30");
        } else if (!parse_num(tok[1], &amp)) {
            log_printf("ERR usage=step <A>, e.g. step 0.30");
        } else {
            switch (app_step_start(amp)) {
            case 0u:
                log_printf("OK step %.2f A, wait 0.3 s for the result", (double)amp);
                break;
            case 2u:
                log_printf("ERR start first");
                break;
            case 4u:
                log_printf("ERR mode 2 first (FOC)");
                break;
            default:
                log_printf("ERR current too high, |I| <= %.2f A",
                           (double)FOC_I_MAX_A);
                break;
            }
        }
    }

    /* ---------------- foc ----------------- */
    else if (strcmp(tok[0], "foc") == 0) {
        const app_foc_info_t *f = app_foc_info();
        if (f->ready == 0u) {
            log_printf("foc idle, run mode 2 + start + id/iq");
        } else {
            log_printf("foc id=%.3f iq=%.3f A, ref %.2f/%.2f",
                       (double)f->id, (double)f->iq,
                       (double)f->id_ref, (double)f->iq_ref);
            log_printf(" vd=%.2f vq=%.2f V, m=%.3f, faults %u",
                       (double)f->vd, (double)f->vq, (double)f->m,
                       (unsigned)f->faults);
        }
    }

    /* ---------------- vf ----------------- */
    else if (strcmp(tok[0], "vf") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "off") == 0)) {
            (void)app_vf_off();
            log_printf("OK vf off, duty 0.50");
        } else if (n == 2u) {
            float hz;

            if (!parse_duty(tok[1], &hz)) {
                log_printf("ERR usage=vf <hz> | vf off");
            } else if (hz <= 0.0f) {
                /* "vf 0" is the same as "vf off": a V/f drive at 0 Hz would
                 * only apply the low-speed boost as a DC vector, which
                 * heats the motor and holds the shaft without rotating. */
                (void)app_vf_off();
                log_printf("OK vf 0 = off, duty 0.50");
            } else {
                switch (app_vf_set(hz)) {
                case 0u:
                    log_printf("OK vf target = %.2f Hz", hz);
                    break;
                case 2u:
                    log_printf("ERR start first");
                    break;
                default:
                    log_printf("ERR calibrating, wait");
                    break;
                }
            }
        } else {
            const app_vf_info_t *v = app_vf_info();
            log_printf("vf = %s %.1f/%.1f Hz, m=%.2f, I=%.2f A, dir=%u",
                       v->on ? "ON" : "OFF", v->f_hz, v->f_target, v->m,
                       v->i_max, (unsigned)v->dir);
        }
    }
    /* ---------------- acc ----------------- */
    else if (strcmp(tok[0], "acc") == 0) {
        float a;

        if ((n != 2u) || (!parse_duty(tok[1], &a))) {
            log_printf("ERR usage=acc <hz/s> 1..200");
        } else {
            app_vf_set_acc(a);
            log_printf("OK acc = %.2f Hz/s", a);
        }
    }
    /* ---------------- dir ----------------- */
    else if (strcmp(tok[0], "dir") == 0) {
        float a;

        if ((n != 2u) || (!parse_duty(tok[1], &a))) {
            log_printf("ERR usage=dir <0|1>");
        } else if (app_vf_set_dir((uint8_t)(a + 0.5f)) != 0u) {
            log_printf("ERR dir must be 0 or 1");
        } else {
            log_printf("OK dir = %u", (unsigned)app_vf_info()->dir);
        }
    }
    /* ---------------- phase ----------------- */
    else if (strcmp(tok[0], "phase") == 0) {
        const app_phase_info_t *ph = app_phase_info();

        if ((n >= 2u) && (strcmp(tok[1], "reset") == 0)) {
            app_phase_reset();
            log_printf("OK phase reset");
        } else {
            /* isr: where in the 50us period the ADC interrupt fired (TIM1 CNT,
             * 1 count = 11.76ns, peak = PWM_ARR). The other three are stage
             * durations in tenths of a microsecond (170 cycles = 1us). */
            uint32_t p0 = (ph->pick.min_cyc * 10u) / PHASE_CYC_PER_US;
            uint32_t p1 = (ph->pick.max_cyc * 10u) / PHASE_CYC_PER_US;
            uint32_t c0 = (ph->comp.min_cyc * 10u) / PHASE_CYC_PER_US;
            uint32_t c1 = (ph->comp.max_cyc * 10u) / PHASE_CYC_PER_US;
            uint32_t w0 = (ph->wr.min_cyc   * 10u) / PHASE_CYC_PER_US;
            uint32_t w1 = (ph->wr.max_cyc   * 10u) / PHASE_CYC_PER_US;

            log_printf("---- phase, stage us (isr cnt = sample point) ----");
            log_printf(" isr cnt = %u..%u",
                       (unsigned)ph->isr.cnt_min, (unsigned)ph->isr.cnt_max);
            log_printf(" pick %u.%u..%u.%u  comp %u.%u..%u.%u  wr %u.%u..%u.%u",
                       (unsigned)(p0 / 10u), (unsigned)(p0 % 10u),
                       (unsigned)(p1 / 10u), (unsigned)(p1 % 10u),
                       (unsigned)(c0 / 10u), (unsigned)(c0 % 10u),
                       (unsigned)(c1 / 10u), (unsigned)(c1 % 10u),
                       (unsigned)(w0 / 10u), (unsigned)(w0 % 10u),
                       (unsigned)(w1 / 10u), (unsigned)(w1 % 10u));
        }
    }
    /* ---------------- trig ----------------- */
    else if (strcmp(tok[0], "trig") == 0) {
        const uint32_t n_iter = 1000u;
        uint32_t t0;
        uint32_t cyc;
        uint32_t per;
        uint32_t ns;
        float    th = 0.0f;
        float    c = 0.0f;
        float    s = 0.0f;
        uint32_t i;

        t0 = bsp_prof_now();
        for (i = 0u; i < n_iter; i++) {
            mid_trig_cos_sin(th, &c, &s);
            th += 0.01f;
        }
        cyc = bsp_prof_now() - t0;
        per = cyc / n_iter;
        ns  = (per * 100u) / 17u;              /* 170 cycles = 1us */

        log_printf("trig = %s, %u cycles/call, %u ns, cos1000=%d sin1000=%d",
                   mid_trig_name(), (unsigned)per, (unsigned)ns,
                   (int)(c * 1000.0f), (int)(s * 1000.0f));
    }
    /* ---------------- cordic ----------------- */
    else if (strcmp(tok[0], "cordic") == 0) {
        static const float  ang[4] = { 0.0f, 1.5707963f, 3.1415927f, -1.5707963f };
        static const char  *nm[4]  = { "0", "pi/2", "pi", "-pi/2" };
        uint8_t i;

        log_printf("---- cordic raw readback ----");
        for (i = 0u; i < 4u; i++) {
            int32_t c32 = 0;
            int32_t s32 = 0;
            float   c = 0.0f;
            float   s = 0.0f;

            bsp_cordic_cos_sin_raw(ang[i], &c, &s, &c32, &s32);
            log_printf(" th=%-5s c=%08x s=%08x", nm[i],
                       (unsigned)c32, (unsigned)s32);
        }
        log_printf("-----------------------------");
    }
    /* ---------------- align ----------------- */
    else if (strcmp(tok[0], "align") == 0) {
        uint8_t rc;

        /* "align ol" = open-loop pre-position, works without an encoder. */
        if ((n >= 2u) && (strcmp(tok[1], "ol") == 0)) {
            rc = app_align_ol_start();
        } else {
            rc = app_align_start();
        }

        switch (rc) {
        case 0u:
            log_printf("OK align started, hold theta=0");
            break;
        case 2u:
            log_printf("ERR start first");
            break;
        default:
            log_printf("ERR already aligning");
            break;
        }
    }
    /* ---------------- if (S10.4 open-loop startup) --------------- */
    else if (strcmp(tok[0], "if") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "off") == 0)) {
            app_if_stop();
            log_printf("OK if off");
        } else if ((n >= 3u) && (strcmp(tok[1], "i") == 0)) {
            float a;

            if (parse_num(tok[2], &a) && (a > 0.05f) && (a <= FOC_I_MAX_A)) {
                app_if_set_amp(a);
                log_printf("OK if i = %.2f A", (double)a);
            } else {
                log_printf("ERR usage=if i <0.05..1.20 A>");
            }
        } else if ((n >= 3u) && (strcmp(tok[1], "acc") == 0)) {
            float hz;

            if (parse_num(tok[2], &hz) && (hz >= 2.0f) && (hz <= 300.0f)) {
                app_if_set_acc(hz);
                log_printf("OK if acc = %.0f Hz/s", (double)hz);
            } else {
                log_printf("ERR usage=if acc <2..300 Hz/s>");
            }
        } else if (n >= 2u) {
            float rpm;
            if (!parse_num(tok[1], &rpm)) {
                log_printf("ERR usage=if <rpm 0..600> | if off | if");
            } else if (app_if_start(rpm) == 0u) {
                log_printf("OK if %.0f rpm (%.1f Hz elec, ramp %.0f Hz/s)",
                           (double)rpm, (double)app_if_freq_hz(),
                           (double)IF_ACC_HZ_S);
            } else {
                log_printf("ERR start first");
            }
        } else {
            log_printf("if %s, f %.2f Hz elec (%.0f rpm), i %.2f A, theta %.1f deg",
                       (app_if_on() != 0u) ? "ON" : "off",
                       (double)app_if_freq_hz(),
                       (double)(app_if_freq_hz() * 60.0f / (float)ENC_POLE_PAIRS),
                       (double)app_if_amp(),
                       (double)(app_if_theta() * 57.29577951308232f));
        }
    }

    /* ---------------- sfoc (S10.5 hand-over) --------------------- */
    else if (strcmp(tok[0], "sfoc") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "off") == 0)) {
            app_if_stop();
            app_ho_reset();        /* idle, not watching */
            /* With an encoder fitted this is a switch to encoder closed loop:
             * the speed loop keeps running at the same target, which is what
             * "back to sensor mode" means. Without an encoder there is
             * nothing to hand back to, so releasing means releasing: leaving
             * the loop armed kept a stale 1500 rpm target and let it push a
             * saturated iq_ref into the telemetry (#169). */
            if (mid_enc_enabled() == 0u) {
                app_spd_off();
            }
            log_printf("OK sfoc off (%s)", (mid_enc_enabled() != 0u)
                       ? "encoder mode"
                       : "released, no encoder: use 'sfoc go'");
        } else if ((n >= 3u) && (strcmp(tok[1], "go") == 0)) {
            float rpm;

            if (!parse_num(tok[2], &rpm)) {
                log_printf("ERR usage=sfoc go <rpm>");
            } else if ((rpm != 0.0f) && (fabsf(rpm) < SFOC_RPM_MIN)) {
                /* The observer cannot hold lock this slow: the back-EMF
                 * is below the usable range. Refuse instead of going
                 * sensorless and dropping out a second later. */
                log_printf("ERR sensorless needs |rpm| >= %d", (int)SFOC_RPM_MIN);
            } else if (app_sfoc_on() != 0u) {
                /* Ask THIS first: app_if_start() used to sit in the
                 * condition below, so it started the I/f even when the
                 * drive was already sensorless (visible as "if run 1"
                 * in the diag line). */
                /* Already sensorless: this is a speed change, not a restart.
                 * Report the result - printing OK while the target was
                 * rejected is worse than an error message. */
                {
                    uint8_t rc2 = app_spd_set(rpm);

                    if (rc2 == 0u) {
                        log_printf("OK sfoc go %.0f rpm (already sensorless)",
                                   (double)rpm);
                    } else if (rc2 == 2u) {
                        log_printf("ERR start first");
                    } else if (rc2 == 4u) {
                        log_printf("ERR mode 2 first (FOC)");
                    } else {
                        log_printf("ERR speed too high, |rpm| <= %d",
                                   (int)SPD_RPM_MAX);
                    }
                }
            } else if (app_if_start(rpm) != 0u) {
                log_printf("ERR start first");
            } else {
                /* S10.6: do not make "obs on" a separate ritual - the
                 * hand-over needs it, so switch it on here. */
                if (app_obs_get_on() == 0u) {
                    (void)app_obs_set_on(1u);
                    log_printf("(obs auto on for the hand-over)");
                }
                app_ho_arm();
                log_printf("OK sfoc go %.0f rpm: I/f then hand over", (double)rpm);
            }
        } else {
            const alg_obs_t *o = (const alg_obs_t *)app_obs_info();
            const mid_enc_t *e = mid_enc_info();
            static const char *st[4] = {"idle", "watching", "blending", "done"};
            uint8_t hs = app_ho_state();

            log_printf("sfoc %s, blend %.2f, ds %.1f deg, sensorless %u",
                       (hs < 4u) ? st[hs] : "?", (double)app_ho_w(),
                       (double)(app_ho_ds() * 57.29577951308232f),
                       (unsigned)app_sfoc_on());
            {
                float eq_a3 = sqrtf((o->e_alpha * o->e_alpha) +
                                    (o->e_beta  * o->e_beta));   /* |e| */
                /* Same reference as the hand-over criterion: while the I/f
                 * is dragging use its commanded speed, otherwise the
                 * observer's own (there is nothing better to compare with). */
                float rpm_d = (double)app_if_freq_hz() * (60.0 / 7.0);
                /* S10.15: only trust the drag speed while the drag is
                 * actually running. During the pre-align the commanded
                 * frequency is the 2 Hz floor, i.e. it says nothing about
                 * the back-EMF the rotor should have - that printed a
                 * meaningless 12.45. */
                uint8_t dragging = ((app_if_on() != 0u) &&
                                    (app_if_phase() == 1u)) ? 1u : 0u;
                float w_a3  = (dragging != 0u)
                            ? (rpm_d * 0.104719755120f * 7.0f)
                            : ((o->omega_e > 0.0f) ? o->omega_e : -o->omega_e);
                float ex_a3 = w_a3 * MOTOR_KE_V_RAD_S;
                float r3    = (ex_a3 > 0.10f) ? (eq_a3 / ex_a3) : 0.0f;

                log_printf(" diag: if run %u phase %u f %.1f Hz, ho cnt %u, |e|/exp %.2f",
                           (unsigned)app_if_on(), (unsigned)app_if_phase(),
                           (double)app_if_freq_hz(), (unsigned)app_ho_cnt(),
                           (double)r3);
            }
            log_printf(" obs w %.0f rpm, enc %.0f rpm, eq %.2f V, locked %u",
                       (double)(o->omega_e * 9.54929658551372f / (float)ENC_POLE_PAIRS),
                       (double)e->spd_rpm, (double)o->eq, (unsigned)o->pll.locked);
        }
    }

    /* ---------------- obs (S10.1 back-EMF observer) --------------- */
    else if (strcmp(tok[0], "obs") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "on") == 0)) {
            (void)app_obs_set_on(1u);
            log_printf("OK obs on (shadow mode, not used for control)");
        } else if ((n >= 2u) && (strcmp(tok[1], "off") == 0)) {
            (void)app_obs_set_on(0u);
            log_printf("OK obs off");
        } else if ((n >= 3u) && (strcmp(tok[1], "bw") == 0)) {
            float hz;

            if (parse_num(tok[2], &hz) && (app_obs_set_bw(hz) == 0u)) {
                log_printf("OK obs bw = %.1f Hz", (double)hz);
            } else {
                log_printf("ERR usage=obs bw <5..500 Hz>");
            }
        } else if ((n >= 3u) && (strcmp(tok[1], "lpf") == 0)) {
            float hz;

            if (parse_num(tok[2], &hz) && (app_obs_set_lpf(hz) == 0u)) {
                log_printf("OK obs lpf = %.0f Hz (%s)", (double)hz,
                           (hz > 0.0f) ? "on" : "off");
            } else {
                log_printf("ERR usage=obs lpf <0..8000 Hz>");
            }
        } else if ((n >= 3u) && (strcmp(tok[1], "comp") == 0)) {
            uint32_t on;

            if (parse_u32(tok[2], &on) && (on <= 2u)) {
                app_obs_set_comp((uint8_t)on);
                log_printf("OK obs comp = %u", (unsigned)on);
            } else {
                log_printf("ERR usage=obs comp <0|1|2>");
            }
        } else if ((n >= 3u) && (strcmp(tok[1], "trim") == 0)) {
            float deg;

            if (parse_num(tok[2], &deg) && (deg > -180.0f) && (deg < 180.0f)) {
                app_obs_set_trim(deg);
                log_printf("OK obs trim = %.2f deg", (double)deg);
            } else {
                log_printf("ERR usage=obs trim <-180..180>");
            }
        } else if ((n >= 3u) && (strcmp(tok[1], "delay") == 0)) {
            float us;

            if (parse_num(tok[2], &us) && (us >= 0.0f) && (us <= 500.0f)) {
                app_obs_set_delay(us);
                log_printf("OK obs delay = %.0f us", (double)us);
            } else {
                log_printf("ERR usage=obs delay <0..500 us>");
            }
        } else if ((n >= 3u) && ((strcmp(tok[1], "r") == 0) ||
                                 (strcmp(tok[1], "l") == 0))) {
            float val;
            const alg_obs_t *o = (const alg_obs_t *)app_obs_info();

            if (!parse_num(tok[2], &val) || (val <= 0.0f)) {
                log_printf("ERR usage=obs r <ohm> | obs l <mH>");
            } else if (tok[1][0] == (char)0x72) {     /* r */
                app_obs_set_rl(val, o->l_h);
                log_printf("OK obs R = %.3f ohm", (double)val);
            } else {                                  /* l */
                app_obs_set_rl(o->r_ohm, val * 0.001f);
                log_printf("OK obs L = %.3f mH", (double)val);
            }
        } else {
            const alg_obs_t *o = (const alg_obs_t *)app_obs_info();
            const mid_enc_t *e = mid_enc_info();
            float err_deg;
            float rpm;

            err_deg = o->theta_out - (e->elec_deg * 0.0174532925199433f);
            while (err_deg >  3.14159265f) { err_deg -= 6.283185307179586f; }
            while (err_deg < -3.14159265f) { err_deg += 6.283185307179586f; }
            err_deg *= 57.29577951308232f;

            rpm = o->omega_e * 9.54929658551372f / (float)ENC_POLE_PAIRS;

            log_printf("obs %s, R %.3f ohm, L %.3f mH, e %.2f/%.2f V",
                       (app_obs_get_on() != 0u) ? "ON (shadow)" : "off",
                       (double)o->r_ohm, (double)(o->l_h * 1000.0f),
                       (double)o->e_alpha, (double)o->e_beta);
            log_printf(" ed %.3f V, eq %.3f V, theta %.1f deg, w %.0f rpm",
                       (double)o->ed, (double)o->eq,
                       (double)(o->theta_out * 57.29577951308232f), (double)rpm);
            log_printf(" enc elec %.1f deg, angle err %.2f deg, locked %u",
                       (double)e->elec_deg, (double)err_deg,
                       (unsigned)o->pll.locked);
        }
    }

    /* ---------------- enc ----------------- */
    /* ---------------- ssi ---------------- */
    else if (strcmp(tok[0], "ssi") == 0) {
        /* Three raw bytes tell a slipped reader apart from real noise: a frame
         * that is rotated by one byte (or by 16 bits) is obvious in the dump,
         * while white noise is not. 18#L-16. */
        const mid_enc_t *e = mid_enc_info();
        uint8_t f[4];

        mid_enc_last_frame(f);
        log_printf("---- ssi (MT6701 frame) ----");
        log_printf(" bytes = %02x %02x %02x %02x (window +%u bit)",
                   (unsigned)f[0], (unsigned)f[1], (unsigned)f[2], (unsigned)f[3],
                   (unsigned)mid_enc_win());
        /* A short frame means the sensor stopped driving mid-word, i.e. its bit
         * counter is parked inside the 24-bit word (docs/00 #053). */
        log_printf(" frame = %u of %u data byte(s)",
                   (unsigned)mid_enc_data_bytes(), (unsigned)BSP_SSI_FRAME_LEN);
        log_printf(" raw   = %u, mg %u, %s",
                   (unsigned)e->raw, (unsigned)e->mg,
                   (e->crc_ok != 0u) ? "crc ok" : "crc BAD");
        log_printf(" errs  = badrun %u, resync %u (busy %u), ovr %u, crc %u/%u",
                   (unsigned)mid_enc_bad_run(), (unsigned)bsp_ssi_resync_count(),
                   (unsigned)bsp_ssi_resync_busy_count(),
                   (unsigned)bsp_ssi_ovr_count(),
                   (unsigned)e->crc_errs, (unsigned)e->frames);
    }
    else if (strcmp(tok[0], "enc") == 0) {
        /* enc off / enc on: no encoder fitted (sensorless build). */
        if ((n == 2u) && (strcmp(tok[1], "off") == 0)) {
            mid_enc_set_enabled(0u);
            log_printf("OK enc off: encoder disabled");
            return;
        }
        if ((n == 2u) && (strcmp(tok[1], "on") == 0)) {
            mid_enc_set_enabled(1u);
            log_printf("OK enc on");
            return;
        }

        const mid_enc_t *e = mid_enc_info();

        if ((n >= 3u) && (strcmp(tok[1], "zero") == 0)) {
            /* Explicit value: restore a previously recorded zero exactly.
             * "enc zero" (no argument) captures the current position. */
            float deg;

            if (!parse_num(tok[2], &deg) || (deg < 0.0f) || (deg >= 360.0f)) {
                log_printf("ERR usage=enc zero [<deg 0..360>]");
            } else if (mid_enc_zero_set(deg) == 0u) {
                log_printf("OK enc zero = %.2f deg (saved)", (double)deg);
            } else {
                log_printf("ERR enc zero write failed");
            }
        } else if ((n >= 2u) && (strcmp(tok[1], "zero") == 0)) {
            if (mid_enc_zero() == 0u) {
                log_printf("OK enc zero, off %.2f deg (saved)",
                           (double)mid_enc_info()->zero_deg);
            } else {
                log_printf("ERR enc read failed");
            }
        } else if ((n >= 3u) && (strcmp(tok[1], "dir") == 0)) {
            uint32_t dv = 0u;

            if ((parse_u32(tok[2], &dv) == 0u) || (dv > 1u)) {
                log_printf("ERR usage=enc dir <0|1>");
            } else if (mid_enc_set_dir((uint8_t)dv) == 0u) {
                log_printf("OK enc dir = %u", (unsigned)dv);
            } else {
                log_printf("ERR enc dir");
            }
        } else if ((n >= 2u) && (strcmp(tok[1], "bench") == 0)) {
            uint8_t  f[BSP_SSI_FRAME_LEN];
            uint32_t k;
            uint32_t t0;
            uint32_t cyc;
            uint32_t cfg1 = 0u;
            uint32_t ccipr2 = 0u;

            /* SPI only: no decode, no speed estimator */
            t0 = bsp_prof_now();
            for (k = 0u; k < 200u; k++) {
                (void)bsp_ssi_read(f);
            }
            cyc = (bsp_prof_now() - t0) / 200u;
            bsp_ssi_diag(&cfg1, &ccipr2);

            log_printf("enc bench = %u cycles/read, %u ns",
                       (unsigned)cyc, (unsigned)((cyc * 100u) / 17u));
            log_printf(" spi cr1=%08x ccipr2=%08x",
                       (unsigned)cfg1, (unsigned)ccipr2);
        } else if ((n >= 2u) && (strcmp(tok[1], "raw") == 0)) {
            uint8_t k;

            for (k = 0u; k < 5u; k++) {
                uint8_t f[BSP_SSI_FRAME_LEN];

                if (bsp_ssi_read(f) == 0u) {
                    log_printf(" enc raw %02x %02x %02x %02x",
                               (unsigned)f[0], (unsigned)f[1], (unsigned)f[2],
                               (unsigned)f[3]);
                } else {
                    log_printf(" enc raw ERR");
                }
            }
        } else if (e->valid == 0u) {
            log_printf("enc = no frame yet (frames=%u io_err=%u)",
                       (unsigned)e->frames, (unsigned)e->io_errs);
        } else {
            log_printf("enc = mech %.2f elec %.2f deg, raw %u, mg %u, crc %s",
                       (double)e->mech_deg, (double)e->elec_deg,
                       (unsigned)e->raw, (unsigned)e->mg,
                       (e->crc_ok != 0u) ? "ok" : "BAD");
            log_printf(" spd = %.2f Hz elec, %.1f rpm mech",
                       (double)e->spd_hz, (double)e->spd_rpm);
            log_printf(" err = %u/%u crc, %u ovr, %u late, zero %.1f deg, dir %u, src %s",
                       (unsigned)e->crc_errs, (unsigned)e->frames,
                       (unsigned)e->ovr, (unsigned)e->late,
                       (double)e->zero_deg,
                       (unsigned)e->dir,
                       (e->src == ENC_SRC_FLASH) ? "flash" : "default");
            log_printf(" pll clears = %u (on stop/fault)",
                       (unsigned)mid_enc_pll_clears());
        }
    }
    /* ---------------- cur ----------------- */
    else if (strcmp(tok[0], "cur") == 0) {
        const current_sample_t *cs = mid_current_get();
        if (cs->ready == 0u) {
            /* No calibration record and no live calibration yet: the nominal
             * 2048 in offset[] is not a measurement, so do not show it as one. */
            log_printf("cur = raw ----/----/----, off ----/----/---- (not calibrated)");
        } else if (bsp_adc_frames() == 0u) {
            /* Nothing has been converted yet: the TIM1_CC4 trigger only reaches
             * the ADC while the bridge is enabled. Show that instead of 0/0/0,
             * which looks like a broken reading. */
            log_printf("cur = raw ----/----/----, off %u/%u/%u",
                       (unsigned)cs->offset[0], (unsigned)cs->offset[1],
                       (unsigned)cs->offset[2]);
        } else {
            log_printf("cur = raw %u/%u/%u, off %u/%u/%u",
                       (unsigned)cs->raw[0], (unsigned)cs->raw[1],
                       (unsigned)cs->raw[2],
                       (unsigned)cs->offset[0], (unsigned)cs->offset[1],
                       (unsigned)cs->offset[2]);
        }
        log_printf("cur = %d/%d/%d mA, sat=%u",
                   (int)(cs->ia * 1000.0f), (int)(cs->ib * 1000.0f),
                   (int)(cs->ic * 1000.0f), (unsigned)cs->saturated);
        log_printf("adc = frames=%u, ovr=%u, cal=%u, src=%s, gen=%u, st=%u",
                   (unsigned)bsp_adc_frames(), (unsigned)bsp_adc_ovr_count(),
                   (unsigned)cs->ready, cal_src_str(cs->src),
                   (unsigned)cs->gen, (unsigned)bsp_adc_status());
        {
            uint32_t cfg[3];
            bsp_adc_get_cfg(cfg);
            log_printf("adc = cr=%08x cfgr=%08x jsqr=%08x",
                       (unsigned)cfg[0], (unsigned)cfg[1], (unsigned)cfg[2]);
        }
    }
    /* ---------------- cal ----------------- */
    else if (strcmp(tok[0], "cal") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "erase") == 0)) {
            /* Erasing restarts the averaging window, so the next 256 frames
             * become the new offset. If the bridge is running, force the zero
             * vector first: otherwise the load current would be averaged in
             * and baked into the stored offset. */
            if (app_status()->state == APP_ST_RUN) {
                bsp_pwm_set_all_duty(PWM_START_DUTY);
            }
            mid_current_erase_cal();
            log_printf("OK cal erased, will calibrate at next start");
        } else if (app_status()->state != APP_ST_RUN) {
            log_printf("ERR start first (ADC trigger needs MOE=1)");
        } else {
            /* Zero vector during calibration: no phase-to-phase voltage, so the
             * offset really is the zero-current reading. */
            bsp_pwm_set_all_duty(PWM_START_DUTY);
            mid_current_restart_cal();
            log_printf("OK cal started, %u frames at duty 0.50, saved to flash",
                       (unsigned)CUR_CAL_SAMPLES);
        }
    }
    /* ---------------- prof ---------------- */
    else if (strcmp(tok[0], "prof") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "reset") == 0)) {
            bsp_prof_reset();
            log_printf("OK prof reset");
        } else if ((n >= 2u) && (strcmp(tok[1], "off") == 0)) {
            bsp_prof_enable(0u);
            log_printf("OK prof off");
        } else if ((n >= 2u) && (strcmp(tok[1], "on") == 0)) {
            bsp_prof_enable(1u);
            log_printf("OK prof on");
        } else {
            static const char *const names[PROF_SLOT_COUNT] = {
                "isr", "isr_period", "adc_read", "conv",
                "wave", "cmd", "txpump", "loop", "enc", "imu", "att", "bal", "balrun"
            };
            uint8_t i;

            log_printf("---- profile (ns) ----");
            for (i = 0u; i < (uint8_t)PROF_SLOT_COUNT; i++) {
                log_printf(" %-10s n=%u min=%u avg=%u max=%u",
                           names[i],
                           (unsigned)bsp_prof_count((prof_slot_t)i),
                           (unsigned)bsp_prof_min_ns((prof_slot_t)i),
                           (unsigned)bsp_prof_avg_ns((prof_slot_t)i),
                           (unsigned)bsp_prof_max_ns((prof_slot_t)i));
            }
            log_printf("----------------------");
        }
    }
    /* ---------------- led ----------------- */
    else if (strcmp(tok[0], "led") == 0) {
        if (n < 2u)                             { log_printf("ERR usage=led <on|off|toggle>"); }
        else if (strcmp(tok[1], "on") == 0)     { bsp_led_write(1u); log_printf("OK led ON"); }
        else if (strcmp(tok[1], "off") == 0)    { bsp_led_write(0u); log_printf("OK led OFF"); }
        else if (strcmp(tok[1], "toggle") == 0) { bsp_led_toggle();   log_printf("OK led %s", bsp_led_is_on() ? "ON" : "OFF"); }
        else                                    { log_printf("ERR usage=led <on|off|toggle>"); }
    }
    /* ---------------- wave ---------------- */
    else if (strcmp(tok[0], "wave") == 0) {
        if (n < 2u) { log_printf("ERR usage=wave <on [hz]|off>"); }
        else if (strcmp(tok[1], "on") == 0) {
            uint16_t rate = app_status()->wave_rate_hz;

            /* #167: 21 channels at 500 Hz is ~685 kbit/s (137 B/frame) on a
             * 1 Mbaud link, i.e. 69% of it;
             * the DAPLink VCP then overflows and the host sees truncated
             * frames ("214.927,0"). Allow the rate to follow the channel
             * count: "wave on 250". */
            if (n >= 3u) {
                if (!parse_u32(tok[2], &val)) {
                    log_printf("ERR usage=wave on [hz]");
                    return;
                }
                rate = (uint16_t)val;
            }
            app_wave_set(1u, rate);
            log_printf("OK wave ON @ %u Hz, select FireWater in VOFA+",
                       (unsigned)app_status()->wave_rate_hz);
        } else if (strcmp(tok[1], "off") == 0) {
            app_wave_set(0u, app_status()->wave_rate_hz);
            log_printf("OK wave OFF");
        } else {
            log_printf("ERR usage=wave <on [hz]|off>");
        }
    }
    /* ---------------- rate ---------------- */
    else if (strcmp(tok[0], "rate") == 0) {
        if (n < 2u)                        { log_printf("ERR usage=rate <hz>"); }
        else if (!parse_u32(tok[1], &val)) { log_printf("ERR bad arg"); }
        else {
            app_wave_set(app_status()->wave_on, (uint16_t)val);
            log_printf("OK rate = %u Hz", (unsigned)app_status()->wave_rate_hz);
        }
    }
    /* ---------------- clr ----------------- */
    else if (strcmp(tok[0], "clr") == 0) {
        app_counters_clear();
        log_printf("OK counters cleared");
    }
    /* ---------------- echo ---------------- */
    else if (strcmp(tok[0], "echo") == 0) {
        if (n < 2u) { log_printf("(empty)"); }
        else        { log_printf("%s", tok[1]); }
    }
    /* ---------------- unknown ------------- */
    else {
        log_printf("ERR unknown cmd '%s', try 'help'", tok[0]);
    }
}

/* ------------------------------------------------------------------ */
/* Init / poll                                                         */
/* ------------------------------------------------------------------ */
void app_cmd_init(void)
{
    s_line_len = 0u;
    s_line[0]  = '\0';
}

void app_cmd_poll(void)
{
    uint8_t  buf[32];
    uint16_t n;
    uint16_t i;

    n = bsp_uart_rx_read(buf, (uint16_t)sizeof(buf));
    for (i = 0u; i < n; i++) {
        char c = (char)buf[i];

        if ((c == '\r') || (c == '\n')) {
            if (s_line_len > 0u) {
                s_line[s_line_len] = '\0';
                cmd_exec(s_line);
                s_line_len = 0u;
            }
        } else if ((c == 0x08) || (c == 0x7F)) {          /* backspace */
            if (s_line_len > 0u) { s_line_len--; }
        } else if ((c >= 0x20) && (c <= 0x7E)) {          /* printable ASCII only */
            if (s_line_len < (CMD_LINE_MAX - 1u)) {
                s_line[s_line_len++] = c;
            } else {
                s_line_len = 0u;
                log_printf("ERR line too long");
            }
        } else {
            /* ignore other control characters */
        }
    }
}
