/**
  ******************************************************************************
  * @file    board_cfg.h
  * @brief   Board-level hardware constants (Board_v2 / STM32G431KBT6).
  *
  *          Every number that is tied to this specific board lives here.
  *          No magic numbers are allowed in the rest of the code.
  *
  *          NOTE: this file must stay pure ASCII. Keil ARMCC V5 parses source
  *          as GBK; UTF-8 multibyte characters inside string literals break
  *          the tokenizer. Chinese documentation lives in /docs instead.
  ******************************************************************************
  */

#ifndef __BOARD_CFG_H
#define __BOARD_CFG_H


#include "motor_cfg.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==========================================================================
 * 0. Board identity
 * ========================================================================== */
#define BOARD_NAME              "cubli/STM32G431KBT6"

/* Firmware build id - bump by 1 on EVERY firmware change.
 *
 * Why this exists: Keil ARMCC V5 builds incrementally. __DATE__/__TIME__ are
 * frozen into whichever .c file happens to contain them, so a build that only
 * recompiles some files keeps the OLD timestamp in the image. That makes
 * "does the board run the newest hex?" impossible to answer from the console.
 * This id is the authoritative answer.
 *
 * Bonus: editing this header touches every file that includes board_cfg.h, so
 * bumping it forces a full rebuild and refreshes __DATE__/__TIME__ as well.
 *
 * The acceptance script (test/accept/s1_accept.py) greps this value out of the
 * source and compares it with what the board reports. */
#define FW_BUILD_ID         1u

/* ---- zero-drift calibration record in flash ---------------------------- */
/* The last 2KB page of the 128KB part is reserved for it; the Keil target ROM
 * region is shrunk to 0x1F800 so the linker never places code there.
 * magic 'CAL1', version bumps if the layout changes (v2 added the generation). */
#define CAL_FLASH_ADDR          0x0801F800u
#define CAL_FLASH_MAGIC         0x314C4143u     /* "CAL1" little endian */
#define CAL_FLASH_VERSION       2u

/* ==========================================================================
 * 1. System clock / control period
 * ========================================================================== */
#define SYS_CORE_CLK_HZ         170000000u
#define FOC_TS                  0.00005f        /* 20kHz current loop = 50us */
#define FOC_FREQ_HZ             20000u
/* Integer version for display: 0.00005f * 1e6 evaluates to 49.99999 and would
 * truncate to 49, so always use this macro when printing Ts. */
#define FOC_TS_US               (1000000u / FOC_FREQ_HZ)
/* DWT cycles per microsecond; used by the "phase" probe to print delays
 * without a 64-bit division (170 cycles = 1us at 170MHz). */
#define PHASE_CYC_PER_US        (SYS_CORE_CLK_HZ / 1000000u)
/* ==========================================================================
 * 2. TIM1 / PWM generation
 *    f_pwm = 170MHz / (2 * (ARR+1)) = 20.000kHz
 * ========================================================================== */
#define PWM_ARR                 4249u
#define PWM_CCR_PERIOD          (PWM_ARR + 1u)          /* 4250 duty steps */
#define PWM_FREQ_HZ_CALC        (SYS_CORE_CLK_HZ / (2u * PWM_CCR_PERIOD))

/* Dead time: DTG=4 -> 4 * 5.882ns = 23.5ns (TIM1 side, nominal value).
 *
 * IMPORTANT: EG3033 has a built-in 60ns dead time with cross-lock logic, so
 * the effective dead time is max(23.5ns, 60ns) = 60ns. Any DTG <= 10 gives
 * the same 60ns result. AO4606 also has ~96ns of "natural" dead time
 * (P-MOS turns off in ~80ns, N-MOS needs ~157ns to reach Vgs(th)), so shoot
 * through is impossible even with DTG = 0.
 * Minimum dead time keeps the phase current waveform clean. */
#define PWM_DTG_REG             4u
/* CH1..CH3 run in PWM mode 2: the high pulse is centred on the counter peak
 * and the low sides conduct around the valley. CCR is therefore the *inactive*
 * (low-side) duration, so CCR_MIN gives the MAXIMUM duty and CCR_MAX the
 * minimum. Keeping 40..4209 never reaches 0% / 100% and always leaves a
 * low-side conduction window for the current sampling at the valley (S3). */
#define PWM_CCR_MIN             40u                     /* duty  99.06% */
#define PWM_CCR_MAX             (PWM_ARR - 40u)         /* duty   0.94% */

/* Duty applied when the bridge is first enabled. 50% on all three phases is
 * the zero vector: no phase-to-phase voltage, so it is safe with or without a
 * motor connected. */
#define PWM_START_DUTY          0.5f

/* CH4 / PA11 (ADC trigger for S3) is configured ENTIRELY by CubeMX: the
 * "Pulse" value of TIM1 CH4 sets CCR4. With PWM2 the pin is high while
 * CNT >= CCR4, so:
 *   - CCR4 = ARR   -> rising edge exactly on the counter peak, ~6ns pulse
 *                     (correct, but invisible on a scope);
 *   - CCR4 = 4000  -> ~2.9us pulse, edge ~1.5us before the peak
 *                     (current CubeMX value: visible, still near the centre).
 * The firmware deliberately does NOT write CCR4, so the CubeMX setting stands. */

/* ==========================================================================
 * 3. Three-phase current sensing (3 x INA240A2 + 25mOhm low-side shunt)
 *    Vout = 50 * Vsense + VREF,  VREF = (REF1+REF2)/2 = (0+3.3)/2 = 1.65V
 * ========================================================================== */
#define RSHUNT_OHM              0.025f          /* 25mOhm fitted (schematic says 10mOhm) */
#define INA240_GAIN             50.0f           /* INA240A2 = 50 V/V */
#define ADC_VREF                3.3f
#define ADC_FULL_SCALE          4096.0f

/* Amps per LSB: 0.806mV / (50 * 0.025) = 0.645mA */
#define CUR_A_PER_LSB           ((ADC_VREF / ADC_FULL_SCALE) / (INA240_GAIN * RSHUNT_OHM))
/* Zero-current ADC code (1.65V bias) */
#define CUR_ZERO_CODE           (ADC_FULL_SCALE * 0.5f)
/* Bipolar range is only +-1.32A (1.65V / 1.25V per A) */
#define CUR_RANGE_A             ((ADC_VREF * 0.5f) / (INA240_GAIN * RSHUNT_OHM))

/* Software over-current trip (used from S7 on) */

/* Zero-drift calibration: average this many injected frames at 20kHz (12.8ms).
 * Runs at "start" with the duty forced to the 50% zero vector, because the
 * TIM1_CC4 trigger only reaches the ADC while the bridge is enabled (MOE=1). */
#define CUR_CAL_SAMPLES         256u

/* Saturation flag threshold: raw code within this many LSB of a rail. */
#define CUR_SAT_MARGIN          8u

/* Sign convention: positive = current flowing OUT of the bridge into the motor.
 * Which way the INA240 actually reads depends on how IN+/IN- are wired across
 * the shunt. Measured on the bench 2026-10-25 (motor, 0.55/0.45/0.45):
 * A phase high -> current leaves the A bridge, but raw dropped below the offset,
 * so the reading is inverted; -1.0f makes "out of the bridge" positive. */
#define CUR_SIGN                (-1.0f)

/* ==========================================================================
 * 4. DC bus voltage sensing
 *    DC_BUS -> R20 12k -> PA1 (VBUS_A) -> R21 2k -> GND
 *    Divider ratio = (12k+2k)/2k = 7
 *    12V bus -> 1.714V -> code ~2128 (max measurable 23.1V)
 * ========================================================================== */
#define VBUS_DIV_R_TOP          12000.0f
#define VBUS_DIV_R_BOT          2000.0f
#define VBUS_DIV_RATIO          ((VBUS_DIV_R_TOP + VBUS_DIV_R_BOT) / VBUS_DIV_R_BOT)
#define VBUS_V_PER_LSB          ((ADC_VREF / ADC_FULL_SCALE) * VBUS_DIV_RATIO)  /* 5.64mV */

#define VBUS_NOMINAL_V          12.0f
/* S7.6 protections (runtime adjustable with the "prot" command). */
#define PROT_OVERSPEED_RPM      2000.0f
#define PROT_OVERSPEED_MS       100u
#define PROT_STALL_RPM          30.0f
#define PROT_STALL_IQ_A         0.6f
#define PROT_STALL_MS           500u
/* The stall judge uses a low-passed speed: a rotor shaking against
 * a stalled speed loop crosses +-30 rpm every oscillation and kept
 * resetting the debounce (measured 04:56, hand-loaded rotor: no trip).
 * tau = 1/(2*pi*fc) with k = 0.01 at 1 kHz is about 100 ms. */
#define PROT_STALL_LP_K         0.01f
#define PROT_VBUS_MS            50u
/* 100 ISR periods = 5 ms at 20 kHz. 20 was too tight: one bit-level
 * frame slip (which the 32-rotation decoder cannot undo) faulted the
 * drive 1 ms after start, and the cube fell on every button press. */
#define PROT_ENC_FRAMES         100u
/* A disconnected sensor reads raw 0 forever (the all-zero CRC also passes),
 * so a long run of exactly zero is treated as a dead encoder: 200 frames
 * at 20 kHz is 10 ms, far longer than a real 0-degree reading would last. */
#define PROT_ENC_ZERO_FRAMES    200u
/* A corrupt SSI frame is rejected by mid_enc, which keeps the previous angle.
 * That is fine for one or two frames, but a sensor that keeps answering with
 * bad CRC (measured 2026-11-05: MT6701 latched a magnetic-field status and
 * drove the frame tail high, crc BAD on ~90 % of frames) leaves the FOC
 * steering on a frozen angle: the current vector lands in the wrong frame and
 * the phase currents run away into the overcurrent trip. Count consecutive
 * bad samples at 1 kHz and fault well before that can matter. */
/* 200 frames = 10 ms at 20 kHz. mid_enc holds the last good angle while
 * frames are corrupt, so a 10 ms window is safe; the rotation decoder
 * made permanent 100 % CRC failure stop being the normal case. */
#define PROT_ENC_BAD_FRAMES     200u
#define PROT_GRACE_MS           200u    /* ignore speed/Vbus after start */

/* The cube runs from a 4S LiPo pack (14.1 V measured, 16.8 V full charge),
 * not from the original 12 V bench supply, so the trip points follow the pack
 * and not the old nominal: OV = 16.8 V + 1.2 V margin, UV = 3.0 V per cell
 * (12.0 V) to stop before the pack is damaged. AO4606 is rated 30 V and the
 * divider saturates at 23.1 V, so both limits sit far inside the hardware. */
#define VBUS_OV_V               18.0f           /* 4S full 16.8 V + 7% margin */
#define VBUS_UV_V               12.0f           /* 3.0 V per cell */
#define VBUS_MAX_MEASURABLE_V   (ADC_VREF * VBUS_DIV_RATIO)   /* 23.1V */

/* ==========================================================================
 * 5. Open-loop V/f (S4)
 *    Motor: 7 pole pairs, 12V, 330KV  ->  no-load 3960rpm = 462Hz electrical
 *    V/f curve: V_LL_rms(f) = VF_BOOST_V + VF_K_V_PER_HZ * f
 *    The modulation index is derived from the line-to-line RMS voltage:
 *      V_LL_rms = (sqrt3/2)/sqrt2 * m * Vdc = 0.6124 * m * Vdc
 *      => m = 1.633 * V_LL_rms / Vdc
 * ========================================================================== */
#define VF_BOOST_V              1.5f            /* low-speed IR boost, V_LL_rms */
#define VF_K_V_PER_HZ           0.026f          /* 12V / 462Hz */
#define VF_M_PER_V              (1.632993f / VBUS_NOMINAL_V)
#define VF_M_MAX                0.95f           /* keep 5% headroom before clamping */
#define VF_F_MAX                200.0f          /* first release: 1714 rpm */
#define VF_ACC_HZ_PER_S         20.0f           /* 0 -> 200Hz in 10s */
#define VF_ACC_MIN_HZ_PER_S     1.0f
#define VF_ACC_MAX_HZ_PER_S     200.0f
#define VF_TS_S                 (1.0f / 20000.0f)

/* Guard bands: the ADC range is only +-1.32A and the motor would draw 3.5A at
 * 12V stall, so the open loop must throttle itself before the sensor clips. */
#define VF_I_FOLD_A             1.00f           /* start folding the modulation */
#define VF_I_FOLD_BACK_A        0.80f           /* recover below this */
#define VF_I_TRIP_A             1.25f           /* hard trip -> FAULT */
#define VF_I_FOLD_FRAMES        20u             /* consecutive frames */
#define VF_I_TRIP_FRAMES        5u
#define VF_M_SCALE_MIN          0.30f

/* ==========================================================================
 * 6. Trigonometry back end (S4 bench experiment)
 *    TRIG_MODE_LUT    : 256-entry table + linear interpolation (default)
 *    TRIG_MODE_CORDIC : hardware CORDIC co-processor
 *    Switching is a one-line rebuild; the LUT stays the default so the
 *    verified behaviour does not change unless explicitly asked for.
 * ========================================================================== */
#define TRIG_MODE_LUT           0
#define TRIG_MODE_CORDIC        1

#ifndef TRIG_MODE
#define TRIG_MODE               TRIG_MODE_LUT
#endif

/* ==========================================================================
 * 5. LED / KEY
 *    LED : 3V3 -> R37 470R -> LED anode..cathode -> PF1  => active LOW
 *    KEY : 3V3 -> R24 10k -> PA0 -> push button -> GND   => pressed = LOW
 * ========================================================================== */
#define LED_PORT                LED_GPIO_Port
#define LED_PIN_HW              LED_Pin
#define LED_ACTIVE_LOW          1

#define KEY_PORT                KEY_GPIO_Port
#define KEY_PIN_HW              KEY_Pin
#define KEY_ACTIVE_LOW          1
#define KEY_DEBOUNCE_MS         20u

/* ==========================================================================
 * 6. Debug UART
 *    USART2 (PA2=TX / PA3=RX) goes to the on-board CH340N -> USB Type-C.
 *    USART1 (PB6/PB7 -> CN3) is reserved for the N100 Mini IMU (see the
 *    hardware document, docs/19).
 *    This is why the debug console moved: plugging the IMU into CN3 kills the
 *    USB-TTL link on COM7, so logs come out of COM13 instead.
 * ========================================================================== */
#define DBG_UART_BAUD           1000000u
#define DBG_RX_BUF_SIZE         128u            /* DMA circular buffer (power of 2) */
#define DBG_RX_RING_SIZE        1024u           /* parser ring buffer (power of 2) */
#define DBG_TX_RING_SIZE        4096u           /* TX ring buffer (power of 2) */
#define DBG_TX_DMA_CHUNK        64u             /* bytes per DMA transfer */

/* TX watchdog. One 64-byte chunk at 1Mbaud takes ~0.64 ms, so a completion
 * event that has not arrived 100 ms later is lost (UART error, DMA glitch).
 * Without this, s_tx_busy would stay set forever and the debug link would go
 * permanently silent while the motor keeps running. See bsp_uart_tx_pump(). */
#define DBG_TX_STUCK_MS         100u
/* Consecutive HAL_BUSY answers from HAL_UART_Transmit_DMA before forcing an
 * abort.  The handle can wedge with gState stuck in BUSY_TX, and then the
 * pump would silently retry forever (tx_pending froze at 412 B). */
#define DBG_TX_BUSY_ABORT       50u

/* Waveform output goes out on the SAME UART as the text (USART2) using the
 * FireWater text protocol, so VOFA+ can plot and you can still read the log. */
/* FireWater CSV with 6 channels is ~44 B/frame:
 *   100 Hz ->  4.4 kB/s ( 4% of 1Mbaud)
 *   500 Hz -> 22.0 kB/s (22% of 1Mbaud)   <- cap
 *  1000 Hz -> 44.0 kB/s (44% of 1Mbaud)   <- theoretical ceiling of
 *                                            HAL_GetTick (1ms), too tight */
#define DBG_WAVE_RATE_MAX       500u
/* Default = max: user asked to run at the highest rate by default.
 * Drop it with the "rate <hz>" command when a lighter stream is wanted. */
#define DBG_WAVE_RATE_HZ        DBG_WAVE_RATE_MAX


/* ==========================================================================
 * 6b. IMU link: N100 Mini nine-axis AHRS over USART1
 *     PB6 = USART1_TX, PB7 = USART1_RX -> CN3 (WAFER-SH1.0-6P)
 *     RX: DMA1_Channel1 circular, TX: DMA1_Channel2 normal (see docs/19 sec.9)
 *     Protocol: FDILink V21.0823, frames 0x40 (IMU) / 0x41 (AHRS),
 *     measured 200 frames/s (5 ms per frame) on the previous project.
 *
 *     Buffer sizing rationale (ported from the gyroscope project, unchanged):
 *       every 5 ms the module sends about 110 B (0x40 = 64 B, 0x41 = 56 B on the
 *       wire), i.e. ~22 kB/s. The main loop runs at a few kHz, so a 2 kB ring
 *       absorbs ~90 ms of stall; that is far more than any loop iteration needs.
 * ========================================================================== */
#define IMU_UART_BAUD           921600u
#define IMU_RX_BUF_SIZE         512u            /* DMA circular buffer (power of 2) */
#define IMU_RX_STREAM_RING      2048u           /* parser ring buffer (power of 2) */
#define IMU_ASCII_RING_SIZE     512u            /* config-mode ASCII replies */
#define IMU_TX_RING_SIZE        128u            /* config commands queued for TX */
#define IMU_TX_DMA_CHUNK        32u             /* bytes per interrupt transfer */
#define IMU_TX_STUCK_MS         50u             /* TX watchdog: panel data is tiny */

/* A frame arrives every 5 ms; 500 ms of silence means the cable is out, the
 * module rebooted, or the baud rate is wrong. Used by imu_stream_ok(), which the
 * balance state machine will require before allowing "bal on". */
#define IMU_FRAME_TIMEOUT_MS    500u
/* Longest wait for a config-command reply (configuration only, never in control) */
#define IMU_CMD_TIMEOUT_MS      800u

/* Sniff ring for "imu raw"/offline frame checks; must be a power of two */
#define IMU_UART_SNIFF_SIZE     256u


/* ==========================================================================
 * 7. MT6701 magnetic encoder (SSI over SPI1)
 *    PA15 = CSN, PB3 = SCK, PB4 = MISO, PB5 = MOSI, SPI mode 1, 10.625 MHz
 *    Frame: D[13:0] angle, Mg[3:0] status, CRC[5:0] (x^6+x+1)
 * ========================================================================== */
#define ENC_POLE_PAIRS          MOTOR_POLE_PAIRS  /* -> motor_cfg.h */
#define ENC_DIR_DEFAULT         0u              /* 0 = CCW positive */

/* Zero offset + direction live in their own page so the current-calibration
 * record (0x0801F800) is never touched by "enc zero". */
#define ENC_FLASH_ADDR          0x0801F000u
#define ENC_FLASH_MAGIC         0x31434E45u     /* ENC1 little endian */
#define ENC_FLASH_VERSION       1u


/* ---- ALIGN: forced alignment for the electrical zero (S5) ------------- */
/* A fixed voltage vector at theta = 0 pulls the rotor d-axis onto the phase A
 * axis. m = 0.20 gives ~1.5V line-to-line, i.e. ~0.33A through 4.6 ohm. */
/* Alignment strategy: bidirectional sweep averaging.
 *
 * A static d-axis vector (the classic method) does not work on this motor:
 * the magnetic torque falls as sin(dtheta) near the equilibrium, and the
 * bearing stiction plus cogging stop the rotor a few degrees short (measured:
 * 0.5A moved it 0.83 deg, 1.0A only 0.27 deg). Spinning the field drags the
 * rotor along, but when the field stops the rotor keeps its lag angle, so the
 * final position varied by +/-9 deg mechanical between runs.
 *
 * So instead of trying to "catch" the rotor, MEASURE the offset:
 *   - sweep the field forward N revs and back N revs at ALIGN_SWEEP_HZ
 *   - every frame accumulate err = encoder_elec - field_elec
 *   - the friction lag has opposite signs in the two directions and cancels,
 *     the encoder zero offset stays, the cogging ripple averages out
 *   - finally shift the stored zero by err_mean / pole_pairs
 * This does not need the torque to beat the stiction, only the rotor to
 * roughly follow the field, which it does. */
#define ALIGN_M_START           0.15f
#define ALIGN_M_MAX             0.45f   /* ~1.17A in phase A (ADC range 1.32A) */
#define ALIGN_RAMP_MS           200u    /* spin up the magnitude first */
#define ALIGN_SWEEP_HZ          2.0f    /* electrical Hz during the sweeps */
/* 7 electrical revs = exactly ONE mechanical rev (7 pole pairs): that makes
 * the encoder eccentricity (period = 1 mech rev) average to zero, which a
 * non-integer number of revs cannot do. */
#define ALIGN_SWEEP_MS          7000u   /* 2 Hz * 7 s = 14 elec revs = 2 mech revs */
#define ALIGN_SETTLE_MS         1000u   /* let the rotor catch up before averaging */
#define ALIGN_I_MAX_A           1.28f   /* abort above this (ADC range 1.32 A) */
#define ALIGN_TIMEOUT_MS        18000u  /* two sweeps plus margin */

/* S10.0c: open-loop pre-position ("align ol"). Holds a fixed vector at
 * theta = 0 while ramping the magnitude, so the rotor is pulled to a
 * known electrical position without any encoder feedback. 500 ms is
 * long enough for the rotor to stop swinging after the step. */
#define ALIGN_OL_MS             500u

/* ==========================================================================
 * S10.4: open-loop I/f startup
 * ==========================================================================
 * Start at a low electrical frequency and ramp slowly: open-loop alignment
 * has a 180 deg ambiguity (measured: two stable positions 27.4 mechanical
 * degrees apart), so a slow ramp lets the rotor slide into sync instead of
 * fighting its starting position. */
#define IF_ACC_HZ_S            30.0f    /* frequency ramp, Hz/s        */
#define IF_I_AMP_A             0.60f    /* forced vector amplitude, A  */
#define IF_F_MIN_HZ            2.0f     /* starting electrical Hz      */
/* d-axis pre-align before the drag: pulls the rotor to a known angle so
 * the 180 deg ambiguity becomes a deterministic start (measured: without
 * it the drag worked or failed purely on the starting position). */
#define IF_ALIGN_MS            600u

/* S10.8: soft start. The pre-align used to reach 0.6 A in 100 ms and then hop
 * the whole vector onto the q axis in one 50 us frame (0 -> 17 mN.m, about
 * 1800 rpm/s on this rotor): that is the first visible "kick" with the motor
 * standing loose on the bench. Now the amplitude takes IF_I_RAMP_MS and the
 * vector rotates d -> q over IF_XFADE_MS. */
#define IF_I_RAMP_MS           300u     /* 0 -> i_amp ramp time        */
#define IF_XFADE_MS            80u      /* d -> q crossfade time       */

/* ==========================================================================
 * S10.5: hand-over from the I/f frame to the observer
 * ==========================================================================
 * Conditions (all must hold continuously):
 *   |theta_obs - theta_if| < HO_ANGLE_DEG,  |eq| > OBS_E_MIN_V
 * then the angle is blended over HO_BLEND_MS so the switch is not a step. */
/* NOTE: the criterion must NOT be "observer agrees with the I/f frame".
 * Measured (#143): the observer tracked the encoder to 5.9 deg while
 * |theta_obs - theta_if| sat at 54..89 deg, because the drag has a load angle of
 * ~74 deg - the forced frame is not where the rotor is. Judge the observer
 * instead: enough back-EMF, enough speed, PLL locked. */
#define HO_MIN_RPM              250.0f  /* below this the back-EMF is too weak */
/* Lowest speed the closed sensorless loop may be asked for. Below this the
 * back-EMF is too small for the observer to stay locked; 300 rpm is the
 * documented floor (decision 2 of the S10 plan). */
#define SFOC_RPM_MIN            300.0f
/* Loss of lock: compare |eq| against what the reported speed implies.
 * An absolute threshold is wrong: during the startup ramp eq starts at 0
 * and grows with the speed, so it is legitimately small (measured 05:35:
 * a 0.25 V absolute limit aborted the drive 1.5 s after "sfoc go").
 * Locked: |eq| / (|omega_e| * flux) ~ 0.87 measured.
 * Lost  : ratio ~ 0.02 (rotor still, observer free-running). */
#define HO_LOST_EQ_RATIO        0.30f
#define HO_LOST_MIN_RPM         100.0f
#define HO_LOST_MS              100u
#define HO_HOLD_MS              20u
#define HO_HOLD_CNT             (HO_HOLD_MS * 20u)   /* control ticks at 20 kHz */
#define HO_BLEND_MS             100u
#define HO_BLEND_S              ((float)HO_BLEND_MS * 0.001f)
#define OBS_E_MIN_V             0.50f
#define IF_RPM_MAX             600.0f   /* ceiling for the "if" command */

/* S10.10: coasting with no angle source (#168).
 * After "sfoc off" (or any release) with the encoder disabled there is no
 * angle to commutate on.  The old code commanded the active zero vector,
 * which is a three-phase short: at 1500 rpm the back-EMF drove 1.316 A
 * through the windings (trip is 1.28 A) and braked the rotor.  Now the
 * current loop stays closed at zero current on the observer angle for as
 * long as the back-EMF is present.  |e| is the only honest "still turning"
 * signal: the observer's omega drifts to ~1000 rpm at standstill. */
#define COAST_E_MIN2            0.04f    /* (0.20 V)^2 back-EMF window     */
#define COAST_TICKS_MAX         100000u  /* 5 s at 20 kHz, then give up    */

/* ==========================================================================
 * S10.1: back-EMF observer (shadow mode)
 * ==========================================================================
 * R and L come from motor_cfg.h. The low pass filters e_alpha/e_beta;
 * the PLL bandwidth is the observer loop (independent of the encoder PLL).
 * w_max keeps the estimate sane if the loop is fed garbage. */
#define OBS_LPF_HZ              1500.0f  /* back-EMF low pass         */
#define OBS_PLL_WN_HZ           80.0f    /* observer PLL bandwidth     */
#define OBS_W_MAX_RADS          (2000.0f * 0.10472f * 7.0f)  /* 2000 rpm elec */

/* S10.13: the PLL clamps at OBS_W_MAX_RADS. Once it does, the angle
 * free-runs and the drive degenerates into a fixed-frequency V/f push
 * whose reported speed is fiction (measured reverse: obs w pinned at
 * exactly -2000 rpm while the rotor was elsewhere). Refuse to drive. */
#define OBS_RAIL_RATIO          0.95f   /* of OBS_W_MAX_RADS           */
#define HO_RAIL_MS              50u     /* dwell before releasing, ms  */
/* S10.2 calibration, measured on hardware (#132, 500..1800 rpm):
 *   comp 2 (retard) with delay 0 and a -3.6 deg trim gives
 *   2.33 / 1.88 / 1.18 / 0.09 deg of angle error at
 *   500 / 800 / 1200 / 1800 rpm.  Without compensation the error
 *   grows 7.3 -> 11.9 deg because the v/i sampling timing rotates
 *   the estimated back-EMF (it is NOT the low pass: removing the
 *   filter made it worse, 16.1 deg).
 *   The trim is a per motor/board calibration; re-measure after a swap. */
#define OBS_COMP_MODE           2u      /* 0 off, 1 advance, 2 retard */
#define OBS_DELAY_US            0.0f    /* extra delay for the comp   */
#define OBS_TRIM_DEG            (-3.6f) /* constant angle correction  */

/* VOFA+ waveform: number of channels per frame (FireWater: 5 bytes each).
 *   0 time, 1..3 duties, 4..6 phase currents, 7..9 encoder,
 *   10 id, 11 iq, 12 id_ref, 13 iq_ref, 14 vd, 15 vq, 16 vbus,
 *   17 window speed (rpm), 18 PLL speed (rpm) -- the last two are for the
 *   S8.1 observer comparison.
 *   19 back-EMF observer angle (deg), 20 observer-vs-encoder error (deg).
 * 21 channels * 5 B = 105 B per frame; at 500 Hz that is 52.5 kB/s of the
 * 100 kB/s the 1 Mbaud link provides. */
#define WAVE_CH 21u

/* FOC current loop (S6).
 * Design start: R = 2.3 ohm/phase, L = 0.5 mH, fc = 500 Hz ->
 *   Kp = 2*pi*fc*L = 1.57 V/A, Ki = 2*pi*fc*R = 7227 V/(A*s).
 * Measured with the "step" command (stall, 20 kHz, step 0.80 A):
 *   kp 0.80 / ki  4000 -> rise 1.05 ms, ripple 27.7 mA
 *   kp 1.57 / ki  7227 -> rise 0.50 ms, ripple 11.1 mA   (old default)
 *   kp 3.00 / ki 11000 -> rise 0.20 ms, ripple 10.0 mA   (adopted)
 *   kp 4.00 / ki 15000 -> rise 0.10 ms, ripple  6.1 mA   (voltage slew limit)
 * 3.0/11000 is about 955 Hz of current bandwidth and still well damped,
 * so it is the new default; "pi" can change it at runtime.
 * The bus voltage is the nominal 12 V until the Vbus ADC is wired (S7). */
#define FOC_VDC_NOMINAL         12.0f
#define FOC_M_MAX               0.95f    /* 0.5*Vdc*0.95 = 5.7 V phase amp */
#define FOC_V_LIM               5.7f
#define FOC_KP                  MOTOR_FOC_KP      /* -> motor_cfg.h */
#define FOC_KI                  MOTOR_FOC_KI      /* -> motor_cfg.h */
/* S7.2 feedforward. K_emf from KV: 330 rpm/V -> 3.03 mV per mechanical rpm,
 * and w_e = rpm*7*2*pi/60 = rpm*0.733 rad/s, so 4.13 mV per electrical rad/s.
 * (The first estimate of 3.98 used 220 Hz for a speed that was really ~175 Hz.) */
#define FOC_K_EMF               MOTOR_KE_V_RAD_S  /* V per elec rad/s */
#define FOC_L_H                 MOTOR_L_H         /* phase inductance, H */
#define FOC_FF_GAIN             1.0f      /* 0 = off, 1 = full          */
#define FOC_FF_LO_W             100.0f    /* FF fades in above 16 Hz elec */
#define FOC_FF_HI_W             300.0f    /* FF full above 48 Hz elec     */

/* S7.3 speed loop. Runs at SPD_FREQ_HZ; the encoder speed is a 10 ms
 * moving average, so the loop bandwidth must stay well below 20 Hz. */
#define SPD_FREQ_HZ             1000u
#define SPD_DIV                 (FOC_FREQ_HZ / SPD_FREQ_HZ)
#define SPD_TS_S                (1.0f / (float)SPD_FREQ_HZ)
#define SPD_KP                  MOTOR_SPD_KP      /* A per rpm */
#define SPD_KI                  MOTOR_SPD_KI      /* A per (rpm*s) */
#define SPD_FF_BASE             0.04f     /* friction feedforward, A     */
#define SPD_FF_SLOPE            0.0f      /* A per rpm (not measurable)  */
#define SPD_KICK                0.20f     /* breakaway current, A        */
/* Kick window: enabled for this long after a target change, so a target that
 * sits near a speed threshold cannot toggle it every loop (measured: spd 50
 * rattled because the gate was at exactly 50 rpm). */
#define SPD_KICK_MS             250u
/* Acceleration limit for the speed reference. Without it a step command
 * makes the PI output Kp*error (0.016*50 = 0.8 A for "spd 50"), which
 * accelerates at ~2900 rpm/s and overshoots before the 10 ms speed window
 * can react (measured: a visible lurch at every start). */
#define SPD_RAMP_RPM_S          600.0f
/* S7.5 current-reference slew for the speed loop, A per 1 ms step.
 * The speed PI can jump from 0 to the 1.20 A clamp in one step (kick plus
 * Kp*error).  The current loop answers a step with ~10% overshoot, so the
 * instantaneous peak reached 1.319 A against FOC_I_TRIP_A 1280 mA and the
 * drive tripped on overcurrent 3.6 s after "spd 500" (measured, 2026-11-07).
 * 0.03 A/ms walks to the clamp in 40 ms, which is faster than the 600 rpm/s
 * reference ramp needs and slow enough that the peak stays ~10% below the
 * trip.  The balance law has its own slew (BAL_SLEW_MNM_STEP) and does not
 * use this path. */
#define SPD_IQ_SLEW_A           0.03f
/* Acceleration feedforward: iq needs J/Kt amperes per rad/s^2 to follow the
 * reference ramp. J/Kt = 9e-5 / 0.0289 = 3.1e-3. Without it the integrator
 * had to build ~0.2 A during the ramp and the 100 rpm step overshot ~96%. */
#define SPD_FF_ACC              MOTOR_FF_ACC_A_RADS2  /* A per rad/s^2 */
/* Gain scheduling. At high speed the 10 ms speed window leaves a ripple at
 * the electrical frequency (58 Hz at 500 rpm) and a high Kp turns it into
 * a current ripple (measured 0.69 A with kp 0.016 vs 0.07 A with 0.004).
 * Scale the gains down linearly between these speeds. */
#define SPD_GAIN_LO_RPM         150.0f   /* full gains below this      */
#define SPD_GAIN_HI_RPM         400.0f   /* scale settles here         */
#define SPD_GAIN_HI_SCALE       1.00f    /* 20% of kp/ki above HI      */
/* Feedback filter, also speed scheduled. The 10 ms speed window yields a
 * staircase (measured: 7-10 rpm steps at 300 rpm) and the loop chases it,
 * which turns cogging into current ripple. 1.0 = no filtering. At 1 kHz a
 * coefficient of 0.05 is a ~20 ms time constant (~8 Hz). */
/* The filter band is separate from the gain band: cogging ripple is already
 * visible from ~200 rpm, well before the gains need to come down. */
#define SPD_LPF_LO_RPM          60.0f
#define SPD_LPF_HI_RPM          180.0f
#define SPD_LPF_LO_K            0.30f    /* no filter below LO         */
#define SPD_LPF_HI_K            0.10f    /* ~8 Hz above HI             */
#define RIP_WINDOW_MS           500u    /* ripple measurement window */
#define SPD_RPM_MAX             MOTOR_RPM_MAX_F   /* target limit */
/* sstep: settle first, then step the speed reference and record. */
#define SSTEP_SETTLE_MS         300u
#define SSTEP_SETTLE_MAX_MS     10000u   /* give up waiting after this */
/* The reference ramp is 600 rpm/s, so an 800 rpm step needs 1.33 s before
 * the steady state is meaningful; 1 s reported "err 28%" for it. */
#define SSTEP_RECORD_MS         2000u


#define FOC_I_MAX_A             MOTOR_I_MAX_A     /* reference vector limit */
/* Runtime tuning limits for the "pi" command. */
#define FOC_KP_MIN              0.05f
#define FOC_KP_MAX              20.0f
#define FOC_KI_MIN              0.0f
#define FOC_KI_MAX              50000.0f

/* "step": settle first, then step iq and record for STEP_MS. */
#define STEP_SETTLE_MS          100u
#define STEP_RECORD_MS          20u

#define FOC_I_TRIP_A            MOTOR_I_TRIP_A    /* abort above this */
#define FOC_TRIP_FRAMES         5u

/* Read the encoder every N control frames (1 = 20kHz).
 *
 * 2 = 10 kHz since docs/00 #059: the SSI ribbon picks up the phase current, so
 * every read is a chance to slip a byte. Halving the rate halves the number of
 * slips per second and leaves the bus idle in between; the electrical angle is
 * then up to 100 us old, which is 2.1 deg at 500 rpm with 7 pole pairs. */
#define ENC_DIV                 2u

/* Poll period while the bridge is off (ms). The ADC only runs in RUN, so a
 * stopped motor needs its own timer: 1ms = 1kHz, plenty for "enc" and
 * "enc zero". */
#define ENC_POLL_MS             1u

/* Speed is measured over this window: the angle delta accumulated over
 * ENC_SPD_WINDOW_MS divided by the elapsed time. A time window instead of
 * a per-sample delta keeps the 14-bit quantisation from dominating at low
 * speed (0.022 deg/LSB). */
#define ENC_SPD_WINDOW_MS       10u
/* S8.1: PLL speed observer. wn is the loop natural frequency in Hz;
 * 100 Hz gives about 1.6 ms of delay versus 15 ms for the window, so the
 * speed loop can run 3-5x faster. 0 selects the window estimator. */
#define PLL_WN_HZ               14.0f
#define PLL_ZETA                0.707f
/* Speed source selector. NOTE: an earlier version defined
 * ENC_SPD_SRC_PLL as 0, which silently selected the window estimator. */
#define ENC_SPD_SRC_WIN         0u
#define ENC_SPD_SRC_PLL         1u
#define ENC_SPD_DEFAULT         ENC_SPD_SRC_PLL

/* S10: a type-2 PLL free-runs once the true speed is outside its
 * capture range, so the estimate is cleared whenever the bridge is
 * disabled (app_stop / app_fault_raise -> mid_enc_pll_clear).
 * An in-run motion detector was tried in #135 and removed: it
 * collided with the PLL gap path and fired during acceleration.
 * Known limitation: a mechanical stall while running (no state
 * change) can still leave the reading high; current limit and the
 * over-current trip are the real protections there. */



/* ==========================================================================
 * 7b. Attitude layer (S2)
 *
 * The cube and the IMU are bolted together, so the sensor's orientation and
 * angular rate are the body's. Three things still have to be pinned down:
 * the frame, the zero, and when the value may be trusted.
 *
 * Body frame (frozen in docs/42, revised 2026-11-04): the cube balances about
 * the IMU X axis, so theta = roll, rotation about +X. Measured on the bench
 * (docs/19 section 6.1): looking along +X, a clockwise rotation makes roll
 * grow. The wheel spin axis has to end up parallel to +X as well, otherwise
 * the torque and the measured angle do not share an axis (docs/18 T4/T6).
 *
 * mid_fdilink.h stores the 0x41 payload in radians and rad/s; mid_att converts
 * to degrees once so the control law, the console and the docs share one unit.
 * ========================================================================== */

/* The cube balances on an edge, which tilts the bolted-on module, so the
 * control axis is the one that carries the edge: measured roll, not pitch
 * (docs/19 section 6.1). */
#define ATT_AXIS                ATT_AXIS_ROLL    /* see ATT_AXIS_PITCH     */

/* +1 keeps the module sign, -1 flips it. The cube may fall either way about
 * X, so the law itself is symmetric; this only fixes "which way is +".
 * Still unverified: the hand-lean test cannot tell a real tilt from a hand
 * wobble. It gets settled in S3 with the wheel reaction test (spin the wheel,
 * watch which side of theta the body rotates towards) - docs/18 T19. */
#define ATT_SIGN                1.0f

/* Raw reading (deg) that means "balanced". COARSE ONLY: 46.0 comes from the
 * hand-held shelf pose (46.35 deg after removing the 2.45 deg mount bias), and
 * a hand is not the equilibrium - the true value is a property of the centre
 * of mass and has to be found on the machine. Two independent trims follow:
 *   1. the operator calls "att zero" while the cube sits near balance;
 *   2. the balance law needs a slow bias trim, because a 1 deg error already
 *      eats a third of the +-2.9 deg capture window (docs/18 T20).
 * 2026-11-05 calibration attempt: the 2492 ms closed loop run held phi inside
 * 1.6 deg with the command never saturated, yet the wheel climbed 0 -> 1694 rpm
 * in 2.49 s. J*alpha = 5.12e-5 * 71.2 = 3.64 mN.m, i.e. the law was paying the
 * friction constant plus 3.64 mN.m of extra torque. mgh*sin(delta) = 3.64 mN.m
 * gives delta = 0.64 deg, so the real balance pose is about 45.36 deg raw and
 * the old constant made the loop hold 0.64 deg of forward lean forever.
 * 45.4 was therefore the measured guess, and 2026-11-06 replaced it with a
 * measured value. With offset 45.4 the bench iteration
 *     trim_new = trim - omega_rpm / (kp/kw in rpm per deg)      (573 rpm per deg)
 * drove the wheel from -427 to +152 to +73 to -46 rpm and settled around
 * trim = 0.87 deg, i.e. theta = 0 at raw 45.4 + 0.87 = 46.27 deg. That agrees
 * with the independent hand-held 46.35 deg and with the equilibrium fit 46.64 deg
 * within their own scatter, so the constant is now that measured 46.27.
 * Consequence: with the wheel at rest the balance law needs no trim at all, and
 * a reboot no longer re-introduces a 0.9 deg bias that winds the wheel up.
 * Nothing here is written to flash on purpose. */
#define ATT_OFFSET_DEG          46.27f

/* First order low pass, 0 = off. Off while measuring: a filter hides exactly
 * the noise and the lag that S2 is supposed to quantify. */
#define ATT_LP_HZ               0.0f

/* A frame older than this is not usable by a balance law. 100 ms is 20 missed
 * 200 Hz frames, i.e. far past the point where a balance loop has to trip. */
#define ATT_TIMEOUT_MS          100u

/* Usable range. Outside it the value is flagged invalid and clamped; the
 * balance stage adds its own trip at 20-25 deg. */
#define ATT_RANGE_DEG           60.0f

/* ==========================================================================
 * 7c. Torque interface (S3)
 *
 * The balance stage thinks in mN.m. The only conversion into the hardware is
 * the q-axis current the current loop already regulates, so iq = t / Kt with
 * Kt taken from motor_cfg.h (MOTOR_KT_NM_A, 28.91 mN.m/A, derived from KE and
 * the pole pair count - a motor swap updates it automatically).
 * ========================================================================== */

/* Command ceiling. 34 mN.m is 1.176 A, just inside FOC_I_MAX_A (1.20 A), so a
 * legal command never sits in the ADC's saturated range. app_torque_set()
 * re-checks the current limit at run time in case Kt changes. */
#define TORQUE_MAX_MNM          34.0f

/* Below this the request is treated as zero: 0.5 mN.m is noise and the balance
 * loop needs 'release' to be unambiguous. */
#define TORQUE_MIN_MNM          1.0f

/* ==========================================================================
 * 7d. Balance law (S4, shadow mode)
 *
 * Designed in sim/s4_design.py and docs/44 4.3. The law works in SI units:
 *
 *   tau[N.m] = kp*phi[rad] + kd*phi_dot[rad/s] + kw*omega[rad/s]
 *
 * The numbers below are N.m per rad (NOT per degree). Everything is converted
 * to radians inside app_bal_update() so this table can be pasted straight out
 * of the design script.
 * ========================================================================== */

/* 2026-11-05: the wheel-speed weight in the LQR cost was moved from an
 * arbitrary 1/50^2 (units ignored, i.e. 50 rad/s ~ 477 rpm) to 50 rpm, which is
 * the quantity the 1700 rpm trip actually cares about. The whole gain set moved
 * with it: kp 0.728 -> 1.314, kd 0.0970 -> 0.1755, kw 4.0e-5 -> 3.82e-4.
 * Reason: with the old set omega_ss = mgh*sin(zero error)/kw, so 1.5 mN.m of
 * constant torque (a USB cable) walked the wheel into the trip in a few
 * seconds. The new set settles the same disturbance near 300 rpm and keeps the
 * capture window (4.5 deg recovers, 5 deg does not, sim/s4_design.py). */

/* Attitude gain. 1.314 N.m/rad = 22.9 mN.m/deg, i.e. 4x the gravity stiffness
 * (mgh = 5.74 mN.m/deg), so the body is held stiffly and the zero error is paid
 * with one fifth of the wheel speed it used to cost. */
#define BAL_KP_NM_RAD           1.313830f

/* Tilt-rate gain, both poles of the fast pair land near -13 rad/s. */
#define BAL_KD_NM_RPS           0.175517f

/* Wheel-speed gain. Positive: a spinning wheel lets the body fall back a
 * little and then brakes, which is how the wheel speed is pulled down.
 * 3.82e-4 N.m/(rad/s) = 0.040 mN.m/rpm. */
#define BAL_KW_NM_RPS           3.819719e-04f

/* Physical sign of the reaction torque. NOT verified yet (docs/18 T19): it is
 * decided in the first closed-loop run on the bench, not by hand-held tests.
 * Keep it an explicit switch so that test flips one number, not a formula. */
#define BAL_SIGN                1.0f

/* Command ceiling. Below TORQUE_MAX_MNM, and below the level where the
 * switching ripple makes the instantaneous phase peak cross FOC_I_TRIP_A.
 * The trip compares the phase peak against 1.28 A with a 5-frame debounce, so
 * the budget is iq + ripple/2. With L = 0.5 mH and 20 kHz the worst-case
 * ripple is 0.30 A pp at 12 V, 0.35 A at 14 V (4S measured) and 0.42 A at the
 * 16.8 V full charge, so a 33 mN.m ceiling (iq 1.14 A) peaked at 1.32 A and
 * tripped the board while the law was saturated (measured 2026-11-04,
 * sim/s6_ripple.py). 28 mN.m keeps the peak at 1.18 A even at 16.8 V. */
#define BAL_MAX_MNM             28.0f

/* Friction feed-forward (docs/45 4.8, docs/18 T20). The motor has to supply
 * the Coulomb torque by itself; otherwise the LQR can only buy it with wheel
 * speed and the wheel parks at (tau_f - kp*theta0)/kw -- 490 rpm with a perfect
 * zero and 1200 rpm for a 0.2 deg zero error (measured 2.05 mN.m in S3).
 * Applied with the sign of the wheel, inside a hysteresis band so noise around
 * standstill cannot flip it every frame. 0 disables it. */
#define BAL_FF_MNM              2.05f
#define BAL_FF_DEADBAND_RPM     30.0f

/* Runtime zero trim, operator adjustable with "bal trim <deg>" (RAM only, so a
 * reboot restores the value below). Bounded because a large trim is just a
 * worse zero: the usable range around the balance point is a couple of degrees. */
#define BAL_TRIM_DEG            0.0f
#define BAL_TRIM_MAX_DEG        3.0f    /* must cover the zero error: the bench
                                         * sat at -2.3 deg (docs/45 4.17) */

/* Slow zero adaptation, walked by the WHEEL (docs/45 4.17). Gravity pins the
 * body at the true balance point, so in steady state
 *     omega_ss = (kp/kw) * (trim - phi_ss)
 * i.e. a zero error is paid for entirely in wheel speed: the bench held
 * phi = -2.32 deg and wheel = 1340 rpm for the whole 60 s window (predicted
 * 1328). The wheel speed is therefore the error signal:
 *     trim -= trim_w_k * omega_rpm * dt     [deg per rpm per second]
 * which stops by itself exactly when the wheel stops. Gated on |phi| and
 * clamped; the walk is only allowed while the loop is engaged and balancing.
 * sim/s6_auto_trim2.py (2.3 deg zero error, 70 s): k = 1e-4 -> 17 rpm, k = 3e-4
 * -> 0 rpm in 17 s with the body moving less than 0.05 deg, k = 1e-3 -> 5 s but
 * 3.11 deg of lean. 3e-4 chosen. 0 freezes the trim and leaves "bal trim" in
 * charge. */
#define BAL_TRIM_W_K            3e-4f    /* on again 2026-11-10: the encoder that made the
                                          * 2026-11-06 runaway possible is fixed (rotating
                                          * crc 0.046%, badrun 0), and app_bal_update() now
                                          * freezes the walk when the encoder reading is
                                          * stale or invalid. "bal walk <k>" retunes it live. */
#define BAL_TRIM_GATE_DEG       4.0f    /* outside this the body is still being
                                         * captured; do not lean it further */

/* The wheel walk is frozen when the encoder reading has not advanced for this
 * long. At 20 kHz a healthy link advances every 50 us, so 100 ms is 2000
 * frames: only a dead or wedged reader trips it. */
#define BAL_WALK_STALE_MS       100u

/* Attitude frame period: the adaptation is advanced once per new 0x41 frame. */
#define BAL_FRAME_DT_S          0.005f

/* ==========================================================================
 * 7e. Closed-loop orchestration (S6)
 *
 * The law itself is a pure function; everything that can hurt the machine
 * lives here: when it may start, when it must stop, and how fast the command
 * may move. Thresholds are compared against the values mid_att and mid_enc
 * already publish, so they read like the screens: degrees, deg/s, rpm.
 * ========================================================================== */

/* First-flight derating. 1.0 = the L1 gains above. Measured capture window
 * (sim/s6_scale_scan.py, nominal inertia, release from rest):
 *   0.4x -> 0.00 deg   0.5x -> 1.50   0.6x -> 3.75   0.7x -> 4.50 (4.25 with
 *   10 ms of extra latency). 0.4x was the original choice and it cannot catch
 *   anything at all, so the derating is 0.7x: still softer than nominal on the
 *   first run, but provably able to hold the ARM window below.
 * Raise to 1.0 only after the direction has been proven (docs/18 T19). */
#define BAL_SCALE               0.7f

/* ARM window: all four must hold before the loop is allowed to push.
 * 2.0 deg leaves 1.75 deg of margin against the 3.75 deg capture of 0.7x, and
 * the operator has to place the cube inside it by hand. */
#define BAL_ARM_DEG             2.0f
#define BAL_ARM_DPS             30.0f   /* do not take over a fast fall     */
#define BAL_ARM_RPM             300.0f  /* wheel should be near standstill  */

/* TRIP thresholds: any one of them ends the run within one attitude frame. */
#define BAL_TRIP_DEG            12.0f   /* 2.73 deg already saturates       */
#define BAL_TRIP_DPS            200.0f
#define BAL_TRIP_RPM            1700.0f /* L-12: at 1700 rpm only 7 mN.m of
                                         * torque is left, above that the
                                         * wheel cannot help any more. 1200 was
                                         * too tight: a 0.5 deg zero error needs
                                         * ~990 rpm of steady wheel speed to
                                         * hold, so 1200 tripped before the
                                         * loop could show anything. */
#define BAL_TRIP_STALE_MS       50u     /* 10 frames at 200 Hz              */

/* Slew: commanded torque may move at most this much per 5 ms frame. The step
 * size is also a current step: 10 mN.m/frame is 0.35 A per frame, and the
 * 20 kHz PI overshoots on such a jump by roughly the step itself - measured
 * 1314 mA against FOC_I_TRIP_A 1280 mA while the law ramped from 0 to the
 * ceiling, so the board tripped on overcurrent in the middle of a normal
 * capture. 3 mN.m/frame (0.104 A) keeps the excursion inside the ripple
 * budget; full scale now takes 50 ms, still fast against the 1.2 Hz loop. */
#define BAL_SLEW_MNM_STEP       3.0f

/* Automatic stop. 0 = no time window: the loop runs until a TRIP or a manual
 * stop. The window existed because a hand-held start can look like a good run
 * (measured twice: phi stayed inside 0.02 deg, i.e. the cube never left the
 * hand), but the engage-on-release detector plus the held detector now cover
 * that case, and the 60 s cap was the only thing ending otherwise healthy runs
 * (measured 2026-11-06: 60 001 ms, trip timeout, phi peak 0.86 deg, wheel
 * 1389 rpm). Set a number only for a bounded experiment. */
#define BAL_MAX_MS              0u

/* One-shot zero learn: BAL_LEARN_MS after ENGAGE the loop has settled onto the
 * real balance point, so the observed theta IS the zero error. Take it once and
 * hand the rest to the wheel-driven walk above (docs/45 4.17). */
#define BAL_LEARN_MS            1000u

/* Held-cube detector. A free cube cannot refuse a torque: if the law keeps
 * asking for a large torque while the body neither tilts nor rotates, the cube
 * is resting on something (hand, USB cable, a stand) and the only thing that
 * grows is the wheel. Measured 2026-11-05: law pinned at 28 mN.m for ~0.35 s
 * while the body rate stayed at 0.13 deg/s -> the wheel reached 1690 rpm and the
 * run died on the wheel interlock, which looks like a tuning bug but is really
 * an operator/harness problem. Trip on it in 150 ms and name it "held". */
#define BAL_HELD_MNM            12.0f   /* law above this counts as "asking"  */
#define BAL_HELD_DEG            1.0f    /* and only when clearly off balance */
#define BAL_HELD_DPS            5.0f    /* body below this counts as "static" */

/* Button start (2026-11-06).
 * Single press + release: the pose being held becomes zero, the drive is
 * started, and the loop engages BAL_KEY_START_DELAY_MS after the button is
 * released - the operator uses those 3 s to settle the cube and get the hand
 * away. Holding KEY for BAL_KEY_HOLD_MS still works as an immediate engage for
 * bench work. Both paths go through app_balance_start_now(), which starts the
 * drive itself (mode FOC + start) so the button alone is enough.
 * Engage-on-release fired on hand tremor and slammed the wheel to the 1700 rpm
 * trip line in 0.47 s (docs/00 #032); the first BAL_ENGAGE_RAMP_MS of the run
 * ramp the torque ceiling so an imperfect release position cannot become a
 * full-scale command. */
#define BAL_KEY_HOLD_MS         3000u   /* still: hold this long = engage now */
#define BAL_KEY_START_DELAY_MS  300u    /* press+release -> engage after this */
#define BAL_ENGAGE_RAMP_MS      800u    /* soft start after engage            */
#define BAL_HELD_MS             150u    /* 30 frames at 200 Hz               */
#define BAL_HELD_GRACE_MS       800u    /* no held trip this soon after engage */
#define BAL_HELD_PHI_DEG        0.30f   /* body must stay this still to count  */

/* Engage-on-release. "bal run on" only arms: no torque flows until the body
 * actually starts to move, i.e. until the operator has let go. This removes the
 * arm-versus-release race - holding the cube used to spin the wheel to ~400 rpm
 * in 265 ms (2026-11-05, trip held) because the law fought the hand. A rolling
 * wheel also passes the ANGLE test, so the angle test is only a fallback for a
 * slow release; the rate test is the primary trigger. Thresholds: after release
 * a 0.5 deg offset grows at Omega=10 rad/s, so 2 deg/s is reached in ~45 ms. */
/* Release detection. A hand wobbles at a couple of deg/s, so a low rate
 * threshold fires while the cube is still held (2026-11-05: 2 deg/s fired at
 * 250 ms with the cube in hand, trip held at 442 ms, wheel 548 rpm). Use a rate
 * a hand cannot fake and require it to persist: 8 deg/s for 3 frames (15 ms).
 * A released cube from 0.5 deg reaches 8 deg/s in ~0.13 s at ~1.7 deg of tilt,
 * which still leaves room inside the 4.5 deg capture window. No angle fallback:
 * a hand can hold any angle, so an angle test only reintroduces the short
 * circuit seen in the first version (0.6 deg < ARM 2.5 deg). */
#define BAL_ENGAGE_DPS          8.0f    /* body rate that means "released"   */
#define BAL_ENGAGE_FRAMES       3u      /* consecutive frames above it       */

/* ==========================================================================
 * 8. Motor parameters -> moved to motor_cfg.h (S10.0)
 *
 * Everything that changes when the motor changes lives in that one file:
 * pole pairs, R, L, KE, J, current limits and top speed. This header only
 * aliases them (see ENC_POLE_PAIRS / FOC_* / SPD_* above), so a motor swap
 * does not touch board_cfg.h at all.
 * ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CFG_H */
