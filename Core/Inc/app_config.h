#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/*
 * MEASURED ON THE BOARD (2026-09-29), not assumed:
 *
 *   CLK_K_X10000 = 15334   ->  k = 1.5334
 *
 * k is 2.2 % away from 1.5 (12 MHz) but only 0.17 % away from 1.536, so OSC_IN
 * carries 12.288 MHz -- the new board's VCTCXO (sys_clk_12m288), fed through
 * the LMK1C1104 fanout, while this file still said 8 MHz (the old board's VCXO
 * frequency).  That made the real SYSCLK 12.288 x 9 = 110.592 MHz -- above the
 * 72 MHz limit -- and every UART 1.536x too fast.
 *
 * 12.288 MHz cannot divide into exactly 72 MHz (72 / 12.288 = 5.859), so the
 * PLL multiplier has to be picked from what is legal:
 *
 *   PREDIV1 = 1, PLLMUL = 5  -> 61.44 MHz   SYSCLK within spec, BRR error 0.13 %
 *   PREDIV1 = 1, PLLMUL = 6  -> 73.728 MHz  exact baud but 2.4 % over the limit
 *   PREDIV1 = 1, PLLMUL = 9  -> 110.592 MHz not allowed
 *
 * 5 is chosen: comfortably inside the limit (margin for temperature and for
 * clone parts), and 61.44 MHz still divides 38400 exactly (800) with only
 * -0.13 % on 115200.  The CPU is 15 % slower than before, which this design has
 * enormous headroom for.
 */
#define APP_HSE_VALUE_HZ                 12288000UL
#define APP_SYSCLK_HZ                    61440000UL

/*
 * Debug UART1 (PA9/PA10) baud rate.
 *
 * The USART divider is derived from PCLK2, which HAL computes from the
 * compile-time APP_HSE_VALUE_HZ.  If the clock actually present on OSC_IN is
 * not that value, the real baud is  k x APP_UART1_BAUD  where
 *     k = f_OSC_IN_real / APP_HSE_VALUE_HZ
 * and a terminal set to 115200 receives nothing but noise -- the ASCII log is
 * never the problem, because the USART shifts bits out in hardware.
 *
 * MEASURED ON THE BOARD (2026-09-29), final answer:
 *
 *   baud sweep  : a nominal 75000 banner was legible at a 115200 terminal
 *   CLK_K_X10000: 15334  ->  k = 1.5334
 *
 * k is 2.2 % away from 1.5 (12 MHz) but only 0.17 % away from 1.536, so OSC_IN
 * carries 12.288 MHz.  The clock configuration has now been corrected
 * (APP_HSE_VALUE_HZ = 12288000, PLLMUL = 5 -> 61.44 MHz), so k is 1.0 and this
 * is back to the plain 115200 the terminal expects.
 *
 * The 75900 midpoint value used while the clock was still wrong is gone.  Note
 * the lesson it taught: the UART link itself CANNOT resolve a 2.4 % rate error
 * (a receiver re-synchronises on every start bit and tolerates ~5 % per frame),
 * so an independent time reference is required -- here DWT_CYCCNT against the
 * PC's own transmit cadence, see APP_CLK_MEASURE_ENABLE below.
 */
#define APP_UART1_BAUD                   115200U
/*
 * GPS UART2 (PA2/PA3) baud rate.
 *
 * README documents "保持 38400 baud" for this port, and 38400 is what the
 * original design used.  But a u-blox GNSS module's factory default is 9600,
 * while most breakout boards (SparkFun, u-blox EVK) ship pre-set to 38400 --
 * so the actual rate depends on which module is fitted.
 *
 * A mismatch has a very specific signature: RX_BYTES keeps growing, but
 * DOLLAR and RMC_LINES stay at zero, because every frame is mis-framed.  If
 * that is what GPS_STAT shows, run the console command GPSBAUD? -- it walks
 * the candidate rates and reports which one produces real '$' frame starts.
 * Then set this macro to the winner.
 */
#define APP_GPS_UART_BAUD                38400U
/*
 * Boot-time baud sweep.
 *
 * The USART divisor comes from PCLK2, which the HAL derives from the
 * compile-time APP_HSE_VALUE_HZ.  If the clock actually present on OSC_IN is
 * not that value, every byte leaves the pin at a rate scaled by
 *     k = f_HSE_real / APP_HSE_VALUE_HZ
 * and a terminal set to 115200 can never read it -- whatever the firmware
 * prints.  That is a chicken-and-egg deadlock: reading the log needs the right
 * rate, finding the rate needs a readable log.
 *
 * A terminal sitting at 115200 reads exactly one candidate legibly: the one
 * sent with nominal 115200 / k.  So sweeping a spread of nominals once at boot
 * identifies k without a scope, and the legible line names the rate that
 * worked.  Set to 0U once the clock question is settled.
 */
#define APP_UART1_BAUD_SWEEP_ENABLE      1U
/*
 * Repeat the sweep from the main loop instead of only at boot.
 *
 * A single boot-time sweep is easy to miss: the terminal buffer may already
 * hold other data, or the board may not have been reset.  Repeating it makes
 * the diagnostic impossible to miss, and it enables a procedure that works for
 * ANY mismatch ratio:
 *
 *   cycle the terminal through 9600 / 19200 / 38400 / 57600 / 115200 / 230400,
 *   pausing ~10 s on each, until one of the swept banners appears as clean
 *   text.  That banner names the nominal rate it was sent at, and
 *
 *       k = terminal_rate / nominal        (then set APP_UART1_BAUD = nominal)
 *
 * While this is 1U the log output is suppressed, because a blocking sweep
 * transmit must not interleave with the interrupt-driven log ring.
 * Set to 0U once the rate question is settled.
 */
#define APP_UART1_BAUD_SWEEP_REPEAT_ENABLE  0U
#define APP_UART1_BAUD_SWEEP_PERIOD_MS      5000U
/*
 * How many times each candidate banner is repeated back to back.
 *
 * NOTE: this does NOT help discriminate two candidates that are ~2.4 % apart.
 * A UART receiver re-synchronises on every start bit, so the timing error only
 * accumulates across one 10 bit frame: 2.4 % x 10 = 0.24 bit, far inside the
 * half-bit sampling margin.  Every candidate within about +-5 % therefore reads
 * perfectly regardless of length.  Kept only to make the swept blocks easy to
 * spot; a real time reference (GPS PPS, see PPS_MS in the banner) is the only
 * thing that can resolve the ratio.
 */
#define APP_UART1_BAUD_SWEEP_REPEATS        4U

#define DAC8571_ADDR_7BIT                0x4CU
#define DAC8571_BROADCAST_WRITE_TEST_ADDR_7BIT 0x48U
#define DAC8571_CONTROL_WRITE_UPDATE     0x10U
#define DAC8571_I2C_TIMEOUT_MS           20U

#define APP_TEMP_INVALID_C               (-999.000f)
#define GPSDO10_USE_VCT_ADC              0U
#define DAC8571_VREF_VOLTS               3.3600f
#define DAC_VREF_MV                      3360U
#define DAC_VREF_UV                      3360000UL
/*
 * CALIBRATED ON THIS BOARD (2026-10-08).
 *
 * Open-loop 3-point sweep against a 53220A locked to an external GPSDO
 * reference (see 编译恢复记录.md, "表定点"):
 *
 *     code 31207 (1.599993 V) -> 12 287 985.125 Hz   (-14.875 Hz)
 *     code 33158 (1.700021 V) -> 12 287 999.162 Hz   ( -0.838 Hz)
 *     code 34328 (1.760007 V) -> 12 288 007.701 Hz   ( +7.701 Hz)
 *
 * Least squares over the three points: residual < 0.05 Hz, non-linearity
 * 1.44 % over the whole span, so the EFC is linear enough to treat as a
 * straight line.  Solving for 12 288 000.000 Hz gives code 33267.
 *
 * Before this the park point was 31253 (inherited from the 8 MHz baseline
 * board), which sat ~700 SW_E50 counts low -- i.e. the board powered up
 * 1.2 ppm off nominal.  Use the EFC? console command to re-check the budget.
 */
#define VCTRL_NEAR_LOCK_UV               1705610L
#define DAC_CODE_INIT                    33267U

/*
 * VCTCXO EFC (Vc) window the discipline loop is allowed to use.
 *
 * These are HARDWARE parameters, not control parameters, so they live here
 * rather than in app_gpsdo.c: they must match the VCTCXO's specified EFC input
 * range, and widening them past it makes the pull non-linear (and, on some
 * parts, out of spec).
 *
 * 1 DAC8571 LSB = DAC_VREF_UV / 65536 = 51.27 uV.
 *
 * RECENTERED 2026-10-08 around the calibrated park point, giving the loop
 * SYMMETRIC authority.  The old window (29000..34000) was left over from the
 * 8 MHz baseline board and its park point sat low in it, so upward travel was
 * only 265 SW_E50 counts (~5.3 Hz) while downward was 1542 -- an asymmetry
 * that silently caps how much warm-up / ageing drift the loop can ever
 * correct.  Since the pull range of this part is at least +/-5 ppm
 * (+/-61 Hz) and its Vc input spans roughly 0.65..2.65 V on a 3.3 V supply,
 * the window below (1.5518..1.8594 V) is comfortably inside the device.
 *
 *     code 30267 -> 1.5518 V      code 33267 (DAC_CODE_INIT) -> 1.7056 V
 *     code 36267 -> 1.8594 V
 *
 * At the measured 141 Hz/V that is +/-21.7 Hz = +/-1084 SW_E50 counts either
 * way.  Use the EFC? console command to see the budget recomputed at runtime.
 */
#define APP_DAC_EFC_CODE_MIN             30267U
#define APP_DAC_EFC_CODE_MAX             36267U

/*
 * Measured VCTCXO EFC slope in Hz per volt -- THE number the pull budget and
 * the loop gain both depend on.
 *
 * MEASURED 2026-10-08 on this board: 141 Hz/V, from the same three-point
 * sweep quoted at DAC_CODE_INIT.  The baseline project assumed 100 Hz/V; the
 * real part is 41 % steeper, which means every LSB is worth more frequency
 * than assumed (1 LSB = 7.23 mHz, not 5.13 mHz) so the same step count moves
 * further -- and the required EFC travel is smaller than the old estimate
 * suggested.
 *
 * Re-measure if the VCTCXO or its Vc conditioning network ever changes; the
 * EFC? budget and the DISC50 step law both scale with this directly.
 */
#define APP_VCTCXO_SLOPE_HZ_PER_V        141L

#define APP_DAC_WRITE_INIT_ON_BOOT       1U

#define GPSDO_DRY_RUN                    0U
#define DAC_AUTO_WRITE                   1U
#define DAC_DISCIPLINE_APPLY_ENABLE      1U
#define GPSDO_DISC10_CONTROL_ENABLE      0U
#define DAC_DISCIPLINE_USE_10S_APPLY     GPSDO_DISC10_CONTROL_ENABLE
#define GPSDO_DISC50_CONTROL_ENABLE      1U
#define DAC_DISCIPLINE_USE_50S_APPLY     GPSDO_DISC50_CONTROL_ENABLE
#define APP_GPSDO_DRY_RUN                GPSDO_DRY_RUN

/*
 * INTERLOCK: refuse to discipline unless the receiver reports a valid GPS time
 * solution.
 *
 * Every window boundary the control loop uses comes from the module's
 * TIMEPULSE, so the loop is only as good as that pulse.  A u-blox receiver
 * emits TIMEPULSE from its own RTC/TCXO BEFORE it has solved time, and that
 * free-running pulse carries the module crystal's error, not GPS accuracy.
 *
 * Measured on this board (2026-10-08): with the module still showing FIX=V and
 * only 17 dBHz of signal, COUNT10 read 122 879 858 -- i.e. -142 counts over
 * 10 s, -1.16 ppm.  The oscillator itself was within +0.01 ppm at the time
 * (checked on a counter locked to a GPSDO reference).  So the -142 counts was
 * almost entirely the PPS SOURCE, and acting on it would have dragged a
 * correctly trimmed VCTCXO ~14 Hz off nominal.
 *
 * A free-running PPS is therefore not a reference, and the loop must wait for a
 * real fix instead of "locking" onto the module's crystal.  The residual risk
 * that remains is losing the fix AFTER locking: the pulse then free-runs again,
 * which is what holdover is for (GPSDO_FIX_LOSS_HOLD_MS).
 */
#define GPSDO_REQUIRE_GPS_FIX_FOR_DISC   1U
/*
 * While already disciplining, a fix that is lost for longer than this means the
 * TIMEPULSE is free running again, so freeze the DAC (HOLDOVER) rather than
 * chase a wrong reference.  Long enough to ride out a brief satellite dropout.
 * Only used when GPSDO_REQUIRE_GPS_FIX_FOR_DISC is 1.
 */
#define GPSDO_FIX_LOSS_HOLD_MS           120000U

#define GPSDO_LOG_GPSDO2_ENABLE          1U
#define GPSDO_LOG_LEGACY_GPSDO10_ENABLE  0U
#define GPSDO_LOG_RTC_PHASE_ENABLE       0U
#define GPSDO_LOG_PHASE10_ENABLE         0U
#define GPSDO_LOG_EVENT_ENABLE           1U
#define GPSDO_LOG_DAC_ADJUST_ENABLE      1U
/*
 * The MAX V build has no phase meter: every app_cpld_read_phase* API returns
 * HAL_ERROR, so the GPSDO_PHASE row is a fixed string of zeros printed once
 * per window.  It buries the rows that actually carry data, so it is off by
 * default (the console command LOGRTC / LOGPHASE 1 turns it back on).
 */
#define GPSDO_LOG_GPSDO_PHASE_ENABLE     0U
#define GPSDO_LOG_GPSDO_AVG_ENABLE       1U
/*
 * GPSDO_RESULT: one fixed-format ASCII row per second carrying the current
 * actuator setting and the frequency error already converted to Hz / ppb, so
 * the acceptance figures can be read straight off the terminal.  See
 * app_gpsdo10_print_result() for the exact conversion.
 */
#define GPSDO_LOG_RESULT_ENABLE          1U
/*
 * GPSDO_MEAS50: one complete record per completed 50 s window -- raw 50 s count,
 * error in mHz and ppb, the running 1000 s average, and which DAC code the
 * controller moved (and why).  This is the acceptance trail behind the 1 Hz
 * GPSDO_RESULT row.  See app_gpsdo10_print_meas50().
 */
#define GPSDO_LOG_MEAS50_ENABLE          1U
/* Pace of the simulated record while the report-only simulator (SIM) is on;
 * there is no real window to trigger on in that case. */
#define GPSDO_MEAS50_PERIOD_SEC          50U
#define GPSDO_SW_E50_WINDOWS             5U
#define GPSDO_SW_E100_WINDOWS            10U
#define GPSDO_SW_E200_WINDOWS            20U
#define GPSDO_SW_E1000_WINDOWS           100U

#define GPSDO_TARGET_COUNT10_NOMINAL     122880000L
#define GPSDO_TARGET_COUNT10_OFFSET      0L
#define GPSDO_TARGET_COUNT10             ((uint32_t)((int32_t)GPSDO_TARGET_COUNT10_NOMINAL + (int32_t)GPSDO_TARGET_COUNT10_OFFSET))
#define GPSDO_TARGET_COUNT50             614400000UL

#define GPSDO_FREQ_DISPLAY_ERR_SIGN      (+1)

#define DAC_INCREASE_MAKES_FREQ_HIGHER   1U
#define DAC_SLOPE_NUM                    24L
#define DAC_SLOPE_DEN                    100000L
#define GPSDO_DAC_SLOPE_DEFAULT_COUNT_PER_100MV 100L
#define GPSDO_DAC_SLOPE_FINE_COUNT_PER_100MV    80L
#define GPSDO_DAC_SLOPE_COARSE_COUNT_PER_100MV  100L
#define GPSDO_DAC_SLOPE_MIN_COUNT_PER_100MV     80L
#define GPSDO_DAC_SLOPE_MAX_COUNT_PER_100MV     300L

#define GPSDO_PHASE_FINE_ENABLE          0U
#define GPSDO_PHASE_FINE_DEADBAND_MHZ    10L
#define GPSDO_PHASE_FINE_CONFIRM_N       3U
#define GPSDO_PHASE_FINE_INTERVAL_MS     30000UL
#define GPSDO_PHASE_FINE_MAX_STEP_CODE   1U
#define GPSDO_PHASE_FINE_MAX_STEP_UV     80UL
#define GPSDO_PHASE_FINE_K_UV_PER_HZ     1000L
#define GPSDO_RTC_PHASE_STEP_ENABLE      0U /* auto-apply disabled; manual test via console still available */
#define GPSDO_RTC_PHASE_STEP_REG         0x16U
#define GPSDO_RTC_PHASE_DEADBAND_NS      120
#define GPSDO_RTC_PHASE_CONFIRM_N        3U
#define GPSDO_RTC_PHASE_MIN_RUN_S        300U
#define GPSDO_RTC_PHASE_MAX_STEP_COUNT   1
#define GPSDO_RTC_PHASE_USE_DAC_FINE     0U
/*
 * PHASE_FINE is a slow NORMAL-state trim using PHASE10 diagnostics only.
 * It is guarded by GPSDO_PHASE_FINE_ENABLE and does not replace 10s/50s control.
 */

#define GPSDO_CLEAR_AFTER_DISC10_ADJUST  0U
#define GPSDO_GPS_INVALID_CONFIRM_N      3U
#define GPSDO_GPS_TIME_INVALID_CONFIRM_N 3U
#define GPSDO_E10_LOCK_ENTER_N           3U
#define GPSDO_TRACK10_DEADBAND_COUNT     1L
#define GPSDO_TRACK10_MAX_STEP_CODE      256U
#define GPSDO_TRACK10_STEP_E2_3          8U
#define GPSDO_TRACK10_STEP_E4_7          16U
#define GPSDO_TRACK10_STEP_E8_15         32U
#define GPSDO_TRACK10_STEP_E16_31        64U
#define GPSDO_TRACK10_STEP_E32_63        128U
#define GPSDO_TRACK10_STEP_E64_UP        256U
#define GPSDO_TRACK10_NEAR_ABS_THRESH        50U
#define GPSDO_TRACK10_NEAR_CONFIRM_N         2U
#define GPSDO_TRACK10_NEAR_JUMP_ABS_MAX      12U
#define GPSDO_TRACK10_NEAR_JUMP_RATIO_NUM    2U
#define GPSDO_TRACK10_NEAR_JUMP_RATIO_DEN    1U
#define GPSDO_TRACK10_SIGN_FLIP_ABS_MAX      7L
#define GPSDO_TRACK10_SIGN_FLIP_MIN_STEP     4U
#define GPSDO_TRACK10_ENTER_NORMAL_ABS_SUM   2L
#define GPSDO_TRACK10_MIN_TIME_MS            60000U
#define GPSDO_DISC10_COOLDOWN_WINDOWS        1U
#define GPSDO_NORMAL_E10_OUT_CONFIRM_N       2U
/*
 * NORMAL -> TRACK10 escape threshold on a single 10 s window.
 * Was 1e6 (= 100 kHz) which can never be reached, so the escape never fired
 * and a genuinely lost lock stayed in NORMAL.  500 counts = 50 Hz, below the
 * +-10 ppm pull limit (123 Hz) so a real unlock still triggers, and far above
 * the +-1..2 count window noise plus the 2-sample confirmation
 * (GPSDO_NORMAL_E10_OUT_CONFIRM_N) so it does not chatter.
 */
#define GPSDO_NORMAL_E10_OUT_FAST_ABS        500U

#define GPSDO_LOG_GPS_INVALID_DETAIL_ENABLE  1U
/*
 * Both of these were 2000 ms, i.e. one row per two seconds each, on top of the
 * 1 Hz GPSDO_RESULT row.  While the GPS has no fix they fire forever and bury
 * the acceptance rows, which is the opposite of what they are for.  10 s keeps
 * the information (a real state change still prints immediately, see
 * app_log_gps_invalid_detail) without dominating the terminal.
 */
#define GPSDO_GPS_INVALID_DETAIL_PERIOD_MS   10000U
#define GPSDO_LOG_WARM_WAIT_ENABLE           1U
#define GPSDO_WARM_WAIT_LOG_PERIOD_MS        10000U

#define DAC_DEADBAND_COUNT10             20L
#define DAC_DEADBAND_COUNT50             100L
#define DAC_TRACK10_GAIN_NUM             1L
#define DAC_TRACK10_GAIN_DEN             1L
#define GPSDO_DISC10_DEADBAND_COUNTS     2L
#define DAC_NORMAL10_DEADBAND_COUNT      GPSDO_DISC10_DEADBAND_COUNTS
#define DAC_NORMAL10_MAX_STEP_UV         5000UL
#define DAC_NORMAL10_GAIN_NUM            1L
#define DAC_NORMAL10_GAIN_DEN            20L
#define GPSDO_DISC50_DEADBAND_COUNTS     0L
#define GPSDO_DISC50_DEADBAND_COUNT      0L
/*
 * DISC50 step law:  step_LSB = clamp(|SW_E50| * LSB_PER_COUNT, MIN, MAX)
 *
 * Scale reference: one count of SW_E50 is 1/50 Hz = 20 mHz of output
 * frequency error, one DAC8571 LSB is 51.27 uV.  With the assumed VCTCXO
 * slope GPSDO_DAC_SLOPE_DEFAULT_COUNT_PER_100MV (100 counts per 100 mV
 * = 0.1 Hz/mV) one LSB moves the frequency by ~5.13 mHz, so ~3.9 LSB per
 * count of SW_E50 gives a deadbeat response.  With the MEASURED 141 Hz/V
 * slope, 1 SW_E50 count = 20 mHz = 2.77 LSB, so deadbeat is 2.77 -- not an
 * integer.  The value below (2) is chosen over 3 because it converges
 * MONOTONICALLY (gain 0.72, error shrinks by 72 % per 50 s window, no sign
 * reversal, ~2-3 windows to settle).  3 would be marginally faster but
 * overshoots by 8 %, and the alternating DAC steps that produces are easy to
 * misread as instability in the logs.  Raise to 3 only if pull-in is too slow
 * AND the SW_E50 sequence shows no overshoot.
 *
 * (Under the old assumed 100 Hz/V this would have been 3.9 LSB per count;
 * measuring the real slope is what makes this number trustworthy.)
 *
 * MAX is the actual fix.  The old table saturated at 8 LSB, i.e. 41 mHz per
 * 50 s window = 0.82 mHz/s of slew.  A 1 Hz error then needed ~20 minutes to
 * pull in, and the loop could not track a warm-up drift at all.  256 LSB
 * gives ~26 mHz/s, still only ~1.3 Hz per step and well inside the
 * DAC_CODE_MIN_SAFE / MAX_SAFE excursion limit.
 *
 * DEADBAND stays 0 on purpose: with a 20 mHz measurement quantum, a deadband
 * of 1 count would already consume the entire 0.02 Hz average budget.
 */
#define GPSDO_DISC50_STEP_LSB_PER_COUNT  2L
#define GPSDO_DISC50_STEP_LSB_MIN        1U
#define GPSDO_DISC50_MAX_STEP_CODE       256U
#define GPSDO_DISC50_MIN_INTERVAL_MS     45000U
#define DAC_NORMAL50_DEADBAND_COUNT      GPSDO_DISC50_DEADBAND_COUNTS
#define GPSDO_TRACK10_FINE_ERR_COUNTS    30L
#define GPSDO_TRACK10_MED_ERR_COUNTS     80L
#define GPSDO_TRACK10_FINE_MAX_STEP_UV   500UL
#define GPSDO_TRACK10_MED_MAX_STEP_UV    2000UL
#define GPSDO_TRACK10_COARSE_ERR_COUNTS  8L
#define DAC_NORMAL50_MAX_STEP_UV         300UL
#define GPSDO_ALLOW_50S_ACCUM_ERR10_COUNTS 1L
#define GPSDO_ALLOW_50S_ACCUM_MIN_WINDOWS  5U
#define DAC_MAX_STEP_10S_UV              20000UL
#define DAC_MAX_STEP_50S_UV              50000UL
#define DAC_RECOMMENDED_STEP_50S_UV      50000UL
/*
 * Sanity bound for accepting a COUNT10 window into the discipline loop.
 * The F103 cannot see the VCTCXO pull range, but the MAX V frame carries a
 * 27 bit count, so a corrupted frame can present values up to ~1.3e8
 * (= 10 kHz).  10000 counts is 1 kHz, far above any physically reachable
 * excursion (+-10 ppm of 12.288 MHz is only 123 Hz = 1230 counts) yet still
 * rejects wild frames that would otherwise slam the DAC into its clamp.
 */
#define DAC_ERR10_ABS_MAX_VALID          10000L
#define DAC_ERR50_ABS_MAX_VALID          5000000L
#define DAC_SKIP_WINDOWS                 1U
#define DISC_PPS_TIMEOUT_TRACK_SEC       25U
#define DISC_WARM_WAIT_LOG_SEC           15U
#define DISC_WIN5_SIZE                   5U

#define APP_TIM3_IRQ_HZ                  100U
#define APP_TIM3_IRQ_MS                  10U
#define APP_TICK20_TIM3_DIV              2U

#define GPSDO_SPI_RAW_DIAG_ENABLE        0U

/*
 * Measure the MCU clock by using the PC's own UART clock as the reference.
 *
 * The UART link cannot measure its own bit-rate error (the receiver
 * re-synchronises on every start bit and tolerates ~5 % per frame), so two
 * candidate rates 2.4 % apart are equally legible.  GPS PPS is the primary
 * reference -- but it needs a position fix, which indoors may never happen.
 *
 * Fallback: bytes the PC sends back to back leave it once every 10 bits, i.e.
 * every 10 / 115200 s = 86.806 us, paced by the PC crystal (typically +-50 ppm
 * -- far better than the 2.4 % we need to resolve).  The firmware timestamps
 * arrivals with DWT_CYCCNT (one tick per HCLK cycle) and solves
 *
 *     k = HCLK_real / 72e6 = cycles_per_frame / 6250     (PC baud 115200)
 *
 * giving 6250 for an 8 MHz OSC_IN, 9375 for 12 MHz and 9600 for 12.288 MHz.
 * Send ~64 characters from the terminal to feed it; the result appears as
 * CLK_K_X10000 / CLK_FRAMES in GPSDO_RESULT.
 */
#define APP_CLK_MEASURE_ENABLE           1U

/*
 * Repeat the PA3/PA0 electrical probe from the main loop while the GPS is
 * still silent.
 *
 * RX_BYTES=0 and PPS_CNT=0 have four very different causes that the counters
 * cannot separate: the module is unpowered/unwired, it is wired elsewhere, the
 * wire is open (pin floats), or traffic IS arriving but is framed wrong.  The
 * probe answers which by sampling GPIOA->IDR and reporting the low-time ratio,
 * the edge counts and the DMA progress; see app_gps_probe_pins().
 *
 * A single boot-time run is easy to miss -- the terminal usually connects
 * after the board has already booted, so the banner has scrolled away.  So it
 * repeats every APP_GPS_PIN_PROBE_PERIOD_MS, and it DISABLES ITSELF as soon as
 * either RX_BYTES or PPS_CNT becomes non-zero, which means it costs nothing
 * once the GPS is healthy.  Set to 0U to silence it manually.
 */
#define APP_GPS_PIN_PROBE_AUTO           0U
#define APP_GPS_PIN_PROBE_PERIOD_MS      5000U
/*
 * Byte count that counts as "the module is really talking" and silences the
 * probe.  An NMEA stream is hundreds of bytes per second, so anything in the
 * tens is conclusive; a single stray byte is not (see the RE gating in
 * app_gps_pull_down_high_permille(), which exists precisely because borrowing
 * the pin can fabricate one byte out of a floating node).
 */
#define APP_GPS_PIN_PROBE_MIN_BYTES      16U
/*
 * PPS edges that count as "PPS is really working" and silence the probe.
 *
 * Not 1, on purpose: a single edge is not evidence of a 1 Hz pulse train, and
 * a borrowed pin can latch exactly one (see the EXTI masking in
 * app_gps_pps_pull_up_high_permille()).  A real PPS reaches 3 within 3 s, and
 * the probe re-checks every APP_GPS_PIN_PROBE_PERIOD_MS, so it still stops
 * promptly once the receiver does lock.
 */
#define APP_GPS_PIN_PROBE_MIN_PPS        3U

#endif /* APP_CONFIG_H */

