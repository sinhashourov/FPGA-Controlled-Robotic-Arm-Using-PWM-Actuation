/* =============================================================================
 * File    : main.c
 * Project : 3-DOF Robotic Arm Controller — ESP32 Firmware
 * SDK     : ESP-IDF v5.x
 *
 * This file was previously MISSING from the project zip — now fully recreated.
 *
 * What changed from previous version:
 *   1. Deadzone S2 reduced: 120 → 30 counts.
 *      Old value of 120 permanently snapped joystick to centre — servo 2 froze.
 *   2. Smoothing alpha reduced: 150/180/150 → 100 for all.
 *      Old values were still sluggish. 100 gives ~200ms response — snappy.
 *   3. Remap defaults tightened: min raised, max lowered to match typical
 *      joystick ranges, ensuring full 0–4095 output for full 0°–180° servo.
 *   4. All three channels use same low deadzone (30) so all servos respond.
 *
 * Joystick wiring:
 *   Joystick 1 VRx → GPIO36 (ADC1_CH0) → Servo 1 (Base)
 *   Joystick 1 VRy → GPIO39 (ADC1_CH3) → Servo 2 (Shoulder)
 *   Joystick 2 VRx → GPIO34 (ADC1_CH6) → Servo 3 (Elbow)
 *   ALL joystick VCC → ESP32 3.3V pin   (NOT 5V)
 *   ALL joystick GND → ESP32 GND
 *   Hardware tip: solder 100nF cap between GPIO39 pin and GND to reduce noise.
 *
 * UART wiring:
 *   ESP32 GPIO17 (UART2 TX) → FPGA PMOD JB Pin 1 (D14)
 *   ESP32 GND               → FPGA PMOD JB GND
 *   Common ground MANDATORY — ESP32 GND = FPGA GND = Servo supply GND
 *
 * Packet format (8 bytes, transmitted every 20 ms at 115200 baud):
 *   [0] 0xAA         — Start marker
 *   [1] S1[11:8]     — Servo 1 upper nibble (4 bits)
 *   [2] S1[7:0]      — Servo 1 lower byte   (8 bits)
 *   [3] S2[11:8]     — Servo 2 upper nibble
 *   [4] S2[7:0]      — Servo 2 lower byte
 *   [5] S3[11:8]     — Servo 3 upper nibble
 *   [6] S3[7:0]      — Servo 3 lower byte
 *   [7] XOR([1]^[2]^[3]^[4]^[5]^[6]) — Checksum
 * ============================================================================= */

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

/* =============================================================================
 * CALIBRATION MODE
 *   Step 1: Set CALIBRATION_MODE 1, build and flash.
 *   Step 2: Open serial monitor (115200 baud).
 *   Step 3: Slowly move EACH joystick axis to its absolute extreme positions.
 *   Step 4: After ~20 seconds, note the printed min/max values.
 *   Step 5: Copy those values into the JOYx_MIN / JOYx_MAX defines below.
 *   Step 6: Set CALIBRATION_MODE 0, rebuild and flash. Done.
 * ============================================================================= */
#define CALIBRATION_MODE   0

/* =============================================================================
 * ADC CHANNEL REMAP CALIBRATION
 *   These define the real minimum and maximum raw ADC values your joysticks
 *   actually reach. The firmware stretches this range to full 0–4095, so the
 *   servo always covers the full 0°–180° range.
 *
 *   GPIO36 (JOY1 VRx) and GPIO39 (JOY1 VRy) are input-only pins on ESP32.
 *   GPIO34 (JOY2 VRx) also input-only — often has a slightly narrower range.
 *
 *   These defaults are safe starting values. Run CALIBRATION_MODE to get the
 *   exact values for YOUR joysticks.
 * ============================================================================= */
#define JOY1_X_MIN     100    /* GPIO36 raw ADC at full left  (Servo 1 / Base)     */
#define JOY1_X_MAX    4000    /* GPIO36 raw ADC at full right                       */

#define JOY1_Y_MIN     100    /* GPIO39 raw ADC at full down  (Servo 2 / Shoulder) */
#define JOY1_Y_MAX    4000    /* GPIO39 raw ADC at full up                          */

#define JOY2_X_MIN     200    /* GPIO34 raw ADC at full left  (Servo 3 / Elbow)    */
#define JOY2_X_MAX    3800    /* GPIO34 raw ADC at full right (narrower than CH0)   */

/* =============================================================================
 * PER-CHANNEL SMOOTHING
 *   IIR low-pass filter. Higher alpha = more smoothing but slower response.
 *   Range: 0 (instant, no smoothing) to 255 (maximum, very sluggish).
 *
 *   alpha=100 → ~200 ms to reach 99% of target — fast and smooth.
 *   alpha=150 → ~400 ms  |  alpha=200 → ~700 ms  |  alpha=230 → ~1.5 s
 *
 *   All channels use 100. GPIO39 (S2) noise is handled by the hardware
 *   100nF cap, not by excessive software smoothing.
 * ============================================================================= */
#define SMOOTH_S1   100
#define SMOOTH_S2   100
#define SMOOTH_S3   100

/* =============================================================================
 * PER-CHANNEL DEADZONE (counts on 0–4095 scale, applied after remap)
 *   Values within DEADZONE of centre (2048) snap to exactly 2048.
 *   This prevents servo jitter when the joystick is at rest.
 *
 *   30 counts = 0.7% of full range — stops jitter, doesn't hide real input.
 *   Old code used 120 for S2 which was 3× too large and froze servo 2.
 * ============================================================================= */
#define DEADZONE_S1   30
#define DEADZONE_S2   30
#define DEADZONE_S3   30

#define JOY_CENTER   2048

/* =============================================================================
 * ADC HARDWARE CONFIGURATION
 * ============================================================================= */
#define JOY1_X_CH    ADC_CHANNEL_0    /* GPIO36 → Servo 1 (Base)     */
#define JOY1_Y_CH    ADC_CHANNEL_3    /* GPIO39 → Servo 2 (Shoulder) */
#define JOY2_X_CH    ADC_CHANNEL_6    /* GPIO34 → Servo 3 (Elbow)    */
#define ADC_ATTEN    ADC_ATTEN_DB_12  /* 0 V – 3.3 V input range      */
#define ADC_SAMPLES  8                /* Average this many readings   */

/* =============================================================================
 * UART CONFIGURATION
 * ============================================================================= */
#define UART_PORT    UART_NUM_2
#define UART_TX_PIN  GPIO_NUM_17      /* → FPGA PMOD JB Pin 1 (D14) */
#define UART_RX_PIN  GPIO_NUM_16      /* Unused, required by driver  */
#define UART_BAUD    115200
#define UART_BUF     256
#define PKT_MS       20               /* Transmit every 20 ms = 50 Hz */

static const char *TAG = "arm";

/* ── Global handles ─────────────────────────────────────────── */
static adc_oneshot_unit_handle_t g_adc;
static adc_cali_handle_t         g_cal = NULL;
static bool                      g_cal_ok = false;

/* Smoothed values as fixed-point × 256 */
static uint32_t gs1 = (uint32_t)JOY_CENTER << 8;
static uint32_t gs2 = (uint32_t)JOY_CENTER << 8;
static uint32_t gs3 = (uint32_t)JOY_CENTER << 8;

/* =============================================================================
 * Helper: read one ADC channel, average ADC_SAMPLES readings
 * ============================================================================= */
static uint32_t adc_avg(adc_channel_t ch)
{
    int sum = 0, raw = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        adc_oneshot_read(g_adc, ch, &raw);
        sum += raw;
    }
    return (uint32_t)(sum / ADC_SAMPLES);
}

/* =============================================================================
 * Helper: remap raw ADC from [in_min..in_max] to [0..4095]
 *   Ensures the servo always covers full 0°–180° even if the joystick
 *   ADC doesn't physically reach 0 or 4095.
 * ============================================================================= */
static uint32_t remap(uint32_t raw, uint32_t in_min, uint32_t in_max)
{
    if (raw <= in_min) return 0;
    if (raw >= in_max) return 4095;
    return (raw - in_min) * 4095UL / (in_max - in_min);
}

/* =============================================================================
 * Helper: snap values within ±dz of JOY_CENTER to exactly JOY_CENTER
 * ============================================================================= */
static uint32_t deadzone(uint32_t v, uint32_t dz)
{
    int32_t off = (int32_t)v - JOY_CENTER;
    if (off > -(int32_t)dz && off < (int32_t)dz)
        return JOY_CENTER;
    return v;
}

/* =============================================================================
 * Helper: IIR low-pass filter
 *   new_smooth = (alpha × old + (256-alpha) × raw) / 256
 *   alpha in [0..255]. All values scaled by 256 (fixed-point).
 * ============================================================================= */
static uint32_t smooth(uint32_t prev_fp, uint32_t raw, uint32_t alpha)
{
    return (alpha * prev_fp + (256 - alpha) * (raw << 8)) >> 8;
}

/* =============================================================================
 * Helper: build and send one 8-byte servo packet over UART
 * ============================================================================= */
static void send_packet(uint16_t s1, uint16_t s2, uint16_t s3)
{
    uint8_t p[8];
    p[0] = 0xAA;
    p[1] = (uint8_t)((s1 >> 8) & 0x0F);
    p[2] = (uint8_t)(s1 & 0xFF);
    p[3] = (uint8_t)((s2 >> 8) & 0x0F);
    p[4] = (uint8_t)(s2 & 0xFF);
    p[5] = (uint8_t)((s3 >> 8) & 0x0F);
    p[6] = (uint8_t)(s3 & 0xFF);
    p[7] = p[1] ^ p[2] ^ p[3] ^ p[4] ^ p[5] ^ p[6];
    uart_write_bytes(UART_PORT, (const char *)p, 8);
}

/* =============================================================================
 * Calibration task — set CALIBRATION_MODE 1 to run this
 * ============================================================================= */
#if CALIBRATION_MODE
static void cal_task(void *arg)
{
    uint32_t mn1=4095, mx1=0, mn2=4095, mx2=0, mn3=4095, mx3=0;
    TickType_t t0 = xTaskGetTickCount();

    ESP_LOGW(TAG, "=== CALIBRATION MODE ===");
    ESP_LOGW(TAG, "Move ALL joystick axes SLOWLY to their full extremes.");

    while (1) {
        uint32_t r1 = adc_avg(JOY1_X_CH);
        uint32_t r2 = adc_avg(JOY1_Y_CH);
        uint32_t r3 = adc_avg(JOY2_X_CH);

        if (r1 < mn1) mn1 = r1;  if (r1 > mx1) mx1 = r1;
        if (r2 < mn2) mn2 = r2;  if (r2 > mx2) mx2 = r2;
        if (r3 < mn3) mn3 = r3;  if (r3 > mx3) mx3 = r3;

        uint32_t secs = (xTaskGetTickCount() - t0) / 1000;
        if (secs % 2 == 0) {
            ESP_LOGI(TAG, "S1(GPIO36) min=%4lu max=%4lu | "
                          "S2(GPIO39) min=%4lu max=%4lu | "
                          "S3(GPIO34) min=%4lu max=%4lu",
                     mn1, mx1, mn2, mx2, mn3, mx3);
            ESP_LOGW(TAG, "Paste these into JOYx_MIN/MAX defines then set "
                          "CALIBRATION_MODE 0 and reflash.");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

/* =============================================================================
 * Main servo control task
 * ============================================================================= */
static void servo_task(void *arg)
{
    TickType_t t0 = xTaskGetTickCount();
    uint32_t log_cnt = 0;

    ESP_LOGI(TAG, "Servo task running — %d ms packets", PKT_MS);

    while (1) {
        /* 1. Read ADC */
        uint32_t r1 = adc_avg(JOY1_X_CH);
        uint32_t r2 = adc_avg(JOY1_Y_CH);
        uint32_t r3 = adc_avg(JOY2_X_CH);

        /* 2. Remap to full 0–4095 */
        r1 = remap(r1, JOY1_X_MIN, JOY1_X_MAX);
        r2 = remap(r2, JOY1_Y_MIN, JOY1_Y_MAX);
        r3 = remap(r3, JOY2_X_MIN, JOY2_X_MAX);

        /* 3. Deadzone */
        r1 = deadzone(r1, DEADZONE_S1);
        r2 = deadzone(r2, DEADZONE_S2);
        r3 = deadzone(r3, DEADZONE_S3);

        /* 4. IIR smooth */
        gs1 = smooth(gs1, r1, SMOOTH_S1);
        gs2 = smooth(gs2, r2, SMOOTH_S2);
        gs3 = smooth(gs3, r3, SMOOTH_S3);

        uint16_t s1 = (uint16_t)((gs1 >> 8) & 0x0FFF);
        uint16_t s2 = (uint16_t)((gs2 >> 8) & 0x0FFF);
        uint16_t s3 = (uint16_t)((gs3 >> 8) & 0x0FFF);

        /* 5. Transmit */
        send_packet(s1, s2, s3);

        /* 6. Log every 1 s */
        if (++log_cnt >= (1000 / PKT_MS)) {
            ESP_LOGI(TAG, "S1(Base)=%4u  S2(Shoulder)=%4u  S3(Elbow)=%4u  "
                          "raw=[%4lu %4lu %4lu]",
                     s1, s2, s3, r1, r2, r3);
            log_cnt = 0;
        }

        vTaskDelayUntil(&t0, pdMS_TO_TICKS(PKT_MS));
    }
}

/* =============================================================================
 * ADC initialisation
 * ============================================================================= */
static void init_adc(void)
{
    /* Create ADC1 unit */
    adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &g_adc));

    /* Configure all three channels identically */
    adc_oneshot_chan_cfg_t ccfg = {
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc, JOY1_X_CH, &ccfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc, JOY1_Y_CH, &ccfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc, JOY2_X_CH, &ccfg));

    /* Try curve fitting calibration, fall back to line fitting */
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    {
        adc_cali_curve_fitting_config_t cfg = {
            .unit_id  = ADC_UNIT_1,
            .atten    = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_cali_create_scheme_curve_fitting(&cfg, &g_cal) == ESP_OK) {
            g_cal_ok = true;
            ESP_LOGI(TAG, "ADC cal: curve fitting");
        }
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!g_cal_ok) {
        adc_cali_line_fitting_config_t cfg = {
            .unit_id  = ADC_UNIT_1,
            .atten    = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_cali_create_scheme_line_fitting(&cfg, &g_cal) == ESP_OK) {
            g_cal_ok = true;
            ESP_LOGI(TAG, "ADC cal: line fitting");
        }
    }
#endif
    if (!g_cal_ok) ESP_LOGW(TAG, "ADC cal unavailable — using raw values");
}

/* =============================================================================
 * UART initialisation
 * ============================================================================= */
static void init_uart(void)
{
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART2 @ %d baud, TX=GPIO%d → FPGA JB1(D14)", UART_BAUD, UART_TX_PIN);
}

/* =============================================================================
 * app_main
 * ============================================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "=== 3-DOF Robotic Arm Controller (ESP-IDF v5) ===");
    ESP_LOGI(TAG, "Calibration mode: %s", CALIBRATION_MODE ? "ON" : "OFF");
    ESP_LOGI(TAG, "Smoothing: %d/%d/%d  Deadzone: %d/%d/%d",
             SMOOTH_S1, SMOOTH_S2, SMOOTH_S3,
             DEADZONE_S1, DEADZONE_S2, DEADZONE_S3);

    init_adc();

#if CALIBRATION_MODE
    ESP_LOGW(TAG, "Running calibration — UART disabled, servos will not move");
    xTaskCreatePinnedToCore(cal_task, "cal", 4096, NULL, 5, NULL, 1);
#else
    init_uart();
    xTaskCreatePinnedToCore(servo_task, "servo", 4096, NULL, 5, NULL, 1);
#endif
}
