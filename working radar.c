#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <inttypes.h>

// =========================================================================
// ==              CONFIGURATION FOR ESP32-C6 BOARD                     ==
// =========================================================================
#define RADAR_UART_PORT_NUM     UART_NUM_1
#define RADAR_ESP_TX_PIN        (GPIO_NUM_1)
#define RADAR_ESP_RX_PIN        (GPIO_NUM_0)
#define LED_PIN                 (GPIO_NUM_15)
#define RADAR_BAUD_RATE         256000
#define RADAR_TASK_STACK_SIZE   4096
#define DISPLAY_TASK_STACK_SIZE 3072
#define DISPLAY_PERIOD_MS       150   // throttle dashboard redraws

// --- Advanced Tracker Configuration ---
#define MAX_TRACKED_TARGETS     3
#define TARGET_TIMEOUT_MS       2000
#define TARGET_MATCH_DIST_MM    500.0f
#define MOVEMENT_THRESH_MM      300.0f
#define STATIC_CONFIRM_TIME_MS  3000

// NOTE: RD-03D usable range is several meters. The previous 200-500mm
// window discarded almost all real detections. Widen (and tune) as needed
// for your enclosure/mounting height.
#define MIN_HUMAN_DISTANCE_MM   200
#define MAX_HUMAN_DISTANCE_MM   6000
#define MAX_SPEED_CM_S          500.0f
// =========================================================================

// ---------- Rd-03D Frame Protocol Constants ----------
typedef enum {
    FRAME_HEADER_1      = 0xAA,
    FRAME_HEADER_2      = 0xFF,
    FRAME_FOOTER_1      = 0x55,
    FRAME_FOOTER_2      = 0xCC
} FrameBytes;

#define FRAME_FULL_LENGTH   30
#define PAYLOAD_START_INDEX 4

// --- Structs for Target Data and Tracking ---
typedef struct {
    float x_pos_mm;
    float y_pos_mm;
    float speed_cm_s;
} RawTargetData;

typedef struct {
    uint8_t id;
    float x_pos_mm;
    float y_pos_mm;
    float speed_cm_s;
    float distance_mm;
    float displacement_mm;
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    bool confirmed_human;
    bool is_static;
    bool is_active;
} TrackedTarget;

// --- Global Tracker Array ---
// Only ever written from radar_rx_task; display task only reads it.
// If you later add another writer (e.g. a web server clearing targets),
// protect this with a mutex.
static TrackedTarget tracked_targets[MAX_TRACKED_TARGETS];
static uint8_t next_target_id = 1;

// Stats for debugging link quality
static volatile uint32_t g_frames_ok = 0;
static volatile uint32_t g_frames_bad_footer = 0;

// =========================================================================
// ==                      Display (decoupled from RX)                  ==
// =========================================================================
static void print_target_dashboard(void) {
    printf("\033[H");
    printf("====== Real-Time Radar Target Dashboard ======\n");
    printf("ID | Status   | X (cm) | Y (cm) | Speed (cm/s) | Dist (m)\n");
    printf("-----------------------------------------------------------\n");

    bool any_target_active = false;
    for (int i = 0; i < MAX_TRACKED_TARGETS; i++) {
        TrackedTarget *t = &tracked_targets[i];
        if (t->is_active) {
            any_target_active = true;
            const char *status = t->is_static ? "Static" : "Active";
            printf("%-2u | %-8s | %6.1f | %6.1f | %12.1f | %7.2f\n",
                   t->id, status,
                   t->x_pos_mm / 10.0f,
                   t->y_pos_mm / 10.0f,
                   t->speed_cm_s,
                   t->distance_mm / 1000.0f);
        } else {
            printf("-  | Inactive |    N/A |    N/A |          N/A |     N/A\n");
        }
    }
    printf("===========================================================\n");
    if (!any_target_active) {
        printf("Status: Awaiting detection...\n");
    }
    printf("Link: %" PRIu32 " ok frames, %" PRIu32 " bad footer\n",
           g_frames_ok, g_frames_bad_footer);
    printf("\n\n");
    fflush(stdout);
}

// Low priority task: rendering never blocks the UART parser.
static void display_task(void *pvParameters) {
    while (1) {
        print_target_dashboard();
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

// =========================================================================
// ==                            Tracking                                ==
// =========================================================================
static void update_tracker(RawTargetData *new_targets, int count) {
    uint32_t current_time_ms = esp_timer_get_time() / 1000;
    bool matched[MAX_TRACKED_TARGETS] = {false};
    int active_human_targets = 0;

    for (int i = 0; i < count; i++) {
        float min_dist = TARGET_MATCH_DIST_MM;
        int best_match_idx = -1;

        for (int j = 0; j < MAX_TRACKED_TARGETS; j++) {
            if (tracked_targets[j].is_active && !matched[j]) {
                float dx = new_targets[i].x_pos_mm - tracked_targets[j].x_pos_mm;
                float dy = new_targets[i].y_pos_mm - tracked_targets[j].y_pos_mm;
                float dist = sqrtf(dx * dx + dy * dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_match_idx = j;
                }
            }
        }

        if (best_match_idx != -1) {
            TrackedTarget *t = &tracked_targets[best_match_idx];
            t->displacement_mm += min_dist;
            t->x_pos_mm = new_targets[i].x_pos_mm;
            t->y_pos_mm = new_targets[i].y_pos_mm;
            t->speed_cm_s = new_targets[i].speed_cm_s;
            t->distance_mm = sqrtf(t->x_pos_mm * t->x_pos_mm + t->y_pos_mm * t->y_pos_mm);
            t->last_seen_ms = current_time_ms;

            if (t->displacement_mm > MOVEMENT_THRESH_MM) {
                t->confirmed_human = true;
                if (t->is_static) {
                    t->is_static = false;
                    t->displacement_mm = 0;
                }
            }
            matched[best_match_idx] = true;
        } else {
            for (int j = 0; j < MAX_TRACKED_TARGETS; j++) {
                if (!tracked_targets[j].is_active) {
                    TrackedTarget *t = &tracked_targets[j];
                    t->id = next_target_id++;
                    if (next_target_id == 0) next_target_id = 1;

                    t->x_pos_mm = new_targets[i].x_pos_mm;
                    t->y_pos_mm = new_targets[i].y_pos_mm;
                    t->speed_cm_s = new_targets[i].speed_cm_s;
                    t->distance_mm = sqrtf(t->x_pos_mm * t->x_pos_mm + t->y_pos_mm * t->y_pos_mm);
                    t->first_seen_ms = current_time_ms;
                    t->last_seen_ms = current_time_ms;
                    t->displacement_mm = 0;
                    t->confirmed_human = false;
                    t->is_static = false;
                    t->is_active = true;
                    break;
                }
            }
        }
    }

    for (int i = 0; i < MAX_TRACKED_TARGETS; i++) {
        TrackedTarget *t = &tracked_targets[i];
        if (t->is_active) {
            if (!matched[i]) {
                if (current_time_ms - t->last_seen_ms > TARGET_TIMEOUT_MS) {
                    memset(t, 0, sizeof(TrackedTarget));
                    t->is_active = false;
                } else if (t->confirmed_human && !t->is_static &&
                           (current_time_ms - t->last_seen_ms > STATIC_CONFIRM_TIME_MS)) {
                    if (t->displacement_mm < 50.0f) {
                        t->is_static = true;
                    }
                }
            }
            if (t->is_active && t->confirmed_human) {
                active_human_targets++;
            }
        }
    }

    gpio_set_level(LED_PIN, (active_human_targets > 0) ? 1 : 0);
    // Dashboard rendering is now handled by display_task, not here.
}

// =========================================================================
// ==                         Frame parsing                              ==
// =========================================================================
static void parse_frame(const uint8_t *frame_buffer) {
    // Validate footer before trusting the payload — catches desync from
    // dropped/corrupted bytes that would otherwise silently misalign parsing.
    if (frame_buffer[FRAME_FULL_LENGTH - 2] != FRAME_FOOTER_1 ||
        frame_buffer[FRAME_FULL_LENGTH - 1] != FRAME_FOOTER_2) {
        g_frames_bad_footer++;
        return;
    }
    g_frames_ok++;

    RawTargetData new_targets[MAX_TRACKED_TARGETS] = {0};
    int valid_target_count = 0;
    const uint8_t *payload = frame_buffer + PAYLOAD_START_INDEX;

    for (int i = 0; i < MAX_TRACKED_TARGETS; ++i) {
        const uint8_t *target_data = payload + (i * 8);

        bool is_empty = true;
        for (int j = 0; j < 8; ++j) {
            if (target_data[j] != 0) {
                is_empty = false;
                break;
            }
        }
        if (is_empty) continue;

        uint16_t raw_x = (uint16_t)((target_data[1] << 8) | target_data[0]);
        uint16_t raw_y = (uint16_t)((target_data[3] << 8) | target_data[2]);
        uint16_t raw_speed = (uint16_t)((target_data[5] << 8) | target_data[4]);

        // Sign-magnitude: bit15 = sign (1=positive, 0=negative), bits0-14 = magnitude
        float final_x_mm = (raw_x & 0x8000) ? (float)(raw_x & 0x7FFF) : -(float)(raw_x & 0x7FFF);
        float final_y_mm = (raw_y & 0x8000) ? (float)(raw_y & 0x7FFF) : -(float)(raw_y & 0x7FFF);
        float final_speed_cm_s = (raw_speed & 0x8000) ? (float)(raw_speed & 0x7FFF) : -(float)(raw_speed & 0x7FFF);

        float distance_mm = sqrtf(final_x_mm * final_x_mm + final_y_mm * final_y_mm);

        if (distance_mm >= MIN_HUMAN_DISTANCE_MM &&
            distance_mm <= MAX_HUMAN_DISTANCE_MM &&
            fabsf(final_speed_cm_s) < MAX_SPEED_CM_S) {

            new_targets[valid_target_count].x_pos_mm = final_x_mm;
            new_targets[valid_target_count].y_pos_mm = final_y_mm;
            new_targets[valid_target_count].speed_cm_s = final_speed_cm_s;
            valid_target_count++;
        }
    }

    update_tracker(new_targets, valid_target_count);
}

static void radar_rx_task(void *pvParameters) {
    uint8_t frame_buffer[FRAME_FULL_LENGTH];
    uint8_t state = 0;
    int frame_index = 0;

    while (1) {
        uint8_t byte;
        int bytes_read = uart_read_bytes(RADAR_UART_PORT_NUM, &byte, 1, pdMS_TO_TICKS(10));

        if (bytes_read > 0) {
            switch (state) {
                case 0:
                    if (byte == FRAME_HEADER_1) {
                        frame_buffer[0] = byte;
                        state = 1;
                    }
                    break;
                case 1:
                    if (byte == FRAME_HEADER_2) {
                        frame_buffer[1] = byte;
                        frame_index = 2;
                        state = 2;
                    } else {
                        state = 0;
                    }
                    break;
                case 2:
                    frame_buffer[frame_index++] = byte;
                    if (frame_index == FRAME_FULL_LENGTH) {
                        parse_frame(frame_buffer);
                        state = 0;
                    }
                    break;
            }
        }
        // No extra vTaskDelay here beyond the read timeout itself — keeps
        // byte-to-byte latency minimal so we don't starve the UART FIFO.
    }
}

// =========================================================================
// ==                       Sensor command helpers                       ==
// =========================================================================
static void send_radar_command(const uint8_t *command, size_t length) {
    uart_flush(RADAR_UART_PORT_NUM);
    uart_write_bytes(RADAR_UART_PORT_NUM, (const char *)command, length);
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Puts the sensor into configuration mode. Must precede any mode-select
// command (multi-target / single-target).
static void radar_enable_config_mode(void) {
    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,  // header
        0x04, 0x00,              // length = 4
        0xFF, 0x00,              // command word: enable config
        0x01, 0x00,              // value: 1
        0x04, 0x03, 0x02, 0x01   // footer
    };
    send_radar_command(cmd, sizeof(cmd));
}

// Selects multi-target tracking mode (up to 3 simultaneous targets).
// This was missing before — without it the sensor may remain in
// single-target mode or fail to leave config mode cleanly.
static void radar_set_multi_target_mode(void) {
    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,  // header
        0x02, 0x00,              // length = 2
        0x90, 0x00,              // command word: multi-target mode
        0x04, 0x03, 0x02, 0x01   // footer
    };
    send_radar_command(cmd, sizeof(cmd));
}

// Ends configuration mode and resumes normal streaming output.
static void radar_end_config_mode(void) {
    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,  // header
        0x02, 0x00,              // length = 2
        0xFE, 0x00,              // command word: end config
        0x04, 0x03, 0x02, 0x01   // footer
    };
    send_radar_command(cmd, sizeof(cmd));
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_ERROR);

    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);

    uart_config_t uart_config = {
        .baud_rate = RADAR_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(RADAR_UART_PORT_NUM, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RADAR_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RADAR_UART_PORT_NUM, RADAR_ESP_TX_PIN, RADAR_ESP_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    printf("Radar tracker waiting for sensor to boot...\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Full config sequence: enter config -> select multi-target -> exit config.
    // Sending only step 1 (as before) leaves the sensor parked in config
    // mode instead of streaming target frames.
    radar_enable_config_mode();
    radar_set_multi_target_mode();
    radar_end_config_mode();

    xTaskCreate(radar_rx_task, "radar_rx_task", RADAR_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(display_task, "display_task", DISPLAY_TASK_STACK_SIZE, NULL, 5, NULL);

    printf("ESP32-C6 Radar Tracker Initialized and Running\n");
    fflush(stdout);
}