/* =========================================================================
 *  RD-03D Radar + WiFi Web Interface for ESP32-C6 (Seeed XIAO)
 *
 *  Based on "working radar.c" — full multi-target tracker with footer
 *  validation, confirmed-human / static classification, and proper
 *  config-mode sequencing.
 *
 *  Additions:
 *   • WiFi STA (connects to your router)
 *   • Embedded HTTP server serving a single-page radar UI
 *   • WebSocket endpoint broadcasting target state at ~7 Hz
 *   • Mutex-protected shared tracker state
 * ========================================================================= */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

static const char *TAG = "RADAR";

// =========================================================================
// ==              CONFIGURATION                                         ==
// =========================================================================

// --- WiFi STA ---
#define WIFI_SSID               "Stambul"
#define WIFI_PASS               "CV123zxb"
#define WIFI_MAXIMUM_RETRY      20

#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1

// --- Radar Hardware ---
#define RADAR_UART_PORT_NUM     UART_NUM_1
#define RADAR_ESP_TX_PIN        (GPIO_NUM_1)
#define RADAR_ESP_RX_PIN        (GPIO_NUM_0)
#define LED_PIN                 (GPIO_NUM_15)
#define RADAR_BAUD_RATE         256000

// --- Task Stack Sizes ---
#define RADAR_TASK_STACK_SIZE   4096
#define DISPLAY_TASK_STACK_SIZE 4096
#define WS_TASK_STACK_SIZE      8192

// --- Timing ---
#define DISPLAY_PERIOD_MS       150
#define WS_BROADCAST_PERIOD_MS  50

// --- Advanced Tracker Configuration ---
#define MAX_TRACKED_TARGETS     3
#define TARGET_TIMEOUT_MS       2000
#define STATIC_TIMEOUT_MS       30000
#define TARGET_MATCH_DIST_MM    350.0f
#define MOVEMENT_THRESH_MM      0.0f
#define STATIC_CONFIRM_TIME_MS  1500
#define MIN_HUMAN_DISTANCE_MM   0
#define MAX_HUMAN_DISTANCE_MM   8000
#define MAX_SPEED_CM_S          500.0f

// --- Rd-03D Frame Protocol ---
typedef enum {
    FRAME_HEADER_1 = 0xAA,
    FRAME_HEADER_2 = 0xFF,
    FRAME_FOOTER_1 = 0x55,
    FRAME_FOOTER_2 = 0xCC
} FrameBytes;

#define FRAME_FULL_LENGTH       30
#define PAYLOAD_START_INDEX     4

// --- HTTP / WebSocket ---
#define MAX_WS_CLIENTS          4

// =========================================================================
// ==              DATA STRUCTURES                                       ==
// =========================================================================

typedef struct {
    float x_pos_mm;
    float y_pos_mm;
    float speed_cm_s;
} RawTargetData;

typedef struct {
    uint8_t  id;
    float    x_pos_mm;
    float    y_pos_mm;
    float    speed_cm_s;
    float    distance_mm;
    float    displacement_mm;
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    bool     confirmed_human;
    bool     is_static;
    bool     is_active;
} TrackedTarget;

// =========================================================================
// ==              GLOBAL STATE                                          ==
// =========================================================================

static TrackedTarget     tracked_targets[MAX_TRACKED_TARGETS];
static uint8_t           next_target_id = 1;
static volatile uint32_t g_frames_ok         = 0;
static volatile uint32_t g_frames_bad_footer = 0;

static SemaphoreHandle_t  tracker_mutex       = NULL;
static httpd_handle_t     http_server         = NULL;
static EventGroupHandle_t s_wifi_event_group  = NULL;
static int                s_retry_num         = 0;

// Embedded HTML (auto-generated header from radar_ui.html)
#include "radar_web_ui.h"

// =========================================================================
// ==              WIFI  (STA MODE)                                      ==
// =========================================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "WiFi retry %d/%d …", s_retry_num, WIFI_MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi connection failed after %d retries", WIFI_MAXIMUM_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected — IP: " IPSTR, IP2STR(&event->ip_info.ip));
        printf("\n=============================================\n");
        printf("  Web UI:  http://" IPSTR "\n", IP2STR(&event->ip_info.ip));
        printf("=============================================\n\n");
        fflush(stdout);
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst_any, inst_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &inst_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &inst_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID \"%s\" …", WIFI_SSID);

    /* Block until connected or failed */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected");
    } else {
        ESP_LOGE(TAG, "WiFi FAILED — web UI will be unavailable");
    }
}

// =========================================================================
// ==              HTTP SERVER  +  WEBSOCKET                             ==
// =========================================================================

/* GET / → serve embedded HTML page */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, radar_ui_html, radar_ui_html_len);
}

/* WS /ws → WebSocket endpoint */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WS client connected (fd %d)", httpd_req_to_sockfd(req));
        return ESP_OK;
    }
    /* Read & discard any incoming client frames */
    httpd_ws_frame_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.type = HTTPD_WS_TYPE_TEXT;
    uint8_t buf[128];
    pkt.payload = buf;
    return httpd_ws_recv_frame(req, &pkt, sizeof(buf));
}

/* GET /favicon.ico → 204 (suppress browser 404 noise) */
static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static const httpd_uri_t uri_root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
};
static const httpd_uri_t uri_ws = {
    .uri          = "/ws",
    .method       = HTTP_GET,
    .handler      = ws_handler,
    .is_websocket = true,
};
static const httpd_uri_t uri_favicon = {
    .uri       = "/favicon.ico",
    .method    = HTTP_GET,
    .handler   = favicon_handler,
};

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = MAX_WS_CLIENTS + 2;
    config.stack_size       = 8192;

    if (httpd_start(&http_server, &config) == ESP_OK) {
        httpd_register_uri_handler(http_server, &uri_root);
        httpd_register_uri_handler(http_server, &uri_ws);
        httpd_register_uri_handler(http_server, &uri_favicon);
        ESP_LOGI(TAG, "HTTP server started (max %d sockets)", config.max_open_sockets);
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
}

// =========================================================================
// ==              JSON BUILDER                                          ==
// =========================================================================

static int build_target_json(char *buf, size_t buf_size)
{
    int off = 0;
    uint32_t uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);

    off += snprintf(buf + off, buf_size - off, "{\"t\":[");

    for (int i = 0; i < MAX_TRACKED_TARGETS; i++) {
        TrackedTarget *t = &tracked_targets[i];
        if (i > 0) buf[off++] = ',';
        off += snprintf(buf + off, buf_size - off,
            "{\"id\":%u,\"x\":%.1f,\"y\":%.1f,\"s\":%.1f,"
            "\"d\":%.1f,\"h\":%d,\"st\":%d,\"a\":%d}",
            t->id, t->x_pos_mm, t->y_pos_mm, t->speed_cm_s,
            t->distance_mm,
            t->confirmed_human ? 1 : 0,
            t->is_static ? 1 : 0,
            t->is_active ? 1 : 0);
    }

    off += snprintf(buf + off, buf_size - off,
        "],\"ok\":%" PRIu32 ",\"bad\":%" PRIu32 ",\"up\":%" PRIu32 "}",
        g_frames_ok, g_frames_bad_footer, uptime_ms);

    return off;
}

// =========================================================================
// ==              WEBSOCKET BROADCAST TASK                              ==
// =========================================================================

static void ws_broadcast_task(void *pvParameters)
{
    char json[1024];
    int  fds[MAX_WS_CLIENTS + 2];

    while (1) {
        if (http_server == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Serialise target state under mutex */
        xSemaphoreTake(tracker_mutex, portMAX_DELAY);
        int len = build_target_json(json, sizeof(json));
        xSemaphoreGive(tracker_mutex);

        /* Broadcast to every WebSocket client */
        size_t n = (size_t)(MAX_WS_CLIENTS + 2);
        if (httpd_get_client_list(http_server, &n, fds) == ESP_OK) {
            for (size_t i = 0; i < n; i++) {
                if (httpd_ws_get_fd_info(http_server, fds[i])
                    == HTTPD_WS_CLIENT_WEBSOCKET)
                {
                    httpd_ws_frame_t pkt = {
                        .type    = HTTPD_WS_TYPE_TEXT,
                        .payload = (uint8_t *)json,
                        .len     = (size_t)len,
                        .final   = true
                    };
                    httpd_ws_send_frame_async(http_server, fds[i], &pkt);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(WS_BROADCAST_PERIOD_MS));
    }
}

// =========================================================================
// ==              DISPLAY  (serial dashboard — debug aid)               ==
// =========================================================================

static void print_target_dashboard(void)
{
    printf("\033[H");
    printf("====== Real-Time Radar Target Dashboard ======\n");
    printf("ID | Status   | X (cm) | Y (cm) | Speed (cm/s) | Dist (m)\n");
    printf("-----------------------------------------------------------\n");

    bool any = false;
    xSemaphoreTake(tracker_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_TRACKED_TARGETS; i++) {
        TrackedTarget *t = &tracked_targets[i];
        if (t->is_active) {
            any = true;
            const char *st = t->is_static ? "Static" : "Active";
            printf("%-2u | %-8s | %6.1f | %6.1f | %12.1f | %7.2f\n",
                   t->id, st,
                   t->x_pos_mm / 10.0f, t->y_pos_mm / 10.0f,
                   t->speed_cm_s, t->distance_mm / 1000.0f);
        } else {
            printf("-  | Inactive |    N/A |    N/A |          N/A |     N/A\n");
        }
    }
    xSemaphoreGive(tracker_mutex);

    printf("===========================================================\n");
    if (!any) printf("Status: Awaiting detection…\n");
    printf("Link: %" PRIu32 " ok, %" PRIu32 " bad footer\n",
           g_frames_ok, g_frames_bad_footer);
    printf("\n\n");
    fflush(stdout);
}

static void display_task(void *pvParameters)
{
    while (1) {
        print_target_dashboard();
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

// =========================================================================
// ==              TRACKING  (from working radar.c)                      ==
// =========================================================================

static void update_tracker(RawTargetData *new_targets, int count)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    bool matched[MAX_TRACKED_TARGETS] = {false};
    int  active_humans = 0;

    xSemaphoreTake(tracker_mutex, portMAX_DELAY);

    /* --- Match new detections to existing targets --- */
    for (int i = 0; i < count; i++) {
        float min_d = TARGET_MATCH_DIST_MM;
        int   best  = -1;

        for (int j = 0; j < MAX_TRACKED_TARGETS; j++) {
            if (tracked_targets[j].is_active && !matched[j]) {
                float dx = new_targets[i].x_pos_mm - tracked_targets[j].x_pos_mm;
                float dy = new_targets[i].y_pos_mm - tracked_targets[j].y_pos_mm;
                float d  = sqrtf(dx * dx + dy * dy);
                if (d < min_d) { min_d = d; best = j; }
            }
        }

        if (best != -1) {
            TrackedTarget *t = &tracked_targets[best];
            t->displacement_mm += min_d;
            t->x_pos_mm   = new_targets[i].x_pos_mm;
            t->y_pos_mm   = new_targets[i].y_pos_mm;
            t->speed_cm_s = new_targets[i].speed_cm_s;
            t->distance_mm = sqrtf(t->x_pos_mm * t->x_pos_mm +
                                   t->y_pos_mm * t->y_pos_mm);
            t->last_seen_ms = now;

            if (t->displacement_mm > MOVEMENT_THRESH_MM) {
                t->confirmed_human = true;
                if (t->is_static) {
                    t->is_static = false;
                    t->displacement_mm = 0;
                }
            }
            matched[best] = true;
        } else {
            /* New target — find a free slot */
            for (int j = 0; j < MAX_TRACKED_TARGETS; j++) {
                if (!tracked_targets[j].is_active) {
                    TrackedTarget *t = &tracked_targets[j];
                    t->id              = next_target_id++;
                    if (next_target_id == 0) next_target_id = 1;
                    t->x_pos_mm        = new_targets[i].x_pos_mm;
                    t->y_pos_mm        = new_targets[i].y_pos_mm;
                    t->speed_cm_s      = new_targets[i].speed_cm_s;
                    t->distance_mm     = sqrtf(t->x_pos_mm * t->x_pos_mm +
                                               t->y_pos_mm * t->y_pos_mm);
                    t->first_seen_ms   = now;
                    t->last_seen_ms    = now;
                    t->displacement_mm = 0;
                    t->confirmed_human = true; // Trust the radar's native ghost filtering
                    t->is_static       = false;
                    t->is_active       = true;
                    matched[j]         = true;
                    break;
                }
            }
        }
    }

    /* --- Time-out & classify --- */
    for (int i = 0; i < MAX_TRACKED_TARGETS; i++) {
        TrackedTarget *t = &tracked_targets[i];
        if (!t->is_active) continue;

        if (!matched[i]) {
            t->speed_cm_s = 0.0f; // Target is missing from radar frame, speed is effectively zero
            uint32_t elapsed = now - t->last_seen_ms;

            if (t->confirmed_human && !t->is_static && elapsed > STATIC_CONFIRM_TIME_MS) {
                t->is_static = true;
            }

            uint32_t timeout = t->is_static ? STATIC_TIMEOUT_MS : TARGET_TIMEOUT_MS;
            if (elapsed > timeout) {
                memset(t, 0, sizeof(TrackedTarget));
                t->is_active = false;
            }
        }
        if (t->is_active && t->confirmed_human) active_humans++;
    }

    xSemaphoreGive(tracker_mutex);

    gpio_set_level(LED_PIN, (active_humans > 0) ? 1 : 0);
}

// =========================================================================
// ==              FRAME PARSING  (from working radar.c)                 ==
// =========================================================================

static void parse_frame(const uint8_t *buf)
{
    /* Footer validation — catches desync from dropped/corrupted bytes */
    if (buf[FRAME_FULL_LENGTH - 2] != FRAME_FOOTER_1 ||
        buf[FRAME_FULL_LENGTH - 1] != FRAME_FOOTER_2) {
        g_frames_bad_footer++;
        return;
    }
    g_frames_ok++;

    RawTargetData det[MAX_TRACKED_TARGETS] = {0};
    int           cnt = 0;
    const uint8_t *payload = buf + PAYLOAD_START_INDEX;

    for (int i = 0; i < MAX_TRACKED_TARGETS; ++i) {
        const uint8_t *td = payload + (i * 8);

        /* Skip empty slots */
        bool empty = true;
        for (int j = 0; j < 8; ++j) if (td[j]) { empty = false; break; }
        if (empty) continue;

        uint16_t rx = (uint16_t)((td[1] << 8) | td[0]);
        uint16_t ry = (uint16_t)((td[3] << 8) | td[2]);
        uint16_t rs = (uint16_t)((td[5] << 8) | td[4]);

        /* Sign-magnitude: bit 15 = sign (1 = positive), bits 0-14 = magnitude */
        float x = (rx & 0x8000) ?  (float)(rx & 0x7FFF) : -(float)(rx & 0x7FFF);
        float y = (ry & 0x8000) ?  (float)(ry & 0x7FFF) : -(float)(ry & 0x7FFF);
        float s = (rs & 0x8000) ?  (float)(rs & 0x7FFF) : -(float)(rs & 0x7FFF);

        float dist = sqrtf(x * x + y * y);
        if (dist >= MIN_HUMAN_DISTANCE_MM &&
            dist <= MAX_HUMAN_DISTANCE_MM &&
            fabsf(s) < MAX_SPEED_CM_S)
        {
            det[cnt].x_pos_mm   = x;
            det[cnt].y_pos_mm   = y;
            det[cnt].speed_cm_s = s;
            cnt++;
        }
    }

    update_tracker(det, cnt);
}

// =========================================================================
// ==              RADAR UART RX TASK                                    ==
// =========================================================================

static void radar_rx_task(void *pvParameters)
{
    uint8_t frame[FRAME_FULL_LENGTH];
    uint8_t state = 0;
    int     idx   = 0;

    while (1) {
        uint8_t byte;
        int n = uart_read_bytes(RADAR_UART_PORT_NUM, &byte, 1, pdMS_TO_TICKS(10));
        if (n <= 0) continue;

        switch (state) {
        case 0:
            if (byte == FRAME_HEADER_1) { frame[0] = byte; state = 1; }
            break;
        case 1:
            if (byte == FRAME_HEADER_2) { frame[1] = byte; idx = 2; state = 2; }
            else state = 0;
            break;
        case 2:
            frame[idx++] = byte;
            if (idx == FRAME_FULL_LENGTH) { parse_frame(frame); state = 0; }
            break;
        }
    }
}

// =========================================================================
// ==              SENSOR COMMAND HELPERS  (from working radar.c)        ==
// =========================================================================

static void send_radar_command(const uint8_t *cmd, size_t len)
{
    uart_flush(RADAR_UART_PORT_NUM);
    uart_write_bytes(RADAR_UART_PORT_NUM, (const char *)cmd, len);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void radar_enable_config_mode(void)
{
    const uint8_t cmd[] = {
        0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0xFF,0x00, 0x01,0x00, 0x04,0x03,0x02,0x01
    };
    send_radar_command(cmd, sizeof(cmd));
}

static void radar_set_multi_target_mode(void)
{
    const uint8_t cmd[] = {
        0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0x90,0x00, 0x04,0x03,0x02,0x01
    };
    send_radar_command(cmd, sizeof(cmd));
}

static void radar_end_config_mode(void)
{
    const uint8_t cmd[] = {
        0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0xFE,0x00, 0x04,0x03,0x02,0x01
    };
    send_radar_command(cmd, sizeof(cmd));
}

// =========================================================================
// ==              APP MAIN                                              ==
// =========================================================================

void app_main(void)
{
    /* ---- Delay to allow Native USB to enumerate ---- */
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* ---- NVS (required by WiFi driver) ---- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ---- Tracker mutex ---- */
    tracker_mutex = xSemaphoreCreateMutex();

    /* ---- Log levels ---- */
    esp_log_level_set("*",   ESP_LOG_WARN);
    esp_log_level_set(TAG,   ESP_LOG_INFO);
    esp_log_level_set("wifi", ESP_LOG_WARN);

    /* ---- GPIO (LED) ---- */
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);

    /* ---- UART for radar ---- */
    uart_config_t uart_cfg = {
        .baud_rate  = RADAR_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(RADAR_UART_PORT_NUM, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RADAR_UART_PORT_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(RADAR_UART_PORT_NUM,
                                 RADAR_ESP_TX_PIN, RADAR_ESP_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    printf("RD-03D Radar + Web Interface\n");
    printf("Waiting for sensor to boot…\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* ---- WiFi ---- */
    printf("Connecting to WiFi \"%s\" …\n", WIFI_SSID);
    fflush(stdout);
    wifi_init_sta();

    /* ---- HTTP server ---- */
    start_webserver();

    /* ---- FreeRTOS tasks ---- */
    xTaskCreate(radar_rx_task,    "radar_rx",    RADAR_TASK_STACK_SIZE,   NULL, 10, NULL);
    xTaskCreate(display_task,     "display",     DISPLAY_TASK_STACK_SIZE, NULL,  3, NULL);
    xTaskCreate(ws_broadcast_task,"ws_bcast",    WS_TASK_STACK_SIZE,      NULL,  5, NULL);

    /* ---- Radar config sequence ---- */
    // Sent AFTER WiFi connects because turning on the WiFi radio causes a 3.3V power dip 
    // that can reboot the radar module, erasing its volatile config.
    printf("Configuring radar for multi-target mode…\n");
    vTaskDelay(pdMS_TO_TICKS(2000)); // wait for power to stabilize after WiFi
    radar_enable_config_mode();
    radar_set_multi_target_mode();
    radar_end_config_mode();
    printf("Radar configured!\n");
    fflush(stdout);

    printf("System running — radar RX + serial dashboard + web UI\n");
    fflush(stdout);
}