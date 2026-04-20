#include "web_monitor_manager.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#define WEB_MONITOR_LOG_BUFFER_SIZE 4096
#define WEB_MONITOR_HISTORY_SIZE    8192
#define WEB_MONITOR_CHUNK_MAX       256
#define WEB_MONITOR_HEARTBEAT_MS    1000

static const char *TAG = "web_monitor";

static httpd_handle_t s_http_server = NULL;
static MessageBufferHandle_t s_log_buffer = NULL;
static SemaphoreHandle_t s_history_lock = NULL;
static bool s_active_viewer = false;
static bool s_initialized = false;
static bool s_capture_started = false;
static bool s_startup_history_frozen = false;
static vprintf_like_t s_previous_vprintf = NULL;
static esp_netif_t *s_ap_netif = NULL;
static char s_log_history[WEB_MONITOR_HISTORY_SIZE] = {0};
static size_t s_log_history_len = 0;

static const char INDEX_HTML[] =
    "<!doctype html>"
    "<html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Wearable Serial Monitor</title>"
    "<style>"
    "body{margin:0;font-family:Consolas,monospace;background:#0f172a;color:#e2e8f0;}"
    "header{padding:16px 20px;background:#111827;position:sticky;top:0;border-bottom:1px solid #334155;}"
    "h1{margin:0;font-size:18px;}p{margin:6px 0 0;color:#94a3b8;font-size:13px;}"
    ".section{padding:16px 20px;border-bottom:1px solid #1e293b;}"
    ".row{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:10px;}"
    ".title{margin:0;font-size:14px;color:#93c5fd;text-transform:uppercase;letter-spacing:.08em;}"
    ".log{white-space:pre-wrap;overflow-wrap:anywhere;background:#020617;border:1px solid #1e293b;"
    "padding:14px;border-radius:10px;min-height:140px;max-height:38vh;overflow:auto;}"
    "button{background:#1d4ed8;color:#eff6ff;border:none;border-radius:8px;padding:8px 12px;cursor:pointer;font:inherit;}"
    "button.stopped{background:#475569;}"
    "</style></head><body>"
    "<header><h1>ESPWearable Serial Monitor</h1>"
    "<p>Startup history is loaded first, then live logs stream while this page stays open.</p></header>"
    "<div class=\"section\"><div class=\"row\"><h2 class=\"title\">Startup / Buffered Logs</h2></div>"
    "<div id=\"history\" class=\"log\">Loading history...</div></div>"
    "<div class=\"section\"><div class=\"row\"><h2 class=\"title\">Live Logs</h2>"
    "<button id=\"toggle\">Pause Live View</button></div>"
    "<div id=\"live\" class=\"log\">Connecting...\n</div></div>"
    "<script>"
    "const historyLog=document.getElementById('history');"
    "const liveLog=document.getElementById('live');"
    "const toggle=document.getElementById('toggle');"
    "let livePaused=false;"
    "let pausedBuffer='';"
    "fetch('/history').then((r)=>r.text()).then((text)=>{historyLog.textContent=text||'[no buffered logs yet]\\n';historyLog.scrollTop=historyLog.scrollHeight;})"
    ".catch(()=>{historyLog.textContent='[failed to load history]\\n';});"
    "const source=new EventSource('/events');"
    "toggle.onclick=()=>{livePaused=!livePaused;toggle.textContent=livePaused?'Resume Live View':'Pause Live View';"
    "toggle.className=livePaused?'stopped':'';"
    "if(!livePaused&&pausedBuffer){if(liveLog.textContent==='Connecting...\\n'){liveLog.textContent='';}"
    "liveLog.textContent+=pausedBuffer;pausedBuffer='';liveLog.scrollTop=liveLog.scrollHeight;}};"
    "source.onmessage=(event)=>{const line=event.data+'\\n';if(livePaused){pausedBuffer+=line;return;}"
    "if(liveLog.textContent==='Connecting...\\n'){liveLog.textContent='';}"
    "liveLog.textContent+=line;liveLog.scrollTop=liveLog.scrollHeight;};"
    "source.onerror=()=>{const line='\\n[stream disconnected]\\n';if(livePaused){pausedBuffer+=line;return;}"
    "liveLog.textContent+=line;liveLog.scrollTop=liveLog.scrollHeight;};"
    "</script></body></html>";

static void append_history(const char *text, size_t len)
{
    if (s_history_lock == NULL || text == NULL || len == 0U) {
        return;
    }

    xSemaphoreTake(s_history_lock, portMAX_DELAY);
    if (s_startup_history_frozen) {
        xSemaphoreGive(s_history_lock);
        return;
    }

    if (len >= WEB_MONITOR_HISTORY_SIZE) {
        text += len - (WEB_MONITOR_HISTORY_SIZE - 1U);
        len = WEB_MONITOR_HISTORY_SIZE - 1U;
        memcpy(s_log_history, text, len);
        s_log_history[len] = '\0';
        s_log_history_len = len;
        xSemaphoreGive(s_history_lock);
        return;
    }

    if ((s_log_history_len + len) >= WEB_MONITOR_HISTORY_SIZE) {
        size_t overflow = (s_log_history_len + len) - (WEB_MONITOR_HISTORY_SIZE - 1U);
        memmove(s_log_history, s_log_history + overflow, s_log_history_len - overflow);
        s_log_history_len -= overflow;
        s_log_history[s_log_history_len] = '\0';
    }

    memcpy(s_log_history + s_log_history_len, text, len);
    s_log_history_len += len;
    s_log_history[s_log_history_len] = '\0';

    xSemaphoreGive(s_history_lock);
}

static int web_monitor_vprintf(const char *format, va_list args)
{
    va_list copy_for_web;
    va_list copy_for_history;
    char line[WEB_MONITOR_CHUNK_MAX];
    int ret;
    int written;

    va_copy(copy_for_web, args);
    va_copy(copy_for_history, args);
    ret = s_previous_vprintf != NULL ? s_previous_vprintf(format, args) : vprintf(format, args);

    written = vsnprintf(line, sizeof(line), format, copy_for_history);
    if (written > 0) {
        size_t len = (size_t)written;
        if (len >= sizeof(line)) {
            len = sizeof(line) - 1U;
        }
        append_history(line, len);
    }

    if (s_active_viewer && s_log_buffer != NULL) {
        written = vsnprintf(line, sizeof(line), format, copy_for_web);
        if (written > 0) {
            size_t len = (size_t)written;
            if (len >= sizeof(line)) {
                len = sizeof(line) - 1U;
            }
            (void)xMessageBufferSend(s_log_buffer, line, len, 0);
        }
    }

    va_end(copy_for_history);
    va_end(copy_for_web);
    return ret;
}

static esp_err_t index_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t history_get_handler(httpd_req_t *req)
{
    esp_err_t err;

    if (s_history_lock == NULL) {
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        return httpd_resp_sendstr(req, "");
    }

    xSemaphoreTake(s_history_lock, portMAX_DELAY);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    err = httpd_resp_send(req, s_log_history, (ssize_t)s_log_history_len);
    xSemaphoreGive(s_history_lock);

    return err;
}

static esp_err_t send_sse_message(httpd_req_t *req, const char *payload)
{
    esp_err_t err = httpd_resp_send_chunk(req, "data: ", HTTPD_RESP_USE_STRLEN);
    if (err != ESP_OK) {
        return err;
    }

    err = httpd_resp_send_chunk(req, payload, HTTPD_RESP_USE_STRLEN);
    if (err != ESP_OK) {
        return err;
    }

    return httpd_resp_send_chunk(req, "\n\n", 2);
}

static esp_err_t events_get_handler(httpd_req_t *req)
{
    char chunk[WEB_MONITOR_CHUNK_MAX];

    if (s_active_viewer) {
        httpd_resp_set_status(req, "503 Busy");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "Another viewer is already connected.");
    }

    s_active_viewer = true;

    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    if (send_sse_message(req, "[monitor connected]") != ESP_OK) {
        s_active_viewer = false;
        return ESP_FAIL;
    }

    while (1) {
        size_t received = xMessageBufferReceive(s_log_buffer, chunk, sizeof(chunk) - 1U,
                                                pdMS_TO_TICKS(WEB_MONITOR_HEARTBEAT_MS));
        if (received == 0U) {
            if (httpd_resp_send_chunk(req, ": keepalive\n\n", HTTPD_RESP_USE_STRLEN) != ESP_OK) {
                break;
            }
            continue;
        }

        chunk[received] = '\0';
        if (send_sse_message(req, chunk) != ESP_OK) {
            break;
        }
    }

    s_active_viewer = false;
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t events_uri = {
        .uri = "/events",
        .method = HTTP_GET,
        .handler = events_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t history_uri = {
        .uri = "/history",
        .method = HTTP_GET,
        .handler = history_get_handler,
        .user_ctx = NULL,
    };

    config.server_port = 80;
    config.max_uri_handlers = 8;
    config.lru_purge_enable = true;

    esp_err_t err = httpd_start(&s_http_server, &config);
    if (err != ESP_OK) {
        return err;
    }

    err = httpd_register_uri_handler(s_http_server, &index_uri);
    if (err != ESP_OK) {
        return err;
    }

    err = httpd_register_uri_handler(s_http_server, &events_uri);
    if (err != ESP_OK) {
        return err;
    }

    err = httpd_register_uri_handler(s_http_server, &history_uri);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

static esp_err_t start_softap(const web_monitor_manager_config_t *config)
{
    wifi_config_t ap_config = {0};
    size_t ssid_len;
    size_t password_len;

    if (config->ap_ssid == NULL || config->ap_password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ssid_len = strlen(config->ap_ssid);
    password_len = strlen(config->ap_password);
    if (ssid_len == 0 || ssid_len > 31) {
        return ESP_ERR_INVALID_ARG;
    }
    if (password_len != 0 && (password_len < 8 || password_len > 63)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == NULL) {
            return ESP_FAIL;
        }
    }

    memcpy(ap_config.ap.ssid, config->ap_ssid, ssid_len);
    memcpy(ap_config.ap.password, config->ap_password, password_len);
    ap_config.ap.ssid_len = (uint8_t)ssid_len;
    ap_config.ap.channel = config->channel;
    ap_config.ap.max_connection = config->max_connections;
    ap_config.ap.authmode = password_len == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    ap_config.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    ESP_LOGI(TAG, "SoftAP ready: SSID=%s channel=%u url=http://192.168.4.1/",
             config->ap_ssid, config->channel);
    return ESP_OK;
}

esp_err_t web_monitor_manager_early_init(void)
{
    if (s_capture_started) {
        return ESP_OK;
    }

    s_history_lock = xSemaphoreCreateMutex();
    if (s_history_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_previous_vprintf = esp_log_set_vprintf(web_monitor_vprintf);
    s_capture_started = true;
    return ESP_OK;
}

esp_err_t web_monitor_manager_init(const web_monitor_manager_config_t *config)
{
    if (s_initialized) {
        return ESP_OK;
    }

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = web_monitor_manager_early_init();
    if (err != ESP_OK) {
        return err;
    }

    s_log_buffer = xMessageBufferCreate(WEB_MONITOR_LOG_BUFFER_SIZE);
    if (s_log_buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    err = start_softap(config);
    if (err != ESP_OK) {
        return err;
    }

    err = start_http_server();
    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Web monitor ready");
    return ESP_OK;
}

void web_monitor_manager_finalize_startup_history(void)
{
    if (s_history_lock == NULL) {
        return;
    }

    xSemaphoreTake(s_history_lock, portMAX_DELAY);
    s_startup_history_frozen = true;
    xSemaphoreGive(s_history_lock);
}

bool web_monitor_manager_has_active_viewer(void)
{
    return s_active_viewer;
}
