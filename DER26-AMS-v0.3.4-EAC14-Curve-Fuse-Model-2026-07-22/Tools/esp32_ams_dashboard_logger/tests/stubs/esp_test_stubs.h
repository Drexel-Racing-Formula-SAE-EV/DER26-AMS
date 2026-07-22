#ifndef DER26_ESP_TEST_STUBS_H
#define DER26_ESP_TEST_STUBS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_NOT_FOUND (-2)
#define ESP_ERR_NVS_NO_FREE_PAGES (-3)
#define ESP_ERR_NVS_NEW_VERSION_FOUND (-4)
#define ESP_ERROR_CHECK(x) do { (void)(x); } while(0)
#define ESP_LOGI(...) do { } while(0)
#define ESP_LOGW(...) do { } while(0)
static inline unsigned long esp_log_timestamp(void) { return 0ul; }

typedef void *SemaphoreHandle_t;
typedef void *TaskHandle_t;
#define portMAX_DELAY 0xffffffffu
#define pdMS_TO_TICKS(x) (x)
static inline int xSemaphoreTake(SemaphoreHandle_t handle, uint32_t timeout)
{ (void)handle; (void)timeout; return 1; }
static inline int xSemaphoreGive(SemaphoreHandle_t handle)
{ (void)handle; return 1; }
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void)
{ return (SemaphoreHandle_t)1; }
static inline void vTaskDelay(uint32_t ticks) { (void)ticks; }
static inline int xTaskCreate(void (*fn)(void *), const char *name,
                              uint32_t stack, void *arg, unsigned priority,
                              TaskHandle_t *handle)
{ (void)fn; (void)name; (void)stack; (void)arg; (void)priority; (void)handle; return 1; }

typedef struct httpd_req httpd_req_t;
typedef void *httpd_handle_t;
typedef struct { int stack_size; } httpd_config_t;
typedef struct {
    const char *uri;
    int method;
    esp_err_t (*handler)(httpd_req_t *);
} httpd_uri_t;
#define HTTPD_DEFAULT_CONFIG() ((httpd_config_t){ .stack_size = 4096 })
#define HTTPD_500_INTERNAL_SERVER_ERROR 500
#define HTTPD_RESP_USE_STRLEN (-1)
#define HTTP_GET 0
static inline esp_err_t httpd_resp_send_err(httpd_req_t *request, int code,
                                             const char *message)
{ (void)request; (void)code; (void)message; return ESP_OK; }
static inline esp_err_t httpd_resp_set_type(httpd_req_t *request,
                                             const char *type)
{ (void)request; (void)type; return ESP_OK; }
static inline esp_err_t httpd_resp_set_hdr(httpd_req_t *request,
                                            const char *name,
                                            const char *value)
{ (void)request; (void)name; (void)value; return ESP_OK; }
static inline esp_err_t httpd_resp_send(httpd_req_t *request,
                                         const char *data, long length)
{ (void)request; (void)data; (void)length; return ESP_OK; }
static inline esp_err_t httpd_start(httpd_handle_t *handle,
                                     const httpd_config_t *config)
{ (void)config; *handle = (void *)1; return ESP_OK; }
static inline esp_err_t httpd_register_uri_handler(httpd_handle_t handle,
                                                    const httpd_uri_t *uri)
{ (void)handle; (void)uri; return ESP_OK; }

typedef struct { int unused; } wifi_init_config_t;
#define WIFI_INIT_CONFIG_DEFAULT() ((wifi_init_config_t){0})
typedef struct {
    struct {
        uint8_t ssid[32];
        uint8_t ssid_len;
        uint8_t channel;
        uint8_t password[64];
        uint8_t max_connection;
        int authmode;
        struct { bool required; } pmf_cfg;
    } ap;
} wifi_config_t;
#define WIFI_AUTH_WPA_WPA2_PSK 1
#define WIFI_AUTH_OPEN 0
#define WIFI_MODE_AP 1
#define WIFI_IF_AP 1
static inline esp_err_t esp_netif_init(void) { return ESP_OK; }
static inline esp_err_t esp_event_loop_create_default(void) { return ESP_OK; }
static inline void *esp_netif_create_default_wifi_ap(void) { return (void *)1; }
static inline esp_err_t esp_wifi_init(const wifi_init_config_t *config)
{ (void)config; return ESP_OK; }
static inline esp_err_t esp_wifi_set_mode(int mode)
{ (void)mode; return ESP_OK; }
static inline esp_err_t esp_wifi_set_config(int interface,
                                             const wifi_config_t *config)
{ (void)interface; (void)config; return ESP_OK; }
static inline esp_err_t esp_wifi_start(void) { return ESP_OK; }

static inline esp_err_t nvs_flash_init(void) { return ESP_OK; }
static inline esp_err_t nvs_flash_erase(void) { return ESP_OK; }

typedef struct {
    uint32_t id;
    bool extended;
    uint8_t dlc;
    uint8_t data[8];
} mcp2515_frame_t;
static inline esp_err_t mcp2515_init(void) { return ESP_OK; }
static inline esp_err_t mcp2515_read_frame(mcp2515_frame_t *frame)
{ (void)frame; return ESP_ERR_NOT_FOUND; }

#endif
