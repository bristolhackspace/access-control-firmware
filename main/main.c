
#include <string.h>
#include <alloca.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "driver/uart.h"
#include "cJSON.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_manage.h"
#include "pn532.h"
#include "http_api.h"
#include "gpio_control.h"

#include "main.h"

static const char* TAG = "main";

ESP_EVENT_DEFINE_BASE(APPLICATION_EVENT);

static volatile bool has_settings;
static volatile bool unlocked;
static volatile bool in_use;

static volatile int idle_timeout = -1;
static volatile int idle_power_limit = 50;

static void handle_json_response(cJSON *root)
{
    cJSON *current_element = NULL;

    cJSON_ArrayForEach(current_element, root) {
        if (current_element->string == NULL) {
            continue;
        }
        if (strcmp(current_element->string, "unlocked") == 0) {
            if (cJSON_IsBool(current_element)) {
                unlocked = cJSON_IsTrue(current_element);
                gpio_set_relay(unlocked);
            }
        } else if (strcmp(current_element->string, "firmware_update") == 0) {
            char* update_url = cJSON_GetStringValue(current_element);
            if (update_url == NULL) {
                ESP_LOGE(TAG, "firmware_update must be a string");
                continue;
            }
            esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_FIRMWARE_UPDATE, update_url, strlen(update_url)+1, portMAX_DELAY);
        } else if (strcmp(current_element->string, "idle_timeout") == 0) {
            double timeout_dbl = cJSON_GetNumberValue(current_element);
            if (isnan(timeout_dbl)) {
                ESP_LOGE(TAG, "idle_timeout must be a number");
                continue;
            }
            idle_timeout = (int)timeout_dbl;
            has_settings = true;
        } else if (strcmp(current_element->string, "idle_power_limit") == 0) {
            double idle_power_dbl = cJSON_GetNumberValue(current_element);
            if (isnan(idle_power_dbl)) {
                ESP_LOGE(TAG, "idle_power_limit must be a number");
                continue;
            }
            idle_power_limit = (int)idle_power_dbl;
            has_settings = true;
        } else if (strcmp(current_element->string, "invert_logout_button") == 0) {
            if (!cJSON_IsBool(current_element)) {
                ESP_LOGE(TAG, "invert_logout_button must be a boolean");
                continue;
            }
            goip_control_set_logout_button_polarity(cJSON_IsTrue(current_element));
            has_settings = true;
        }
    }

    cJSON_Delete(root);
}


void handle_app_event(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    switch (event_id)
    {
    case APPLICATION_EVENT_CARD_SCANNED:
        card_id_t* card_id = (card_id_t*)event_data;
        ESP_LOGI(TAG, "Card found");
        http_api_unlock(card_id->id, sizeof(card_id_t));
        break;
    case APPLICATION_EVENT_BUTTON_PRESS:
        ESP_LOGI(TAG, "Button pressed");
        http_api_lock();
        break;
    case APPLICATION_EVENT_STATUS:
        cJSON* status = cJSON_CreateObject();
        cJSON_AddBoolToObject(status, "unlocked", unlocked);

        cJSON_AddBoolToObject(status, "has_settings", has_settings);

        if (http_api_status(status) == ESP_OK) {
            esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
            const esp_partition_t* running_partition = esp_ota_get_running_partition();
            esp_ota_get_state_partition(running_partition, &state);
            if (state == ESP_OTA_IMG_PENDING_VERIFY) {
                esp_ota_mark_app_valid_cancel_rollback();
                ESP_LOGI(TAG, "Update successful, cancelling rollback");
            }
        }
        cJSON_Delete(status);

        break;
    case APPLICATION_EVENT_RESPONSE_JSON:
        cJSON* root = *(cJSON**)event_data;
        handle_json_response(root);
        break;
    case APPLICATION_EVENT_BLINK_ERROR:
        ESP_LOGI(TAG, "Blinking the LED in anger");
        break;
    case APPLICATION_EVENT_FIRMWARE_UPDATE:
        char* update_url = (char*)event_data;
        ESP_LOGI(TAG, "Updating from %s", update_url);
        http_api_ota(update_url);
        break;
    default:
        break;
    }
}

int uart_vprintf(const char* fmt, va_list arguments_list)
{
    char buffer[256];

    int num_bytes = vsnprintf(buffer,sizeof(buffer),fmt,arguments_list);//Storing the outptut into the string 

    uart_write_bytes(UART_NUM_2, buffer, num_bytes);
    return num_bytes;
}


esp_err_t logging_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 19, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    esp_log_set_vprintf(uart_vprintf);
    return ESP_OK;
}

void app_main(void)
{
    logging_init();

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_sta();

    ESP_ERROR_CHECK(pn532_init());
    ESP_ERROR_CHECK(http_api_init());

    ESP_ERROR_CHECK(esp_event_handler_register(APPLICATION_EVENT, ESP_EVENT_ANY_ID, handle_app_event, NULL));

    gpio_control_init();

    while (1)
    {
        esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_STATUS, NULL, 0, portMAX_DELAY);
        pn532_full_scan_sequence();
    }
}