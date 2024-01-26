
#include "app_events.h"

#include <math.h>
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "gpio_control.h"
#include "http_api.h"
#include "pn532.h"

static const char* TAG = "app_events";

static esp_event_loop_handle_t app_event_loop_handle;

static enum app_state current_state = APPLICATION_STATE_INITIALISING;

static StaticTimer_t status_update_timer_buffer;
static TimerHandle_t status_update_timer;

void handle_app_event(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

void on_state_change(void);

ESP_EVENT_DEFINE_BASE(APPLICATION_EVENT);

static void status_update_timer_handler(TimerHandle_t timer)
{
    app_event_post_isr(APPLICATION_EVENT_STATUS_UPDATE, NULL, 0);
}

void app_event_loop_init()
{
    esp_event_loop_args_t loop_args = {
        .queue_size = 32,
        .task_name = "app_evt",
        .task_stack_size = CONFIG_APP_EVENT_TASK_STASK_SIZE,
        .task_priority = CONFIG_APP_EVENT_TASK_PRIORITY,
        .task_core_id = 0
    };

    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &app_event_loop_handle));

    ESP_ERROR_CHECK(esp_event_handler_register_with(
        app_event_loop_handle,
        APPLICATION_EVENT,
        ESP_EVENT_ANY_ID,
        handle_app_event,
        NULL
    ));

    status_update_timer = xTimerCreateStatic("debounceTimer",
                                CONFIG_STATUS_UPDATE_INTERVAL/portTICK_PERIOD_MS,
                                true,
                                NULL,
                                status_update_timer_handler,
                                &status_update_timer_buffer);
    xTimerStart(status_update_timer, 0);
}


esp_err_t app_event_post(int32_t event_id, const void* event_data, size_t event_data_size, TickType_t ticks_to_wait)
{
    return esp_event_post_to(
        app_event_loop_handle,
        APPLICATION_EVENT,
        event_id, 
        event_data,
        event_data_size,
        ticks_to_wait
    );
}

esp_err_t app_event_post_isr(int32_t event_id, const void* event_data, size_t event_data_size)
{
    return esp_event_isr_post_to(
        app_event_loop_handle,
        APPLICATION_EVENT,
        event_id,
        event_data,
        event_data_size,
        NULL
    );
}

static void do_status_update()
{
    cJSON* status = cJSON_CreateObject();
    cJSON_AddBoolToObject(status, "unlocked", current_state == APPLICATION_STATE_UNLOCKED);

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
}


static void handle_json_response(cJSON *root)
{
    cJSON *current_element = NULL;

    cJSON_ArrayForEach(current_element, root) {
        if (current_element->string == NULL) {
            continue;
        }
        if (strcmp(current_element->string, "unlocked") == 0) {
            if (cJSON_IsBool(current_element)) {
                bool unlocked = cJSON_IsTrue(current_element);
                current_state = unlocked? APPLICATION_STATE_UNLOCKED:APPLICATION_STATE_LOCKED;
            }
        } else if (strcmp(current_element->string, "idle_timeout") == 0) {
            double timeout_dbl = cJSON_GetNumberValue(current_element);
            if (isnan(timeout_dbl)) {
                ESP_LOGE(TAG, "idle_timeout must be a number");
                continue;
            }
            // idle_timeout = (int)timeout_dbl;
            if (current_state == APPLICATION_STATE_INITIALISING)
            {
                current_state = APPLICATION_STATE_LOCKED;
            }
        } else if (strcmp(current_element->string, "idle_power_limit") == 0) {
            double idle_power_dbl = cJSON_GetNumberValue(current_element);
            if (isnan(idle_power_dbl)) {
                ESP_LOGE(TAG, "idle_power_limit must be a number");
                continue;
            }
            // idle_power_limit = (int)idle_power_dbl;
            if (current_state == APPLICATION_STATE_INITIALISING)
            {
                current_state = APPLICATION_STATE_UNLOCKED;
            }
        } else if (strcmp(current_element->string, "firmware_update") == 0) {
            char* update_url = cJSON_GetStringValue(current_element);
            if (update_url == NULL) {
                ESP_LOGE(TAG, "firmware_update must be a string");
                continue;
            }
            if (current_state != APPLICATION_STATE_FW_UPDATE)
            {
                current_state = APPLICATION_STATE_FW_UPDATE;
                gpio_set_led_mode(LED_MODE_ERROR);
                ESP_LOGI(TAG, "Updating from %s", update_url);
                http_api_ota(update_url);
            }
        } 
    }
}

void handle_app_event(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    enum app_state last_state = current_state;

    switch (event_id)
    {
    case APPLICATION_EVENT_CARD_SCANNED:
        switch (last_state)
        {
            case APPLICATION_STATE_LOCKED:
            case APPLICATION_STATE_UNLOCKED:
                card_id_t* card_id = (card_id_t*)event_data;
                ESP_LOGI(TAG, "Card found");
                http_api_unlock(card_id->id, sizeof(card_id_t));
                break;
            case APPLICATION_STATE_BUTTON_HOLD:
                current_state = APPLICATION_STATE_INDUCTION;
                break;
            case APPLICATION_STATE_INDUCTION:
                break;
            default:
                break;
        }
        break;
    case APPLICATION_EVENT_BUTTON_PRESS:
        switch (last_state)
        {
            case APPLICATION_STATE_LOCKED:
            case APPLICATION_STATE_UNLOCKED:
            case APPLICATION_STATE_INDUCTION:
                current_state = APPLICATION_STATE_BUTTON_HOLD;
                break;
            default:
                break;
        }
        break;
    case APPLICATION_EVENT_BUTTON_RELEASE:
        if (last_state == APPLICATION_STATE_BUTTON_HOLD)
        {
            current_state = APPLICATION_STATE_LOCKED;
        }
        break;
    case APPLICATION_EVENT_STATUS_UPDATE:
        if (last_state == APPLICATION_STATE_INITIALISING)
        {
            http_api_settings();
        }
        else if(last_state != APPLICATION_STATE_FW_UPDATE)
        {
            do_status_update();
        }
        break;
    case APPLICATION_EVENT_RESPONSE_JSON:
        cJSON* root = *(cJSON**)event_data;
        handle_json_response(root);
        cJSON_Delete(root);
        break;
    case APPLICATION_EVENT_HTTP_ERROR:
        gpio_led_error_flash();
        break;
    case APPLICATION_EVENT_FIRMWARE_UPDATE:
        break;
    default:
        break;
    }

    if (last_state != current_state)
    {
        on_state_change();
    }
}

void on_state_change(void)
{
    switch (current_state)
    {
    case APPLICATION_STATE_INITIALISING:
        break;
    case APPLICATION_STATE_LOCKED:
        gpio_set_relay(false);
        gpio_set_led_mode(LED_MODE_IDLE);
        break;
    case APPLICATION_STATE_UNLOCKED:
        gpio_set_relay(true);
        gpio_set_led_mode(LED_MODE_OFF);
        break;
    case APPLICATION_STATE_BUTTON_HOLD:
        break;
    case APPLICATION_STATE_INDUCTION:
        gpio_set_led_mode(LED_MODE_INDUCT);
        break;
    case APPLICATION_STATE_FW_UPDATE:
        break;
    default:
        break;
    } 
}