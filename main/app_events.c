
#include "app_events.h"
#include "esp_ota_ops.h"

static esp_event_loop_handle_t app_event_loop_handle;

static enum app_state current_state = APPLICATION_STATE_INITIALISING;

void handle_app_event(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

void on_card_scanned(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void on_button_state_change(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void on_status_update_request(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void on_response_json(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void on_http_error(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

void app_event_loop_init()
{
    esp_event_loop_args_t loop_args = {
        .queue_size = 32,
        .task_name = "app_evt",
        .task_stack_size = CONFIG_APP_EVENT_TASK_STASK_SIZE,
        .task_priority = CONFIG_APP_EVENT_TASK_PRIORITY,
        .task_core_id = 0
    };

    esp_event_loop_create(&loop_args, &app_event_loop_handle);

    ESP_ERROR_CHECK(esp_event_handler_register_with(
        &app_event_loop_handle,
        APPLICATION_EVENT,
        APPLICATION_EVENT_CARD_SCANNED,
        on_card_scanned,
        NULL
    ));

    ESP_ERROR_CHECK(esp_event_handler_register_with(
        &app_event_loop_handle,
        APPLICATION_EVENT,
        APPLICATION_EVENT_BUTTON_PRESS,
        on_button_state_change,
        NULL
    ));

    ESP_ERROR_CHECK(esp_event_handler_register_with(
        &app_event_loop_handle,
        APPLICATION_EVENT,
        APPLICATION_EVENT_STATUS,
        on_status_update_request,
        NULL
    ));

    ESP_ERROR_CHECK(esp_event_handler_register_with(
        &app_event_loop_handle,
        APPLICATION_EVENT,
        APPLICATION_EVENT_RESPONSE_JSON,
        on_response_json,
        NULL
    ));

    ESP_ERROR_CHECK(esp_event_handler_register_with(
        &app_event_loop_handle,
        APPLICATION_EVENT,
        APPLICATION_EVENT_CARD_SCANNED,
        on_http_error,
        NULL
    ));
}

esp_err_t app_event_post(int32_t event_id, const void* event_data, size_t event_data_size, TickType_t ticks_to_wait)
{
    return esp_event_post_to(
        &app_event_loop_handle,
        APPLICATION_EVENT,
        event_id, 
        event_data,
        event_data_size,
        ticks_to_wait
    );
}


ESP_EVENT_DEFINE_BASE(APPLICATION_EVENT);

static volatile int idle_timeout = -1;
static volatile int idle_power_limit = 50;

void on_card_scanned(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    switch (current_state) {
        case APPLICATION_STATE_LOCKED:
        case APPLICATION_STATE_UNLOCKED:
            card_id_t* card_id = (card_id_t*)event_data;
            ESP_LOGI(TAG, "Card found");
            http_api_unlock(card_id->id, sizeof(card_id_t));
            break;
        case APPLICATION_STATE_BUTTON_HOLD:
            break;
        default:
            break;
    }
}

void on_button_state_change(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{

}

void on_status_update_request(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    cJSON* status = cJSON_CreateObject();
    cJSON_AddBoolToObject(status, "unlocked", current_state == APPLICATION_STATE_UNLOCKED);

    cJSON_AddBoolToObject(status, "has_settings", current_state != APPLICATION_STATE_INITIALISING);

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
}

void on_response_json(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    cJSON* root = *(cJSON**)event_data;
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
            idle_power_limit = (int)idle_power_dbl;
            if (current_state == APPLICATION_STATE_INITIALISING)
            {
                current_state = APPLICATION_STATE_LOCKED;
            }
        }
    }

    cJSON_Delete(root);
}

void on_http_error(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{

}


void handle_app_event(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    switch (event_id)
    {
    case APPLICATION_EVENT_CARD_SCANNED:
        
        break;
    case APPLICATION_EVENT_BUTTON_PRESS:
        ESP_LOGI(TAG, "Button pressed");
        http_api_lock();
        break;
    case APPLICATION_EVENT_STATUS:
        
    case APPLICATION_EVENT_RESPONSE_JSON:
        cJSON* root = *(cJSON**)event_data;
        handle_json_response(root);
        break;
    case APPLICATION_EVENT_HTTP_ERROR:
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