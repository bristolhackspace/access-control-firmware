
#include <string.h>
#include <alloca.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "cJSON.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_manage.h"
#include "pn532.h"
#include "http_api.h"

#include "main.h"

static const char* TAG = "main";

#define CARD_SCAN_INTERVAL_MS 4000
#define ERROR_TIMEOUT_MS 2000

ESP_EVENT_DEFINE_BASE(APPLICATION_EVENT);

static volatile bool unlocked;
static volatile bool in_use;

typedef struct
{
    int id_len;
    uint8_t id[];
} card_id_t;


void handle_app_event(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    switch (event_id)
    {
    case APPLICATION_EVENT_CARD_SCANNED:
        card_id_t* card_id = (card_id_t*)event_data;
        ESP_LOGI(TAG, "Card found");
        http_api_unlock(card_id->id, card_id->id_len);
        break;
    case APPLICATION_EVENT_STATUS:
        cJSON* status = cJSON_CreateObject();
        cJSON_AddBoolToObject(status, "unlocked", unlocked);

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
    case APPLICATION_EVENT_SET_UNLOCKED:
        unlocked = *(bool*)event_data;
        if (unlocked) {
            ESP_LOGI(TAG, "Machine unlocked");
        } else {
            ESP_LOGI(TAG, "Machine locked");
        }
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

bool is_unlocked(void);
bool is_update_pending(void);
bool is_in_use(void);


void app_main(void)
{
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

    uint8_t loop_count = 0;

    card_id_t* card_id = alloca(sizeof(card_id_t) + CARD_ID_MAX_LEN);

    while (1)
    {
        // Re-initialise the PN532 every 16 interations just in case it has failed for whatever reason.
        if (loop_count % 16 == 0) {
            ESP_LOGI(TAG, "Reconfiguring pn532");
            pn532_wakeup();
            ret = pn532_sam_config();
            if (ret) {
                ESP_LOGE(TAG, "Error re-configuring pn532");
                vTaskDelay(ERROR_TIMEOUT_MS/portTICK_PERIOD_MS);
                continue;
            }
        }
        loop_count += 1;


        ESP_LOGI(TAG, "Scanning for card");
        ret = pn532_listen_for_passive_target();
        if (ret < 0) {
            ESP_LOGE(TAG, "Error iistening for targets");
            vTaskDelay(ERROR_TIMEOUT_MS/portTICK_PERIOD_MS);
            continue;
        }

        memset(card_id, 0, sizeof(card_id_t) + CARD_ID_MAX_LEN);
        ret = pn532_get_passive_target(card_id->id, CARD_ID_MAX_LEN, CARD_SCAN_INTERVAL_MS/portTICK_PERIOD_MS);
        if (ret == 0) {
            ESP_LOGI(TAG, "No card found");
            esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_STATUS, NULL, 0, portMAX_DELAY);
            continue;
        }

        if (ret < 0) {
            ESP_LOGE(TAG, "Error reading card");
            vTaskDelay(ERROR_TIMEOUT_MS/portTICK_PERIOD_MS);
            continue;
        }

        card_id->id_len = ret;
        // Obfuscate just for testing
        card_id->id[0] = 0x55;
        card_id->id[1] = 0xaa;
        card_id->id[2] = 0x55;

        esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_CARD_SCANNED, card_id, sizeof(card_id_t)+card_id->id_len, portMAX_DELAY);

        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}