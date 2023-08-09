
#include <string.h>
#include <alloca.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "cJSON.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_manage.h"
#include "pn532.h"
#include "http_api.h"

#include "main.h"

static const char* TAG = "main";

#define CARD_SCAN_INTERVAL_MS 2000
#define RELAY_GPIO_PIN 26
#define BUTTON_GPIO_PIN 25
#define PWR_DETECT_GPIO_PIN 4
#define DEBOUNCE_INTERVAL_MS 250

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
    case APPLICATION_EVENT_BUTTON_PRESS:
        ESP_LOGI(TAG, "Button pressed");
        http_api_lock();
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
            gpio_set_level(RELAY_GPIO_PIN, 1);
            ESP_LOGI(TAG, "Machine unlocked");
        } else {
            gpio_set_level(RELAY_GPIO_PIN, 0);
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

static void IRAM_ATTR button_isr_handler(void* arg)
{
    static TickType_t last_interrupt_tick = 0;

    TickType_t current_interrupt_tick = xTaskGetTickCount();
    TickType_t delta = current_interrupt_tick - last_interrupt_tick;

    if (delta*portTICK_PERIOD_MS < DEBOUNCE_INTERVAL_MS)
    {
        return;
    }

    last_interrupt_tick = current_interrupt_tick;

    uint32_t gpio_num = (uint32_t) arg;
    if (BUTTON_GPIO_PIN == gpio_num)
    {
        esp_event_isr_post(APPLICATION_EVENT, APPLICATION_EVENT_BUTTON_PRESS, NULL, 0, NULL);
    }
}

void gpio_init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << RELAY_GPIO_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);


    uint32_t button_gpio_pin = BUTTON_GPIO_PIN;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << button_gpio_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(button_gpio_pin, button_isr_handler, (void*) button_gpio_pin);

    uint32_t pwr_detect_gpio_pin = PWR_DETECT_GPIO_PIN;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << pwr_detect_gpio_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

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

    gpio_init();

    uint8_t loop_count = 0;

    card_id_t* card_id = alloca(sizeof(card_id_t) + CARD_ID_MAX_LEN);

    while (1)
    {
        esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_STATUS, NULL, 0, portMAX_DELAY);

        // Re-initialise the PN532 every 16 interations just in case it has failed for whatever reason.
        if (loop_count % 16 == 0) {
            ESP_LOGI(TAG, "Reconfiguring pn532");
            pn532_wakeup();
            ret = pn532_sam_config();
            if (ret) {
                ESP_LOGE(TAG, "Error re-configuring pn532");
                vTaskDelay(CARD_SCAN_INTERVAL_MS/portTICK_PERIOD_MS);
                continue;
            }
        }
        loop_count += 1;


        ESP_LOGI(TAG, "Scanning for card");
        ret = pn532_listen_for_passive_target();
        if (ret < 0) {
            ESP_LOGE(TAG, "Error iistening for targets");
            vTaskDelay(CARD_SCAN_INTERVAL_MS/portTICK_PERIOD_MS);
            continue;
        }

        memset(card_id, 0, sizeof(card_id_t) + CARD_ID_MAX_LEN);
        ret = pn532_get_passive_target(card_id->id, CARD_ID_MAX_LEN, CARD_SCAN_INTERVAL_MS/portTICK_PERIOD_MS);
        if (ret == 0) {
            ESP_LOGI(TAG, "No card found");
            continue;
        }

        if (ret < 0) {
            ESP_LOGE(TAG, "Error reading card");
            vTaskDelay(CARD_SCAN_INTERVAL_MS/portTICK_PERIOD_MS);
            continue;
        }

        card_id->id_len = ret;

        esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_CARD_SCANNED, card_id, sizeof(card_id_t)+card_id->id_len, portMAX_DELAY);

        vTaskDelay(CARD_SCAN_INTERVAL_MS/portTICK_PERIOD_MS);
    }
}