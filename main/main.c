
#include <string.h>
#include <alloca.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
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

    gpio_control_init();

    app_event_loop_init();

    while (1)
    {
        esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_STATUS, NULL, 0, portMAX_DELAY);
        pn532_full_scan_sequence();
    }
}