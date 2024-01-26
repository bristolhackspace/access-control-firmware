
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <string.h>

#include "pn532.h"
#include "app_events.h"

static const char* TAG = "pn532";

#define BUF_SIZE (1024)
#define CARD_SCAN_INTERVAL_MS 2000

static const uart_port_t uart_num = UART_NUM_0;
static const int tx_pin = UART_PIN_NO_CHANGE;
static const int rx_pin = UART_PIN_NO_CHANGE;
static const TickType_t default_timeout = 500/portTICK_PERIOD_MS;

static uint8_t frame_buffer[300];
static StaticTask_t pn532_poll_task;
static StackType_t pn532_poll_task_stack[CONFIG_NFC_TASK_STACK_SIZE];

int pn532_write_frame(uint8_t* data, size_t data_len);
int pn532_send_command(uint8_t command, uint8_t* data, size_t data_len, TickType_t ack_timeout);
int pn532_get_response(uint8_t command, uint8_t* data, size_t data_len, TickType_t rsp_timeout);
void pn532_flush_until_empty(void);
void pn532_poll_task_handler(void* params);

esp_err_t pn532_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));


    xTaskCreateStatic(pn532_poll_task_handler,
                      "nfc_task",
                      CONFIG_NFC_TASK_STACK_SIZE,
                      NULL,
                      tskIDLE_PRIORITY,
                      pn532_poll_task_stack,
                      &pn532_poll_task);

    return ESP_OK;
}

void pn532_wakeup(void)
{
    uint8_t data[] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uart_write_bytes(uart_num, data, sizeof(data));
}

int pn532_send_command(uint8_t command, uint8_t* data, size_t data_len, TickType_t ack_timeout)
{
    ESP_LOGD(TAG, "pn532_send_command %x", (unsigned int)command);
    uint8_t checksum = 0;

    frame_buffer[0] = PN532_PREAMBLE;
    frame_buffer[1] = PN532_STARTCODE1;
    frame_buffer[2] = PN532_STARTCODE2;

    uint8_t frame_len = data_len + 2; // TFI + PD0
    frame_buffer[3] = frame_len;
    frame_buffer[4] = ~frame_len + 1;

    frame_buffer[5] = PN532_HOSTTOPN532; //TFI
    checksum += frame_buffer[5];
    frame_buffer[6] = command;
    checksum += frame_buffer[6];

    for(size_t i=0; i<data_len;i++)
    {
        frame_buffer[7+i] = data[i];
        checksum += data[i];
    }
    checksum = ~checksum + 1;
    frame_buffer[7+data_len] = checksum;
    frame_buffer[7+data_len+1] = PN532_POSTAMBLE;

    uart_write_bytes(uart_num, frame_buffer, (7+data_len+1)+1);

    ESP_LOGD(TAG, "pn532_send_command wait for ack");
    uint8_t ack_frame[6] = {0};
    int rc = uart_read_bytes(uart_num, ack_frame, sizeof(ack_frame), ack_timeout);
    if (rc < 0) {
        return rc;
    }
    return ESP_OK;
}

int pn532_get_response(uint8_t command, uint8_t* data, size_t data_len, TickType_t rsp_timeout)
{
    ESP_LOGD(TAG, "pn532_get_response %x", (unsigned int)command);
    int rc = 0;
    uint8_t header;
    while (1)
    {
        rc = uart_read_bytes(uart_num, &header, sizeof(header), rsp_timeout);
        if (rc <= 0) {
            return rc;
        }

        if (header == PN532_PREAMBLE) {
            continue;
        }else if (header == PN532_STARTCODE2) {
            break;
        }else{
            ESP_LOGE(TAG, "Invalid preamble");
            pn532_flush_until_empty();
            return ESP_FAIL;
        }
    }

    uint8_t frame_len[2] = {0};
    rc = uart_read_bytes(uart_num, frame_len, sizeof(frame_len), rsp_timeout);
    if (rc <= 0) {
        ESP_LOGE(TAG, "Incomplete response");
        return ESP_FAIL;
    }
    if (((frame_len[0] + frame_len[1]) & 0xff) != 0) {
        ESP_LOGE(TAG, "Length checksum error");
        pn532_flush_until_empty();
        return ESP_FAIL;
    }

    if (frame_len[0] < 2) {
        ESP_LOGE(TAG, "No data returned");
        pn532_flush_until_empty();
        return ESP_FAIL;
    }

    if (frame_len[0]-2 > data_len) {
        ESP_LOGE(TAG, "Data buffer too small");
        pn532_flush_until_empty();
        return ESP_FAIL;
    }

    uint8_t tfi = 0;
    rc = uart_read_bytes(uart_num, &tfi, 1, rsp_timeout);
    if (rc != 1) {
        ESP_LOGE(TAG, "Incomplete response");
        return ESP_FAIL;
    }
    uint8_t checksum = tfi;
    frame_len[0] -= 1;

    uint8_t got_command = 0;
    rc = uart_read_bytes(uart_num, &got_command, 1, rsp_timeout);
    if (rc != 1) {
        ESP_LOGE(TAG, "Incomplete response");
        return ESP_FAIL;
    }
    checksum += got_command;
    frame_len[0] -= 1;

    if (frame_len[0]) {
        rc = uart_read_bytes(uart_num, data, frame_len[0], rsp_timeout);
        if (rc != frame_len[0]) {
            ESP_LOGE(TAG, "Incomplete response");
            return ESP_FAIL;
        }
        for (size_t i=0; i<frame_len[0]; i++)
        {
            checksum += data[i];
        }
    }


    uint8_t footer[2] = {0};
    rc = uart_read_bytes(uart_num, footer, sizeof(footer), rsp_timeout);
    if (rc != sizeof(footer)) {
        ESP_LOGE(TAG, "Incomplete response");
        return ESP_FAIL;
    }
    checksum += footer[0];
    if (checksum) {
        ESP_LOGE(TAG, "Data checksum error");
        return ESP_FAIL;
    }

    if (tfi != PN532_PN532TOHOST) {
        ESP_LOGE(TAG, "Unexpected frame identifier");
        return ESP_FAIL;
    }

    if (got_command != command+1) {
        ESP_LOGE(TAG, "Incorrect command ID");
        return ESP_FAIL;
    }

    return frame_len[0];
}

void pn532_flush_until_empty(void)
{
    ESP_LOGD(TAG, "Flushing RX buffer");
    size_t data_available;
    do {
        uart_flush(uart_num);
        vTaskDelay(5/portTICK_PERIOD_MS);
        uart_get_buffered_data_len(uart_num, &data_available);
    } while (data_available);
}

esp_err_t pn532_firmware_version(uint8_t* ic, uint8_t* ver, uint8_t* rev, uint8_t* support)
{
    int rc = pn532_send_command(PN532_COMMAND_GETFIRMWAREVERSION, NULL, 0, default_timeout);
    if (rc) {
        return rc;
    }
    uint8_t buf[4] = {0};
    rc = pn532_get_response(PN532_COMMAND_GETFIRMWAREVERSION, buf, 4, default_timeout);
    if (rc < 0) {
        return rc;
    }
    *ic = buf[0];
    *ver = buf[1];
    *rev = buf[2];
    *support = buf[3];
    return ESP_OK;
}

esp_err_t pn532_sam_config(void)
{
    // Send SAM configuration command with configuration for:
    // - 0x01, normal mode
    // - 0x14, timeout 50ms * 20 = 1 second
    // - 0x01, use IRQ pin
    uint8_t sam_config[3] = {0x01, 0x14, 0x01};
    int rc = pn532_send_command(PN532_COMMAND_SAMCONFIGURATION, sam_config, sizeof(sam_config), default_timeout);
    if (rc) {
        return rc;
    }
    rc = pn532_get_response(PN532_COMMAND_SAMCONFIGURATION, NULL, 0, default_timeout);
    if (rc < 0) {
        return rc;
    }
    return ESP_OK;
}

esp_err_t pn532_listen_for_passive_target(void)
{
    uint8_t inlist_params[] = {0x01, PN532_MIFARE_ISO14443A};
    return pn532_send_command(PN532_COMMAND_INLISTPASSIVETARGET, inlist_params, sizeof(inlist_params), default_timeout);
}

int pn532_get_passive_target(uint8_t* data, size_t data_len, TickType_t timeout)
{
    uint8_t rsp_buffer[30] = {0};
    int rc = pn532_get_response(PN532_COMMAND_INLISTPASSIVETARGET, rsp_buffer, sizeof(rsp_buffer), timeout);
    if (rc <= 0) {
        return rc;
    }

    if (rc < 10) {
        ESP_LOGE(TAG, "Get target response too small");
        return -ESP_ERR_INVALID_SIZE;
    }

    if (rsp_buffer[0] != 0x01) {
        ESP_LOGE(TAG, "More than one tag detected");
        return -ESP_ERR_INVALID_SIZE;
    }

    uint8_t uid_len = rsp_buffer[5];

    if (uid_len > 7) {
        ESP_LOGE(TAG, "Found card with unexpectedly long UID");
        return -ESP_ERR_INVALID_SIZE;
    }

    if (uid_len > data_len) {
        ESP_LOGE(TAG, "Data buffer not large enough for tag UID");
        return -ESP_ERR_INVALID_SIZE;
    }
    memcpy(data, &rsp_buffer[6], uid_len);
    return uid_len;

}

void pn532_full_scan_sequence(void)
{
    static uint8_t loop_count = 0;
    int ret = 0;

    // Re-initialise the PN532 every 16 interations just in case it has failed for whatever reason.
    if (loop_count % 16 == 0) {
        ESP_LOGI(TAG, "Reconfiguring pn532");
        pn532_wakeup();
        ret = pn532_sam_config();
        if (ret) {
            ESP_LOGE(TAG, "Error re-configuring pn532");
            return;
        }
    }
    loop_count += 1;


    ESP_LOGI(TAG, "Scanning for card");
    ret = pn532_listen_for_passive_target();
    if (ret < 0) {
        ESP_LOGE(TAG, "Error iistening for targets");
        return;
    }

    card_id_t card_id = {0};

    ret = pn532_get_passive_target(card_id.id, sizeof(card_id), CARD_SCAN_INTERVAL_MS/portTICK_PERIOD_MS);
    if (ret == 0) {
        ESP_LOGI(TAG, "No card found");
        return;
    }

    if (ret < 0) {
        ESP_LOGE(TAG, "Error reading card");
        return;
    }

    if (ret != 4) {
        ESP_LOGE(TAG, "Card UID not 4 bytes");
        return;
    }

    app_event_post(APPLICATION_EVENT_CARD_SCANNED,  &card_id, sizeof(card_id), portMAX_DELAY);
}

void pn532_poll_task_handler(void* params)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while(true)
    {
        pn532_full_scan_sequence();

        if(!xTaskDelayUntil(&last_wake_time, CARD_SCAN_INTERVAL_MS/portTICK_PERIOD_MS))
        {
            last_wake_time = xTaskGetTickCount();
        }
    }
}