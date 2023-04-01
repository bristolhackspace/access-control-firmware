
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "pn532.h"

static const char* TAG = "pn532";

#define BUF_SIZE (1024)

static const uart_port_t uart_num = UART_NUM_2;
static const int tx_pin = 17;
static const int rx_pin = 16;
static const TickType_t default_timeout = 500/portTICK_PERIOD_MS;

static uint8_t frame_buffer[300];

void pn532_wakeup(void);
int pn532_write_frame(uint8_t* data, size_t data_len);
int pn532_send_command(uint8_t command, uint8_t* data, size_t data_len, TickType_t ack_timeout);
int pn532_get_response(uint8_t command, uint8_t* data, size_t data_len, TickType_t rsp_timeout);
void pn532_flush_until_empty(void);

esp_err_t pn532_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void pn532_wakeup(void)
{
    uint8_t data[] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uart_write_bytes(uart_num, data, sizeof(data));
}

int pn532_send_command(uint8_t command, uint8_t* data, size_t data_len, TickType_t ack_timeout)
{
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
    uart_wait_tx_done(uart_num, (TickType_t)portMAX_DELAY);

    uint8_t ack_frame[6] = {0};
    int rc = uart_read_bytes(uart_num, ack_frame, sizeof(ack_frame), ack_timeout);
    if (rc < 0) {
        return rc;
    }
}

int pn532_get_response(uint8_t command, uint8_t* data, size_t data_len, TickType_t rsp_timeout)
{
    int rc = 0;
    uint8_t header;
    while (1)
    {
        rc = uart_read_bytes(uart_num, &header, sizeof(header), rsp_timeout);
        if (rc < 0) {
            ESP_LOGE(TAG, "Timeout waiting for response");
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
    if (rc < 0) {
        ESP_LOGE(TAG, "Timeout waiting for response");
        return rc;
    }
    if ((frame_len[0] + frame_len[1]) & 0xff != 0) {
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
    if (rc < 0) {
        ESP_LOGE(TAG, "Timeout waiting for response");
        return rc;
    }
    uint8_t checksum = tfi;
    frame_len[0] -= 1;

    uint8_t got_command = 0;
    rc = uart_read_bytes(uart_num, &got_command, 1, rsp_timeout);
    if (rc < 0) {
        ESP_LOGE(TAG, "Timeout waiting for response");
        return rc;
    }
    checksum += got_command;
    frame_len[0] -= 1;

    if (frame_len[0]) {
        rc = uart_read_bytes(uart_num, data, frame_len[0], rsp_timeout);
        if (rc < 0) {
            ESP_LOGE(TAG, "Timeout waiting for response");
            return rc;
        }
        for (size_t i=0; i<frame_len[0]; i++)
        {
            checksum += data[i];
        }
    }


    uint8_t footer[2] = {0};
    rc = uart_read_bytes(uart_num, footer, sizeof(footer), rsp_timeout);
    if (rc < 0) {
        ESP_LOGE(TAG, "Timeout waiting for response");
        return rc;
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
    if (rc) {
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
    if (rc) {
        return rc;
    }
    return ESP_OK;
}