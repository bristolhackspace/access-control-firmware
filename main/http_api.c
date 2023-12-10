
#include "esp_http_client.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "cJSON.h"
#include <string.h>
#include <stdbool.h>

#include "main.h"
#include "http_api.h"

static const char* TAG = "http_api";

#define MAX_HTTP_OUTPUT_BUFFER 1024
#define BASE_URL CONFIG_BASE_URL
#define MACHINE_MAC_LEN 6

static char mac_str[(MACHINE_MAC_LEN*2)+1] = {0};
static char url[200];

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

esp_err_t _http_generic_event_handler(esp_http_client_event_t *evt)
{
    static char output_buffer[MAX_HTTP_OUTPUT_BUFFER];
    static int output_len = 0;

    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            int copy_len = MIN(evt->data_len, (MAX_HTTP_OUTPUT_BUFFER - output_len));
            if (copy_len) {
                memcpy(output_buffer + output_len, evt->data, copy_len);
            }
            output_len += copy_len;
            break;
        case HTTP_EVENT_ON_FINISH:
            if (output_len) {
                cJSON *root = cJSON_ParseWithLength(output_buffer, output_len);
                if (!cJSON_IsObject(root)) {
                    ESP_LOGE(TAG, "Response is not JSON object");
                    cJSON_Delete(root);
                    break;
                }
                // post a pointer to the cJSON pointer as cJSON handles the allocation
                esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_RESPONSE_JSON, &root, sizeof(cJSON*), portMAX_DELAY);
            }

            int status_code = esp_http_client_get_status_code(evt->client);
            if (status_code >= HttpStatus_MultipleChoices) {
                esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_HTTP_ERROR, NULL, 0, portMAX_DELAY);
            }

            break;
        case HTTP_EVENT_DISCONNECTED:
            output_len = 0;
            break;
        default:
            break;
    }

    return ESP_OK;
}


esp_err_t http_api_init(void)
{
    uint8_t mac[MACHINE_MAC_LEN];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char* mac_str_end = mac_str;
    for (int i=0; i<MACHINE_MAC_LEN; i++)
    {
        sprintf(mac_str_end, "%02hhx", mac[i]);
        mac_str_end += 2;
    }
    return ESP_OK;
}

esp_err_t http_api_unlock(uint8_t* card_id, int len)
{
    esp_err_t err;

    if (len > CARD_ID_MAX_LEN) {
        return ESP_FAIL;
    }

    char card_id_str[(CARD_ID_MAX_LEN*2)+1] = {0};
    char* card_id_str_end = card_id_str;
    for (int i=len-1; i>=0; i--)
    {
        sprintf(card_id_str_end, "%02hhx", card_id[i]);
        card_id_str_end += 2;
    }

    snprintf(url, sizeof(url), BASE_URL "/api/machines/%s/unlock?card_id=%s", mac_str, card_id_str);

    esp_http_client_config_t http_cfg = {.url = url, .event_handler = _http_generic_event_handler};
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);

    err = esp_http_client_perform(client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

esp_err_t http_api_lock(void)
{
    esp_err_t err;

    snprintf(url, sizeof(url), BASE_URL "/api/machines/%s/lock", mac_str);

    esp_http_client_config_t http_cfg = {.url = url, .event_handler = _http_generic_event_handler};
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);

    err = esp_http_client_perform(client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

esp_err_t http_api_status(cJSON *status)
{
    esp_err_t err;

    snprintf(url, sizeof(url), BASE_URL "/api/machines/%s/status", mac_str);

    esp_http_client_config_t http_cfg = {
        .url = url,
        .event_handler = _http_generic_event_handler,
        .method = HTTP_METHOD_POST,
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);

    char* json = cJSON_PrintUnformatted(status);

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json, strlen(json));

    err = esp_http_client_perform(client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    cJSON_free(json);
    return err;
}

#define BUFFSIZE 1024

esp_err_t http_api_ota(const char* url)
{
    esp_err_t err;
    const esp_partition_t *update_partition = NULL;
    esp_ota_handle_t update_handle = 0 ;
    static char ota_write_data[BUFFSIZE + 1] = { 0 };

    esp_http_client_config_t http_cfg = {
        .url = url,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);

    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        goto http_cleanup;
    }

    esp_http_client_fetch_headers(client);

    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%"PRIx32,
             update_partition->subtype, update_partition->address);

    int binary_file_length = 0;

    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        goto ota_cleanup;
    }

    ESP_LOGI(TAG, "Beginning firmware download");
    while (1) {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
        if (data_read < 0) {
            err = data_read;
            ESP_LOGE(TAG, "Data read error");
            goto ota_cleanup;
        } else if (data_read > 0) {
            err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK) {
                goto ota_cleanup;
            }
            binary_file_length += data_read;


        } else if (data_read == 0) {
             if (errno == ECONNRESET || errno == ENOTCONN) {
                ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
                break;
            }
            if (esp_http_client_is_complete_data_received(client) == true) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
        }
    }

    ESP_LOGI(TAG, "Total firmware update size: %d", binary_file_length);
    if (esp_http_client_is_complete_data_received(client) != true) {
        ESP_LOGE(TAG, "Error in receiving complete file");
        goto ota_cleanup;
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        goto http_cleanup;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        goto http_cleanup;
    }

    ESP_LOGI(TAG, "Rebooting application");
    esp_restart();

ota_cleanup:
    esp_ota_abort(update_handle);
http_cleanup:
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return err;
}