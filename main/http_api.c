
#include "esp_http_client.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>

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
                cJSON *current_element = NULL;

                cJSON_ArrayForEach(current_element, root) {
                    if (current_element->string == NULL) {
                        continue;
                    }
                    if (strcmp(current_element->string, "unlocked") == 0) {
                        int unlocked = 0;
                        if (cJSON_IsTrue(current_element)) {
                            unlocked = 1;
                        } else if (cJSON_IsFalse(current_element)) {
                            unlocked = 0;
                        } else {
                            continue;
                        }
                        esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_SET_UNLOCKED, &unlocked, sizeof(unlocked), portMAX_DELAY);
                    } else if (strcmp(current_element->string, "firmware_update") == 0) {
                        char* update_url = cJSON_GetStringValue(current_element);
                        if (update_url == NULL) {
                            ESP_LOGE(TAG, "firmware_update must be a string");
                            continue;
                        }
                        esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_FIRMWARE_UPDATE, update_url, strlen(update_url)+1, portMAX_DELAY);
                    } else if (strcmp(current_element->string, "reboot") == 0) {
                        if (cJSON_IsTrue(current_element)) {
                            esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_REBOOT, NULL, 0, portMAX_DELAY);
                        }
                    }
                }

                cJSON_Delete(root);
            }

            int status_code = esp_http_client_get_status_code(evt->client);
            if (status_code >= HttpStatus_MultipleChoices) {
                esp_event_post(APPLICATION_EVENT, APPLICATION_EVENT_BLINK_ERROR, NULL, 0, portMAX_DELAY);
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
    for (int i=0; i<len; i++)
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
