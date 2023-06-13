#pragma once

#include "esp_err.h"

esp_err_t http_api_init(void);
esp_err_t http_api_unlock(uint8_t* card_id, int len);
esp_err_t http_api_lock(void);
esp_err_t http_api_status(cJSON *status);
esp_err_t http_api_ota(const char* url);