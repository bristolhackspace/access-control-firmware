#pragma once

#include "esp_event.h"
#include <stdbool.h>

ESP_EVENT_DECLARE_BASE(APPLICATION_EVENT);

enum {
    APPLICATION_EVENT_CARD_SCANNED,
    APPLICATION_EVENT_BUTTON_PRESS,
    APPLICATION_EVENT_STATUS,
    APPLICATION_EVENT_RESPONSE_JSON,
    APPLICATION_EVENT_BLINK_ERROR,
    APPLICATION_EVENT_FIRMWARE_UPDATE,
};

#define CARD_ID_MAX_LEN 8

bool is_unlocked(void);
bool is_update_pending(void);
bool is_in_use(void);