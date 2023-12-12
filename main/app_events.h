#pragma once

#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(APPLICATION_EVENT);

enum app_event_type {
    APPLICATION_EVENT_CARD_SCANNED,
    APPLICATION_EVENT_BUTTON_PRESS,
    APPLICATION_EVENT_BUTTON_RELEASE,
    APPLICATION_EVENT_STATUS_UPDATE,
    APPLICATION_EVENT_RESPONSE_JSON,
    APPLICATION_EVENT_HTTP_ERROR,
    APPLICATION_EVENT_FIRMWARE_UPDATE,

    APPLICATION_EVENT_MAX,
};

enum app_state {
    APPLICATION_STATE_INITIALISING,
    APPLICATION_STATE_LOCKED,
    APPLICATION_STATE_UNLOCKED,
    APPLICATION_STATE_BUTTON_HOLD,
    APPLICATION_STATE_INDUCTION,
    APPLICATION_STATE_FW_UPDATE,

    APPLICATION_STATE_MAX,
};

void app_event_loop_init();
esp_err_t app_event_post(int32_t event_id, const void* event_data, size_t event_data_size, TickType_t ticks_to_wait);
esp_err_t app_event_post_isr(int32_t event_id, const void* event_data, size_t event_data_size);