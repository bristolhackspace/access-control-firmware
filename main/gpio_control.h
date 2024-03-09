#pragma once

#include <esp_event.h>
#include <stdbool.h>

ESP_EVENT_DECLARE_BASE(GPIO_EVENTS);

typedef enum {
    GPIO_EVENT_ANY = ESP_EVENT_ANY_ID,
    GPIO_EVENT_NONE,
    GPIO_EVENT_BUTTON_PRESS,
    GPIO_EVENT_BUTTON_RELEASE,
} gpio_event_t;

esp_err_t gpio_control_init(esp_event_loop_handle_t event_handle);
void gpio_set_relay(bool on);