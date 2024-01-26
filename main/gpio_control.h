#pragma once

#include <stdbool.h>

enum led_mode_type {
    LED_MODE_OFF,
    LED_MODE_IDLE,
    LED_MODE_ON,
    LED_MODE_ERROR,
    LED_MODE_INDUCT,
    LED_MODE_MAX,
};

void gpio_control_init();
void goip_control_set_logout_button_polarity(bool invert);
void gpio_set_relay(bool on);
void gpio_set_led_mode(enum led_mode_type mode);
void gpio_led_error_flash();