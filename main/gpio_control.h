#pragma once

#include <stdbool.h>

void gpio_control_init();
void goip_control_set_logout_button_polarity(bool invert);
void gpio_set_relay(bool on);
