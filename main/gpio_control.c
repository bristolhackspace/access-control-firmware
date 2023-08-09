#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_system.h"

#include "main.h"

#include "gpio_control.h"

#define RELAY_GPIO_PIN 26
#define BUTTON_GPIO_PIN 25
#define PWR_DETECT_GPIO_PIN 4
#define DEBOUNCE_INTERVAL_MS 250

static void IRAM_ATTR button_isr_handler(void* arg)
{
    static TickType_t last_interrupt_tick = 0;

    TickType_t current_interrupt_tick = xTaskGetTickCount();
    TickType_t delta = current_interrupt_tick - last_interrupt_tick;

    if (delta*portTICK_PERIOD_MS < DEBOUNCE_INTERVAL_MS)
    {
        return;
    }

    last_interrupt_tick = current_interrupt_tick;

    uint32_t gpio_num = (uint32_t) arg;
    if (BUTTON_GPIO_PIN == gpio_num)
    {
        esp_event_isr_post(APPLICATION_EVENT, APPLICATION_EVENT_BUTTON_PRESS, NULL, 0, NULL);
    }
}

void gpio_control_init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << RELAY_GPIO_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);


    uint32_t button_gpio_pin = BUTTON_GPIO_PIN;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << button_gpio_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(button_gpio_pin, button_isr_handler, (void*) button_gpio_pin);

    uint32_t pwr_detect_gpio_pin = PWR_DETECT_GPIO_PIN;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << pwr_detect_gpio_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

}

void gpio_set_relay(bool on)
{
    gpio_set_level(RELAY_GPIO_PIN, on?1:0);
}