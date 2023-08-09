#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_system.h"

#include "main.h"

#include "gpio_control.h"

#define RELAY_GPIO_PIN 26
#define BUTTON_GPIO_PIN 4
#define DEBOUNCE_INTERVAL_MS 100

static StaticTimer_t debounce_timer_buffer;
static TimerHandle_t debounce_timer;
static bool logout_invert = false;

static void IRAM_ATTR button_isr_handler(void* arg)
{
    // Start the timer to check the GPIO after the debounce time
    BaseType_t wake_higher_prio_task = pdFALSE;
    xTimerStartFromISR(debounce_timer, &wake_higher_prio_task);
}

static void debounce_timer_handler(TimerHandle_t timer)
{
    if((!!gpio_get_level(BUTTON_GPIO_PIN)) != logout_invert)
    {
        esp_event_isr_post(APPLICATION_EVENT, APPLICATION_EVENT_BUTTON_PRESS, NULL, 0, NULL);
    }
}

void gpio_control_init()
{
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << RELAY_GPIO_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void goip_control_set_logout_button_polarity(bool invert)
{
    gpio_config_t io_conf = {0};
    logout_invert = invert;
    
    if (invert)
    {
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
    } else {
        io_conf.intr_type = GPIO_INTR_POSEDGE;
    }
    
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    debounce_timer = xTimerCreateStatic("debounceTimer",
                                DEBOUNCE_INTERVAL_MS/portTICK_PERIOD_MS,
                                false,
                                NULL,
                                debounce_timer_handler,
                                &debounce_timer_buffer);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO_PIN, button_isr_handler, NULL);
}

void gpio_set_relay(bool on)
{
    gpio_set_level(RELAY_GPIO_PIN, on?1:0);
}