#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"

#include "app_events.h"

#include "main.h"

#include "gpio_control.h"

#define RELAY_GPIO_PIN (26)
#define LED_GPIO_PIN (19)
#define BUTTON_GPIO_PIN 0
#define DEBOUNCE_INTERVAL_MS 100

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH1_GPIO       LED_GPIO_PIN
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

#define LEDC_CHOSEN_DUTY LEDC_TIMER_13_BIT
#define LEDC_MAX_DUTY (1 << LEDC_CHOSEN_DUTY)

#define LEDC_FADE_DURATION (500)

static StaticTask_t led_task;
static StackType_t led_task_stack[CONFIG_LED_TASK_STACK_SIZE];

static const char* TAG = "gpio";

static StaticTimer_t debounce_timer_buffer;
static TimerHandle_t debounce_timer;
static bool last_button_state = false;
static enum led_mode_type led_mode = LED_MODE_IDLE;
static bool led_error_flash = false;

static const ledc_channel_config_t led_config = {
    .channel    = LEDC_HS_CH1_CHANNEL,
    .duty       = 0,
    .gpio_num   = LEDC_HS_CH1_GPIO,
    .speed_mode = LEDC_HS_MODE,
    .hpoint     = 0,
    .timer_sel  = LEDC_HS_TIMER,
    .flags.output_invert = 0
};

void led_task_handler(void* params);

static void IRAM_ATTR button_isr_handler(void* arg)
{
    // Start the timer to check the GPIO after the debounce time
    BaseType_t wake_higher_prio_task = pdFALSE;
    xTimerStartFromISR(debounce_timer, &wake_higher_prio_task);
}

static void debounce_timer_handler(TimerHandle_t timer)
{
    bool current_button_state = (!gpio_get_level(BUTTON_GPIO_PIN));
    if(current_button_state != last_button_state)
    {
        last_button_state = current_button_state;
        
        if (current_button_state)
        {
            ESP_LOGI(TAG, "Button press");
            app_event_post_isr(APPLICATION_EVENT_BUTTON_PRESS, NULL, 0);
        }
        else
        {
            ESP_LOGI(TAG, "Button release");
            app_event_post_isr(APPLICATION_EVENT_BUTTON_RELEASE, NULL, 0);
        }


        // esp_event_isr_post(APPLICATION_EVENT, APPLICATION_EVENT_BUTTON_PRESS, NULL, 0, NULL);
    }
}

void gpio_control_init()
{
    gpio_config_t relay_conf = {0};
    relay_conf.intr_type = GPIO_INTR_DISABLE;
    relay_conf.mode = GPIO_MODE_OUTPUT;
    relay_conf.pin_bit_mask = (1ULL << RELAY_GPIO_PIN);
    relay_conf.pull_down_en = 0;
    relay_conf.pull_up_en = 0;
    gpio_config(&relay_conf);

    gpio_config_t btn_conf = {0};
    btn_conf.intr_type = GPIO_INTR_ANYEDGE;
    btn_conf.pin_bit_mask = (1ULL << BUTTON_GPIO_PIN);
    btn_conf.mode = GPIO_MODE_INPUT;
    btn_conf.pull_down_en = 1;
    gpio_config(&btn_conf);
    debounce_timer = xTimerCreateStatic("debounceTimer",
                                DEBOUNCE_INTERVAL_MS/portTICK_PERIOD_MS,
                                false,
                                NULL,
                                debounce_timer_handler,
                                &debounce_timer_buffer);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO_PIN, button_isr_handler, NULL);


    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_CHOSEN_DUTY, // resolution of PWM duty
        .freq_hz = 4000,                      // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config(&led_config);
    ledc_fade_func_install(0);

    xTaskCreateStatic(led_task_handler,
                      "led_task",
                      CONFIG_LED_TASK_STACK_SIZE,
                      NULL,
                      tskIDLE_PRIORITY,
                      led_task_stack,
                      &led_task);

}

void gpio_set_relay(bool on)
{
    gpio_set_level(RELAY_GPIO_PIN, on?1:0);
}

void gpio_set_led_mode(enum led_mode_type mode)
{
    led_mode = mode;
}

void gpio_led_error_flash()
{
    led_error_flash = true;
}

void blink_helper(int count, TickType_t on_time, TickType_t off_time)
{
    for(int i=0;i<count;i++)
    {
        ledc_set_duty_and_update(led_config.speed_mode,
                                 led_config.channel,
                                 LEDC_MAX_DUTY-1,
                                 0);
        vTaskDelay(on_time);
        ledc_set_duty_and_update(led_config.speed_mode,
                                 led_config.channel,
                                 0,
                                 0);
        vTaskDelay(off_time);
    }
}

void led_task_handler(void* params)
{
    while(true)
    {
        enum led_mode_type active_mode = led_mode;
        int error_blinks = 1;
        if(led_error_flash)
        {
            active_mode = LED_MODE_ERROR;
            error_blinks = 5;
            led_error_flash = false;
        }
        switch (active_mode)
        {
        case LED_MODE_OFF:
            ledc_set_duty_and_update(led_config.speed_mode,
                                     led_config.channel,
                                     0,
                                     0);
            vTaskDelay(250/portTICK_PERIOD_MS);
            break;
        case LED_MODE_IDLE:
            ledc_set_fade_time_and_start(led_config.speed_mode,
                                         led_config.channel,
                                         LEDC_MAX_DUTY-1,
                                         LEDC_FADE_DURATION,
                                         LEDC_FADE_WAIT_DONE);
            ledc_set_fade_time_and_start(led_config.speed_mode,
                                         led_config.channel,
                                         0,
                                         LEDC_FADE_DURATION,
                                         LEDC_FADE_WAIT_DONE);
            break;
        case LED_MODE_ON:
            ledc_set_duty_and_update(led_config.speed_mode,
                                     led_config.channel,
                                     LEDC_MAX_DUTY-1,
                                     0);
            vTaskDelay(250/portTICK_PERIOD_MS);
            break;
        case LED_MODE_ERROR:
            blink_helper(error_blinks, 250/portTICK_PERIOD_MS, 250/portTICK_PERIOD_MS);
            break;
        case LED_MODE_INDUCT:
            blink_helper(1, 100/portTICK_PERIOD_MS, 900/portTICK_PERIOD_MS);
            break;
        default:
            
            break;
        }
        
    }
}