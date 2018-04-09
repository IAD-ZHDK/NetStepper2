#include <driver/ledc.h>

#include "led.h"

#define LED_RED_PIN 14
#define LED_GREEN_PIN 27
#define LED_BLUE_PIN 26

void led_init() {
  // prepare ledc timer config
  ledc_timer_config_t t = {
      .bit_num = LEDC_TIMER_10_BIT, .freq_hz = 5000, .speed_mode = LEDC_HIGH_SPEED_MODE, .timer_num = LEDC_TIMER_0};

  // configure ledc timer
  ESP_ERROR_CHECK(ledc_timer_config(&t));

  // prepare ledc channel config
  ledc_channel_config_t c = {
      .duty = 1024, .intr_type = LEDC_INTR_DISABLE, .speed_mode = LEDC_HIGH_SPEED_MODE, .timer_sel = LEDC_TIMER_0};

  // configure red led
  c.gpio_num = LED_RED_PIN;
  c.channel = LEDC_CHANNEL_0;
  ESP_ERROR_CHECK(ledc_channel_config(&c));

  // configure green led
  c.gpio_num = LED_GREEN_PIN;
  c.channel = LEDC_CHANNEL_1;
  ESP_ERROR_CHECK(ledc_channel_config(&c));

  // configure blue led
  c.gpio_num = LED_BLUE_PIN;
  c.channel = LEDC_CHANNEL_2;
  ESP_ERROR_CHECK(ledc_channel_config(&c));
}

void led_set(uint16_t r, uint16_t g, uint16_t b) {
  // set duties
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 1024 - r));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 1024 - g));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 1024 - b));

  // update channels
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2));
}
