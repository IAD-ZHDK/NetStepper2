#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "encoder.h"

// https://github.com/PaulStoffregen/Encoder/blob/master/Encoder.h

#define ENCODER_A_SEL GPIO_SEL_22
#define ENCODER_B_SEL GPIO_SEL_21

#define ENCODER_A_NUM GPIO_NUM_22
#define ENCODER_B_NUM GPIO_NUM_21

#define ENCODER_RESOLUTION 400
#define ENCODER_ACCURACY 40

static encoder_handler_t encoder_handler;

static volatile uint8_t encoder_state = 0;
static volatile int32_t encoder_total = 0;
static volatile int32_t encoder_last = 0;

static QueueHandle_t encoder_queue;

static void encoder_isr(void *_) {
  // read pins
  int a = gpio_get_level(ENCODER_A_NUM);
  int b = gpio_get_level(ENCODER_B_NUM);

  // calculate encoder change
  uint8_t state = (uint8_t)(encoder_state & 3);
  if (a) state |= 4;
  if (b) state |= 8;

  // save new state
  encoder_state = (state >> 2);

  // save relative change
  switch (state) {
    case 1:
    case 7:
    case 8:
    case 14: {
      encoder_total += 1;
      break;
    }
    case 2:
    case 4:
    case 11:
    case 13: {
      encoder_total -= 1;
      break;
    }
    case 3:
    case 12: {
      encoder_total += 2;
      break;
    }
    case 6:
    case 9: {
      encoder_total -= 2;
      break;
    }
    default: {
      // no movement
    }
  }

  // check if an update is due
  int32_t diff = ENCODER_RESOLUTION / ENCODER_ACCURACY;
  if (encoder_total > encoder_last + diff || encoder_total < encoder_last - diff) {
    // remember value
    encoder_last = encoder_total;

    // send on queue
    xQueueSendFromISR(encoder_queue, (void *)&encoder_total, NULL);
  }
}

static void encoder_task(void *_) {
  // loop forever
  for (;;) {
    // get all button events
    int32_t n;
    while (xQueueReceive(encoder_queue, &n, portMAX_DELAY) == pdTRUE) {
      // calculate real position
      double pos = n;
      pos /= ENCODER_RESOLUTION;

      // call handler with position
      encoder_handler(pos);
    }
  }
}

void encoder_init(encoder_handler_t handler) {
  // set handler
  encoder_handler = handler;

  // create queue
  encoder_queue = xQueueCreate(16, sizeof(int32_t));

  // configure rotation pins
  gpio_config_t rc;
  rc.pin_bit_mask = ENCODER_A_SEL | ENCODER_B_SEL;
  rc.mode = GPIO_MODE_INPUT;
  rc.intr_type = GPIO_INTR_ANYEDGE;
  rc.pull_up_en = GPIO_PULLUP_ENABLE;
  rc.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&rc);

  // preset state
  if (gpio_get_level(ENCODER_A_NUM)) encoder_state |= 1;
  if (gpio_get_level(ENCODER_B_NUM)) encoder_state |= 2;

  // add interrupt handlers
  gpio_isr_handler_add(ENCODER_A_NUM, encoder_isr, NULL);
  gpio_isr_handler_add(ENCODER_B_NUM, encoder_isr, NULL);

  // run task
  xTaskCreatePinnedToCore(encoder_task, "enc", 2048, NULL, 2, NULL, 1);
}
