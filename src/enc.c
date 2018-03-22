#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "enc.h"
#include "ext.h"

// https://github.com/PaulStoffregen/Encoder/blob/master/Encoder.h

#define ENC_A_SEL GPIO_SEL_15
#define ENC_B_SEL GPIO_SEL_22

#define ENC_A_NUM GPIO_NUM_15
#define ENC_B_NUM GPIO_NUM_22

#define ENC_RESOLUTION 400
#define ENC_ACCURACY 40

static enc_handler_t enc_handler;

static volatile uint8_t enc_state = 0;
static volatile int32_t enc_total = 0;
static volatile int32_t enc_last = 0;

static QueueHandle_t enc_queue;

static void enc_isr(void *_) {
  // read pins
  int a = gpio_get_level(ENC_A_NUM);
  int b = gpio_get_level(ENC_B_NUM);

  // calculate encoder change
  uint8_t state = (uint8_t)(enc_state & 3);
  if (a) state |= 4;
  if (b) state |= 8;

  // save new state
  enc_state = (state >> 2);

  // save relative change
  switch (state) {
    case 1:
    case 7:
    case 8:
    case 14: {
      enc_total += 1;
      break;
    }
    case 2:
    case 4:
    case 11:
    case 13: {
      enc_total -= 1;
      break;
    }
    case 3:
    case 12: {
      enc_total += 2;
      break;
    }
    case 6:
    case 9: {
      enc_total -= 2;
      break;
    }
    default: {
      // no movement
    }
  }

  // check if an update is due
  int32_t diff = ENC_RESOLUTION / ENC_ACCURACY;
  if (enc_total > enc_last + diff || enc_total < enc_last - diff) {
    // remember value
    enc_last = enc_total;

    // send on queue
    xQueueSendFromISR(enc_queue, (void *)&enc_total, NULL);
  }
}

static void enc_task(void *_) {
  // loop forever
  for (;;) {
    // get all button events
    int32_t n;
    while (xQueueReceive(enc_queue, &n, portMAX_DELAY) == pdTRUE) {
      // calculate real position
      double pos = n;
      pos /= ENC_RESOLUTION;

      // call handler with position
      enc_handler(pos);
    }
  }
}

void enc_init(enc_handler_t handler) {
  // set handler
  enc_handler = handler;

  // create queue
  enc_queue = xQueueCreate(16, sizeof(int32_t));

  // configure rotation pins
  gpio_config_t rc;
  rc.pin_bit_mask = ENC_A_SEL | ENC_B_SEL;
  rc.mode = GPIO_MODE_INPUT;
  rc.intr_type = GPIO_INTR_ANYEDGE;
  rc.pull_up_en = GPIO_PULLUP_ENABLE;
  rc.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&rc);

  // preset state
  if (gpio_get_level(ENC_A_NUM)) enc_state |= 1;
  if (gpio_get_level(ENC_B_NUM)) enc_state |= 2;

  // add interrupt handlers
  gpio_isr_handler_add(ENC_A_NUM, enc_isr, NULL);
  gpio_isr_handler_add(ENC_B_NUM, enc_isr, NULL);

  // run task
  xTaskCreatePinnedToCore(enc_task, "enc", 2048, NULL, 2, NULL, 1);
}
