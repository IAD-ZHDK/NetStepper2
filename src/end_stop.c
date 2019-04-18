#include <driver/adc.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "end_stop.h"

#define END_STOP_1_SEL GPIO_SEL_32
#define END_STOP_2_SEL GPIO_SEL_33

#define END_STOP_1_NUM GPIO_NUM_32
#define END_STOP_2_NUM GPIO_NUM_33

typedef struct {
  end_stop_pin_t pin;
  bool state;
} end_stop_event_t;

static bool end_stop_analog = false;

static end_stop_handler_t end_stop_handler;

static QueueHandle_t end_stop_queue;

static volatile bool end_stop_state[2] = {0, 0};

static void end_stop_isr(void* arg) {
  // get type
  end_stop_pin_t pin = (end_stop_pin_t)arg;

  // prepare state
  bool state = false;

  // check
  switch (pin) {
    case END_STOP_PIN_1:
      state = gpio_get_level(END_STOP_1_NUM) == 1;
      break;
    case END_STOP_PIN_2:
      state = gpio_get_level(END_STOP_2_NUM) == 1;
      break;
  }

  // skip event if same as last time
  if (end_stop_state[pin] == state) {
    return;
  }

  // save state
  end_stop_state[pin] = state;

  // allocate event
  end_stop_event_t event = {.pin = pin, .state = state};

  // send event
  xQueueSendFromISR(end_stop_queue, &event, NULL);
}

void end_stop_task(void* _) {
  // loop forever
  for (;;) {
    // get all end stop events
    end_stop_event_t event;
    while (xQueueReceive(end_stop_queue, &event, portMAX_DELAY) == pdTRUE) {
      // call handler with event
      end_stop_handler(event.pin, event.state);
    }
  }
}

void end_stop_init(end_stop_handler_t handler, bool analog) {
  // set flag
  end_stop_analog = analog;

  // setup adc
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_11db);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_11db);

  // skip isr setup in analog mode
  if (analog) {
    return;
  }

  // set handler
  end_stop_handler = handler;

  // create queue
  end_stop_queue = xQueueCreate(16, sizeof(end_stop_event_t));

  // configure end stop pins
  gpio_config_t esc;
  esc.pin_bit_mask = END_STOP_1_SEL | END_STOP_2_SEL;
  esc.mode = GPIO_MODE_INPUT;
  esc.intr_type = GPIO_INTR_ANYEDGE;
  esc.pull_up_en = GPIO_PULLUP_DISABLE;
  esc.pull_down_en = GPIO_PULLDOWN_ENABLE;
  gpio_config(&esc);

  // add interrupt handler
  gpio_isr_handler_add(END_STOP_1_NUM, end_stop_isr, (void*)END_STOP_PIN_1);
  gpio_isr_handler_add(END_STOP_2_NUM, end_stop_isr, (void*)END_STOP_PIN_2);

  // run task
  xTaskCreatePinnedToCore(end_stop_task, "end_stop", 2048, NULL, 2, NULL, 1);
}

double end_stop_read_1() {
  // return zero if not in analog mode
  if (!end_stop_analog) {
    return 0;
  }

  // read sensor
  return adc1_get_voltage(ADC1_CHANNEL_4) / 3.3 * 3.6;
}

double end_stop_read_2() {
  // return zero if not in analog mode
  if (!end_stop_analog) {
    return 0;
  }

  // read sensor
  return adc1_get_voltage(ADC1_CHANNEL_5) / 3.3 * 3.6;
}
