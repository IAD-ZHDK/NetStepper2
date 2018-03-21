#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "buttons.h"
#include "ext.h"

#define BUTTONS_CW_SEL GPIO_SEL_33
#define BUTTONS_CCW_SEL GPIO_SEL_26
#define BUTTONS_STOP_SEL GPIO_SEL_12
#define BUTTONS_HOME_SEL GPIO_SEL_27

#define BUTTONS_CW_NUM GPIO_NUM_33
#define BUTTONS_CCW_NUM GPIO_NUM_26
#define BUTTONS_STOP_NUM GPIO_NUM_12
#define BUTTONS_HOME_NUM GPIO_NUM_27

typedef struct {
  buttons_type_t type;
  bool state;
} buttons_event_t;

static buttons_handler_t buttons_handler;

static QueueHandle_t buttons_queue;

static volatile bool buttons_state[4] = {0, 0, 0, 0};

static void buttons_isr(void* arg) {
  // get type
  buttons_type_t type = (buttons_type_t)arg;

  // prepare state
  bool state = false;

  // check
  switch (type) {
    case BUTTONS_TYPE_CW:
      state = gpio_get_level(BUTTONS_CW_NUM) == 1;
      break;
    case BUTTONS_TYPE_CCW:
      state = gpio_get_level(BUTTONS_CCW_NUM) == 1;
      break;
    case BUTTONS_TYPE_STOP:
      state = gpio_get_level(BUTTONS_STOP_NUM) == 1;
      break;
    case BUTTONS_TYPE_HOME:
      state = gpio_get_level(BUTTONS_HOME_NUM) == 1;
      break;
  }

  // skip event if same as last time
  if (buttons_state[type] == state) {
    return;
  }

  // save state
  buttons_state[type] = state;

  // allocate event
  buttons_event_t event = {type : type, state : state};

  // send event
  xQueueSendFromISR(buttons_queue, &event, NULL);
}

void buttons_task() {
  // get all button events
  buttons_event_t event;
  while (xQueueReceive(buttons_queue, &event, 0) == pdTRUE) {
    // call handler with event
    buttons_handler(event.type, event.state);
  }
}

void buttons_init(buttons_handler_t handler) {
  // set handler
  buttons_handler = handler;

  // create queue
  buttons_queue = xQueueCreate(16, sizeof(buttons_event_t));

  // configure button pins
  gpio_config_t bc;
  bc.pin_bit_mask = BUTTONS_CW_SEL | BUTTONS_CCW_SEL | BUTTONS_STOP_SEL | BUTTONS_HOME_SEL;
  bc.mode = GPIO_MODE_INPUT;
  bc.intr_type = GPIO_INTR_ANYEDGE;
  bc.pull_up_en = GPIO_PULLUP_DISABLE;
  bc.pull_down_en = GPIO_PULLDOWN_ENABLE;
  gpio_config(&bc);

  // install gpio interrupt service
  gpio_install_isr_service(0);

  // add interrupt handler
  gpio_isr_handler_add(BUTTONS_CW_NUM, buttons_isr, (void*)BUTTONS_TYPE_CW);
  gpio_isr_handler_add(BUTTONS_CCW_NUM, buttons_isr, (void*)BUTTONS_TYPE_CCW);
  gpio_isr_handler_add(BUTTONS_STOP_NUM, buttons_isr, (void*)BUTTONS_TYPE_STOP);
  gpio_isr_handler_add(BUTTONS_HOME_NUM, buttons_isr, (void*)BUTTONS_TYPE_HOME);

  // run task
  naos_run("buttons_task", 2048, buttons_task);
}
