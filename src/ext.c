#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ext.h"

static void naos_runner(void* arg) {
  // get task
  naos_task_t task = (naos_task_t)arg;

  // run task forever
  for (;;) {
    task();
  }
}

void naos_run(const char* name, uint32_t stack, naos_task_t task) {
  // start runner with task as the argument
  xTaskCreatePinnedToCore(naos_runner, name, stack, task, 2, NULL, 1);
};
