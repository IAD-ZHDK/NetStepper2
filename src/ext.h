#include <stdint.h>

typedef void (*naos_task_t)();

void naos_run(const char* name, uint32_t stack, naos_task_t task);
