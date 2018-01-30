#include <naos.h>

static naos_config_t config = {.device_type = "NetStepper2", .firmware_version = "0.1.0"};

void app_main() {
  // initialize naos
  naos_init(&config);
}
