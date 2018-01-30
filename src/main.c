#include <naos.h>

#include "l6470.h"

void online() {}

void message(const char *topic, uint8_t *payload, size_t len, naos_scope_t scope) {}

void update(const char *param, const char *value) {}

void loop() {}

void offline() {
  // TODO: Stop motor.
}

static naos_config_t config = {
    .device_type = "NetStepper2",
    .firmware_version = "0.1.0",
    .online_callback = online,
    .update_callback = update,
    .message_callback = message,
    .loop_callback = loop,
    .offline_callback = offline,
    .crash_on_mqtt_failures = true,
};

void app_main() {
  // initialize naos
  naos_init(&config);

  // initialize l6470
  l6470_init();

  // get status
  l6470_status_t status = l6470_get_status();
  naos_log("status: %b", status.data);
}
