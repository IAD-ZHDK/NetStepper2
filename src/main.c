#include <art32/strconv.h>
#include <naos.h>
#include <string.h>

#include "l6470.h"

int32_t new_position = 0;
int32_t old_position = 0;

void online() {
  // subscribe to topics
  naos_subscribe("target", 0, NAOS_LOCAL);
}

void message(const char *topic, uint8_t *payload, size_t len, naos_scope_t scope) {
  // make string
  char *str = (char *)payload;

  // handle "target" command
  if (strcmp(topic, "target") == 0 && scope == NAOS_LOCAL) {
    new_position = (int32_t)a32_str2l(str);
  }
}

void update(const char *param, const char *value) {}

void loop() {
  // get info
  uint32_t speed = l6470_get_speed();
  l6470_dir_t dir = l6470_get_status().dir;

  // check pos change
  if (new_position != old_position) {
    // change to run command
    l6470_run(dir, speed);
    // TODO: Wait for finish (busy).

    // set new position
    l6470_go_to(new_position);

    // set new position
    old_position = new_position;
  }

  // log info
  naos_log("speed: %ld, dir: %d", speed, dir);
}

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

  // set step mode
  l6470_set_step_mode(L6470_STEP_MODE_1_128);
}
