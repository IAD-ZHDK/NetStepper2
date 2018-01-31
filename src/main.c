#include <art32/strconv.h>
#include <naos.h>
#include <string.h>

#include "l6470.h"

void online() {
  // subscribe to topics
  naos_subscribe("forward", 0, NAOS_LOCAL);
  naos_subscribe("backward", 0, NAOS_LOCAL);
  naos_subscribe("target", 0, NAOS_LOCAL);
  naos_subscribe("stop", 0, NAOS_LOCAL);
  naos_subscribe("reset", 0, NAOS_LOCAL);
  naos_subscribe("home", 0, NAOS_LOCAL);
}

void message(const char *topic, uint8_t *payload, size_t len, naos_scope_t scope) {
  // make string
  char *str = (char *)payload;

  // handle "forward" command
  if (strcmp(topic, "forward") == 0 && scope == NAOS_LOCAL) {
    // run forward
    l6470_run(L6470_DIR_FWD, l6470_calc_speed(500));
  }

  // handle "backward command
  if (strcmp(topic, "backward") == 0 && scope == NAOS_LOCAL) {
    // run backward
    l6470_run(L6470_DIR_REV, l6470_calc_speed(500));
  }

  // handle "target" command
  if (strcmp(topic, "target") == 0 && scope == NAOS_LOCAL) {
    // get new position
    int32_t pos = (int32_t)a32_str2l(str);

    // get info
    uint32_t speed = l6470_get_speed();
    l6470_status_t status = l6470_get_status();

    // change to run command and wait until speed is reached
    l6470_run(status.dir, speed);
    l6470_wait();

    // set new position
    l6470_go_to(pos);
  }

  // handle "stop" command
  if (strcmp(topic, "stop") == 0 && scope == NAOS_LOCAL) {
    // stop motor
    l6470_soft_stop();
  }

  // handle "reset" command
  if (strcmp(topic, "reset") == 0 && scope == NAOS_LOCAL) {
    // reset position
    l6470_reset_pos();
  }

  // handle "home" command
  if (strcmp(topic, "home") == 0 && scope == NAOS_LOCAL) {
    // get info
    uint32_t speed = l6470_get_speed();
    l6470_status_t status = l6470_get_status();

    // change to run command and wait until speed is reached
    l6470_run(status.dir, speed);
    l6470_wait();

    // set new position
    l6470_go_home();
  }
}

void update(const char *param, const char *value) {}

void loop() {
  // log info info
  // uint32_t speed = l6470_get_speed();
  // l6470_status_t status = l6470_get_status();
  // naos_log("speed: %ld, dir: %d, busy: %d", speed, status.dir, status.busy);
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

  // set max speed to 500 steps/s
  l6470_set_max_speed(l6470_calc_max_speed(500));
}
