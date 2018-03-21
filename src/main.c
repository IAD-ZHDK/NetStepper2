#include <art32/strconv.h>
#include <naos.h>
#include <string.h>

#include "buttons.h"
#include "l6470.h"

static void approach_home() {
  // get info
  uint32_t speed = l6470_get_speed();
  l6470_status_t status = l6470_get_status();

  // change to run command and wait until speed is reached
  l6470_run(status.dir, speed);
  l6470_wait();

  // set new position
  l6470_go_home();
}

static void approach_target(int32_t pos) {
  // get info
  uint32_t speed = l6470_get_speed();
  l6470_status_t status = l6470_get_status();

  // change to run command and wait until speed is reached
  l6470_run(status.dir, speed);
  l6470_wait();

  // set new position
  l6470_go_to(pos);
}

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

    // approach target
    approach_target(pos);
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
    // approach home
    approach_home();
  }
}

void press(buttons_type_t type, bool pressed) {
  // prepare home press counter
  static uint32_t home_press = 0;

  // turn forward if cw is released
  if (type == BUTTONS_TYPE_CW && !pressed) {
    // run forward
    l6470_run(L6470_DIR_FWD, l6470_calc_speed(500));
  }

  // turn backwards if ccw is released
  if (type == BUTTONS_TYPE_CCW && !pressed) {
    // run backward
    l6470_run(L6470_DIR_REV, l6470_calc_speed(500));
  }

  // stop if stop is released
  if (type == BUTTONS_TYPE_STOP && !pressed) {
    // stop motor
    l6470_soft_stop();
  }

  // set time if pressed
  if (type == BUTTONS_TYPE_HOME && pressed) {
    home_press = naos_millis();
  }

  if (type == BUTTONS_TYPE_HOME && !pressed) {
    // get time difference
    uint32_t diff = naos_millis() - home_press;

    // approach home if buttons has been released quickly
    if (diff < 2000) {
      // approach home
      approach_home();

      return;
    }

    // otherwise reset position and stop

    // set home pos
    l6470_reset_pos();

    // stop motor
    l6470_soft_stop();
  }
}

void offline() {
  // stop motor
  l6470_soft_stop();
}

static naos_config_t config = {
    .device_type = "NetStepper2",
    .firmware_version = "0.1.0",
    .online_callback = online,
    .message_callback = message,
    .offline_callback = offline,
    .crash_on_mqtt_failures = true,
};

void app_main() {
  // initialize naos
  naos_init(&config);

  // initialize buttons
  buttons_init(press);

  // initialize l6470
  l6470_init();

  // set step mode
  l6470_set_step_mode(L6470_STEP_MODE_1_128);

  // set max speed to 500 steps/s
  l6470_set_max_speed(l6470_calc_max_speed(500));
}
