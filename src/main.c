#include <art32/numbers.h>
#include <art32/strconv.h>
#include <naos.h>
#include <string.h>
#include <driver/gpio.h>

#include "buttons.h"
#include "encoder.h"
#include "end_stop.h"
#include "l6470.h"
#include "led.h"

// 128 micro stepping results in the smoothest motion
#define STEP_MODE L6470_STEP_MODE_128

// 900 steps per second is the maximum speed before we encounter jitter
#define MAX_SPEED 900

// steppers usually have 200 full steps per revolution (1.8°)
#define FULL_STEPS_PER_REV 200

#define MICRO_STEPS 128

#define GEAR_RATIO 5.18

static naos_status_t current_status = NAOS_DISCONNECTED;

double max_speed = 0;
double acceleration = 0;
double deceleration = 0;

bool blocked = false;

static void set_status() {
  // set led accordingly
  switch (current_status) {
    case NAOS_DISCONNECTED:
      led_set(1024, 0, 0);
      break;
    case NAOS_CONNECTED:
      led_set(0, 0, 1024);
      break;
    case NAOS_NETWORKED:
      if (blocked) {
        led_set(1024, 1024, 0);
      } else {
        led_set(0, 1024, 0);
      }
      break;
  }
}

static void status(naos_status_t status) {
  // set last status
  current_status = status;

  // set new status
  set_status();
}

static void ping() {
  // blink white once
  led_set(1024, 1024, 1024);
  naos_delay(300);

  // set status
  set_status();
}

static void online() {
  // subscribe to topics
  naos_subscribe("forward", 0, NAOS_LOCAL);
  naos_subscribe("backward", 0, NAOS_LOCAL);
  naos_subscribe("target", 0, NAOS_LOCAL);
  naos_subscribe("stop", 0, NAOS_LOCAL);
  naos_subscribe("reset", 0, NAOS_LOCAL);
  naos_subscribe("home", 0, NAOS_LOCAL);
}

static void update(const char *param, const char *value) {
  // handle "max-speed"
  if (strcmp(param, "max-speed") == 0) {
    // constrain value
    max_speed = a32_constrain_d(naos_get_d("max-speed"), 0, MAX_SPEED);

    // set constrained value
    naos_set_d("max-speed", max_speed);

    // set setting
    l6470_set_maximum_speed(l6470_calculate_maximum_speed(max_speed));
  }

  // handle "acceleration"
  if (strcmp(param, "acceleration") == 0) {
    // constrain value
    acceleration = a32_constrain_d(naos_get_d("acceleration"), 0, MAX_SPEED);

    // set constrained value
    naos_set_d("acceleration", acceleration);

    // set setting
    l6470_set_acceleration(l6470_calculate_acceleration(acceleration));
  }

  // handle "deceleration"
  if (strcmp(param, "deceleration") == 0) {
    // constrain value
    deceleration = a32_constrain_d(naos_get_d("deceleration"), 0, MAX_SPEED);

    // set constrained value
    naos_set_d("deceleration", deceleration);

    // set setting
    l6470_parse_deceleration(l6470_calculate_deceleration(deceleration));
  }
}

static void message(const char *topic, uint8_t *payload, size_t len, naos_scope_t scope) {
  // immediately return if blocked
  if (blocked) {
    return;
  }

  // make string
  char *str = (char *)payload;

  // handle "forward" command
  if (strcmp(topic, "forward") == 0 && scope == NAOS_LOCAL) {
    // run forward
    l6470_run(L6470_FORWARD, l6470_calculate_speed(MAX_SPEED));
  }

  // handle "backward command
  if (strcmp(topic, "backward") == 0 && scope == NAOS_LOCAL) {
    // run backward
    l6470_run(L6470_REVERSE, l6470_calculate_speed(MAX_SPEED));
  }

  // handle "target" command
  if (strcmp(topic, "target") == 0 && scope == NAOS_LOCAL) {
    // get target
    double target = a32_str2d(str);

    // calculate real position
    int32_t pos = (int32_t)(target * MICRO_STEPS * FULL_STEPS_PER_REV * GEAR_RATIO);

    // approach target
    l6470_approach_target(pos);
  }

  // handle "stop" command
  if (strcmp(topic, "stop") == 0 && scope == NAOS_LOCAL) {
    // stop motor
    l6470_soft_stop();
  }

  // handle "reset" command
  if (strcmp(topic, "reset") == 0 && scope == NAOS_LOCAL) {
    // reset position
    l6470_reset_position();
  }

  // handle "home" command
  if (strcmp(topic, "home") == 0 && scope == NAOS_LOCAL) {
    // approach home
    l6470_approach_home();
  }
}

static void press(buttons_type_t type, bool pressed) {
  // prepare home press counter
  static uint32_t home_press = 0;
  static uint32_t stop_press = 0;

  // turn forward if cw is released
  if (type == BUTTONS_TYPE_CW && !pressed) {
    // run forward
    l6470_run(L6470_FORWARD, l6470_calculate_speed(MAX_SPEED));
  }

  // turn backwards if ccw is released
  if (type == BUTTONS_TYPE_CCW && !pressed) {
    // run backward
    l6470_run(L6470_REVERSE, l6470_calculate_speed(MAX_SPEED));
  }

  // stop motor and block if stop is pressed
  if (type == BUTTONS_TYPE_STOP && pressed) {
    // stop motor
    l6470_soft_stop();

    // set stop flag
    blocked = true;
    set_status();

    // save time
    stop_press = naos_millis();
  }

  // handle stop button release
  if (type == BUTTONS_TYPE_STOP && !pressed) {
    // get time difference
    uint32_t diff = naos_millis() - stop_press;

    // reset blocked flag if button has been released quickly
    if (diff < 500) {
      // set flag
      blocked = false;
      set_status();

      return;
    }

    // otherwise keep blocked flag set
  }

  // set time if pressed
  if (type == BUTTONS_TYPE_HOME && pressed) {
    home_press = naos_millis();
  }

  // handle home button release
  if (type == BUTTONS_TYPE_HOME && !pressed) {
    // get time difference
    uint32_t diff = naos_millis() - home_press;

    // approach home if buttons has been released quickly
    if (diff < 500) {
      // approach home
      l6470_approach_home();

      return;
    }

    // otherwise reset position and stop

    // set home pos
    l6470_reset_position();

    // stop motor
    l6470_soft_stop();
  }
}

static void position(double p) {
  // naos_log("pos %f", p);
}

static void end_stop(end_stop_pin_t pin, bool on) {
  naos_log("end stop: %d - %d", pin, on);
}

static void offline() {
  // stop motor
  l6470_soft_stop();
}

static naos_config_t config = {
    .device_type = "NetStepper2",
    .firmware_version = "0.2.0",
    .ping_callback = ping,
    .status_callback = status,
    .online_callback = online,
    .update_callback = update,
    .message_callback = message,
    .offline_callback = offline,
    .crash_on_mqtt_failures = true,
};

void app_main() {
  // install gpio interrupt service
  gpio_install_isr_service(0);

  // initialize led
  led_init();

  // initialize buttons
  buttons_init(press);

  // initialize encoder
  encoder_init(position);

  // initialize l6470
  l6470_init();

  // set step mode
  l6470_set_step_mode(STEP_MODE);

  // set full step mode to two times max speed (no full stepping)
  l6470_set_full_step_speed(l6470_calculate_full_step_speed(MAX_SPEED * 2));

  // reset minimum speed
  l6470_set_minimum_speed(l6470_calculate_minimum_speed(0));

  // initialize naos
  naos_init(&config);

  // initialize end stops
  end_stop_init(end_stop);

  // ensure parameters
  naos_ensure_d("max-speed", MAX_SPEED);
  naos_ensure_d("acceleration", MAX_SPEED / 2);
  naos_ensure_d("deceleration", MAX_SPEED / 2);

  // get speeds
  max_speed = a32_constrain_d(naos_get_d("max-speed"), 0, MAX_SPEED);
  acceleration = a32_constrain_d(naos_get_d("acceleration"), 0, MAX_SPEED);
  deceleration = a32_constrain_d(naos_get_d("deceleration"), 0, MAX_SPEED);

  // set speeds
  l6470_set_maximum_speed(l6470_calculate_maximum_speed(max_speed));
  l6470_set_acceleration(l6470_calculate_acceleration(acceleration));
  l6470_set_deceleration(l6470_calculate_deceleration(deceleration));
}
