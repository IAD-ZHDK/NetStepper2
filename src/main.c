#include <art32/numbers.h>
#include <art32/smooth.h>
#include <art32/strconv.h>
#include <driver/adc.h>
#include <driver/gpio.h>
#include <naos.h>
#include <string.h>

#include "buttons.h"
#include "encoder.h"
#include "end_stop.h"
#include "l6470.h"
#include "led.h"
#include "sharp.h"

static naos_status_t current_status = NAOS_DISCONNECTED;

int32_t micro_steps = 128;
double gear_ratio = 0;
int32_t resolution = 0;
double max_speed = 0;
double acceleration = 0;
double deceleration = 0;
bool use_sensor_1 = false;
bool use_sensor_2 = false;
bool convert_sharp = false;

a32_smooth_t *sensor_smooth_1;
a32_smooth_t *sensor_smooth_2;

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
  // handle "micro-steps"
  if (strcmp(param, "micro-steps") == 0) {
    // make motor stop (remove power)
    l6470_hard_hiz();

    // set setting
    micro_steps = l6470_set_step_mode_int(naos_get_l("micro-steps"));

    // set corrected value
    naos_set_l("micro-steps", micro_steps);

    // reset position
    l6470_reset_position();
  }

  // handle "max-speed"
  if (strcmp(param, "max-speed") == 0) {
    // constrain value
    max_speed = naos_get_d("max-speed");

    // set constrained value
    naos_set_d("max-speed", max_speed);

    // set setting
    l6470_set_maximum_speed(l6470_calculate_maximum_speed(max_speed));

    // set full step mode to two times max speed (no full stepping)
    l6470_set_full_step_speed(l6470_calculate_full_step_speed(max_speed * 2));
  }

  // handle "acceleration"
  if (strcmp(param, "acceleration") == 0) {
    // constrain value
    acceleration = naos_get_d("acceleration");

    // set constrained value
    naos_set_d("acceleration", acceleration);

    // set setting
    l6470_set_acceleration(l6470_calculate_acceleration(acceleration));
  }

  // handle "deceleration"
  if (strcmp(param, "deceleration") == 0) {
    // constrain value
    deceleration = naos_get_d("deceleration");

    // set constrained value
    naos_set_d("deceleration", deceleration);

    // set setting
    l6470_set_deceleration(l6470_calculate_deceleration(deceleration));
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
    l6470_run(L6470_FORWARD, l6470_calculate_speed(max_speed));
  }

  // handle "backward command
  if (strcmp(topic, "backward") == 0 && scope == NAOS_LOCAL) {
    // run backward
    l6470_run(L6470_REVERSE, l6470_calculate_speed(max_speed));
  }

  // handle "target" command
  if (strcmp(topic, "target") == 0 && scope == NAOS_LOCAL) {
    // calculate position factor
    double steps_per_rev = micro_steps * resolution * gear_ratio;

    // calculate minimum and maximum position
    double min_target = L6470_I22_MIN / steps_per_rev;
    double max_target = L6470_I22_MAX / steps_per_rev;

    // get target
    double target = a32_str2d(str);

    // constrain target
    target = a32_constrain_d(target, min_target, max_target);

    // calculate real position
    int32_t pos = (int32_t)(target * steps_per_rev);

    // constrain position
    pos = a32_constrain_l(pos, L6470_I22_MIN, L6470_I22_MAX);

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

static void loop() {
  // this loop is called at a rate of 100/s

  // get time
  uint32_t now = naos_millis();

  // check sensor 1
  if (use_sensor_1) {
    // prepare counter
    static uint32_t last_send_1 = 0;

    // read sensor
    double es1 = end_stop_read_1();

    // convert value if requested
    if (convert_sharp) {
      es1 = sharp_convert(es1);
    }

    // smooth value
    es1 = a32_smooth_update(sensor_smooth_1, es1);

    // check if 100ms passed
    if (last_send_1 + 100 < now) {
      // set time
      last_send_1 = now;

      // publish sensor value
      naos_publish_d("sensor1", es1, 0, false, NAOS_LOCAL);
    }
  }

  // check sensor 2
  if (use_sensor_2) {
    // prepare counter
    static uint32_t last_send_2 = 0;

    // read sensors
    double es2 = end_stop_read_2();

    // convert value if requested
    if (convert_sharp) {
      es2 = sharp_convert(es2);
    }

    // smooth value
    es2 = a32_smooth_update(sensor_smooth_2, es2);

    // check if 100ms passed
    if (last_send_2 + 100 < now) {
      // set time
      last_send_2 = now;

      // publish sensor value
      naos_publish_d("sensor2", es2, 0, false, NAOS_LOCAL);
    }
  }
}

static void press(buttons_type_t type, bool pressed) {
  // prepare home press counter
  static uint32_t home_press = 0;
  static uint32_t stop_press = 0;

  // turn forward if cw is released
  if (type == BUTTONS_TYPE_CW && !pressed) {
    // run forward
    l6470_run(L6470_FORWARD, l6470_calculate_speed(max_speed));
  }

  // turn backwards if ccw is released
  if (type == BUTTONS_TYPE_CCW && !pressed) {
    // run backward
    l6470_run(L6470_REVERSE, l6470_calculate_speed(max_speed));
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

    // flash led
    led_set(255, 255, 255);
    naos_delay(300);
    set_status();
  }
}

static void position(double p) {}

static void end_stop(end_stop_pin_t pin, bool on) {}

static void offline() {
  // stop motor
  l6470_soft_stop();
}

static naos_param_t params[] = {
    {.name = "micro-steps", .type = NAOS_LONG, .default_l = 128},
    {.name = "gear-ratio", .type = NAOS_DOUBLE, .default_d = 5.18, .sync_d = &gear_ratio},
    {.name = "resolution", .type = NAOS_LONG, .default_l = 200, .sync_l = &resolution},
    {.name = "max-speed", .type = NAOS_DOUBLE, .default_d = 900},
    {.name = "acceleration", .type = NAOS_DOUBLE, .default_d = 900},
    {.name = "deceleration", .type = NAOS_DOUBLE, .default_d = 900},
    {.name = "use-sensor-1", .type = NAOS_BOOL, .default_b = false, .sync_b = &use_sensor_1},
    {.name = "use-sensor-2", .type = NAOS_BOOL, .default_b = false, .sync_b = &use_sensor_2},
    {.name = "convert-sharp", .type = NAOS_BOOL, .default_b = false, .sync_b = &convert_sharp},
};

static naos_config_t config = {
    .device_type = "NetStepper2",
    .firmware_version = "0.5.0",
    .parameters = params,
    .num_parameters = 9,
    .ping_callback = ping,
    .status_callback = status,
    .online_callback = online,
    .update_callback = update,
    .message_callback = message,
    .loop_callback = loop,
    .loop_interval = 10,
    .offline_callback = offline,
};

void app_main() {
  // install gpio interrupt service
  gpio_install_isr_service(0);

  // set adc capture width
  adc1_config_width(ADC_WIDTH_10Bit);

  // initialize led
  led_init();

  // initialize buttons
  buttons_init(press);

  // initialize encoder
  encoder_init(position);

  // initialize l6470
  l6470_init();

  // initialize sensors
  sensor_smooth_1 = a32_smooth_new(16);
  sensor_smooth_2 = a32_smooth_new(16);

  // initialize end stops
  end_stop_init(end_stop, true);

  // initialize naos
  naos_init(&config);

  // set step mode
  micro_steps = l6470_set_step_mode_int(naos_get_l("micro-steps"));

  // get speeds
  max_speed = naos_get_d("max-speed");
  acceleration = naos_get_d("acceleration");
  deceleration = naos_get_d("deceleration");

  // reset minimum speed
  l6470_set_minimum_speed(l6470_calculate_minimum_speed(0));

  // set initial speeds
  l6470_set_maximum_speed(l6470_calculate_maximum_speed(max_speed));
  l6470_set_acceleration(l6470_calculate_acceleration(acceleration));
  l6470_set_deceleration(l6470_calculate_deceleration(deceleration));

  // set full step mode to two times max speed (no full stepping)
  l6470_set_full_step_speed(l6470_calculate_full_step_speed(max_speed * 2));
}
