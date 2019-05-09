#include <stdint.h>

/* DATA STRUCTURES */

typedef enum { L6470_FORWARD = 0x01, L6470_REVERSE = 0x00 } l6470_direction_t;

typedef union {
  uint16_t data;
  struct {
    uint8_t first;
    uint8_t second;
  };
  struct {
    uint8_t hiz : 1;
    uint8_t busy : 1;
    uint8_t _1 : 2;
    l6470_direction_t direction : 1;
    uint16_t _2 : 11;
  };
} l6470_status_t;

typedef enum {
  L6470_STEP_MODE_1 = 0x00,
  L6470_STEP_MODE_2 = 0x01,
  L6470_STEP_MODE_4 = 0x02,
  L6470_STEP_MODE_8 = 0x03,
  L6470_STEP_MODE_16 = 0x04,
  L6470_STEP_MODE_32 = 0x05,
  L6470_STEP_MODE_64 = 0x06,
  L6470_STEP_MODE_128 = 0x07
} l6470_step_mode_t;

#define L6470_I22_MAX ((1<<21) - 1)
#define L6470_I22_MIN ((-1u) << 21)
#define L6470_U22_MAX ((1<<22) - 1)

/* INITIALIZATION */

void l6470_init();

/* COMMANDS */

void l6470_run(l6470_direction_t dir, uint32_t steps_per_tick);

void l6470_move(l6470_direction_t dir, uint32_t steps);

void l6470_go_to(int32_t pos);

void l6470_go_to_direction(int32_t pos, l6470_direction_t dir);

void l6470_go_home();

void l6470_go_mark();

void l6470_reset_position();

void l6470_reset_device();

void l6470_soft_stop();

void l6470_hard_stop();

void l6470_soft_hiz();

void l6470_hard_hiz();

l6470_status_t l6470_get_status_and_clear();

/* PARAMETERS */

void l6470_set_absolute_position(int32_t pos);

int32_t l6470_get_absolute_position();

void l6470_set_mark(int32_t pos);

int32_t l6470_get_mark();

uint32_t l6470_get_speed();

void l6470_set_acceleration(uint32_t steps_per_tick);

uint32_t l6470_get_acceleration();

void l6470_set_deceleration(uint32_t steps_per_tick);

uint32_t l6470_get_deceleration();

void l6470_set_maximum_speed(uint16_t steps_per_tick);

uint16_t l6470_get_maximum_speed();

void l6470_set_minimum_speed(uint16_t steps_per_tick);

uint16_t l6470_get_minimum_speed();

void l6470_set_full_step_speed(uint16_t steps_per_tick);

uint16_t l6470_get_full_step_speed();

void l6470_set_step_mode(l6470_step_mode_t mode);

int l6470_set_step_mode_int(int mode);

l6470_step_mode_t l6470_get_step_mode();

l6470_status_t l6470_get_status();

/* HELPERS */

void l6470_wait();

void l6470_approach_home();

void l6470_approach_target(int32_t pos);

/* CALCULATION */

uint32_t l6470_calculate_speed(double steps_per_sec);

double l6470_parse_speed(uint32_t steps_per_sec);

uint16_t l6470_calculate_acceleration(double steps_per_sec_per_sec);

double l6470_parse_acceleration(uint16_t steps_per_sec_per_sec);

uint16_t l6470_calculate_deceleration(double steps_per_sec_per_sec);

double l6470_parse_deceleration(uint16_t steps_per_sec_per_sec);

uint16_t l6470_calculate_maximum_speed(double steps_per_sec);

double l6470_parse_maximum_speed(uint16_t steps_per_sec);

uint16_t l6470_calculate_minimum_speed(double steps_per_sec);

double l6470_parse_minimum_speed(uint16_t steps_per_sec);

uint16_t l6470_calculate_full_step_speed(double steps_per_sec);

double l6470_parse_full_step_speed(uint16_t steps_per_sec);
