#include <stdint.h>

/* DATA STRUCTURES */

typedef enum { L6470_DIR_FWD = 0x01, L6470_DIR_REV = 0x00 } l6470_dir_t;

typedef union {
  uint16_t data;
  struct {
    uint8_t first;
    uint8_t second;
  };
  struct {
    uint8_t hiz : 1;
    uint8_t busy : 1;
    uint8_t sw_f : 1;
    uint8_t sw_evn : 1;
    l6470_dir_t dir : 1;
    uint8_t mot_status : 2;
    uint8_t notperf_cmd : 1;
    uint8_t wrong_cmd : 1;
    uint8_t uvlo : 1;
    uint8_t th_wrn : 1;
    uint8_t th_sd : 1;
    uint8_t ocd : 1;
    uint8_t step_loss_a : 1;
    uint8_t step_loss_b : 1;
    uint8_t sck_mod : 1;
  };
} l6470_status_t;

typedef enum {
  L6470_STEP_MODE_1 = 0x00,
  L6470_STEP_MODE_1_2 = 0x01,
  L6470_STEP_MODE_1_4 = 0x02,
  L6470_STEP_MODE_1_8 = 0x03,
  L6470_STEP_MODE_1_16 = 0x04,
  L6470_STEP_MODE_1_32 = 0x05,
  L6470_STEP_MODE_1_64 = 0x06,
  L6470_STEP_MODE_1_128 = 0x07
} l6470_step_mode_t;

/* INITIALIZATION */

void l6470_init();

/* COMMANDS */

void l6470_set_param(uint8_t param, uint32_t value);

uint32_t l6470_get_param(uint8_t param);

void l6470_run(l6470_dir_t dir, uint32_t steps_per_tick);

void l6470_move(uint8_t dir, uint32_t steps);

void l6470_go_to(int32_t pos);

void l6470_go_to_dir(int32_t pos, l6470_dir_t dir);

void l6470_go_home();

void l6470_go_mark();

void l6470_reset_pos();

void l6470_reset_device();

void l6470_soft_stop();

void l6470_hard_stop();

void l6470_soft_hiz();

void l6470_hard_hiz();

l6470_status_t l6470_get_status_and_clear();

/* PARAMETERS */

void l6470_set_abs_pos(int32_t value);

int32_t l6470_get_abs_pos();

void l6470_set_mark(int32_t value);

int32_t l6470_get_mark();

uint32_t l6470_get_speed();

void l6470_set_acc(uint32_t steps_per_tick);

uint32_t l6470_get_acc();

void l6470_set_decel(uint32_t steps_per_tick);

uint32_t l6470_get_decel();

void l6470_set_max_speed(uint16_t steps_per_tick);

uint16_t l6470_get_max_speed();

void l6470_set_min_speed(uint16_t steps_per_tick);

uint16_t l6470_get_min_speed();

void l6470_set_fs_speed(uint16_t steps_per_tick);

uint16_t l6470_get_fs_speed();

void l6470_set_step_mode(l6470_step_mode_t value);

l6470_step_mode_t l6470_set_get_step_mode();

l6470_status_t l6470_get_status();

/* HELPERS */

void l6470_wait();

/* CALCULATION */

uint32_t l6470_calc_speed(double stepsPerSec);

double l6470_parse_speed(uint32_t stepsPerSec);

uint16_t l6470_calc_acc(double stepsPerSecPerSec);

double l6470_parse_acc(uint16_t stepsPerSecPerSec);

uint16_t l6470_calc_dec(double stepsPerSecPerSec);

double l6470_parse_dec(uint16_t stepsPerSecPerSec);

uint16_t l6470_calc_max_speed(double stepsPerSec);

double l6470_parse_max_speed(uint16_t stepsPerSec);

uint16_t l6470_calc_min_speed(double stepsPerSec);

double l6470_parse_min_speed(uint16_t stepsPerSec);

uint16_t l6470_calc_fs_peed(double stepsPerSec);

double l6470_parse_fs_speed(uint16_t stepsPerSec);
