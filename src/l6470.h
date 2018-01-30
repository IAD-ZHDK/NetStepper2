#include <stdint.h>

typedef union {
  uint16_t data;
  struct {
    uint8_t first;
    uint8_t second;
  };
  struct {
    uint8_t sck_mod : 1;
    uint8_t step_loss_b : 1;
    uint8_t step_loss_a : 1;
    uint8_t ocd : 1;
    uint8_t th_sd : 1;
    uint8_t th_wrn : 1;
    uint8_t uvlo : 1;
    uint8_t wrong_cmd : 1;
    uint8_t notperf_cmd : 1;
    uint8_t mot_status : 2;
    uint8_t dir : 1;
    uint8_t sw_evn : 1;
    uint8_t sw_f : 1;
    uint8_t busy : 1;
    uint8_t hiz : 1;
  };
} l6470_status_t;

void l6470_init();

/* COMMANDS */

void l6470_set_param(uint8_t param, uint32_t value);

uint32_t l6470_get_param(uint8_t param);

void l6470_go_to(int32_t pos);

l6470_status_t l6470_get_status_and_clear();

/* PARAMETERS */

typedef enum {
  L6570_STEP_MODE_1 = 0x00,
  L6570_STEP_MODE_1_2 = 0x01,
  L6570_STEP_MODE_1_4 = 0x02,
  L6570_STEP_MODE_1_8 = 0x03,
  L6570_STEP_MODE_1_16 = 0x04,
  L6570_STEP_MODE_1_32 = 0x05,
  L6570_STEP_MODE_1_64 = 0x06,
  L6570_STEP_MODE_1_128 = 0x07
} l6470_step_mode_t;

void l6470_set_step_mode(l6470_step_mode_t value);

l6470_step_mode_t l6470_set_get_step_mode();

#define STEP_MODE_STEP_SEL 0x07  // Mask for these bits only.
