#include <stdint.h>

typedef union {
  uint16_t data;
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
  } reg;
} l6470_status_t;

void l6470_init();

l6470_status_t l6470_get_status();
