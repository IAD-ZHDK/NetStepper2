#include <stdint.h>

#define L6470_REG_ABS_POS 0x01
#define L6470_REG_EL_POS 0x02
#define L6470_REG_MARK 0x03
#define L6470_REG_SPEED 0x04
#define L6470_REG_ACC 0x05
#define L6470_REG_DECEL 0x06
#define L6470_REG_MAX_SPEED 0x07
#define L6470_REG_MIN_SPEED 0x08
#define L6470_REG_FS_SPD 0x15
#define L6470_REG_KVAL_HOLD 0x09
#define L6470_REG_KVAL_RUN 0x0A
#define L6470_REG_KVAL_ACC 0x0B
#define L6470_REG_KVAL_DEC 0x0C
#define L6470_REG_INT_SPD 0x0D
#define L6470_REG_ST_SLP 0x0E
#define L6470_REG_FN_SLP_ACC 0x0F
#define L6470_REG_FN_SLP_DEC 0x10
#define L6470_REG_K_THERM 0x11
#define L6470_REG_ADC_OUT 0x12
#define L6470_REG_OCD_TH 0x13
#define L6470_REG_STALL_TH 0x14
#define L6470_REG_STEP_MODE 0x16
#define L6470_REG_ALARM_EN 0x17
#define L6470_REG_CONFIG 0x18
#define L6470_REG_STATUS 0x19

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

void l6470_set_param(uint8_t param, uint32_t value);

uint32_t l6470_get_param(uint8_t param);

void l6470_go_to(int32_t pos);

l6470_status_t l6470_get_status_and_clear();
