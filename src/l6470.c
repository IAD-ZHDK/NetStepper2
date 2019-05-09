#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <naos.h>

#include "l6470.h"

#define L6470_CMD_SET_PARAM 0x00
#define L6470_CMD_GET_PARAM 0x20
#define L6470_CMD_RUN 0x50
#define L6470_CMD_MOVE 0x40
#define L6470_CMD_GO_TO 0x60
#define L6470_CMD_GO_TO_DIR 0x68
#define L6470_CMD_GO_HOME 0x70
#define L6470_CMD_GO_MARK 0x78
#define L6470_CMD_RESET_POSITION 0xD8
#define L6470_CMD_RESET_DEVICE 0xC0
#define L6470_CMD_SOFT_STOP 0xB0
#define L6470_CMD_HARD_STOP 0xB8
#define L6470_CMD_SOFT_HIZ 0xA0
#define L6470_CMD_HARD_HIZ 0xA8
#define L6470_CMD_GET_STATUS 0xD0

#define L6470_REG_ABSOLUTE_POSITION 0x01
#define L6470_REG_MARK 0x03
#define L6470_REG_SPEED 0x04
#define L6470_REG_ACCELERATION 0x05
#define L6470_REG_DECELERATION 0x06
#define L6470_REG_MAXIMUM_SPEED 0x07
#define L6470_REG_MINIMUM_SPEED 0x08
#define L6470_REG_FULL_STEP_SPEED 0x15
#define L6470_REG_STEP_MODE 0x16
#define L6470_REG_STATUS 0x19

#define L6470_CS 25
#define L6470_RESET GPIO_NUM_16

spi_device_handle_t l6470_spi;

void l6470_init() {
  // prepare bus config
  spi_bus_config_t bus_config = {
      .miso_io_num = 19, .mosi_io_num = 23, .sclk_io_num = 18, .quadwp_io_num = -1, .quadhd_io_num = -1};

  // Initialize the SPI bus
  ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 0));

  // prepare device config
  spi_device_interface_config_t device_config = {
      .mode = 3,
      .clock_speed_hz = 4 * 1000 * 1000,  // 4 MHz
      .spics_io_num = L6470_CS,
      .queue_size = 1,
  };

  // attach device to the SPI bus
  ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &device_config, &l6470_spi));

  // initialize non-SPI GPIOs
  ESP_ERROR_CHECK(gpio_set_direction(L6470_RESET, GPIO_MODE_OUTPUT));

  // reset chip
  ESP_ERROR_CHECK(gpio_set_level(L6470_RESET, 0));
  naos_delay(100);
  ESP_ERROR_CHECK(gpio_set_level(L6470_RESET, 1));
  naos_delay(100);
}

/* COMMUNICATION */

static uint8_t l6470_transmit(uint8_t data) {
  // prepare transaction
  spi_transaction_t t = {
    .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
  };

  // set tx and rx data
  t.tx_data[0] = data;
  t.length = 8;
  t.rxlength = 8;

  // send command
  ESP_ERROR_CHECK(spi_device_transmit(l6470_spi, &t));

  return t.rx_data[0];
}

static uint32_t l6470_transfer(uint32_t data, size_t bits) {
  // get full bytes
  size_t bytes = bits / 8;
  if (bits % 8 > 0) {
    bytes++;
  }

  // prepare return value
  uint32_t ret = 0;

  // read all bytes
  for (int i = 0; i < bytes; i++) {
    // shift already read bytes
    ret = ret << 8;

    // read next byte
    uint8_t next = l6470_transmit((uint8_t)(data >> ((bytes - i - 1) * 8)));

    // add read byte to result
    ret |= next;
  }

  // create mask
  uint32_t mask = 0xffffffff >> (32 - bits);

  return ret & mask;
}

static uint32_t l6470_handle_param(uint8_t param, uint32_t value) {
  switch (param) {
    case L6470_REG_ABSOLUTE_POSITION:
      return l6470_transfer(value, 22);
    case L6470_REG_MARK:
      return l6470_transfer(value, 22);
    case L6470_REG_SPEED:
      return l6470_transfer(0, 20);
    case L6470_REG_ACCELERATION:
      return l6470_transfer(value, 12);
    case L6470_REG_DECELERATION:
      return l6470_transfer(value, 12);
    case L6470_REG_MAXIMUM_SPEED:
      return l6470_transfer(value, 10);
    case L6470_REG_MINIMUM_SPEED:
      return l6470_transfer(value, 13);
    case L6470_REG_FULL_STEP_SPEED:
      return l6470_transfer(value, 10);
    case L6470_REG_STEP_MODE:
      return l6470_transfer(value, 8);
    case L6470_REG_STATUS:
      return l6470_transfer(0, 16);
    default:
      return 0;
  }
}

static void l6470_set_param(uint8_t param, uint32_t value) {
  // send command
  l6470_transmit(param | (uint8_t)L6470_CMD_SET_PARAM);

  // set param
  l6470_handle_param(param, value);
}

static uint32_t l6470_get_param(uint8_t param) {
  // send command
  l6470_transmit(param | (uint8_t)L6470_CMD_GET_PARAM);

  // handle param
  return l6470_handle_param(param, 0);
}

/* COMMANDS */

void l6470_run(l6470_direction_t dir, uint32_t steps_per_tick) {
  // send command
  l6470_transmit((uint8_t)(L6470_CMD_RUN | dir));

  // clamp to 20 bits
  if (steps_per_tick > 0xFFFFF) {
    steps_per_tick = 0xFFFFF;
  };

  // get pos as pointer
  uint8_t* _steps_per_tick = (uint8_t*)&steps_per_tick;

  // send steps per tick
  l6470_transmit(_steps_per_tick[2]);
  l6470_transmit(_steps_per_tick[1]);
  l6470_transmit(_steps_per_tick[0]);
}

void l6470_move(l6470_direction_t dir, uint32_t steps) {
  // send command
  l6470_transmit((uint8_t)(L6470_CMD_MOVE | dir));

  // clamp to 22 bits
  if (steps > L6470_U22_MAX) {
    steps = L6470_U22_MAX;
  }

  // get pos as pointer
  uint8_t* _steps = (uint8_t*)&steps;

  // send steps
  l6470_transmit(_steps[2]);
  l6470_transmit(_steps[1]);
  l6470_transmit(_steps[0]);
}

void l6470_go_to(int32_t pos) {
  // clamp to 22 bits
  if (pos > L6470_I22_MAX) {
    pos = L6470_I22_MAX;
  } else if(pos < L6470_I22_MIN) {
    pos = L6470_I22_MIN;
  }

  // get pos as pointer
  uint8_t* _pos = (uint8_t*)&pos;

  // send command
  l6470_transmit(L6470_CMD_GO_TO);

  // send position
  l6470_transmit(_pos[2]);
  l6470_transmit(_pos[1]);
  l6470_transmit(_pos[0]);
}

void l6470_go_to_direction(int32_t pos, l6470_direction_t dir) {
  // clamp to 22 bits
  if (pos > L6470_I22_MAX) {
    pos = L6470_I22_MAX;
  } else if(pos < L6470_I22_MIN) {
    pos = L6470_I22_MIN;
  }

  // get pos as pointer
  uint8_t* _pos = (uint8_t*)&pos;

  // send command
  l6470_transmit((uint8_t)(L6470_CMD_GO_TO_DIR | dir));

  // send position
  l6470_transmit(_pos[2]);
  l6470_transmit(_pos[1]);
  l6470_transmit(_pos[0]);
}

void l6470_go_home() {
  // send command
  l6470_transmit(L6470_CMD_GO_HOME);
}

void l6470_go_mark() {
  // send command
  l6470_transmit(L6470_CMD_GO_MARK);
}

void l6470_reset_position() {
  // send command
  l6470_transmit(L6470_CMD_RESET_POSITION);
}

void l6470_reset_device() {
  // send command
  l6470_transmit(L6470_CMD_RESET_DEVICE);
}

void l6470_soft_stop() {
  // send command
  l6470_transmit(L6470_CMD_SOFT_STOP);
}

void l6470_hard_stop() {
  // send command
  l6470_transmit(L6470_CMD_HARD_STOP);
}

void l6470_soft_hiz() {
  // send command
  l6470_transmit(L6470_CMD_SOFT_HIZ);
}

void l6470_hard_hiz() {
  // send command
  l6470_transmit(L6470_CMD_HARD_HIZ);
}

l6470_status_t l6470_get_status_and_clear() {
  // prepare status
  l6470_status_t status;

  // send command
  l6470_transmit(L6470_CMD_GET_STATUS);

  // read status
  status.second = l6470_transmit(0);
  status.first = l6470_transmit(0);

  return status;
}

/* PARAMETER HANDLING */

void l6470_set_absolute_position(int32_t pos) {
  // set parameter
  l6470_set_param(L6470_REG_ABSOLUTE_POSITION, (uint32_t)pos);
}

int32_t l6470_get_absolute_position() {
  // read parameter
  uint32_t value = l6470_get_param(L6470_REG_ABSOLUTE_POSITION);

  // fix sign for 22bit 2s complement
  if (value & 0x00200000) {
    value |= 0xffc00000;
  }

  return (int32_t)value;
}

void l6470_set_mark(int32_t pos) {
  // set parameter
  l6470_set_param(L6470_REG_MARK, (uint32_t)pos);
}

int32_t l6470_get_mark() {
  // read parameter
  uint32_t temp = l6470_get_param(L6470_REG_MARK);

  // fix sign for 22bit 2s complement
  if (temp & 0x00200000) {
    temp |= 0xffC00000;
  }

  return (int32_t)temp;
}

uint32_t l6470_get_speed() {
  // read parameter
  return l6470_get_param(L6470_REG_SPEED);
}

void l6470_set_acceleration(uint32_t steps_per_tick) {
  // set param
  l6470_set_param(L6470_REG_ACCELERATION, steps_per_tick);
}

uint32_t l6470_get_acceleration() {
  // read parameter
  return l6470_get_param(L6470_REG_ACCELERATION);
}

void l6470_set_deceleration(uint32_t steps_per_tick) {
  // set param
  l6470_set_param(L6470_REG_DECELERATION, steps_per_tick);
}

uint32_t l6470_get_deceleration() {
  // read parameter
  return l6470_get_param(L6470_REG_DECELERATION);
}

void l6470_set_maximum_speed(uint16_t steps_per_tick) {
  // set param
  l6470_set_param(L6470_REG_MAXIMUM_SPEED, steps_per_tick);
}

uint16_t l6470_get_maximum_speed() {
  // read parameter
  return (uint16_t)l6470_get_param(L6470_REG_MAXIMUM_SPEED);
}

void l6470_set_minimum_speed(uint16_t steps_per_tick) {
  // get param and clear min speed
  uint16_t current = (uint16_t)l6470_get_param(L6470_REG_MINIMUM_SPEED) & (uint16_t)0x1000;

  // set new value and respect mask
  current |= (steps_per_tick & 0xFFF);

  // set param
  l6470_set_param(L6470_REG_MINIMUM_SPEED, current);
}

uint16_t l6470_get_minimum_speed() {
  // read parameter
  return (uint16_t)l6470_get_param(L6470_REG_MINIMUM_SPEED);
}

void l6470_set_full_step_speed(uint16_t steps_per_tick) {
  // set param
  l6470_set_param(L6470_REG_FULL_STEP_SPEED, steps_per_tick);
}

uint16_t l6470_get_full_step_speed() {
  // read parameter
  return (uint16_t)l6470_get_param(L6470_REG_FULL_STEP_SPEED);
}

void l6470_set_step_mode(l6470_step_mode_t mode) {
  // get current register and clear step mode
  uint8_t current = (uint8_t)l6470_get_param(L6470_REG_STEP_MODE) & (uint8_t)0xF8;

  // set new mode and respect mask
  current |= (mode & 0x07);

  // update register
  l6470_set_param(L6470_REG_STEP_MODE, current);
}

int l6470_set_step_mode_int(int mode) {
  switch (mode) {
    case 1:
      l6470_set_step_mode(L6470_STEP_MODE_1);
      return 1;
    case 2:
      l6470_set_step_mode(L6470_STEP_MODE_2);
      return 2;
    case 4:
      l6470_set_step_mode(L6470_STEP_MODE_4);
      return 4;
    case 8:
      l6470_set_step_mode(L6470_STEP_MODE_8);
      return 8;
    case 16:
      l6470_set_step_mode(L6470_STEP_MODE_16);
      return 16;
    case 32:
      l6470_set_step_mode(L6470_STEP_MODE_32);
      return 32;
    case 64:
      l6470_set_step_mode(L6470_STEP_MODE_64);
      return 64;
    case 128:
      l6470_set_step_mode(L6470_STEP_MODE_128);
      return 128;
    default:
      l6470_set_step_mode(L6470_STEP_MODE_1);
      return 1;
  }
}

l6470_step_mode_t l6470_get_step_mode() {
  // set parameter
  return (l6470_step_mode_t)(l6470_get_param(L6470_REG_STEP_MODE) & 0x07);
}

l6470_status_t l6470_get_status() {
  // prepare status
  l6470_status_t status;

  // send command
  status.data = (uint16_t)l6470_get_param(L6470_REG_STATUS);

  return status;
}

/* HELPERS */

void l6470_wait() {
  while (l6470_get_status().busy == 0) {
  }
}

void l6470_approach_home() {
  // get info
  uint32_t speed = l6470_get_speed();
  l6470_status_t status = l6470_get_status();

  // change to run command and wait until speed is reached
  l6470_run(status.direction, speed);
  l6470_wait();

  // set new position
  l6470_go_home();
}

void l6470_approach_target(int32_t pos) {
  // get info
  uint32_t speed = l6470_get_speed();
  l6470_status_t status = l6470_get_status();

  // change to run command and wait until speed is reached
  l6470_run(status.direction, speed);
  l6470_wait();

  // set new position
  l6470_go_to(pos);
}

/* CALCULATION */

uint32_t l6470_calculate_speed(double steps_per_sec) {
  // calculate internal value
  uint32_t value = (uint32_t)(steps_per_sec * 67.106);

  // clamp to 20 bits
  if (value > 0xFFFFF) {
    value = 0xFFFFF;
  };

  return value;
}

double l6470_parse_speed(uint32_t steps_per_sec) {
  // calculate real value
  return (steps_per_sec & 0x000FFFFF) / 67.106;
}

uint16_t l6470_calculate_acceleration(double steps_per_sec_per_sec) {
  // calculate internal value
  uint16_t value = (uint16_t)(steps_per_sec_per_sec * 0.137438);

  // clamp to 12 bits
  if (value > 0xFFF) {
    value = 0xFFF;
  }

  return value;
}

double l6470_parse_acceleration(uint16_t steps_per_sec_per_sec) {
  // calculate real value
  return (steps_per_sec_per_sec & 0xFFF) / 0.137438;
}

uint16_t l6470_calculate_deceleration(double steps_per_sec_per_sec) {
  // calculate internal value
  uint16_t value = (uint16_t)(steps_per_sec_per_sec * 0.137438);

  // clamp to 12 bits
  if (value > 0xFFF) {
    value = 0xFFF;
  }

  return value;
}

double l6470_parse_deceleration(uint16_t steps_per_sec_per_sec) {
  // calculate real value
  return (steps_per_sec_per_sec & 0x00000FFF) / 0.137438;
}

uint16_t l6470_calculate_maximum_speed(double steps_per_sec) {
  // calculate internal value
  uint16_t value = (uint16_t)(steps_per_sec * 0.065536);

  // clamp to 10 bits
  if (value > 0x3FF) {
    return 0x3FF;
  };

  return value;
}

double l6470_parse_maximum_speed(uint16_t steps_per_sec) {
  // calculate real value
  return (steps_per_sec & 0x000003FF) / 0.065536;
}

uint16_t l6470_calculate_minimum_speed(double steps_per_sec) {
  // calculate internal value
  uint16_t value = (uint16_t)(steps_per_sec / 0.238);

  // clamp to 12 bits
  if (value > 0xFFF) {
    value = 0xFFF;
  }

  return value;
}

double l6470_parse_minimum_speed(uint16_t steps_per_sec) {
  // calculate real value
  return (steps_per_sec & 0x00000FFF) * 0.238;
}

uint16_t l6470_calculate_full_step_speed(double steps_per_sec) {
  // calculate internal value
  uint16_t value = (uint16_t)((steps_per_sec * .065536) - .5);

  // clamp to 10 bits
  if (value > 0x3FF) {
    return 0x3FF;
  };

  return value;
}

double l6470_parse_full_step_speed(uint16_t steps_per_sec) {
  // calculate real value
  return ((steps_per_sec & 0x000003FF) + 0.5) / 0.065536;
}
