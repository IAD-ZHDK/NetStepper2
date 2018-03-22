#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <naos.h>

#include "l6470.h"

#define L6470_CS 5

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
    flags : SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
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

/* COMMANDS */

static uint32_t l6470_handle_param(uint8_t param, uint32_t value) {
  switch (param) {
    // ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
    // in two's complement. At power up, this value is 0. It cannot be written when
    // the motor is running, but at any other time, it can be updated to change the
    // interpreted position of the motor.
    case L6470_REG_ABS_POS:
      return l6470_transfer(value, 22);

    // MARK is a second position other than 0 that the motor can be told to go to. As
    // with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
    case L6470_REG_MARK:
      return l6470_transfer(value, 22);

    // SPEED contains information about the current speed. It is read-only. It does
    // NOT provide direction information.
    case L6470_REG_SPEED:
      return l6470_transfer(0, 20);

    // ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF
    //  to get infinite acceleration/deceleration- there is no way to get infinite
    //  deceleration w/o infinite acceleration (except the HARD STOP command).
    //  Cannot be written while motor is running. Both default to 0x08A on power up.
    // AccCalc() and DecCalc() functions exist to convert steps/s/s values into
    //  12-bit values for these two registers.
    case L6470_REG_ACC:
      return l6470_transfer(value, 12);
    case L6470_REG_DECEL:
      return l6470_transfer(value, 12);

    // MAX_SPEED is just what it says- any command which attempts to set the speed
    //  of the motor above this value will simply cause the motor to turn at this
    //  speed. Value is 0x041 on power up.
    // MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
    //  for this register.
    case L6470_REG_MAX_SPEED:
      return l6470_transfer(value, 10);

    // MIN_SPEED controls two things- the activation of the low-speed optimization
    //  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
    //  is the 13th bit, and when it is set, the minimum allowed speed is automatically
    //  set to zero. This value is 0 on startup.
    // MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
    //  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
    case L6470_REG_MIN_SPEED:
      return l6470_transfer(value, 13);

    // FS_SPD register contains a threshold value above which microstepping is disabled
    //  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
    // FSCalc() function exists to convert steps/s value into 10-bit integer for this
    //  register.
    case L6470_REG_FS_SPD:
      return l6470_transfer(value, 10);

    // STEP_MODE controls the microstepping settings, as well as the generation of an
    //  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
    //  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
    //  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
    //  of the output signal relative to the full-step frequency; see datasheet for
    //  that relationship as it is too complex to reproduce here.
    // Most likely, only the microsteps per step value will be needed; there is a set
    //  of constants provided for ease of use of these values.
    case L6470_REG_STEP_MODE:
      return l6470_transfer(value, 8);

    // STATUS contains read-only information about the current condition of the chip. A
    //  comprehensive set of constants for masking and testing this register is provided, but
    //  users should refer to the datasheet to ensure that they fully understand each one of
    //  the bits in the register.
    case L6470_REG_STATUS:  // STATUS is a read-only register
      return l6470_transfer(0, 16);

    default:
      return 0;
  }
}

void l6470_set_param(uint8_t param, uint32_t value) {
  // send command
  l6470_transmit(param | (uint8_t)L6470_CMD_SET_PARAM);

  // set param
  l6470_handle_param(param, value);
}

uint32_t l6470_get_param(uint8_t param) {
  // send command
  l6470_transmit(param | (uint8_t)L6470_CMD_GET_PARAM);

  // handle param
  return l6470_handle_param(param, 0);
}

void l6470_run(l6470_dir_t dir, uint32_t steps_per_tick) {
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

void l6470_move(uint8_t dir, uint32_t steps) {
  // send command
  l6470_transmit((uint8_t)(L6470_CMD_MOVE | dir));

  // clamp to 22 bits
  if (steps > 0x3FFFFF) {
    steps = 0x3FFFFF;
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
  if (pos > 0x3FFFFF) {
    pos = 0x3FFFFF;
  }

  // get pos as pointer
  uint8_t* _pos = (uint8_t*)&pos;

  // send command
  l6470_transmit(L6470_CMD_GOTO);

  // send position
  l6470_transmit(_pos[2]);
  l6470_transmit(_pos[1]);
  l6470_transmit(_pos[0]);
}

void l6470_go_to_dir(int32_t pos, l6470_dir_t dir) {
  // clamp to 22 bits
  if (pos > 0x3FFFFF) {
    pos = 0x3FFFFF;
  }

  // get pos as pointer
  uint8_t* _pos = (uint8_t*)&pos;

  // send command
  l6470_transmit((uint8_t)(L6470_CMD_GOTO_DIR | dir));

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

void l6470_reset_pos() {
  // send command
  l6470_transmit(L6470_CMD_RESET_POS);
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

void l6470_set_abs_pos(int32_t value) {
  // set parameter
  l6470_set_param(L6470_REG_ABS_POS, (uint32_t)value);
}

/* PARAMETER HANDLING */

int32_t l6470_get_abs_pos() {
  // read parameter
  uint32_t value = l6470_get_param(L6470_REG_ABS_POS);

  // fix sign for 22bit 2s complement
  if (value & 0x00200000) {
    value |= 0xffc00000;
  }

  return (int32_t)value;
}

void l6470_set_mark(int32_t value) {
  // set parameter
  l6470_set_param(L6470_REG_MARK, (uint32_t)value);
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
  l6470_set_param(L6470_REG_ACC, steps_per_tick);
}

uint32_t l6470_get_acceleration() {
  // read parameter
  return l6470_get_param(L6470_REG_ACC);
}

void l6470_set_deceleration(uint32_t steps_per_tick) {
  // set param
  l6470_set_param(L6470_REG_DECEL, steps_per_tick);
}

uint32_t l6470_get_deceleration() {
  // read parameter
  return l6470_get_param(L6470_REG_DECEL);
}

void l6470_set_max_speed(uint16_t steps_per_tick) {
  // set param
  l6470_set_param(L6470_REG_MAX_SPEED, steps_per_tick);
}

uint16_t l6470_get_max_speed() {
  // read parameter
  return (uint16_t)l6470_get_param(L6470_REG_MAX_SPEED);
}

void l6470_set_min_speed(uint16_t steps_per_tick) {
  // get param and clear min speed
  uint16_t current = (uint16_t)l6470_get_param(L6470_REG_MIN_SPEED) & (uint16_t)0x1000;

  // set new value and respect mask
  current |= (steps_per_tick & 0xFFF);

  // set param
  l6470_set_param(L6470_REG_MIN_SPEED, current);
}

uint16_t l6470_get_min_speed() {
  // read parameter
  return (uint16_t)l6470_get_param(L6470_REG_MIN_SPEED);
}

void l6470_set_fs_speed(uint16_t steps_per_tick) {
  // set param
  l6470_set_param(L6470_REG_FS_SPD, steps_per_tick);
}

uint16_t l6470_get_fs_speed() {
  // read parameter
  return (uint16_t)l6470_get_param(L6470_REG_FS_SPD);
}

void l6470_set_step_mode(l6470_step_mode_t value) {
  // get current register and clear step mode
  uint8_t current = (uint8_t)l6470_get_param(L6470_REG_STEP_MODE) & (uint8_t)0xF8;

  // set new value and respect mask
  current |= (value & 0x07);

  // update register
  l6470_set_param(L6470_REG_STEP_MODE, current);
}

l6470_step_mode_t l6470_set_get_step_mode() {
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

/* CALCULATION */

uint32_t l6470_calc_speed(double steps_per_sec) {
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

uint16_t l6470_calc_acceleration(double steps_per_sec_per_sec) {
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

uint16_t l6470_calc_deceleration(double steps_per_sec_per_sec) {
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

uint16_t l6470_calc_max_speed(double steps_per_sec) {
  // calculate internal value
  uint16_t value = (uint16_t)(steps_per_sec * .065536);

  // clamp to 10 bits
  if (value > 0x3FF) {
    return 0x3FF;
  };

  return value;
}

double l6470_parse_max_speed(uint16_t steps_per_sec) {
  // calculate real value
  return (steps_per_sec & 0x000003FF) / 0.065536;
}

uint16_t l6470_calc_min_speed(double steps_per_sec) {
  // calculate internal value
  uint16_t value = (uint16_t)(steps_per_sec / 0.238);

  // clamp to 12 bits
  if (value > 0xFFF) {
    value = 0xFFF;
  }

  return value;
}

double l6470_parse_min_speed(uint16_t steps_per_sec) {
  // calculate real value
  return (steps_per_sec & 0x00000FFF) * 0.238;
}

uint16_t l6470_calc_fs_peed(double steps_per_sec) {
  // calculate internal value
  uint16_t value = (uint16_t)((steps_per_sec * .065536) - .5);

  // clamp to 10 bits
  if (value > 0x3FF) {
    return 0x3FF;
  };

  return value;
}

double l6470_parse_fs_speed(uint16_t steps_per_sec) {
  // calculate real value
  return ((steps_per_sec & 0x000003FF) + 0.5) / 0.065536;
}
