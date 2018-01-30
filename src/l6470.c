#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <naos.h>

#include "l6470.h"
#include "l6470_constants.h"

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

    // EL_POS is the current electrical position in the step generation cycle. It can
    // be set when the motor is not in motion. Value is 0 on power up.
    case L6470_REG_EL_POS:
      return l6470_transfer(value, 9);

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

    // KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
    //  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
    // The implications of different KVAL settings is too complex to dig into here, but
    //  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
    //  HOLD may result in excessive power dissipation when the motor is not running.
    case L6470_REG_KVAL_HOLD:
      return l6470_transfer(value, 8);
    case L6470_REG_KVAL_RUN:
      return l6470_transfer(value, 8);
    case L6470_REG_KVAL_ACC:
      return l6470_transfer(value, 8);
    case L6470_REG_KVAL_DEC:
      return l6470_transfer(value, 8);

    // INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
    //  compensation functionality. Please see the datasheet for details of this
    //  function- it is too complex to discuss here. Default values seem to work
    //  well enough.
    case L6470_REG_INT_SPD:
      return l6470_transfer(value, 14);
    case L6470_REG_ST_SLP:
      return l6470_transfer(value, 8);
    case L6470_REG_FN_SLP_ACC:
      return l6470_transfer(value, 8);
    case L6470_REG_FN_SLP_DEC:
      return l6470_transfer(value, 8);

    // K_THERM is motor winding thermal drift compensation. Please see the datasheet
    //  for full details on operation- the default value should be okay for most users.
    case L6470_REG_K_THERM:
      value &= 0x0F;
      return l6470_transfer(value, 8);

    // ADC_OUT is a read-only register containing the result of the ADC measurements.
    //  This is less useful than it sounds; see the datasheet for more information.
    case L6470_REG_ADC_OUT:
      return l6470_transfer(value, 8);

    // Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
    //  A set of defined constants is provided for the user's convenience. Default
    //  value is 3.375A- 0x08. This is a 4-bit value.
    case L6470_REG_OCD_TH:
      value &= 0x0F;
      return l6470_transfer(value, 8);

    // Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
    //  4A in 31.25mA steps. This is a 7-bit value.
    case L6470_REG_STALL_TH:
      value &= 0x7F;
      return l6470_transfer(value, 8);

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

    // ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
    //  is provided to make this easy to interpret. By default, ALL alarms will trigger the
    //  FLAG pin.
    case L6470_REG_ALARM_EN:
      return l6470_transfer(value, 8);

    // CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
    //  set of reasonably self-explanatory constants is provided, but users should refer
    //  to the datasheet before modifying the contents of this register to be certain they
    //  understand the implications of their modifications. Value on boot is 0x2E88; this
    //  can be a useful way to verify proper start up and operation of the dSPIN chip.
    case L6470_REG_CONFIG:
      return l6470_transfer(value, 16);

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

// TODO: RUN 0x50
// TODO: STEP_CLOCK 0x58

void l6470_move(uint8_t dir, uint32_t steps) {
  // send command
  l6470_transmit(L6470_CMD_MOVE);

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

// TODO: GOTO_DIR 0x68
// TODO: GO_UNTIL 0x82
// TODO: RELEASE_SW 0x92
// TODO: GO_HOME 0x70
// TODO: GO_MARK 0x78
// TODO: RESET_POS 0xD8
// TODO: RESET_DEVICE 0xC0
// TODO: SOFT_STOP 0xB0
// TODO: HARD_STOP 0xB8
// TODO: SOFT_HIZ 0xA0
// TODO: HARD_HIZ 0xA8

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

// TODO: EL_POS 0x02

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

// TODO: SPEED 0x04
// TODO: ACC 0x05
// TODO: DECEL 0x06
// TODO: MAX_SPEED 0x07
// TODO: MIN_SPEED 0x08
// TODO: FS_SPD 0x15
// TODO: KVAL_HOLD 0x09
// TODO: KVAL_RUN 0x0A
// TODO: KVAL_ACC 0x0B
// TODO: KVAL_DEC 0x0C
// TODO: INT_SPD 0x0D
// TODO: ST_SLP 0x0E
// TODO: FN_SLP_ACC 0x0F
// TODO: FN_SLP_DEC 0x10
// TODO: K_THERM 0x11
// TODO: ADC_OUT 0x12
// TODO: OCD_TH 0x13
// TODO: STALL_TH 0x14
// TODO: STEP_MODE
// TODO: ALARM_EN 0x17
// TODO: CONFIG 0x18
// TODO: STATUS 0x19

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
