#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <naos.h>
#include <string.h>

#include "l6470.h"

#define L6470_MISO 13
#define L6470_MOSI 12
#define L6470_SCLK 14
#define L6470_CS 32

#define L6470_RESET GPIO_NUM_18

spi_device_handle_t l6470_spi;

void l6470_init() {
  // prepare bus config
  spi_bus_config_t bus_config = {.miso_io_num = L6470_MISO,
                                 .mosi_io_num = L6470_MOSI,
                                 .sclk_io_num = L6470_SCLK,
                                 .quadwp_io_num = -1,
                                 .quadhd_io_num = -1};

  // Initialize the SPI bus
  ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 1));

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
  gpio_set_direction(L6470_RESET, GPIO_MODE_OUTPUT);

  // reset chip
  gpio_set_level(L6470_RESET, 0);
  naos_delay(100);
  gpio_set_level(L6470_RESET, 1);
  naos_delay(100);
}

static void l6470_send(const void *tx, size_t tx_len, void *rx, size_t rx_len) {
  // prepare transaction
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));

  // set tx
  t.tx_buffer = tx;
  t.length = tx_len;

  // set rx
  t.rx_buffer = rx;
  t.rxlength = rx_len;

  // send command
  ESP_ERROR_CHECK(spi_device_transmit(l6470_spi, &t));
}

l6470_status_t l6470_get_status() {
  uint8_t tx = 0xD0;
  l6470_status_t status;
  l6470_send(&tx, 1, &status.data, 2);
  return status;
}
