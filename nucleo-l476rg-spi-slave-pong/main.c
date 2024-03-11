#include "main.h"
#include "clock.h"
#include "led.h"
#include "mco.h"
#include "spi.h"
#include "system.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

int main(void) {
  clock_config();
  mco_config();
  led_config();
  uart_config();
  spi_config();
  uint8_t buffer[4] = {0};

  /* 64MHz 1KHz firing requency (each ms) */
  SysTick_Config(64000);
  __enable_irq();
  uint8_t tx_buffer[8] = "CRAPPONG";
  uint8_t rx_buffer[8] = {0};
  spi_enable();
  while (1) {
    spi_transmit_receive(tx_buffer, 8, rx_buffer, 8);
    uart_transmit((char *)rx_buffer, 8);
  }
}
