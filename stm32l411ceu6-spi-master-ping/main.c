#include "main.h"
#include "clock.h"
#include "led.h"
#include "spi.h"
#include "system.h"
#include "uart.h"
#include <stdint.h>

int main(void) {
  clock_config();
  led_config();
  uart_config();
  spi_config();

  /* 16MHz 1KHz firing requency (each ms) */
  SysTick_Config(16000);
  __enable_irq();
  spi_enable();
  uint8_t tx_buffer[8] = "PINGCRAP";
  uint8_t rx_buffer[8] = {0};
  spi_enable();
  while (1) {
    led_toggle();
    spi_chip_select();
    spi_transmit_receive(tx_buffer, 8, rx_buffer, 8);
    spi_chip_deselect();
    uart_transmit((char *)rx_buffer, 8);
    delay_ms(1000);
  }
}
