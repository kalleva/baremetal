#include "main.h"
#include "clock.h"
#include "led.h"
#include "spi.h"
#include "system.h"
#include "uart.h"

int main(void) {
  clock_config();
  led_config();
  uart_config();
  spi_config();

  /* 16MHz 1KHz firing requency (each ms) */
  SysTick_Config(16000);
  __enable_irq();
  spi_enable();

  while (1) {
    led_toggle();
    uart_transmit("TEST\r\n", 6);
    spi_transmit((uint8_t *)"PING", 4);
    delay_ms(1000);
  }
}
