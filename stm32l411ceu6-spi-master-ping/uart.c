#include "uart.h"
#include "clock.h"

void uart_config(void) {
  /* Enable UART clock */
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  /* Enable GPIOA Clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* Configure PA2 as AF for UART */
  GPIOA->MODER &= ~(GPIO_MODER_MODE2);
  GPIOA->MODER |= GPIO_MODER_MODE2_1;
  GPIOA->OTYPER &= GPIO_OTYPER_OT2;

  /* Configure AF7 (USART2) for PA2 */
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2);
  GPIOA->AFR[0] |=
      (GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL2_2);

  /* Default UART settings after reset:
  - 16X Oversampling
  - 1 Start bit
  - 8 data bits
  - no stop bit
  - no parity bit
  */

  /* Configuration of baudrate 9600
  For oversampling 16x:
  USARTDIV = 32MHz / 9600 baud */
  uint16_t usart_div = SYSTEM_CLOCK / (9600);
  USART2->BRR = usart_div;
  /* Enable UART and TX over UART */
  USART2->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

int uart_transmit(char *data, int size) {
  int count = size;
  while (count--) {
    while (!(USART2->SR & USART_SR_TXE))
      __NOP();
    USART2->DR = *data++;
  }
  /* This line is inserted so UART will complete transmitting character
  before
   * MCU will go to sleep */
  while ((USART2->SR & USART_SR_TC) == 0)
    __NOP();
  return size;
}