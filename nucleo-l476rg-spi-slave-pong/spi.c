#include "spi.h"
#include "uart.h"
#include <stdint.h>

void spi_config(void) {
  /* 1. Configure GPIO pins */
  /* AF5 */
  /* SPI1_NSS PA4 */
  /* SPI1_SCK PA5 */
  /* SPI1_MISO PA6 */
  /* SPI1_MOSI PA7 */
  /* For SPI configure pins as push pull */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 |
                    GPIO_MODER_MODE7);

  /* Pins 4, 5, 6, 7 as Alternate function 0xb10 */
  GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 |
                   GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);

  /* Configure pins as no pull-up no pull-down */

  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 |
                    GPIO_PUPDR_PUPD7);

  /* Configure alternate function register to set mode 5 for these pins */

  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6 |
                     GPIO_AFRL_AFSEL7);

  /* AF5 0b0101 */
  GPIOA->AFR[0] |=
      (GPIO_AFRL_AFSEL4_2 | GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL5_2 |
       GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL6_2 | GPIO_AFRL_AFSEL6_0 |
       GPIO_AFRL_AFSEL7_2 | GPIO_AFRL_AFSEL7_0);

  /* Enable SPI clock */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  /* 2. Configure SPI_CR1 register */
  uint32_t reg = 0;

  /* 2.1 Configure clock baud rate with BR bits */
  /* System clock is 64 MHz */
  /* Baudrate of the slave is not set because SCK is obtained from master */

  /* 2.2 Configure CPOL and CPHA bits */
  /* CPOL = 0 - to zero on idel and CPHA = 0 first clock transition is first
   * data capture. No need to configure  */

  /* 2.3 Select mode (full duplex, half duplex, simplex) with
    RXONLY or BIDIMODE and BIDIOE bits */
  /* Transmit only -> RXONLY - 0 */
  /* 2 line unidirectional BIDIMODE - 0 */

  /* 2.4 Configure LSBFIRST bit */
  /* MSB first */

  /* 2.5 Configure CRCL and CRCEN bits */
  /* Not using CRC and CRCEN */

  /* 2.6 Configure SSM and SSI  */
  /* Use sw slave select so both of these are 1 */

  /* 2.7 Configure MSTR bit */

  /* 3. Configure SPI_CR2 register */

  /* 3.1 Configure DS bits for data length */
  /* 8 bits 0b0111 */
  reg |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;

  /* 3.2 Configure SSOE */
  /* Slave select so SSOE bit is 0 */

  /* 3.3 Configure Rx FIFO queue for 8bit data receive*/
  reg |= SPI_CR2_FRXTH;

  /* 3.3 Configure NSSP */

  SPI1->CR2 = reg;
}

void spi_transmit_receive(uint8_t *tx_buffer, uint32_t tx_length,
                          uint8_t *rx_buffer, uint32_t rx_length) {
  uint32_t count = tx_length;
  while (count--) {
    while (!(SPI1->SR & SPI_SR_TXE))
      ;
    *((uint8_t *)&(SPI1->DR)) = *tx_buffer++;

    while (!(SPI1->SR & SPI_SR_RXNE))
      ;
    *rx_buffer++ = *((uint8_t *)&(SPI1->DR));
  }
}