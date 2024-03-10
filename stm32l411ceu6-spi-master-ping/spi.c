#include "spi.h"
#include "stm32f411xe.h"
#include <stdint.h>

void spi_config(void) {
  /* 1. Configure GPIO pins */
  /* AF5 */
  /* SPI1_NSS PA4 */
  /* SPI1_SCK PA5 */
  /* SPI1_MISO PA6 */
  /* SPI1_MOSI PA7 */
  /* For SPI configure pins as push pull */
  RCC->AHB2ENR |= RCC_AHB1ENR_GPIOAEN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 |
                    GPIO_MODER_MODE7);

  /* Output for pin 4, Alternate function for pins 5, 6, 7, 0xb10 */
  GPIOA->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_1 |
                   GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);

  /* Configure output pins as output push-pull */
  GPIOA->OTYPER &=
      ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);

  /* Configure pins as no pull-up no pull-down */
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 |
                    GPIO_PUPDR_PUPD7);

  /* Configure alternate function register to set mode 5 for these pins */
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);

  /* AF5 0b0101 */
  GPIOA->AFR[0] |=
      (GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL6_2 |
       GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL7_2 | GPIO_AFRL_AFSEL7_0);

  /* Put CS High */
  GPIOA->ODR |= GPIO_ODR_OD4;

  /* Enable SPI clock */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  /* 2. Configure SPI_CR1 register */
  uint32_t reg = 0;

  /* 2.1 Configure clock baud rate with BR bits */
  /* APB2 is 4MHz desired SPI freq is 500kHz so BRR 0b010 */
  reg |= SPI_CR1_BR_2;

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
  /* Use sw slave select with active 0, so SSM 1, SSI 1 */
  reg |= SPI_CR1_SSM | SPI_CR1_SSI;

  /* 2.7 Configure MSTR bit */
  reg |= SPI_CR1_MSTR;

  /* 2.8 Configure DFF bit for 8bit data format*/

  SPI1->CR1 = reg;

  reg = 0;

  /* 3. Configure SPI_CR2 register */

  /* 3.1 Configure SSOE */
  /* Manual slave select so SSOE bit is 0 */

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

  /* Wait first for TXE */
  while (!(SPI1->SR & SPI_SR_TXE))
    ;

  /* And only after that wait for BSY */
  /* Without this last byte of transmisson is not received */
  while (SPI1->SR & SPI_SR_BSY)
    ;
}