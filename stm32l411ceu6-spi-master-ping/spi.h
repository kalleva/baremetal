#ifndef SPI_H__
#define SPI_H__

#include "main.h"
#include <stdint.h>

#define spi_enable(void) (SPI1->CR1 |= SPI_CR1_SPE)
#define spi_disable(void) (SPI1->CR1 &= ~SPI_CR1_SPE)
#define spi_chip_select(void) (GPIOA->ODR &= ~GPIO_ODR_OD4)
#define spi_chip_deselect(void) (GPIOA->ODR |= GPIO_ODR_OD4)

void spi_config(void);
void spi_transmit(uint8_t *buffer, uint32_t len);
void spi_receive_master(uint8_t *buffer, uint32_t len);
void spi_transmit_receive(uint8_t *tx_buffer, uint32_t tx_length,
                          uint8_t *rx_buffer, uint32_t rx_length);

#endif /* SPI_H__ */