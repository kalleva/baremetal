#ifndef SPI_H__
#define SPI_H__

#include "main.h"
#include <stdint.h>

#define spi_enable(void) (SPI1->CR1 |= SPI_CR1_SPE)
#define spi_disable(void) (SPI1->CR1 &= ~SPI_CR1_SPE)

void spi_config(void);
void spi_transmit(uint8_t *buffer, uint32_t len);

#endif /* SPI_H__ */