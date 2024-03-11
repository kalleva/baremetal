#ifndef SPI_H___
#define SPI_H___

#include "main.h"

#define spi_enable(void) (SPI1->CR1 |= SPI_CR1_SPE)
#define spi_disable(void) (SPI1->CR1 &= ~SPI_CR1_SPE)

void spi_config(void);
void spi_transmit(uint8_t *buffer, uint32_t len);
void spi_master_transmit_receive(uint8_t *tx_buffer, uint32_t tx_len,
                                 uint8_t *rx_buffer, uint32_t rx_len);
void spi_receive(uint8_t *buffer, uint32_t lenth);
void spi_transmit_slave(uint8_t *buffer, uint32_t len);
void spi_transmit_receive(uint8_t *tx_buffer, uint32_t tx_length,
                          uint8_t *rx_buffer, uint32_t rx_length);

#endif /* SPI_H___ */