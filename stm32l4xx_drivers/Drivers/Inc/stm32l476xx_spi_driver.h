#ifndef STM32L476XX_SPI_DRIVER__
#define STM32L476XX_SPI_DRIVER__

#include "stm32l476xx.h"
#include <stdint.h>

typedef struct SPI_Config_s {
  uint8_t SPI_DeviceMode;
  uint8_t SPI_BusConfig;
  uint8_t SPI_SclkSpeed;
  uint8_t SPI_DS;
  uint8_t SPI_CPOL;
  uint8_t SPI_CPHA;
  uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct SPI_Handle_s {
  SPI_RegDef_t *pSPIx;
  SPI_Config_t SPIConfig;
} SPI_Handle_t;

#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0

#define SPI_BUS_CONFIG_FD 0
#define SPI_BUS_CONFIG_HD 1
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY 2

#define SPI_SCLK_SPEED_DIV2 0
#define SPI_SCLK_SPEED_DIV4 1
#define SPI_SCLK_SPEED_DIV8 2
#define SPI_SCLK_SPEED_DIV16 3
#define SPI_SCLK_SPEED_DIV32 4
#define SPI_SCLK_SPEED_DIV64 5
#define SPI_SCLK_SPEED_DIV128 6
#define SPI_SCLK_SPEED_DIV256 7

#define SPI_DS_4 3
#define SPI_DS_5 4
#define SPI_DS_6 5
#define SPI_DS_7 6
#define SPI_DS_8 7
#define SPI_DS_9 8
#define SPI_DS_10 9
#define SPI_DS_11 10
#define SPI_DS_12 11
#define SPI_DS_13 12
#define SPI_DS_14 13
#define SPI_DS_15 14
#define SPI_DS_16 15

#define SPI_DFF_8BITS 0
#define SPI_DFF_16BITS 1

#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

#define SPI_SSM_EN 1
#define SPI_SSM_DI 0

/* SPI register bit positions */

#define SPI_CR1_CPHA 0
#define SPI_CR1_CPOL 1
#define SPI_CR1_MSTR 2
#define SPI_CR1_BR 3
#define SPI_CR1_SPE 6
#define SPI_CR1_LSBFIRST 8
#define SPI_CR1_SSI 8
#define SPI_CR1_SSM 9
#define SPI_CR1_RXONLY 10
#define SPI_CR1_CRCL 11
#define SPI_CR1_CRCNEXT 12
#define SPI_CR1_CRCEN 13
#define SPI_CR1_BIDIOE 14
#define SPI_CR1_BIDIMODE 15

#define SPI_CR2_RXDMAEN 0
#define SPI_CR2_TXDMAEN 1
#define SPI_CR2_SSOE 2
#define SPI_CR2_NSSP 3
#define SPI_CR2_FRF 4
#define SPI_CR2_ERRIE 5
#define SPI_CR2_RXNEIE 6
#define SPI_CR2_TXNEIE 7
#define SPI_CR2_DS 8
#define SPI_CR2_FRXTH 12
#define SPI_CR2_LDMA_RX 13
#define SPI_CR2_LDMA_TX 14

#define SPI_CR2_DS_SIZE 4

#define SPI_SR_RXNE 0
#define SPI_SR_TXE 1
#define SPI_SR_CRCERR 4
#define SPI_SR_MODF 5
#define SPI_SR_OVR 6
#define SPI_SR_BUSY 7
#define SPI_SR_FRE 8
#define SPI_SR_FRLVL 9
#define SPI_SR_FTLVL 11

#define SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG (1 << SPI_SR_BUSY)

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

void SPI_PerpheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

#endif /* STM32L476XX_SPI_DRIVER__ */