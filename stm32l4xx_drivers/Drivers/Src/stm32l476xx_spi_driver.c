#include "stm32l476xx_spi_driver.h"
#include "stm32l476xx.h"
#include <stdint.h>

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
  if (EnorDi == ENABLE) {
    if (pSPIx == SPI1) {
      SPI1_PCLK_EN();
    } else if (pSPIx == SPI2) {
      SPI2_PCLK_EN();
    } else if (pSPIx == SPI3) {
      SPI3_PCLK_EN();
    }
  } else {
    if (pSPIx == SPI1) {
      SPI1_PCLK_DI();
    } else if (pSPIx == SPI2) {
      SPI2_PCLK_DI();
    } else if (pSPIx == SPI3) {
      SPI3_PCLK_DI();
    }
  }
}

void SPI_Init(SPI_Handle_t *pSPIHandle) {
  uint32_t tempreg = 0;

  SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
  tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

  if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
    /* BIDI clear */
    tempreg &= ~(1 << SPI_CR1_BIDIMODE);
  } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
    /* BIDI set */
    tempreg |= (1 << SPI_CR1_BIDIMODE);
  } else if (pSPIHandle->SPIConfig.SPI_BusConfig ==
             SPI_BUS_CONFIG_SIMPLEX_RX_ONLY) {
    /* BIDI clear, RXONLY set */
    tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    tempreg |= (1 << SPI_CR1_RXONLY);
  }

  tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
  tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
  tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
  tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

  pSPIHandle->pSPIx->CR1 = tempreg;

  pSPIHandle->pSPIx->CR2 &= ~(0xF << SPI_CR2_DS);
  pSPIHandle->pSPIx->CR2 |=
      (((uint32_t)pSPIHandle->SPIConfig.SPI_DS) << SPI_CR2_DS);
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
  if (pSPIx->SR & FlagName) {
    return FLAG_SET;
  }
  return FLAG_RESET;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
  if (pSPIx == SPI1) {
    SPI1_REG_RESET();
  } else if (pSPIx == SPI2) {
    SPI2_REG_RESET();
  } else if (pSPIx == SPI3) {
    SPI3_REG_RESET();
  }
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
  while (len > 0) {
    while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
      ;

    /* Only for 16 or 8 bit DS */
    if (((pSPIx->CR2 >> SPI_CR2_DS) & 0xF) == SPI_DS_16) {
      /* 16 bit */
      pSPIx->DR = *((uint16_t *)pTxBuffer);
      len -= 2;
      (uint16_t *)pTxBuffer++;
    } else {
      /* 8 bit */
      pSPIx->DR = *pTxBuffer;
      len--;
      pTxBuffer++;
    }
  }
}

void SPI_PerpheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
  if (EnorDi == ENABLE) {
    pSPIx->CR1 |= (1 << SPI_CR1_SPE);
  } else {
    pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
  }
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
  if (EnorDi == ENABLE) {
    pSPIx->CR1 |= (1 << SPI_CR1_SSI);
  } else {
    pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
  }
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
  if (EnorDi == ENABLE) {
    pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
  } else {
    pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
  }
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {
  while (len > 0) {
    while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
      ;

    /* Only for 16 or 8 bit DS */
    if (((pSPIx->CR2 >> SPI_CR2_DS) & 0xF) == SPI_DS_16) {
      /* 16 bit */
      *((uint16_t *)pRxBuffer) = pSPIx->DR;
      len -= 2;
      (uint16_t *)pRxBuffer++;
    } else {
      /* 8 bit */
      *pRxBuffer = pSPIx->DR;
      len--;
      pRxBuffer++;
    }
  }
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {}
void SPI_IRQHandling(SPI_Handle_t *pHandle) {}