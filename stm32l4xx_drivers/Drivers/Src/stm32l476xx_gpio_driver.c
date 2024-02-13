#include "stm32l476xx_gpio_driver.h"
#include "stm32l476xx.h"
#include <stdint.h>

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
  if (EnorDi == ENABLE) {
    if (pGPIOx == GPIOA) {
      GPIOA_PCLK_EN();
    } else if (pGPIOx == GPIOB) {
      GPIOB_PCLK_EN();
    } else if (pGPIOx == GPIOC) {
      GPIOC_PCLK_EN();
    } else if (pGPIOx == GPIOD) {
      GPIOD_PCLK_EN();
    } else if (pGPIOx == GPIOE) {
      GPIOE_PCLK_EN();
    } else if (pGPIOx == GPIOF) {
      GPIOF_PCLK_EN();
    } else if (pGPIOx == GPIOG) {
      GPIOG_PCLK_EN();
    } else if (pGPIOx == GPIOH) {
      GPIOH_PCLK_EN();
    }
  } else {
    if (pGPIOx == GPIOA) {
      GPIOA_PCLK_DI();
    } else if (pGPIOx == GPIOB) {
      GPIOB_PCLK_DI();
    } else if (pGPIOx == GPIOC) {
      GPIOC_PCLK_DI();
    } else if (pGPIOx == GPIOD) {
      GPIOD_PCLK_DI();
    } else if (pGPIOx == GPIOE) {
      GPIOE_PCLK_DI();
    } else if (pGPIOx == GPIOF) {
      GPIOF_PCLK_DI();
    } else if (pGPIOx == GPIOG) {
      GPIOG_PCLK_DI();
    } else if (pGPIOx == GPIOH) {
      GPIOH_PCLK_DI();
    }
  }
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
  uint32_t temp = 0;

  /* 1. Configure mode of gpio pin */

  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
            << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->MODER &=
        ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->MODER |= temp;
  } else {
    /* Interrupt mode */
  }

  /* 2. Configure speed */

  temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
          << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
  pGPIOHandle->pGPIOx->OSPEEDR &=
      ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
  pGPIOHandle->pGPIOx->OSPEEDR |= temp;

  /* 3. Configure pupd settings */

  temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
          << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
  pGPIOHandle->pGPIOx->PUPDR &=
      ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
  pGPIOHandle->pGPIOx->PUPDR |= temp;

  /* 4. Configure optype */

  temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
          << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  pGPIOHandle->pGPIOx->OTYPER &=
      ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  pGPIOHandle->pGPIOx->OTYPER |= temp;

  /* 5. Configure alt functionality */
  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
    uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
    uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
    pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
    pGPIOHandle->pGPIOx->AFR[temp1] |=
        pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
  }
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
  if (pGPIOx == GPIOA) {
    GPIOA_REG_RESET();
  } else if (pGPIOx == GPIOB) {
    GPIOB_REG_RESET();
  } else if (pGPIOx == GPIOC) {
    GPIOC_REG_RESET();
  } else if (pGPIOx == GPIOD) {
    GPIOD_REG_RESET();
  } else if (pGPIOx == GPIOE) {
    GPIOE_REG_RESET();
  } else if (pGPIOx == GPIOF) {
    GPIOF_REG_RESET();
  } else if (pGPIOx == GPIOG) {
    GPIOG_REG_RESET();
  } else if (pGPIOx == GPIOH) {
    GPIOH_REG_RESET();
  }
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber) {
  uint8_t value = (uint8_t)((pGPIOx->IDR >> GPIO_PinNumber) & 0x00000001);
  return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
  uint16_t value = (uint16_t)(pGPIOx->IDR);
  return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber,
                           uint8_t value) {
  if (value == GPIO_PIN_RESET) {
    pGPIOx->ODR &= ~(1 << GPIO_PinNumber);
  } else {
    pGPIOx->ODR |= value << GPIO_PinNumber;
  }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
  pGPIOx->ODR &= ~(0xF);
  pGPIOx->ODR |= value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber) {
  pGPIOx->ODR ^= 1 << GPIO_PinNumber;
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {}
void GPIO_IRQHandling(uint8_t GPIO_PinNumber) {}