#ifndef STM32L476XX_GPIO_DRIVER_H__
#define STM32L476XX_GPIO_DRIVER_H__

#include "stm32l476xx.h"
#include <stdint.h>

typedef struct GPIO_PinConfig_s {
  uint8_t GPIO_PinNumber;      /* Values from @GPIO_PIN_NUMBERS */
  uint8_t GPIO_PinMode;        /* Values from @GPIO_PIN_MODES */
  uint8_t GPIO_PinSpeed;       /* Values from @GPIO_PIN_SPEED */
  uint8_t GPIO_PinPuPdControl; /* Values from @GPIO_PIN_PULL */
  uint8_t GPIO_PinOPType;      /* Values from @GPIO_PIN_OT */
  uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct GPIO_Handle_s {
  GPIO_RegDef_t *pGPIOx; /* Base address of GPIO port for a pin */
  GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/* GPIO pin numbers @GPIO_PIN_NUMBERS */

#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/* GPIO pin possible modes @GPIO_PIN_MODES */

#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6

/* GPIO pin possible output types @GPIO_PIN_OT */

#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

/* GPIO pin possible output speed @GPIO_PIN_SPEED */

#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

/* GPIO pin pull up pull down configs @GPIO_PIN_PULL */

#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber,
                           uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIO_PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t GPIO_PinNumber);

#endif /* STM32L476XX_GPIO_DRIVER_H__ */
