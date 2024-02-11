#ifndef STM32L476XX_H__
#define STM32L476XX_H__

/* Base addresses of FLASH, SRAM and ROM */

#include <stdint.h>

#define FLASH1_BASEADDR 0x08000000U
#define FLASH2_BASEADDR 0x08080000U
#define FLASH FLASH1_BASEADDR
#define SRAM1_BASEADDR 0x200000000U
#define SRAM2_BASEADDR 0x100000000U
#define SRAM SRAM1_BASEADDR
#define ROM_BASE 0x1FFF0000U

/* AHBx and APBx Bus peripheral base adresses */

#define PERIPH_BASEADDR 0x40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 0x40010000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x48000000U

/* Base adresses of peripherals on APB1 bus */

#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASEADDR + 0x5000)

#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00)

/* Base adresses of peripherals on APB2 bus */

#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + 0x0000)
#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + 0x0400)
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x3800)

/* Base adresses of peripherals on AHB1 bus */

#define RCC_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000)

/* Base adresses of peripherals on AHB2 bus */

#define GPIOA_BASEADDR (AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB2PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB2PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB2PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB2PERIPH_BASEADDR + 0x1C00)

/* Peripheral registers definition structures */

typedef struct RCC_RegDef_s {
  volatile uint32_t CR;
  volatile uint32_t ICSCR;
  volatile uint32_t CFGR;
  volatile uint32_t PLL_CFGR;
  volatile uint32_t PLLSAI1CFGR;
  volatile uint32_t PLLSAI2CFGR;
  volatile uint32_t CIER;
  volatile uint32_t CIFR;
  volatile uint32_t CICR;
  volatile uint32_t RESERVED0;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  volatile uint32_t AHB3RSTR;
  volatile uint32_t RESERVED1;
  volatile uint32_t APB1RSTR1;
  volatile uint32_t APB1RSTR2;
  volatile uint32_t APB2RSTR;
  volatile uint32_t RESRVED3;
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  volatile uint32_t AHB3ENR;
  volatile uint32_t RESRVED4;
  volatile uint32_t APB1ENR1;
  volatile uint32_t APB1ENR2;
  volatile uint32_t APB2ENR;
  volatile uint32_t RESRVED5;
  volatile uint32_t AHB1SMENR;
  volatile uint32_t AHB2SMENR;
  volatile uint32_t AHB3SMENR;
  volatile uint32_t RESRVED6;
  volatile uint32_t APB1SMENR1;
  volatile uint32_t APB1SMENR2;
  volatile uint32_t APB2SMENR;
  volatile uint32_t RESRVED7;
  volatile uint32_t CCIPR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  volatile uint32_t CRRCR;
  volatile uint32_t CCIPR2;
} RCC_RegDef_t;

typedef struct GPIO_RegDef_s {
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
  volatile uint32_t BRR;
  volatile uint32_t ASCR;
} GPIO_RegDef_t;

/* Peripheral definitions */

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)

/* GPIO peripheral clock Enable */

#define GPIOA_PCLK_EN() (RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB2ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB2ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB2ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB2ENR |= (1 << 7))

/* I2C peripheral clock Enable */

#define I2C1_PCLK_EN() (RCC->APB1ENR1 |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR1 |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR1 |= (1 << 23))

/* SPI peripheral clock Enable */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12));
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14));
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15));

/* UART peripheral clock Enable */

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))

/* SYSCFG peripheral clock Enable */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 0))

/* GPIO peripheral clock Disable */

#define GPIOA_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 7))

/* I2C peripheral clock Disable */

#define I2C1_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 23))

/* SPI peripheral clock Disable */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12));
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14));
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15));

/* UART peripheral clock Disable */

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))

/* SYSCFG peripheral clock Enable */

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 0))

/* Reset GPIOx peripherals */

#define GPIOA_REG_RESET()                                                      \
  do {                                                                         \
    (RCC->AHB2RSTR |= (1 << 0));                                               \
    (RCC->AHB2RSTR &= ~(1 << 0));                                              \
  } while (0)

#define GPIOB_REG_RESET()                                                      \
  do {                                                                         \
    (RCC->AHB2RSTR |= (1 << 1));                                               \
    (RCC->AHB2RSTR &= ~(1 << 1));                                              \
  } while (0)

#define GPIOC_REG_RESET()                                                      \
  do {                                                                         \
    (RCC->AHB2RSTR |= (1 << 2));                                               \
    (RCC->AHB2RSTR &= ~(1 << 2));                                              \
  } while (0)

#define GPIOD_REG_RESET()                                                      \
  do {                                                                         \
    (RCC->AHB2RSTR |= (1 << 3));                                               \
    (RCC->AHB2RSTR &= ~(1 << 3));                                              \
  } while (0)

#define GPIOE_REG_RESET()                                                      \
  do {                                                                         \
    (RCC->AHB2RSTR |= (1 << 4));                                               \
    (RCC->AHB2RSTR &= ~(1 << 4));                                              \
  } while (0)

#define GPIOF_REG_RESET()                                                      \
  do {                                                                         \
    (RCC->AHB2RSTR |= (1 << 5));                                               \
    (RCC->AHB2RSTR &= ~(1 << 5));                                              \
  } while (0)

#define GPIOG_REG_RESET()                                                      \
  do {                                                                         \
    (RCC->AHB2RSTR |= (1 << 6));                                               \
    (RCC->AHB2RSTR &= ~(1 << 6));                                              \
  } while (0)

#define GPIOH_REG_RESET()                                                      \
  do {                                                                         \
    (RCC->AHB2RSTR |= (1 << 7));                                               \
    (RCC->AHB2RSTR &= ~(1 << 7));                                              \
  } while (0)

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#include "stm32l476xx_gpio_driver.h"

#endif /* STM32L476XX_H__ */