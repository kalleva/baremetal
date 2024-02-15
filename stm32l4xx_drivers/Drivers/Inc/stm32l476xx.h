#ifndef STM32L476XX_H__
#define STM32L476XX_H__

#include "stm32l476xx.h"
#include <stdint.h>

/* ARM Cortex Mx NVIC ISERx register addresses */

#define NVIC_ISER0 ((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1 ((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2 ((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3 ((volatile uint32_t *)0xE000E10C)

/* ARM Cortex Mx NVIC ICERx register addresses */

#define NVIC_ICER0 ((volatile uint32_t *)0xE000E180)
#define NVIC_ICER1 ((volatile uint32_t *)0xE000E184)
#define NVIC_ICER2 ((volatile uint32_t *)0xE000E188)
#define NVIC_ICER3 ((volatile uint32_t *)0xE000E18C)

/* ARM Cortex Mx NVIC Priority register addresses */

#define NVIC_PR_BASE_ADDR ((volatile uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4

/* Base addresses of FLASH, SRAM and ROM */
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

typedef struct EXTI_RegDef_s {
  volatile uint32_t IMR1;
  volatile uint32_t EMR1;
  volatile uint32_t RTSR1;
  volatile uint32_t FTSR1;
  volatile uint32_t SWIER1;
  volatile uint32_t PR1;
  volatile uint32_t EXTI_RESERVED1[2];
  volatile uint32_t IMR2;
  volatile uint32_t EMR2;
  volatile uint32_t RTSR2;
  volatile uint32_t FTSR2;
  volatile uint32_t SWIER2;
  volatile uint32_t PR2;
} EXTI_RegDef_t;

typedef struct SYSCFG_RegDef_s {
  volatile uint32_t MEMRMP;
  volatile uint32_t CFGR1;
  volatile uint32_t EXTICR[4];
  volatile uint32_t SCSR;
  volatile uint32_t CFGR2;
  volatile uint32_t SWPR;
  volatile uint32_t SKR;
} SYSCFG_RegDef_t;

typedef struct SPI_RegDef_s {
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
} SPI_RegDef_t;

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

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)

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

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR1 |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR1 |= (1 << 15))

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

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR1 &= ~(1 << 15))

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

#define SPI1_REG_RESET()                                                       \
  do {                                                                         \
    (RCC->APB2RSTR |= (1 << 12));                                              \
    (RCC->APB2RSTR &= ~(1 << 12));                                             \
  } while (0)

#define SPI2_REG_RESET()                                                       \
  do {                                                                         \
    (RCC->APB1RSTR1 |= (1 << 14));                                             \
    (RCC->APB1RSTR1 &= ~(1 << 14));                                            \
  } while (0)

#define SPI3_REG_RESET()                                                       \
  do {                                                                         \
    (RCC->APB1RSTR1 |= (1 << 15));                                             \
    (RCC->APB1RSTR1 &= ~(1 << 15));                                            \
  } while (0)

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET RESET
#define FLAG_SET SET

#define GPIO_BASEADDR_TO_CODE(x)                                               \
  ((x == GPIOA)   ? 0                                                          \
   : (x == GPIOB) ? 1                                                          \
   : (x == GPIOC) ? 2                                                          \
   : (x == GPIOD) ? 3                                                          \
   : (x == GPIOE) ? 4                                                          \
   : (x == GPIOF) ? 5                                                          \
   : (x == GPIOH) ? 6                                                          \
   : (x == GPIOG) ? 7                                                          \
                  : 0)

/* Interrupts */

#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40

#define NVIC_IRQ_PRI0 0
#define NVIC_IRQ_PRI1 1
#define NVIC_IRQ_PRI2 2
#define NVIC_IRQ_PRI3 3
#define NVIC_IRQ_PRI4 4
#define NVIC_IRQ_PRI5 5
#define NVIC_IRQ_PRI6 6
#define NVIC_IRQ_PRI7 7
#define NVIC_IRQ_PRI8 8
#define NVIC_IRQ_PRI9 9
#define NVIC_IRQ_PRI10 10
#define NVIC_IRQ_PRI11 11
#define NVIC_IRQ_PRI12 12
#define NVIC_IRQ_PRI13 13
#define NVIC_IRQ_PRI14 14
#define NVIC_IRQ_PRI15 15

#include "stm32l476xx_gpio_driver.h"
#include "stm32l476xx_spi_driver.h"

#endif /* STM32L476XX_H__ */