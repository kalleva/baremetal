#include "stm32l476xx.h"
#include "stm32l4xx.h"

#define LED_Pin 5
#define led_toggle() (GPIOA->ODR ^= (1 << LED_Pin))

static void clock_config(void);
static void led_config(void);
static void mco_config(void);

volatile int counter = 0;

int main(void) {
  clock_config();
  led_config();
  mco_config();

  while (1) {
    led_toggle();
    for (int i = 0; i < 1000000; i++)
      __NOP();
  }
}

static void clock_config(void) {
  /* Turn on HSE clock source in bypass mode from STLink 8MHz crystal Nucleo */
  RCC->CR |= (RCC_CR_HSEBYP | RCC_CR_HSEON);

  /* Wait until HSE is ready */
  while (!(RCC->CR & RCC_CR_HSERDY))
    __NOP();

  /* Max MCU speed is 80MHz */

  /* To run at max speed select PWR dynamic voltage range 1 by writing 0b01 */
  RCC->APB1ENR1 |= (RCC_APB1ENR1_PWREN);
  // PWR->CR1 &= ~(PWR_CR1_VOS);
  PWR->CR1 |= PWR_CR1_VOS_0;

  /* Configure flash latency. PWR voltage range 1 and 80MHz -> 4 WS (5 CPU
   * cycles) on reset there is zero wait state so no need to clear it up before
   * setting */
  FLASH->ACR |= FLASH_ACR_LATENCY_4WS;

  /* Configure MCU to run at 80MHz from PLL */

  /* Disable PLL */
  RCC->CR &= ~(RCC_CR_PLLON);
  while ((RCC->CR & RCC_CR_PLLRDY))
    __NOP();

  /* Configure HSE as PLL source */
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

  /* Configure PLL parameters / (M = 2, VCO = 4MHz) * (N = 40, VCO = 160Mhz) /
  (R = 2, VCO = 80MHz) */

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
  /* 0b001 - 2 */
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_0;

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
  /* 40 = 0b0101000 */
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_3;

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLR);
  /* 2 = 0b00 so no need to set after clearing */

  /* APB1 can run at 80MHz as well so no need to divide for it */

  /* Enable PLL */
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    __NOP();

  /* Enable PLLR output */
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;

  /* Configure System Clock to use PLLCLK */

  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while (!(RCC->CFGR & RCC_CFGR_SWS_PLL))
    __NOP();
}

static void led_config(void) {
  /* Configure LED PA5 PIN */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE5);
  GPIOA->MODER |= GPIO_MODER_MODE5_0;
}

static void mco_config(void) {
  /* To check the frequncy of input clock route it out to MCO pin */
  /* Configure RCC_CFGR MCO1 bit fields to select HSE as clock source */
  RCC->CFGR &= ~(RCC_CFGR_MCOSEL);
  RCC->CFGR |= (RCC_CFGR_MCOSEL_2);

  /* Configure MC01 prescaler to divided frequency by 16 */
  RCC->CFGR &= ~(RCC_CFGR_MCO_PRE);
  RCC->CFGR |= RCC_CFGR_MCOPRE_DIV16;

  /* Configure PA8 as AF0 to output MC01 */
  /* Enable GPIOA Clock */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  /* Configure PA8 as Alternate function */
  GPIOA->MODER &= ~(GPIO_MODER_MODE8);
  GPIOA->MODER |= (GPIO_MODER_MODE8_1);

  /* Configure Alternate function register to set mode 0 for PA8 */
  GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL8);

  /* Connect logic analyzer of oscilloscop to PA8 and observe signal with
   * frequency of 500 kHz (8MHz / 16) */
}