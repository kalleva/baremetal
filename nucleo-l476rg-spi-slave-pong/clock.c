#include "clock.h"

void clock_config(void) {
  /* Turn on HSE clock source in bypass mode from STLink 8MHz crystal Nucleo */
  RCC->CR |= (RCC_CR_HSEBYP | RCC_CR_HSEON);

  /* Wait until HSE is ready */
  while (!(RCC->CR & RCC_CR_HSERDY))
    __NOP();

  /* Configure CPU speed for 64MHz */

  /* To run at max speed select PWR dynamic voltage range 1 by writing 0b01 */
  RCC->APB1ENR1 |= (RCC_APB1ENR1_PWREN);
  // PWR->CR1 &= ~(PWR_CR1_VOS);
  PWR->CR1 |= PWR_CR1_VOS;

  /* Configure flash latency. PWR voltage range 1 and 64MHz -> 3 WS (4 CPU
   * cycles) on reset there is zero wait state so no need to clear it up before
   * setting */
  FLASH->ACR |= FLASH_ACR_LATENCY_3WS;

  /* Configure MCU to run at 80MHz from PLL */

  /* Disable PLL */
  RCC->CR &= ~(RCC_CR_PLLON);
  while ((RCC->CR & RCC_CR_PLLRDY))
    __NOP();

  /* Configure HSE as PLL source */
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

  /* Configure PLL parameters / (M = 2, VCO = 4MHz) * (N = 32, VCO = 128MHz) /
  (R = 2, VCO = 64MHz) */

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
  /* 0b001 - 2 */
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_0;

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
  /* 32 = 0b00100000 */
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_5;

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLR);
  /* 2 = 0b00 so no need to set after clearing */

  /* APB1 can run at 64MHz as well so no need to divide for it */

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