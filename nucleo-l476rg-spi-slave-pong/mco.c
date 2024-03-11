#include "mco.h"

void mco_config(void) {
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