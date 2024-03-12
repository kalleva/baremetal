#include "led.h"

void led_config(void) {
  /* Configure LED PC13 PIN */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  GPIOC->MODER &= ~(GPIO_MODER_MODE13);
  GPIOC->MODER |= GPIO_MODER_MODE13_0;
}