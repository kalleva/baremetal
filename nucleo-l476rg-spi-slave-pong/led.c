#include "led.h"

void led_config(void) {
  // TODO: OpenDrain
  /* Configure LED PA5 PIN */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE5);
  GPIOA->MODER |= GPIO_MODER_MODE5_0;
}