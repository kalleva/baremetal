#include "stm32l4xx.h"

#define LED_Pin 5

volatile int counter = 0;

int main(void) {

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE5);
  GPIOA->MODER |= GPIO_MODER_MODE5_0;

  while (1) {
    GPIOA->ODR ^= (1 << LED_Pin);
    for (int i = 0; i < 100000; i++)
      __NOP();
  }
}