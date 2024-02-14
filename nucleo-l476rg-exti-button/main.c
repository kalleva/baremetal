#include "stm32l476xx.h"
#include "stm32l4xx.h"
#include <stdint.h>

#define LED_Pin 5
#define BTN_PIN 13

volatile int counter = 0;

int main(void) {

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE5);
  GPIOA->MODER |= GPIO_MODER_MODE5_0;

  /* Enable clock for GPIOC */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

  /* Configure pin as Input mode 0b00 */
  GPIOC->MODER &= ~(GPIO_MODER_MODE13);

  /* Enable pull-up resistor 0b01 */
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);
  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD13_0);

  /* Configure Trigger for Falling Edge */
  EXTI->FTSR1 |= (1 << BTN_PIN);

  /* Clear Trigger on Rising Edge */
  EXTI->RTSR1 &= ~(1 << BTN_PIN);

  /* Enable clock for SYSCFG */
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  /* Configure from PC13 pin and port wait for the interrupt */
  uint8_t exti_idx = BTN_PIN / 4;
  uint8_t exti_reg_offset = BTN_PIN % 4;
  SYSCFG->EXTICR[exti_idx] &= 0xF << (exti_reg_offset * 4);
  /* 13 pin PC */
  SYSCFG->EXTICR[exti_idx] |= 0x2 << (exti_reg_offset * 4);

  /* Unmask EXTI line */
  EXTI->IMR1 |= 1 << BTN_PIN;

  /* Configure Priority for the interrupt in NVIC */
  NVIC_SetPriority(EXTI15_10_IRQn, 15);

  /* Enable IRQ line in NVIC */
  __NVIC_EnableIRQ(EXTI15_10_IRQn);

  while (1)
    ;
}

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR1 & (1 << BTN_PIN)) {
    EXTI->PR1 |= (1 << BTN_PIN);

    for (int i = 0; i < 100000; i++)
      ;
    GPIOA->ODR ^= (1 << LED_Pin);
  }
}