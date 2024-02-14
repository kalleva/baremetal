#include "stm32l476xx.h"
#include "stm32l476xx_gpio_driver.h"

#define HIGH 1
#define LOW 0
#define BUTTON_PRESSED LOW

int main(void) {
  GPIO_Handle_t GpioLed = {GPIOA,
                           {GPIO_PIN_NO_5, GPIO_MODE_OUT, GPIO_SPEED_FAST,
                            GPIO_OP_TYPE_PP, GPIO_NO_PUPD}};

  GPIO_Handle_t GpioButton = {GPIOC,
                              {.GPIO_PinNumber = GPIO_PIN_NO_13,
                               .GPIO_PinMode = GPIO_MODE_IT_FT,
                               .GPIO_PinPuPdControl = GPIO_PIN_PU}};

  GPIO_PeriClockControl(GPIOA, ENABLE);
  GPIO_PeriClockControl(GPIOC, ENABLE);

  GPIO_Init(&GpioLed);
  GPIO_Init(&GpioButton);

  GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
  GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

  while (1) {
  }
}

void EXTI15_10_IRQHandler(void) {
  for (int i = 0; i < 100000; i++)
    ;
  GPIO_IRQHandling(GPIO_PIN_NO_13);
  GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}