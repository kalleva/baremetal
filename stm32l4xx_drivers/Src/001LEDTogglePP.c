#include "stm32l476xx.h"
#include "stm32l476xx_gpio_driver.h"

int main(void) {
  GPIO_Handle_t GpioLed = {GPIOA,
                           {GPIO_PIN_NO_5, GPIO_MODE_OUT, GPIO_SPEED_FAST,
                            GPIO_OP_TYPE_PP, GPIO_NO_PUPD}};

  GPIO_PeriClockControl(GPIOA, ENABLE);

  GPIO_Init(&GpioLed);

  while (1) {
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
    for (int i = 0; i < 1000000; i++)
      ;
  }
}
