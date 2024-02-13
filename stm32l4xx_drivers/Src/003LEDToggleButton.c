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
                               .GPIO_PinMode = GPIO_MODE_IN,
                               .GPIO_PinSpeed = GPIO_SPEED_FAST,
                               .GPIO_PinPuPdControl = GPIO_NO_PUPD}};

  GPIO_PeriClockControl(GPIOA, ENABLE);
  GPIO_PeriClockControl(GPIOC, ENABLE);

  GPIO_Init(&GpioLed);
  GPIO_Init(&GpioButton);

  while (1) {
    if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BUTTON_PRESSED) {
      /* Debounce */
      for (int i = 0; i < 1000000; i++)
        ;

      GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
    }
  }
}
