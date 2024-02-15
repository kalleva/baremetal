#include "stm32l476xx.h"
#include "stm32l476xx_spi_driver.h"
#include <stdbool.h>

/* AF5 */
/* SPI1_NSS PA4 */
/* SPI1_SCK PA5 */
/* SPI1_MISO PA6 */
/* SPI1_MOSI PA7 */

#define HIGH 1
#define LOW 0
#define BUTTON_PRESSED LOW

volatile bool btn_pressed = false;

void SPI1_GPIOInits(void) {
  /* SCK */
  GPIO_Handle_t SPIPins = {GPIOA,
                           {.GPIO_PinNumber = GPIO_PIN_NO_5,
                            .GPIO_PinAltFunMode = 5,
                            .GPIO_PinMode = GPIO_MODE_ALTFN,
                            .GPIO_PinSpeed = GPIO_SPEED_FAST,
                            .GPIO_PinOPType = GPIO_OP_TYPE_PP,
                            .GPIO_PinPuPdControl = GPIO_NO_PUPD}};
  GPIO_Init(&SPIPins);

  /* MOSI */
  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
  GPIO_Init(&SPIPins);

  /* MISO */
  // SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
  // GPIO_Init(&SPIPins);

  /* NSS */
  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
  GPIO_Init(&SPIPins);
}

void GPIO_Button_Init(void) {
  GPIO_Handle_t GpioButton = {GPIOC,
                              {.GPIO_PinNumber = GPIO_PIN_NO_13,
                               .GPIO_PinMode = GPIO_MODE_IT_FT,
                               .GPIO_PinPuPdControl = GPIO_PIN_PU}};

  GPIO_Init(&GpioButton);

  GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
  GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
}

void SPI1_Inits(void) {
  SPI_Handle_t SPI1handle = {SPI1,
                             {.SPI_BusConfig = SPI_BUS_CONFIG_FD,
                              .SPI_DeviceMode = SPI_DEVICE_MODE_MASTER,
                              .SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2,
                              .SPI_DS = SPI_DS_8,
                              .SPI_CPOL = SPI_CPOL_LOW,
                              .SPI_CPHA = SPI_CPHA_LOW,
                              .SPI_SSM = SPI_SSM_DI}};
  SPI_Init(&SPI1handle);
}

int main(void) {
  GPIO_Button_Init();
  SPI1_GPIOInits();
  SPI1_Inits();
  SPI_SSOEConfig(SPI1, ENABLE);

  while (1) {
    if (btn_pressed) {

      SPI_PerpheralControl(SPI1, ENABLE);

      char data[] = "HELLO, WORLD!\r\n";
      SPI_SendData(SPI1, (uint8_t *)data, 15);
      while (SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG))
        ;

      SPI_PerpheralControl(SPI1, DISABLE);
      btn_pressed = false;
    }
  }
}

void EXTI15_10_IRQHandler(void) {
  for (int i = 0; i < 100000; i++)
    ;

  if (!btn_pressed)
    btn_pressed = true;

  GPIO_IRQHandling(GPIO_PIN_NO_13);
}
