#include "stm32l4xx.h"
#include <stdint.h>

#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS 0
#define NANOPRINTF_VISIBILITY_STATIC 0
#define NANOPRINTF_IMPLEMENTATION

#include "nanoprintf.h"

#define LED_Pin 5
#define led_toggle() (GPIOA->ODR ^= (1 << LED_Pin))
#define SYSTEM_CLOCK 80000000

static void clock_config(void);
static void led_config(void);
static void mco_config(void);
static void uart_config(void);
static int uart_transmit(char *data, int size);
static void delay_ms(uint32_t ms);

volatile int counter = 0;
volatile uint32_t ticks = 0;

int main(void) {
  clock_config();
  led_config();
  mco_config();
  uart_config();

  /* 80MHz 1KHz firing requency (each ms) */
  SysTick_Config(80000);
  __enable_irq();

  char buf[32];
  while (1) {
    led_toggle();
    int len = npf_snprintf(buf, sizeof(buf), "[%lu]%s, %s!\r\n", ticks, "Hello",
                           "World");
    uart_transmit(buf, len);
    delay_ms(1000);
  }
}

static void clock_config(void) {
  /* Turn on HSE clock source in bypass mode from STLink 8MHz crystal Nucleo */
  RCC->CR |= (RCC_CR_HSEBYP | RCC_CR_HSEON);

  /* Wait until HSE is ready */
  while (!(RCC->CR & RCC_CR_HSERDY))
    __NOP();

  /* Max MCU speed is 80MHz */

  /* To run at max speed select PWR dynamic voltage range 1 by writing 0b01 */
  RCC->APB1ENR1 |= (RCC_APB1ENR1_PWREN);
  // PWR->CR1 &= ~(PWR_CR1_VOS);
  PWR->CR1 |= PWR_CR1_VOS_0;

  /* Configure flash latency. PWR voltage range 1 and 80MHz -> 4 WS (5 CPU
   * cycles) on reset there is zero wait state so no need to clear it up before
   * setting */
  FLASH->ACR |= FLASH_ACR_LATENCY_4WS;

  /* Configure MCU to run at 80MHz from PLL */

  /* Disable PLL */
  RCC->CR &= ~(RCC_CR_PLLON);
  while ((RCC->CR & RCC_CR_PLLRDY))
    __NOP();

  /* Configure HSE as PLL source */
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

  /* Configure PLL parameters / (M = 2, VCO = 4MHz) * (N = 40, VCO = 160Mhz) /
  (R = 2, VCO = 80MHz) */

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
  /* 0b001 - 2 */
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_0;

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
  /* 40 = 0b0101000 */
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_3;

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLR);
  /* 2 = 0b00 so no need to set after clearing */

  /* APB1 can run at 80MHz as well so no need to divide for it */

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

static void led_config(void) {
  /* Configure LED PA5 PIN */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  GPIOA->MODER &= ~(GPIO_MODER_MODE5);
  GPIOA->MODER |= GPIO_MODER_MODE5_0;
}

static void mco_config(void) {
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

static void uart_config(void) {
  /* Enable UART clock */
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

  /* Enable GPIOA Clock */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  /* Configure PA2 as AF for UART */
  GPIOA->MODER &= ~(GPIO_MODER_MODE2);
  GPIOA->MODER |= GPIO_MODER_MODE2_1;
  GPIOA->OTYPER &= GPIO_OTYPER_IDR_2;

  /* Configure AF7 (USART2) for PA2 */
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2);
  GPIOA->AFR[0] |=
      (GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL2_2);

  /* Default UART settings after reset:
  - 16X Oversampling
  - 1 Start bit
  - 8 data bits
  - no stop bit
  - no parity bit
  */

  /* Configuration of baudrate 9600
  For oversampling 16x:
  USARTDIV = 80MHz / 9600 baud */
  uint16_t usart_div = SYSTEM_CLOCK / 9600;
  USART2->BRR = usart_div;
  /* Enable UART and TX over UART */
  USART2->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

static int uart_transmit(char *data, int size) {
  int count = size;
  while (count--) {
    while (!(USART2->ISR & USART_ISR_TXE))
      __NOP();
    USART2->TDR = *data++;
  }
  /* This line is inserted so UART will complete transmitting character before
   * MCU will go to sleep */
  while ((USART2->ISR & USART_ISR_TC) == 0)
    __NOP();
  return size;
}

void SysTick_Handler(void) { ticks++; }

static void delay_ms(uint32_t ms) {
  /* Busy loop. ticks variable is updated in SysTick_Handler*/
  uint32_t start = ticks;
  uint32_t end = start + ms;

  /* if ticks will wrap */
  if (end < start) {
    /* wait until ticks wraps and goes to 0 */
    while (ticks > start)
      __NOP();
  }

  while (ticks < end) {
    __NOP();
  }
}