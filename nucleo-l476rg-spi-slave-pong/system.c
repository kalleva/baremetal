#include "system.h"

static volatile uint32_t ticks = 0;

void SysTick_Handler(void) { ticks++; }

void delay_ms(uint32_t ms) {
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