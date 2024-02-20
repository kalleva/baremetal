Enables SysTick:

- Overwrites weak ```SysTick_Handler```
- Sets ```SysTick``` reload counter 
- Calls cmsis ```SysTick_Config``` with reload value based on clock frequncy 80MHz and 
desired systick firing frequency of 1kHz
- Enables global interrupts with ```__enable_irq()```
- Uses ```delay_ms``` which monitors global ```ticks``` value, instead of loop