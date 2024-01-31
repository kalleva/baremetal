#include <stdint.h>

#define SRAM1_START 0x20000000U
#define SRAM1_SIZE (96 * 1024U) /* 96*/
#define SRAM1_END ((SRAM1_START) + (SRAM1_SIZE))

#define STACK_START SRAM1_END

extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _la_data;

extern uint32_t _sbss;
extern uint32_t _ebss;

/* Prototype of main */
int main(void);

/* Prototypes of STM32L476xx system exceptions and IRQ handlers */
void Reset_Handler(void);
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void SVCall_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DebugMonitor_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void WWDG_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void PVD_PVM_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void RCC_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void ADC1_2_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_TX_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_RX0_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_RX1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_SCE_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM15_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM16_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM17_IRQn(void)
    __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void USART1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void USART2_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void USART3_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DFSDM1_FLT3_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_BRK_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_UP_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_TRG_COM_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_CC_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void ADC3_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void FMC_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void SDMMC1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM5_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void UART4_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void UART5_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM6_DAC_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TIM7_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel2_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel3_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel4_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel5_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DFSDM1_FLT0_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DFSDM1_FLT1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DFSDM1_FLT2_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void COMP_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void LPTIM1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void LPTIM2_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void OTG_FS_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel6_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel7_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void LPUART1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void QUADSPI_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void SAI1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void SAI2_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void SWPMI1_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void TSC_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void LCD_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void RNG_IRQn(void) __attribute__((weak, alias("Default_Handler")));
void FPU_IRQn(void) __attribute__((weak, alias("Default_Handler")));

uint32_t vectors[] __attribute__((section(".isr_vector"))) = {
    STACK_START,
    (uint32_t)Reset_Handler,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
    (uint32_t)MemManage_Handler,
    (uint32_t)BusFault_Handler,
    (uint32_t)UsageFault_IRQn,
    0,
    0,
    0,
    0,
    (uint32_t)SVCall_IRQn,
    (uint32_t)DebugMonitor_IRQn,
    0,
    (uint32_t)PendSV_IRQn,
    (uint32_t)SysTick_IRQn,
    (uint32_t)WWDG_IRQn,
    (uint32_t)PVD_PVM_IRQn,
    (uint32_t)TAMP_STAMP_IRQn,
    (uint32_t)RTC_WKUP_IRQn,
    (uint32_t)FLASH_IRQn,
    (uint32_t)RCC_IRQn,
    (uint32_t)EXTI0_IRQn,
    (uint32_t)EXTI1_IRQn,
    (uint32_t)EXTI2_IRQn,
    (uint32_t)EXTI3_IRQn,
    (uint32_t)EXTI4_IRQn,
    (uint32_t)DMA1_Channel1_IRQn,
    (uint32_t)DMA1_Channel2_IRQn,
    (uint32_t)DMA1_Channel3_IRQn,
    (uint32_t)DMA1_Channel4_IRQn,
    (uint32_t)DMA1_Channel5_IRQn,
    (uint32_t)DMA1_Channel6_IRQn,
    (uint32_t)DMA1_Channel7_IRQn,
    (uint32_t)ADC1_2_IRQn,
    (uint32_t)CAN1_TX_IRQn,
    (uint32_t)CAN1_RX0_IRQn,
    (uint32_t)CAN1_RX1_IRQn,
    (uint32_t)CAN1_SCE_IRQn,
    (uint32_t)EXTI9_5_IRQn,
    (uint32_t)TIM1_BRK_TIM15_IRQn,
    (uint32_t)TIM1_UP_TIM16_IRQn,
    (uint32_t)TIM1_TRG_COM_TIM17_IRQn,
    (uint32_t)TIM1_CC_IRQn,
    (uint32_t)TIM2_IRQn,
    (uint32_t)TIM3_IRQn,
    (uint32_t)TIM4_IRQn,
    (uint32_t)I2C1_EV_IRQn,
    (uint32_t)I2C1_ER_IRQn,
    (uint32_t)I2C2_EV_IRQn,
    (uint32_t)I2C2_ER_IRQn,
    (uint32_t)SPI1_IRQn,
    (uint32_t)SPI2_IRQn,
    (uint32_t)USART1_IRQn,
    (uint32_t)USART2_IRQn,
    (uint32_t)USART3_IRQn,
    (uint32_t)EXTI15_10_IRQn,
    (uint32_t)RTC_Alarm_IRQn,
    (uint32_t)DFSDM1_FLT3_IRQn,
    (uint32_t)TIM8_BRK_IRQn,
    (uint32_t)TIM8_UP_IRQn,
    (uint32_t)TIM8_TRG_COM_IRQn,
    (uint32_t)TIM8_CC_IRQn,
    (uint32_t)ADC3_IRQn,
    (uint32_t)FMC_IRQn,
    (uint32_t)SDMMC1_IRQn,
    (uint32_t)TIM5_IRQn,
    (uint32_t)SPI3_IRQn,
    (uint32_t)UART4_IRQn,
    (uint32_t)UART5_IRQn,
    (uint32_t)TIM6_DAC_IRQn,
    (uint32_t)TIM7_IRQn,
    (uint32_t)DMA2_Channel1_IRQn,
    (uint32_t)DMA2_Channel2_IRQn,
    (uint32_t)DMA2_Channel3_IRQn,
    (uint32_t)DMA2_Channel4_IRQn,
    (uint32_t)DMA2_Channel5_IRQn,
    (uint32_t)DFSDM1_FLT0_IRQn,
    (uint32_t)DFSDM1_FLT1_IRQn,
    (uint32_t)DFSDM1_FLT2_IRQn,
    (uint32_t)COMP_IRQn,
    (uint32_t)LPTIM1_IRQn,
    (uint32_t)LPTIM2_IRQn,
    (uint32_t)OTG_FS_IRQn,
    (uint32_t)DMA2_Channel6_IRQn,
    (uint32_t)DMA2_Channel7_IRQn,
    (uint32_t)LPUART1_IRQn,
    (uint32_t)QUADSPI_IRQn,
    (uint32_t)I2C3_EV_IRQn,
    (uint32_t)I2C3_ER_IRQn,
    (uint32_t)SAI1_IRQn,
    (uint32_t)SAI2_IRQn,
    (uint32_t)SWPMI1_IRQn,
    (uint32_t)TSC_IRQn,
    (uint32_t)LCD_IRQn,
    (uint32_t)RNG_IRQn,
    (uint32_t)FPU_IRQn,
};

void Reset_Handler(void) {
  uint32_t size = (uint32_t)&_edata - (uint32_t)&_sdata;

  /* SRAM */
  uint8_t *p_dst = (uint8_t *)&_sdata;
  /* FLASH */
  uint8_t *p_src = (uint8_t *)&_la_data;

  /* Copy data from FLASH to RAM */
  for (uint32_t i = 0; i < size; i++)
    *p_dst++ = *p_src++;

  /* Init .bss with 0 */
  size = (uint32_t)&_ebss - (uint32_t)&_sbss;
  p_dst = (uint8_t *)&_sbss;
  for (uint32_t i = 0; i < size; i++)
    *p_dst++ = 0;

  main();
}

void Default_Handler(void) {
  while (1)
    ;
}