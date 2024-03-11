#ifndef LED_H__
#define LED_H__

#include "main.h"

#define LED_Pin 5
#define led_toggle() (GPIOA->ODR ^= (1 << LED_Pin))
void led_config(void);

#endif /* LED_H__ */