/**
 * @file LCD_pins.h
 * @brief Піни для підключення LCD 16x2 до STM32
 * @author Вадзік
 */

#ifndef LCD_PINS_H_
#define LCD_PINS_H_

#include "stm32f4xx_hal.h"  // або ваш HAL

// RS - Register Select
#define RS_PORT    GPIOE
#define RS_PIN     GPIO_PIN_7

// E - Enable
#define E_PORT     GPIOE
#define E_PIN      GPIO_PIN_10

// Data pins D4..D7
#define D4_PORT    GPIOE
#define D4_PIN     GPIO_PIN_11

#define D5_PORT    GPIOE
#define D5_PIN     GPIO_PIN_12

#define D6_PORT    GPIOE
#define D6_PIN     GPIO_PIN_13

#define D7_PORT    GPIOE
#define D7_PIN     GPIO_PIN_14

#endif /* LCD_PINS_H_ */
