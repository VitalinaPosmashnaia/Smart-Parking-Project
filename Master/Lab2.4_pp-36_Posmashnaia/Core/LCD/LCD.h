#ifndef LCD_H_
#define LCD_H_

#include "stdint.h"

void LCD_Init(void);
void LCD_SendChar(char ch);
void LCD_SendString(char *str, uint8_t size);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);

#endif /* LCD_H_ */
