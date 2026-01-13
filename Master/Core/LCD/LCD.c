#include "LCD.h"
#include "wh1602.h"

void LCD_Init(void)
{
    wh1602_Init();
}

void LCD_SendChar(char ch)
{
    wh1602_SendChar(ch);
}

void LCD_SendString(char *str, uint8_t size)
{
    for(uint8_t i = 0; i < size; i++)
    {
        wh1602_SendChar(str[i]);
    }
}

void LCD_Clear(void)
{
    wh1602_Clear();
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t address;
    // Адреса початку рядків: 0x80 - 1-й рядок, 0xC0 - 2-й рядок
    if (row == 0) {
        address = 0x80;
    } else {
        address = 0xC0;
    }

    address += col; // Додаємо зміщення по стовпцю
    wh1602_Command(address); // Відправляємо як команду
}
