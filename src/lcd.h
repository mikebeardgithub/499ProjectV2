/*
 * lcd.h
 *
 *  Created on: Jul 1, 2017
 *      Author: jjank
 */

#include "stm32f4xx.h"
#define LCD_ENTRY_INCR (1)
#define LCD_ENTRY_DECR (0)
#define LCD_ENTRY_SHIFTDISPLAY (1)
#define LCD_ENTRY_NOSHIFT (0)
#define LCD_ON (1)
#define LCD_OFF (0)
#define LCD_NOCURSOR (0)
#define LCD_CURSOR (1)
#define LCD_NOBLINK (0)
#define LCD_BLINK (1)
#define LCD_8BIT (1)
#define LCD_4BIT (0)
#define LCD_ENGLISHJAP (0b00)
#define LCD_EUROPE1 (0b01)
#define LCD_ENGLISHRUS (0b10)
#define LCD_EUROPE2 (0b11)
#define LCD_SINEWAVE (1)
#define LCD_TRIANGLE (2)
#define LCD_SQUARE   (3)



// alexes code
void SPI_TX(int tx);
void lcd_init(void);
int reverse_DC(double);
int reverse_Av(double);
void lcd_update(int,double, int, double, double,double);
void lcd_scaled_update(int, int, int, double, int, double);
void delay(int);
void LCD_CLEAR(void);
void LCD_WRITE(int data);
void lcd_writeline(char * str, int row);
void LCD_SETDDRAM(int DDADress);
