/*
 *	LiquidCrystal_I2C.h
 *
 *      Author: Ngoc Tuan
 */
#ifndef LIQUIDCRYSTAL_I2C_H
#define LIQUIDCRYSTAL_I2C_H

#include "stm32f1xx_hal.h"
// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define LCD_RS_SET	0x01
#define LCD_RS_CLEAR	0x00

#define LCD_EN_SET	0x04
#define LCD_EN_CLEAR	0x00

#define LCD_ADDR_DEFAULT 0x27
typedef struct
{
	uint8_t Addr;//dia chi cua LCD mac dinh 0x27
	I2C_HandleTypeDef *pI2c;
	uint8_t DisplayControl;
	uint8_t Backlightval;
}LiquidCrystal_I2C;

void lcd_init (LiquidCrystal_I2C *lcd,I2C_HandleTypeDef *_hi2c,uint8_t addr);   // initialize lcd
void lcd_clear_display (LiquidCrystal_I2C *lcd);	//clear display lcd
void lcd_set_cursor (LiquidCrystal_I2C *lcd,uint8_t row, uint8_t col); //set proper location on screen
void lcd_display_off(LiquidCrystal_I2C *lcd);
void lcd_display_on(LiquidCrystal_I2C *lcd);
void lcd_backlight_on(LiquidCrystal_I2C *lcd) ;
void lcd_backlight_off(LiquidCrystal_I2C *lcd) ;
// Turns the underline cursor on/off
void lcd_set_cursor_on(LiquidCrystal_I2C *lcd);
void lcd_set_cursor_off(LiquidCrystal_I2C *lcd);
// Turn on and off the blinking cursor
void lcd_set_cursor_blink_on(LiquidCrystal_I2C *lcd);
void lcd_set_cursor_blink_off(LiquidCrystal_I2C *lcd) ;
// Custom char
void lcd_create_char(LiquidCrystal_I2C *lcd,uint8_t location, uint8_t charmap[]);
void lcd_write(LiquidCrystal_I2C *lcd,uint8_t location);
//
void lcd_putchar(LiquidCrystal_I2C *lcd,uint8_t data);
// Print string
void lcd_printf(LiquidCrystal_I2C *lcd,const char* str, ...);
#endif
