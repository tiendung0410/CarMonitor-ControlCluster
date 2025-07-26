/*
 *	LiquidCrystal_I2C.h
 *
 *      Author: Ngoc Tuan
*/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "LiquidCrystal_I2C.h"
static void lcd_send_cmd (LiquidCrystal_I2C *lcd, uint8_t cmd)
{
  uint8_t data_h, data_l;
	uint8_t data_t[4];
	data_h = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_h|(lcd->Backlightval|LCD_RS_CLEAR|LCD_EN_SET); //0xXXXX.1100  en=1, rs=0
	data_t[1] = data_h|(lcd->Backlightval|LCD_RS_CLEAR|LCD_EN_CLEAR); //0xXXXX.1000  en=0, rs=0
	data_t[2] = data_l|(lcd->Backlightval|LCD_RS_CLEAR|LCD_EN_SET); //0xXXXX.1100  en=1, rs=0
	data_t[3] = data_l|(lcd->Backlightval|LCD_RS_CLEAR|LCD_EN_CLEAR); //0xXXXX.1000  en=0, rs=0
	HAL_I2C_Master_Transmit(lcd->pI2c, lcd->Addr,(uint8_t *) data_t, 4, 100);
}

static void lcd_send_data(LiquidCrystal_I2C *lcd,uint8_t data)
{
	uint8_t data_h, data_l;
	uint8_t data_t[4];
	data_h = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_h|(lcd->Backlightval|LCD_RS_SET|LCD_EN_SET); //0xXXXX.1101  en=1, rs=1
	data_t[1] = data_h|(lcd->Backlightval|LCD_RS_SET|LCD_EN_CLEAR); //0xXXXX.1001  en=0, rs=1
	data_t[2] = data_l|(lcd->Backlightval|LCD_RS_SET|LCD_EN_SET); //0xXXXX.1101  en=1, rs=1
	data_t[3] = data_l|(lcd->Backlightval|LCD_RS_SET|LCD_EN_CLEAR); //0xXXXX.1001  en=0, rs=1
	HAL_I2C_Master_Transmit (lcd->pI2c, lcd->Addr,(uint8_t *) data_t, 4, 100);
}
void lcd_putchar(LiquidCrystal_I2C *lcd,uint8_t data)
{
	lcd_send_data(lcd,data);
}

void lcd_clear_display (LiquidCrystal_I2C *lcd)
{
	lcd_send_cmd (lcd,LCD_CLEARDISPLAY);
	HAL_Delay(1);
}
void lcd_display_off(LiquidCrystal_I2C *lcd) 
{
	lcd->DisplayControl &= ~LCD_DISPLAYON;
	lcd_send_cmd(lcd,LCD_DISPLAYCONTROL | lcd->DisplayControl);
}
void lcd_display_on(LiquidCrystal_I2C *lcd) 
{
	lcd->DisplayControl |= LCD_DISPLAYON;
	lcd_send_cmd(lcd,LCD_DISPLAYCONTROL | lcd->DisplayControl);
}

void lcd_backlight_on(LiquidCrystal_I2C *lcd)
{
	lcd->Backlightval = LCD_BACKLIGHT;
	HAL_I2C_Master_Transmit (lcd->pI2c, lcd->Addr,&lcd->Backlightval, 1, 100);
}

void lcd_backlight_off(LiquidCrystal_I2C *lcd)
{
	lcd->Backlightval = LCD_NOBACKLIGHT;
	HAL_I2C_Master_Transmit (lcd->pI2c, lcd->Addr,&lcd->Backlightval, 1, 100);
}

void lcd_set_cursor(LiquidCrystal_I2C *lcd,uint8_t row, uint8_t col)
{
	uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	lcd_send_cmd(lcd,LCD_SETDDRAMADDR | (col + row_offsets[row]));
}
// Turns the underline cursor on/off
void lcd_set_cursor_on(LiquidCrystal_I2C *lcd) 
{
	lcd->DisplayControl |= LCD_CURSORON;
	lcd_send_cmd(lcd,LCD_DISPLAYCONTROL | lcd->DisplayControl);
}
void lcd_set_cursor_off(LiquidCrystal_I2C *lcd)
{
	lcd->DisplayControl &= ~LCD_CURSORON;
	lcd_send_cmd(lcd,LCD_DISPLAYCONTROL | lcd->DisplayControl);
}

// Turn on and off the blinking cursor
void lcd_set_cursor_blink_on(LiquidCrystal_I2C *lcd) 
{
	
	lcd->DisplayControl |= LCD_BLINKON;
	lcd_send_cmd(lcd,LCD_DISPLAYCONTROL | lcd->DisplayControl);
}
void lcd_set_cursor_blink_off(LiquidCrystal_I2C *lcd) 
{
	lcd->DisplayControl &= ~LCD_BLINKON;
	lcd_send_cmd(lcd,LCD_DISPLAYCONTROL | lcd->DisplayControl);
}
void lcd_create_char(LiquidCrystal_I2C *lcd,uint8_t location, uint8_t charmap[]) 
{
	location &= 0x7; 
	lcd_send_cmd(lcd,LCD_SETCGRAMADDR | (location << 3));
	for (int i=0; i<8; i++) {
		lcd_send_data(lcd,charmap[i]);
	}
}
void lcd_write(LiquidCrystal_I2C *lcd,uint8_t location)
{
	lcd_send_data(lcd,location);
}
void lcd_printf(LiquidCrystal_I2C *lcd,const char* str, ...)
{
  char stringArray[20];
	
  va_list args;
  va_start(args, str);
  vsprintf(stringArray, str, args);
  va_end(args);
	
  for(uint8_t i=0;  i<strlen(stringArray) && i<16; i++)
  {
    lcd_send_data(lcd,(uint8_t)stringArray[i]);
  }
}
void lcd_init (LiquidCrystal_I2C *lcd,I2C_HandleTypeDef *_hi2c,uint8_t addr)
{
	lcd->pI2c = _hi2c;
	lcd->Addr = addr<<1;
	lcd->DisplayControl |= LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	lcd_send_cmd (lcd,0x33); 
	lcd_send_cmd (lcd,0x32);
	HAL_Delay(5);
	lcd_send_cmd (lcd,LCD_CLEARDISPLAY); 
	HAL_Delay(5);
	lcd_send_cmd (lcd,0x0c); 	
	HAL_Delay(5);
	lcd_send_cmd (lcd,LCD_RETURNHOME); 
	HAL_Delay(5);
	lcd_send_cmd (lcd,LCD_SETDDRAMADDR);
	lcd_set_cursor_blink_off(lcd);
	lcd->Backlightval = LCD_BACKLIGHT;
}

