/*
 * SP328PB_display_functions.h
 *
 * Created: 6/10/2019 11:44:00 AM
 *  Author: Steve
 */ 


#ifndef SP328PB_DISPLAY_FUNCTIONS_H_
#define SP328PB_DISPLAY_FUNCTIONS_H_

#include <string.h>
#include <smd_avr_i2clib.h>

extern SMD_I2C_Device lcd;
extern char printBuf[];
extern bool lcd_present;
extern const char * msg_prefix[];

// Forward declarations
void displayBuffer();
void displayInit();
void displayMsg(const char msg[], lcd_msg_type msg_type);
void _writeCharToLCD(uint8_t chr);


void displayBuffer()
{
	if(lcd_present) {
		lcd.start(I2C_WRITE_MODE);
		lcd.sendByte(0);
		lcd.sendByte(3);
		lcd.sendByte(2);	// y pos
		lcd.sendByte(1);	// x pos : first col
		lcd.sendByte(0);
		bool done = false;
		uint8_t index = 0;
		while (!done) {
			switch(printBuf[index]) {
				case 0:		// null terminator
				case 10:	// LF
				case 13:	// CR
					done = true;
					break;
				default:
					_writeCharToLCD(printBuf[index]);
					//lcd.sendByte(char(printBuf[index]));
			}
			index++;
			if(index == 16) done = true;
		}
		if(index < 15) {
			for(uint8_t i = index; i <= 16; i++) {
				lcd.sendByte(32);
			}
		}
		lcd.stop();
		_delay_ms(1500);
	}
}

void displayInit()
{
	lcd.start(I2C_WRITE_MODE);
	lcd.sendByte(0);
	lcd.sendByte(12);	// clear screen
	lcd.sendByte(19);	// backlight on
	lcd.sendByte(4);	// cursor off
	lcd.stop();
}

void displayMsg(const char msg[], lcd_msg_type msg_type)
{
	if(lcd_present) {
		lcd.start(I2C_WRITE_MODE);
		lcd.sendByte(0);
		lcd.sendByte(3);
		lcd.sendByte(msg_type + 1);	// y pos
		lcd.sendByte(1);	// x pos : first col
		lcd.sendByte(0);
		for(uint8_t i = 0; i < 4; i++) {
			_writeCharToLCD(msg_prefix[msg_type][i]);
			//lcd.sendByte(char(msg_prefix[msg_type][i]));
		}
		for(uint8_t i = 0; i < strlen(msg); i++) {
			_writeCharToLCD(msg[i]);
			//lcd.sendByte(char(msg[i]));
		}
		// clear rest of line with spaces
		for(uint8_t i = strlen(msg); i < 12; i++) {
			_writeCharToLCD(0x20);
			//lcd.sendByte(0x20);	// space
		}
		lcd.stop();
	}
}

void _writeCharToLCD(uint8_t chr)
{
	// need to limit the range of characters that are sent to LCD
	if( (31 < chr && chr < 127) || (159 < chr && chr < 255) ) {
		lcd.sendByte(chr);
	}
}


#endif /* SP328PB_DISPLAY_FUNCTIONS_H_ */