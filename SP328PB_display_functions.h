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

enum lcd_msg_type {PRINTER, SERIAL};

// DISPLAY FUNCTIONS
void displayBuffer();
void displayInit();
void displayMsg(const char msg[], lcd_msg_type msg_type);
void writeCharToLCD(uint8_t chr);

#endif /* SP328PB_DISPLAY_FUNCTIONS_H_ */