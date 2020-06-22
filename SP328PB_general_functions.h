/*
 * SP328PB_general_functions.h
 *
 * Created: 6/10/2019 12:34:37 PM
 *  Author: Steve
 */ 


#ifndef SP328PB_GENERAL_FUNCTIONS_H_
#define SP328PB_GENERAL_FUNCTIONS_H_

#include <smd_std_macros.h>
#include "SP328PB_defines.h"

uint8_t readPin(volatile uint8_t * portreg, uint8_t pin)
{
	// The input register - eg, PINB - is passed by reference
	if((*portreg & (1 << pin)) == (1 << pin)) { // pin is high
		return HIGH;
	} else {
		return LOW;
	}
}

void setPin(volatile uint8_t * portreg, uint8_t pin, uint8_t hilo)
{
	// example call: writeBit(&PORTB, PB1, HIGH);
	if(hilo == HIGH) {
		*portreg |= (1 << pin);
	} else {
		*portreg &= ~(1 << pin);
	}
}

void setLED(uint8_t pin, uint8_t onOff)
{
	setPin(&LED_REG, pin, onOff);
}

#endif /* SP328PB_GENERAL_FUNCTIONS_H_ */