/*
 * SP328PB_serial_functions.h
 *
 * Created: 6/10/2019 12:30:20 PM
 *  Author: Steve
 */ 


#ifndef SP328PB_SERIAL_FUNCTIONS_H_
#define SP328PB_SERIAL_FUNCTIONS_H_

#include <smd_avr_seriallib.h>
#include "SP328PB_defines.h"
#include "SP328PB_general_functions.h"

extern SMD_AVR_Serial serialport;
extern const char * prt_state_comm_msg[];
extern const char * prt_state_disp_msg[]; // This shouldn't have to be here - see below
extern printer_state curr_state;

void sendStateMsg()
{
	serialport.writeln(prt_state_comm_msg[curr_state]);
	displayMsg(prt_state_disp_msg[curr_state], PRINTER);	// this is wrong place to be doing this
}

void setCTS(uint8_t hilo)
{
	setPin(&SERIAL_REG, CTS_PIN, hilo);
}


#endif /* SP328PB_SERIAL_FUNCTIONS_H_ */