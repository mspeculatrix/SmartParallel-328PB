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

extern SMD_AVR_Serial SerialPort;
extern const char * prt_state_comm_msg[];
extern const char * prt_state_disp_msg[]; // This shouldn't have to be here - see below
extern printerState printer;

void sendStateMsg()
{
	SerialPort.writeln(prt_state_comm_msg[printer.state]);
	displayMsg(prt_state_disp_msg[printer.state], PRINTER);	// this is wrong place to be doing this
}

void setCTS(uint8_t hilo)
{
	setPin(&SERIAL_REG, CTS_PIN, hilo);
}


#endif /* SP328PB_SERIAL_FUNCTIONS_H_ */