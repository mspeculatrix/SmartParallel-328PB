/*
  SmartParallel.cpp

  Code for Atmel ATMEGA328PB micronctroller on the SmartParallel serial to
  parallel interface.

  Intended for use with an Epson MX-80 F/T III dot matrix printer.

  Designed to work with the Devantech 40 char x 4-line I2C LDC display (optional). The code checks
  for the presence of the display during initialisation.

  Epson MX-80 F/T III parallel interface has a transfer rate of 1000 cps.

  Colours for LEDs:
	Status LEDs:
		Printing			GREEN
		Serial Recv			YELLOW
		Stat3 (poss. error)	RED
	Interface LEDS:
		Error				RED
		Offline				ORANGE
		Autofeed			BLUE
		Busy				GREEN

	
  Codes supported by Epson MX-80 & Devantech LCD include:
				EPSON						LCD
  7		0x07	BEL - Bell					-- does nothing
  8		0x08	BS - Backspace				Backspace
  9		0x09	HT - Horizontal Tab			Horizontal tab
  10	0x0A	LF - Linefeed				Linefeed
  11	0x0B	VT - Vertical tab			Vertical tab
  12	0x0C	FF - Form feed				Clear screen
  13	0x0D	CR - Carriage return		Carriage return
  27	0x1B	ESC - Escape				Custom char generator
  32-126		ASCII						ASCII
  127	0x7F	DEL - Delete				
  128-135		(135 is BEL)				Custom chars
  The extended (8-bit) character set is actually the same as the 7-bit, just with 128 added, so we
  can ignore characters in this range (or deduct 128 from them).
 */

#define TESTING true

#ifndef F_CPU			// if F_CPU was not defined in Project -> Properties
#define F_CPU 16000000UL // define it now as 8MHz unsigned long - prob change to 16MHz
#endif
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <smd_std_macros.h>
#include <uproc/smd_atmega_88_168_328.h>
#include <smd_avr_i2clib.h>
#include <smd_avr_seriallib.h>
#include <ShiftReg74hc595.h>
#include "SP328PB_defines.h"
#include "SP328PB_general_functions.h"
#include "SP328PB_display_functions.h"
#include "SP328PB_serial_functions.h"

/********************************************************************************
*****   GLOBALS                                                             *****
*********************************************************************************/
// printer messages for displaying on the LCD
const char *prt_state_disp_msg[] = {"READY", "DONE", "INITIALISING", "OFFLINE", "PRINTING", "BUSY", "ERROR", "PAPER_END", "ACK TIMEOUT", "BUSY TIMEOUT"};
// matching printer messages for sending via serial to the client
const char *prt_state_comm_msg[] = {"PRT_READY", "PRT_DONE", "PRT_INIT", "PRT_OFFLINE", "PRT_PRINTING", "PRT_BUSY", "PRT_ERROR", "PRT_PE", "PRT_ACK_TIMEOUT", "PRT_BUSY_TIMEOUT"};

// serial port messages for displaying on the LCD
const char *serial_disp_msg[] = {"OK", "READ TIMEOUT", "BUF CLEARED"};
// matching serial port messages for sending to the client via serial
const char *serial_comm_msg[] = {"SER_OK", "SER_READ_TIMEOUT", "SER_BUF_CLEARED"};
// prefixes to use when displaying messages on the LCD
const char *msg_prefix[] = {"Prt:", "Ser:"};

// LCD object - I'm using a Devantech 20x2 module which has its own I2C backpack
SMD_I2C_Device lcd = SMD_I2C_Device(LCD_ADDRESS, I2C_BUS_SPEED_STD);
// Serial port object
SMD_AVR_Serial SerialPort = SMD_AVR_Serial(SERIAL_BAUD_RATE); // default baudrate 19200
// Shift register object - the shift reg handles data output to the printer
ShiftReg74hc595 DataRegister = ShiftReg74hc595(SHCP_PIN, STCP_PIN, DATA_PIN, &SHIFTREG_REG, &SHIFTREG_DDR);

char printBuf[PRINT_BUF_LEN]; // line buffer of which last element will always be null terminator
uint8_t buf_index = 0;
bool buf_ready = false;
bool lcd_present = false;     // assume no display

/********************************************************************************
*****   FORWARD DECLARATIONS                                                *****
*********************************************************************************/
// PRINTER FUNCTIONS
void clearBuffer();
void initialisePrinter();
void printBuffer();
printer_state printChar(uint8_t chardata, ack_state waitForACK);
void resetAck();
printer_state updatePrinterState(bool setCts=true);


/********************************************************************************
*****   INTERRUPT HANDLERS                                                  *****
*********************************************************************************/

ISR(INT0_vect) // for /ACK line
{
	printer.ackstate = ACK;
}

ISR(INT1_vect) // for /ERROR line
{
	printer.state = ERROR;
	clearBuffer();
}

/********************************************************************************
*****   PRINTER FUNCTIONS                                                   *****
*********************************************************************************/

/**
Set all bytes in printer buffer to 0. 
**/
void clearBuffer()
{
	for (uint8_t i = 0; i < PRINT_BUF_LEN; i++) {
		printBuf[i] = 0;
	}
	buf_index = 0;
	buf_ready = false;
	//SerialPort.clearBuffer();
}

void initialisePrinter()
{
	setCTS(CTS_OFFLINE);		// deter serial input
	resetAck();					// probably not necessary, but what the hell
	printer.state = INIT;
	printer.prev_state = INIT;
	printer.state_changed = false;
	sendStateMsg();						  // show we're initialising
	clearBuffer();						  // clear the print data buffer
	SerialPort.clearBuffer();			  // clear serial port's input buffer
	DataRegister.shiftOut(0);			  // set data lines to all zeroes
	setPin(&OUTPUT_PORT, INIT_PIN, LOW);  // send /INIT pulse
	_delay_us(INIT_PULSE_LENGTH);		  // - keep line low
	setPin(&OUTPUT_PORT, INIT_PIN, HIGH); // - release line
	_delay_ms(POST_INIT_DELAY);			  // - let the printer settle down
	//updatePrinterState(true);				  // let's see how the printer's doing - sets CTS
	//return printer.state;
}

void printBuffer()
{
	setCTS(CTS_OFFLINE);						// deter further incoming data
	printer.state = PRINTING;
	sendStateMsg();								// update display & send serial message
	setLED(STAT_PRINTING, ON);
	uint8_t buf_index = 0;
	bool done = false;
	while (!done && !printer.error)	{
		if (printBuf[buf_index] != 0) {
			printer.state = printChar(printBuf[buf_index], printer.useAck);
			if (printer.state != DONE) {
				done = true; // because we've encountered an error
				SerialPort.writeln(prt_state_disp_msg[printer.state]);
			}
			buf_index++;
			if (buf_index == PRINT_BUF_LEN - 1)	{
				// we're at the penultimate byte of the buffer, so finished
				done = true; 
			}
		} else {
			done = true; // we've encountered a 0, hence end of data
		}
	}
	setLED(STAT_PRINTING, OFF);
	clearBuffer();
	updatePrinterState(true);	// resets CTS
}

printer_state printChar(uint8_t chardata, ack_state waitForACK)
{
	updatePrinterState(false);
	if (printer.state == ERROR || printer.state == OFFLINE || printer.state == PAPER_END || printer.error) {
		 if(TESTING) SerialPort.write("- printer is not ready - state: ");
		 if(TESTING) SerialPort.writeln(printer.state);
	 } else {
		bool done = false;
		uint8_t waitTimeoutCounter = 0;
		while (!done) {
			if (printer.state == READY) {
				resetAck();								   // an excess of caution
				DataRegister.shiftOut(chardata, MSBFIRST); // put data on shift reg
				_delay_us(PRE_STROBE_DELAY);			   // settle down for min 0.5us
				setPin(&OUTPUT_PORT, STROBE_PIN, LOW);	 // start strobe pulse
				_delay_us(STROBE_PULSE_LENGTH);			   // - pulse time
				setPin(&OUTPUT_PORT, STROBE_PIN, HIGH);	// - end pulse
				if (waitForACK == ACK) {				   // did we specify in param to wait for ACK?
					uint8_t timeout_count = 0; // NB: the printer will hold ACK low for around 5us
					while (timeout_count <= ACK_TIMEOUT_LOOP_COUNTER && printer.ackstate == NO_ACK) {
						// the ACK line is set up on interrupt INT0 so we need to wait
						// for the interrupt to trigger or our timeout to expire
						_delay_us(ACK_TIMEOUT_LOOP_DELAY);
						timeout_count++;
					}
					if (printer.ackstate == NO_ACK)	{
						// we exited the loop above but still no ACK, so it must have been a timeout
						printer.state = ACK_TIMEOUT;
					} else {
						// we exited the loop above because ACK got set
						printer.state = DONE;
						resetAck();
					}
				} else {
					printer.state = DONE; // we decided not to bother with ACK
				}
				done = true;
			} else { // curr_state wasn't READY - printer might be busy
				_delay_ms(BUSY_TIMEOUT_LOOP_DELAY);
				updatePrinterState(false);
				waitTimeoutCounter++;
				if (waitTimeoutCounter == BUSY_TIMEOUT_LOOP_COUNTER) {
					printer.state = BUSY_TIMEOUT;
					done = true;
				}
			}
		}
	}
	return printer.state;
}

void resetAck()
{
	printer.ackstate = NO_ACK;
	EIFR = (1 << INTF0); // clear the interrupt flag to be sure
}

/*
	The possible states this can return are:
		READY 		- only one where CTS should be CTS_ONLINE
		ERROR
		BUSY
		OFFLINE
		PAPER_END
*/
printer_state updatePrinterState(bool setCts)
{
	// read state of input pins
	printer.state = READY;							  // let's be optimistic
	printer.busy = readPin(&INPUT_REG, BUSY_PIN);	  // active high
	printer.error = 1 - readPin(&ERR_REG, ERR_PIN);   // active low
	printer.pe = readPin(&INPUT_REG, PE_PIN);		  // active high - /ERROR and BUSY will also be set
	printer.select = readPin(&INPUT_REG, SELECT_PIN); // active high - low indicates offline

	// set overall state
	if (printer.error) printer.state = ERROR;			// also true if BUSY
	if (printer.busy) printer.state = BUSY;				// ERROR will also be set
	if (!printer.select) printer.state = OFFLINE;
	if (printer.pe) printer.state = PAPER_END;			// ERROR and BUSY will also be set
	if (setCts) {
		if (printer.state == READY) {
			setCTS(CTS_ONLINE);
		} else {
			setCTS(CTS_OFFLINE);
		}
	}
	if (printer.state != printer.prev_state) {
		printer.state_changed = true;
		printer.prev_state = printer.state;
	}
	return printer.state;
}

void setSelectIn(uint8_t hilo)
{
	setPin(&OUTPUT_PORT, SELIN_PIN, hilo); // set pin
	printer.selectIn = hilo;			   // store state
}

void setAutofeed(autofeed_state af_state)
{
	// autofeed is active low, but we usually want it disabled
	setPin(&OUTPUT_PORT, AUTOFEED_PIN, af_state);
	printer.autofeed = af_state;
}

void disableAutofeed(void)
{
	setAutofeed(AF_DISABLED);
}

void enableAutofeed(void)
{
	setAutofeed(AF_ENABLED);
}

int main(void)
{
	/********************************************************************************
	*****   SETUP                                                               *****
	*********************************************************************************/

	// Set pin directions
	SHIFTREG_DDR |= (1 << SHCP_PIN | 1 << STCP_PIN | 1 << DATA_PIN);						// shift reg pins as outputs
	OUTPUT_DDR |= (1 << STROBE_PIN | 1 << INIT_PIN | 1 << AUTOFEED_PIN | 1 << SELIN_PIN);	// outputs
	INPUT_DDR &= ~(1 << BUSY_PIN | 1 << PE_PIN | 1 << SELECT_PIN);							// BUSY, PE & SEL as inputs
	INPUT_INT_DDR &= ~(1 << ACK_PIN | 1 << ERR_PIN);										// ACK and ERR as inputs
	LED_DDR |= (1 << STAT_PRINTING | 1 << STAT_SERIAL_RECV | 1 << STAT_CTS_OFFLINE);		// LEDS as outputs
	CTS_DDR |= (1 << CTS_PIN);																// CTS as output

	// Set outputs at initial values
	setCTS(CTS_OFFLINE);					// active low - refuse serial data while getting set up
	setPin(&OUTPUT_PORT, STROBE_PIN, HIGH); // active low
	setPin(&OUTPUT_PORT, INIT_PIN, HIGH);   // active low
	setAutofeed(AF_DISABLED);				// active low - disable by default
	setSelectIn(LOW);						// active low - enable by default
	printer.ackstate = NO_ACK;
	printer.useAck = ACK;
	printer.addCR = false;
	printer.addLF = false;

	setLED(STAT_PRINTING, OFF);
	setLED(STAT_SERIAL_RECV, OFF);
	setLED(STAT_CTS_OFFLINE, OFF);

	SerialPort.begin(); // initialise serial port
	_delay_ms(250);		// pause to allow everything to stabilise - DO WE NEED THIS?

	SerialPort.clearBuffer();
	SerialPort.writeln("--");
	SerialPort.writeln("PRT_START");

	// Do we have an LCD panel?
	lcd_present = !(1 & lcd.checkAlive());
	if (lcd_present) {
		SerialPort.writeln("LCD_OK");
		displayInit(); // initialise LCD panel
		char msg_buf[21];
		sprintf(msg_buf, "LCD_VERSION_%i", lcd.readRegister(3));
		displayText(msg_buf, 1);
		SerialPort.writeln(msg_buf);
	} else {
		SerialPort.writeln("LCD_NONE");
	}

	displayMsg(serial_disp_msg[SER_OK], SERIAL);
	DataRegister.shiftOut(0, MSBFIRST);

	// flash LEDs
	setLED(STAT_PRINTING, ON);
	_delay_ms(BOOT_LED_DELAY);
	setLED(STAT_SERIAL_RECV, ON);
	_delay_ms(BOOT_LED_DELAY);
	setLED(STAT_CTS_OFFLINE, ON);
	_delay_ms(BOOT_LED_DELAY);
	setLED(STAT_PRINTING, OFF);
	_delay_ms(BOOT_LED_DELAY);
	setLED(STAT_SERIAL_RECV, OFF);
	_delay_ms(BOOT_LED_DELAY);
	setLED(STAT_CTS_OFFLINE, OFF);

	initialisePrinter();		// also sets CTS to OFFLINE again

	// Set up interrupts
	EICRA = 0b00001010;					   // INT0 & INT1 to trigger on falling edge
	EIMSK |= ((1 << INT0) | (1 << INT1));  // enable INT0 & INT1
	EIFR |= ((1 << INTF0) | (1 << INTF1)); // clear the interrupt flags
	sei();								   // enable global interrupts

	updatePrinterState(true);	// also sets CTS if printer ready
	sendStateMsg();

	/********************************************************************************
	*****   MAIN LOOP                                                           *****
	*********************************************************************************/
	bool runloop = true;
	while (runloop) {
		if (SerialPort.inWaiting()) {
			// We have serial data waiting. We're assuming a string terminated with a null.
			// Any characters after a null will be ignored this time through the loop.
			//update_serial_state = true;
			//setCTS(CTS_OFFLINE);
			setLED(STAT_SERIAL_RECV, ON);
			bool ser_err = false;
			// Loop, grabbing a byte at a time from the serial input.
			// Keep going until we've got a sensible string (ie, we've encountered
			// a terminator), or have reached the end of the buffer, or have
			// encountered an error.
			while (!buf_ready && !ser_err && buf_index < PRINT_BUF_LEN) {
				bool gotByte = false;
				uint8_t byte = 0;			// to hold value for current byte. Default to terminator.
				uint32_t no_data_count = 0; // for timeout
				while (!gotByte && !ser_err) {
					byte = SerialPort.readByte();
					if (byte != NO_DATA) { // NO_DATA = 128, the high-order version of null
						gotByte = true;
					} else {
						no_data_count++;
						if (no_data_count >= SERIAL_TIMEOUT_LOOP_COUNTER) {
							ser_err = true;
							SerialPort.writeln(serial_comm_msg[SER_READ_TO]);
							displayMsg(serial_disp_msg[SER_READ_TO], SERIAL);
							clearBuffer();
							// maybe also clear the serial buffer here?
							sendStateMsg();
						}
					}
				}
				if (!ser_err) {
					switch (byte) {
						case 0:							// string termination. Buffer is now ready.
							printBuf[buf_index] = byte; // add null termination to string
							buf_ready = true;
							break;
						case 10: // linefeed/newline 0x0A
							if(printer.autofeed == AF_DISABLED) {							
								// The printer is NOT set to issue a linefeed whenever it receives a
								// carriage return, so include linefeed character in buffer.
								printBuf[buf_index] = byte; // add newline to buffer
							}
							break;
						default:
							printBuf[buf_index] = byte;
							break;
					} // /switch
					// check on buffer length						
					if (buf_index == PRINT_BUF_LEN - 2) {
						// We're at the penultimate element in the array.
						// The next one must be a terminator.
						printBuf[buf_index + 1] = 0;
						buf_ready = true;
					} else {
						buf_index++;
						//serialport.write(".");
					}
				} else {
					if (TESTING) SerialPort.writeln("*** serial error ***");
				}
			}
			setLED(STAT_SERIAL_RECV, OFF);
		}

		if (buf_ready && buf_index > 0 && !printer.error) {
			// Is this message a regular bit of text to be printed, or is it a command
			// that the client can use to interact with the SmartParallel?
			if (printBuf[0] == SERIAL_COMMAND_CHAR)	{				// It's a  command.
				switch (printBuf[1]) { // The next byte is the instruction
					case CMD_PING: // SERIAL STUFF
						SerialPort.writeln("SER_PONG");
						break;
					case CMD_ACK_DISABLE: // switch print mode to use NO_ACK
						printer.useAck = NO_ACK;
						SerialPort.writeln("PIF_ACK_DISABLED");
						break;
					case CMD_ACK_ENABLE: // switch print mode to use ACK
						printer.useAck = ACK;
						SerialPort.writeln("PIF_ACK_ENABLED");
						break;
					case CMD_AUTOFEED_DISABLE: // disable AUTOFEED
						printer.autofeed = AF_DISABLED;
						setPin(&OUTPUT_PORT, AUTOFEED_PIN, AF_DISABLED);
						SerialPort.writeln("PIF_LF_DISABLED");
						break;
					case CMD_AUTOFEED_ENABLE:
						printer.autofeed = AF_ENABLED; // enable AUTOFEED
						setPin(&OUTPUT_PORT, AUTOFEED_PIN, AF_ENABLED);
						SerialPort.writeln("PIF_LF_ENABLED");
						break;
					case CMD_PRT_MODE_NORMAL:
						break;
					case CMD_PRT_MODE_COND:
						break;
					case CMD_PRT_MODE_DBL:
						break;
					case CMD_REPORT_STATE:		// report printer state back to client
						updatePrinterState(false);
						sendStateMsg();
						displayText("Sent report",2);
						break;
					case CMD_REPORT_ACK:		// report status of ACK setting to client
						if (printer.useAck) {
							SerialPort.writeln("PIF_ACK_ENABLED");
						} else {
							SerialPort.writeln("PIF_ACK_DISABLED");
						}
						break;
					case CMD_REPORT_AUTOFEED:	// report status of AUTOFEED setting to client
						if (printer.autofeed) {
							SerialPort.writeln("PIF_AF_ENABLED");
						} else {
							SerialPort.writeln("PIF_AF_DISABLED");
						}
						break;
				}
				clearBuffer();
			} else { 
				// It's not a special command, so it's text to print
				printBuffer();
				// sendStateMsg();
				displayMsg(serial_disp_msg[SER_OK], SERIAL);
			}
		}
		// Keep checking the status of the printer. If it changes, report the 
		// new state to the client and the LCD display.
		updatePrinterState(true);
		if (printer.state_changed) {
			sendStateMsg();
			printer.state_changed = false;
		}
	} // main while loop
} // main func
