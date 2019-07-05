/*
  SmartParallel.cpp

  Code for Atmel ATMEGA328PB micronctroller on the SmartParallel serial to
  parallel interface.

  Intended for use with an Epson MX-80 F/T III dot matrix printer.

  Designed to work with the Devantech 40 char x 4-line I2C LDC display (optional). The code checks
  for the presence of the display during initialisation.

  Epson MX-80 F/T III parallel interface has a transfer rate of 1000 cps.

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

#ifndef F_CPU					// if F_CPU was not defined in Project -> Properties
	#define F_CPU 8000000UL		// define it now as 8MHz unsigned long - prob change to 16MHz
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
const char * prt_state_disp_msg[] = {"READY", "DONE", "INITIALISING", "OFFLINE", "PRINTING", "BUSY", "ERROR", "PAPER_END", "ACK TIMEOUT", "BUSY TIMEOUT"};
const char * prt_state_comm_msg[] = {"PRT_READY", "PRT_DONE", "PRT_INIT", "PRT_OFFLINE", "PRT_PRINTING", "PRT_BUSY", "PRT_ERROR", "PRT_PE", "PRT_ACK_TIMEOUT", "PRT_BUSY_TIMEOUT"};

const char * serial_disp_msg[] = {"OK", "READ TIMEOUT", "BUF CLEARED"};
const char * serial_comm_msg[] = {"SER_OK", "SER_READ_TIMEOUT", "SER_BUF_CLEARED"};

const char * msg_prefix[] = {"Prt:", "Ser:"};

SMD_I2C_Device lcd = SMD_I2C_Device(LCD_ADDRESS, I2C_BUS_SPEED_STD);
SMD_AVR_Serial SerialPort = SMD_AVR_Serial(SERIAL_BAUD_RATE);	// default baudrate 19200
ShiftReg74hc595 DataRegister = ShiftReg74hc595(SHCP_PIN, STCP_PIN, DATA_PIN, &SHIFTREG_REG, &SHIFTREG_DDR);

char printBuf[PRINT_BUF_LEN];	// line buffer of which last element will always be null terminator
uint8_t buf_index = 0;
bool buf_ready = false;
bool lcd_present = false;			// assume no display

/********************************************************************************
*****   FORWARD DECLARATIONS                                                *****
*********************************************************************************/
// PRINTER FUNCTIONS
void clearBuffer();
printer_state initialisePrinter();
printer_state printBuffer();
printer_state printChar(uint8_t chardata, ack_state waitForACK);
void resetAck();
printer_state updatePrinterState();

/********************************************************************************
*****   INTERRUPT HANDLERS                                                  *****
*********************************************************************************/

ISR(INT0_vect)		// for /ACK line
{
	printer.ackstate = ACK;
}

ISR(INT1_vect)		// for /ERROR line
{
	printer.state = ERROR;
	clearBuffer();
}

/********************************************************************************
*****   PRINTER FUNCTIONS                                                   *****
*********************************************************************************/

void clearBuffer()
{
	for(uint8_t i = 0; i < PRINT_BUF_LEN; i++) {
		printBuf[i] = 0;
	}
	buf_index = 0;
	buf_ready = false;
	//serialport.writeln("PRT_BUF_CLEARED");
	SerialPort.clearBuffer();
	//serialport.writeln(serial_comm_msg[SER_BUF_CLEARED]);
	//serialport.writeln(serial_comm_msg[SER_OK]);
}

printer_state initialisePrinter()
{
	setCTS(LOW);							// deter serial input
	resetAck();								// probably not necessary, but what the hell
	printer.state = INIT;
	sendStateMsg();						    // show we're initialising
	clearBuffer();							// clear the print data buffer
	SerialPort.clearBuffer();				// clear serial port's input buffer
	DataRegister.shiftOut(0);				// set data lines to all zeroes
	setPin(&OUTPUT_PORT, INIT_PIN, LOW);	// send /INIT pulse
	_delay_us(INIT_PULSE_LENGTH);			// - keep line low
	setPin(&OUTPUT_PORT, INIT_PIN, HIGH);	// - release line
	_delay_ms(POST_INIT_DELAY);				// - let the printer settle down
	updatePrinterState();					// let's see how the printer's doing
	if(printer.state == READY) setCTS(HIGH);	// tell remote machine we're ready to receive
	printer.LF_received = false;
	printer.CR_received = false;
	return printer.state;
}

printer_state printBuffer()
{
	setCTS(LOW);							// deter further incoming data
	printer.state = PRINTING;
	sendStateMsg();
	setLED(STAT_LED1_PIN, ON);
	uint8_t buf_index = 0;
	bool done = false;
	while(!done && !printer.error) {
		if(printBuf[buf_index] != 0) {
			printer.state = printChar(printBuf[buf_index], printer.useAck);	// print next character
			if(printer.state != DONE) {
				done = true;	// because we've encountered an error
				SerialPort.writeln(prt_state_disp_msg[printer.state]);
			}
		} else {
			done = true;	// we've encountered a 0, hence end of data
		}
		buf_index++;
		if(buf_index == PRINT_BUF_LEN - 1) done = true;	// we're at the penultimate byte of the buffer, so finished
	}
	//if(TESTING) displayBuffer();
	clearBuffer();
	setCTS(HIGH);					// signal that we're ready to receive again
	updatePrinterState();
	setLED(STAT_LED1_PIN, OFF);
	printer.LF_received = false;
	printer.CR_received = false;
	return printer.state;
}

printer_state printChar(uint8_t chardata, ack_state waitForACK)
{
	updatePrinterState();
	waitForACK = NO_ACK;	// FOR DEBUGGING ONLY
	if( !(printer.state == ERROR || printer.state == OFFLINE || printer.state == PAPER_END || !printer.error) ) {
		bool done = false;
		uint8_t waitTimeoutCounter = 0;
		while(!done) {
			if(printer.state == READY) {
				resetAck();									// an excess of caution
				DataRegister.shiftOut(chardata, MSBFIRST);	// put data on shift reg
				_delay_us(PRE_STROBE_DELAY);				// settle down for min 0.5us
				setPin(&OUTPUT_PORT, STROBE_PIN, LOW);		// start strobe pulse
				_delay_us(STROBE_PULSE_LENGTH);				// - pulse time
				setPin(&OUTPUT_PORT, STROBE_PIN, HIGH);		// - end pulse
				if(waitForACK == ACK) {						// did we specify in param to wait for ACK?
					uint8_t timeout_count = 0;				// NB: the printer will hold ACK low for around 5us
					while(timeout_count <= ACK_TIMEOUT_LOOP_COUNTER && printer.ackstate == NO_ACK) {
						// the ACK line is set up on interrupt INT0 so we need to wait
						// for the interrupt to trigger or our timeout to expire
						_delay_us(ACK_TIMEOUT_LOOP_DELAY);
						timeout_count++;
					}
					if(printer.ackstate == NO_ACK) {
						printer.state = ACK_TIMEOUT;
					} else {
						printer.state = DONE;
						resetAck();
					}
				} else {
					printer.state = DONE;						// we decided not to bother with ACK
				}
				done = true;
			} else {										// curr_state wasn't READY - printer might be busy
				_delay_ms(BUSY_TIMEOUT_LOOP_DELAY);
				updatePrinterState();
				waitTimeoutCounter++;
				if(waitTimeoutCounter == BUSY_TIMEOUT_LOOP_COUNTER) {
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
	EIFR = (1 << INTF0);	// clear the interrupt flag to be sure
}

printer_state updatePrinterState()
{
	// read state of input pins
	printer.state = READY;								// let's be optimistic
	printer.busy = readPin(&INPUT_REG, BUSY_PIN);		// active high
	printer.error = 1 - readPin(&ERR_REG, ERR_PIN);		// active low
	printer.pe = readPin(&INPUT_REG, PE_PIN);			// active high - /ERROR and BUSY will also be set
	printer.select = readPin(&INPUT_REG, SELECT_PIN);	// active high - low indicates offline

	// set overall state
	if(printer.error) printer.state = ERROR;
	if(printer.busy) printer.state = BUSY;
	if(!printer.select) printer.state = OFFLINE;
	if(printer.pe) printer.state = PAPER_END;

	//if(!readPin(&ERR_REG, ERR_PIN)) curr_state = ERROR;			// active low
	//if(readPin(&INPUT_REG, BUSY_PIN)) curr_state = BUSY;		// active high
	//if(!readPin(&INPUT_REG, SELECT_PIN)) curr_state = OFFLINE;	// active high - low indicates problem
	//if(readPin(&INPUT_REG, PE_PIN)) curr_state = PAPER_END;		// active high - /ERROR and BUSY will also be set
	return printer.state;
}

void setSelectIn(uint8_t hilo)
{
	setPin(&OUTPUT_PORT, SELIN_PIN, hilo);	// set pin
	printer.selectIn = hilo;				// store state
}

void setAutofeed(uint8_t hilo)
{
	// autofeed is active low, but we usually want it disabled
	setPin(&OUTPUT_PORT, AUTOFEED_PIN, hilo);
	printer.autofeed = hilo;
}

void disableAutofeed(void)
{
	setAutofeed(HIGH);
}

void enableAutofeed(void)
{
	setAutofeed(LOW);
}

int main(void)
{
	/********************************************************************************
	*****   SETUP                                                               *****
	*********************************************************************************/
	// Set pin directions
	SHIFTREG_DDR |= (1 << SHCP_PIN | 1 << STCP_PIN | 1 << DATA_PIN);			// shift reg pins as outputs
	OUTPUT_DDR |= (1 << STROBE_PIN | 1 << INIT_PIN | 1 << AUTOFEED_PIN | 1 << SELIN_PIN);	// outputs
	INPUT_DDR &= ~(1 << BUSY_PIN | 1 << PE_PIN | 1 << SELECT_PIN);				// BUSY, PE & SEL as inputs
	INPUT_INT_DDR &= ~(1 << ACK_PIN | 1 << ERR_PIN);							// ACK and ERR as inputs
	LED_DDR |= (1 << STAT_LED1_PIN | 1 << STAT_LED2_PIN | 1 << STAT_LED3_PIN);	// LEDS as outputs
	SERIAL_DDR |= (1 << CTS_PIN);												// CTS as output

	// Set outputs at initial values
	setCTS(LOW);								// refuse serial data while getting set up
	setPin(&OUTPUT_PORT, STROBE_PIN, HIGH);		// active low
	setPin(&OUTPUT_PORT, INIT_PIN, HIGH);		// active low
	setAutofeed(HIGH);							// active low - disable by default
	setSelectIn(LOW);							// active low - enable by default
	printer.ackstate = NO_ACK;
	printer.useAck = NO_ACK;

	setLED(STAT_LED1_PIN, OFF);
	setLED(STAT_LED2_PIN, OFF);
	setLED(STAT_LED3_PIN, OFF);
	
	updatePrinterState();
	printer_state prev_state = printer.state;

	SerialPort.begin();							// initialise serial port
	_delay_ms(250);								// pause to allow everything to stabilise - DO WE NEED THIS?

	SerialPort.clearBuffer();
	SerialPort.writeln("--");
	SerialPort.writeln("PRT_START");

	// Do we have an LCD panel?
	lcd_present = !(1 & lcd.checkAlive());
	if (lcd_present) {
		SerialPort.writeln("LCD_OK");
		displayInit();								// initialise LCD panel
		SerialPort.write("LCD_VERSION_"); SerialPort.writeln(lcd.readRegister(3));
	} else {
		SerialPort.writeln("LCD_NONE");
	}

	displayMsg(serial_disp_msg[SER_OK], SERIAL);

	bool runloop = true;
	initialisePrinter();	// also sets CTS HIGH again
	sendStateMsg();
	//if(curr_state != READY) {
		////runloop = false;
		//char err_buf[21];
		//sprintf(err_buf, "HALT/%s", prt_state_disp_msg[curr_state]);
		//displayMsg(err_buf, PRINTER);
		//serialport.writeln(err_buf);
	//} else {
		// flash the LEDs to show everything went well
		uint8_t testData = 0;
		for(uint8_t i=0; i<8; i++) {
			testData = testData | (1 << i);
			DataRegister.shiftOut(testData, MSBFIRST);
			_delay_ms(BOOT_LED_DELAY);
		}
		setLED(STAT_LED1_PIN, ON);
		_delay_ms(BOOT_LED_DELAY);
		setLED(STAT_LED2_PIN, ON);
		_delay_ms(BOOT_LED_DELAY);
		setLED(STAT_LED3_PIN, ON);
		_delay_ms(BOOT_LED_DELAY);

		// Turn off LEDs
		DataRegister.shiftOut(0, MSBFIRST);
		setLED(STAT_LED1_PIN, OFF);
		setLED(STAT_LED2_PIN, OFF);
		setLED(STAT_LED3_PIN, OFF);
	//}

	// Set up interrupts
	EICRA = 0b00001010;						// INT0 & INT1 to trigger on falling edge
	EIMSK |= ((1 << INT0) | (1 << INT1));	// enable INT0 & INT1
	EIFR |= ((1 << INTF0) | (1 << INTF1));	// clear the interrupt flags
	sei();									// enable global interrupts

	/********************************************************************************
	*****   MAIN LOOP                                                           *****
	*********************************************************************************/
    while (runloop)
    {
		if(SerialPort.inWaiting()) {
			// We have serial data waiting. We're assuming a string terminated with a null,
			// a linefeed or a carriage return. Any characters after any one of those three
			// will be ignored this time through the loop.
			//update_serial_state = true;
			//if(TESTING) serialport.write(":");
			setLED(STAT_LED2_PIN, ON);
			bool ser_err = false;
			// Loop, grabbing a byte at a time from the serial input.
			// Keep going until we've got a sensible string (ie, we've encountered
			// a terminator), or have reached the end of the buffer, or have
			// encountered an error.
			while(!buf_ready && !ser_err && buf_index < PRINT_BUF_LEN) {
				//serialport.write("'");
				bool gotByte = false;			// have we retrieved a byte from input?
				uint8_t byte = 0;				// value for current byte. Default to terminator.
				uint32_t no_data_count = 0;		// for timeout
				while(!gotByte && !ser_err) {
					byte = SerialPort.readByte();
					if(byte != NO_DATA) {		// NO_DATA = 128, the high-order version of null
						gotByte = true;
					} else {
						no_data_count++;
						if(no_data_count >= SERIAL_TIMEOUT_LOOP_COUNTER) {
							ser_err = true;
							SerialPort.writeln(serial_comm_msg[SER_READ_TO]);
							displayMsg(serial_disp_msg[SER_READ_TO], SERIAL);
							clearBuffer();
							//byte = 0;
							sendStateMsg();
						}
					}
				}
				if (!ser_err) {
					switch(byte) {
						case 0:							// string termination. Buffer is now ready.
							printBuf[buf_index] = byte;	// add null termination to string
							buf_ready = true;
							break;
						case 10:							// newline 0x0A
							//printer.LF_received = true;
							if(printer.autofeed) {
								// The printer is set to issue a linefeed whenever it receives a
								// carriage return. In that case, we'll ignore linefeeds
								// -- do nothing here --
							} else {
								printBuf[buf_index] = byte;		// add newline to buffer
							}
							// if(buf_index < PRINT_BUF_LEN - 2) {
							// 	// there are still at least 2 bytes left in buffer
							// 	printBuf[buf_index + 1] = 13;	// add a carriage return
							// 	printBuf[buf_index + 2] = 0;	// also add null termination
							// } else {
							// 	printBuf[buf_index + 1] = 0;	// we're at penultimate byte - last one has to be null
							// }
							// buf_ready = true;
							break;
						// case 13:							// carriage return 0x0D
						// 	printer.CR_received = true;
						// 	printBuf[buf_index] = byte;		// keep CR in string
						// 	if(buf_index < PRINT_BUF_LEN - 2) {
						// 		// there are still at least 2 bytes left in buffer
						// 		if(printer.autofeed) {
						// 			// the newline will be added by the printer
						// 			printBuf[buf_index + 1] = 0;
						// 		} else {
						// 			printBuf[buf_index + 1] = 10;	// add a newline
						// 			printBuf[buf_index + 2] = 0;	// also add null termination
						// 		}
						// 	} else {
						// 		printBuf[buf_index + 1] = 0;	// we're at penultimate byte - last one has to be null
						// 	}
						// 	buf_ready = true;
						// 	break;
						default:
							printBuf[buf_index] = byte;
							// check on buffer length
							if(buf_index == PRINT_BUF_LEN - 2) {
								// We're at the penultimate element in the array.
								// The next one must be a terminator.
								printBuf[buf_index + 1] = 0;
								buf_ready = true;
							} else {
								buf_index++;
								//serialport.write(".");
							}
							break;
					}
				} else {
					if(TESTING) SerialPort.writeln("*** serial error ***");
				}
			}
			setLED(STAT_LED2_PIN, OFF);
		}

		if(buf_ready && buf_index > 0 && !printer.error) {
			if(TESTING) SerialPort.writeln("Buffer ready");
			//serialstate = SER_OK;			// must have been okay because we've got decent buffer
			//update_serial_state = true;
			if(printBuf[0] == SERIAL_COMMAND_CHAR) {	// SPECIAL COMMAND	
				switch(printBuf[1]) {					// The next byte is the instruction
					case 1:								// report printer interface settings
						if(printer.useAck) {
							SerialPort.writeln("PIF_ACK_ENABLED");
						} else {
							SerialPort.writeln("PIF_ACK_DISABLED");
						}
						if(printer.autofeed) {
							SerialPort.writeln("PIF_LF_ENABLED");
						} else {
							SerialPort.writeln("PIF_LF_DISABLED");
						}
						break;
					case 2:					// switch print mode to use ACK
						printer.useAck = ACK;
						SerialPort.writeln("PIF_ACK_ENABLED");
						break;
					case 3:					// switch print mode to use NO_ACK
						printer.useAck = NO_ACK;
						SerialPort.writeln("PIF_ACK_DISABLED");
						break;
					case 4:
						printer.autofeed = true;		// enable AUTOFEED
						setPin(&OUTPUT_PORT, AUTOFEED_PIN, LOW);
						SerialPort.writeln("PIF_LF_ENABLED");
						break;
					case 5:					// disable AUTOFEED
						printer.autofeed = false;
						setPin(&OUTPUT_PORT, AUTOFEED_PIN, HIGH);
						SerialPort.writeln("PIF_LF_DISABLED");
						break;
					case 32:
						updatePrinterState();
						sendStateMsg();
					case 64:				// SERIAL STUFF
						SerialPort.writeln("SER_PONG");
						break;
				}
				clearBuffer();
			} else {
				printBuffer();
				sendStateMsg();
				displayMsg(serial_disp_msg[SER_OK], SERIAL);
			}
		}

		updatePrinterState();
		if(printer.state != prev_state) {
			sendStateMsg();
			prev_state = printer.state;
		}
    }
}

