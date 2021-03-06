/*
 * SmartParallel.cpp
 *
 * Created: 4/23/2018 6:53:13 PM
 * Author : Steve

  Epson MX-80 F/T III parallel interface has a transfer rate of 1000 cps.

  328PB (SMD) version

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
	#define F_CPU 8000000UL		// define it now as 16 MHz unsigned long
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
#include <ShiftReg74hc595.cpp>

#define PRINT_BUF_LEN 255
#define SERIAL_BAUD_RATE 19200

// TIME-OUTS
// while the ACK pulse lasts around 5us on the Epson, it might take as long as 1ms after
// the strobe pulse is finished for the ACK pulse to be sent.
#define ACK_TIMEOUT_LOOP_DELAY 1			// us - microseconds
#define ACK_TIMEOUT_LOOP_COUNTER 1000		// xACK_TIMEOUT_LOOP_DELAY

#define BUSY_TIMEOUT_LOOP_DELAY 10			// milliseconds
#define BUSY_TIMEOUT_LOOP_COUNTER 200		// xBUSY_TIMEOUT_LOOP_DELAY

#define SERIAL_TIMEOUT_LOOP_COUNTER 1710000	// roughly 570000 = 1 sec

// TIMINGS
#define INIT_PULSE_LENGTH 75				// us - time to hold init low - min 50us
#define POST_INIT_DELAY 2000				// ms - time for printer to settle down after init
#define PRE_STROBE_DELAY 1					// us - time between setting data & strobe pulse, min 0.5us
#define STROBE_PULSE_LENGTH 1				// us - min 0.5us

enum printer_state {READY, DONE, INIT, OFFLINE, PRINTING, BUSY, ERROR, PAPER_END, ACK_TIMEOUT, BUSY_TIMEOUT};
const char * prt_state_disp_msg[] = {"READY", "DONE", "INITIALISING", "OFFLINE", "PRINTING", "BUSY", "ERROR", "PAPER_END", "ACK TIMEOUT", "BUSY TIMEOUT"};
const char * prt_state_comm_msg[] = {"PRT_READY", "PRT_DONE", "PRT_INIT", "PRT_OFFLINE", "PRT_PRINTING", "PRT_BUSY", "PRT_ERROR", "PRT_PE", "PRT_ACK_TIMEOUT", "PRT_BUSY_TIMEOUT"};

enum serial_state {SER_OK, SER_READ_TO, SER_BUF_CLEARED};
const char * serial_comm_msg[] = {"SER_OK", "SER_READ_TIMEOUT", "SER_BUF_CLEARED"};
const char * serial_disp_msg[] = {"OK", "READ TIMEOUT", "BUF CLEARED"};

enum ack_state {ACK, NO_ACK};
enum error_state {NO_ERR, ERR};

#define LCD_ADDRESS 0xC6
enum lcd_msg_type {PRINTER, SERIAL};
const char * msg_prefix[] = {"Prt:", "Ser:"};

// ----- Outputs ----------------------------------
#define STROBE_PIN		PC0
#define INIT_PIN		PC1
#define AUTOFEED_PIN	PC2
#define SELIN_PIN		PC3		// probably not using
#define CTS_PIN			PD7
#define OUTPUT_PORT		PORTC
#define OUTPUT_DDR		DDRC
// ----- Serial Port ------------------------------
#define CTS_PORT		PORTD
#define SERIAL_DDR		DDRD
// ----- LEDS -------------------------------------
#define LED_PRINTING_PIN	PE1
#define LED_ERROR_PIN		PE2
#define LED_SERIAL_PIN		PE3
#define LED_PORT			PORTE
#define LED_DDR				DDRE

// ----- Inputs -----------------------------------
#define ACK_PIN		PD2
#define ERR_PIN		PD3
#define BUSY_PIN	PD4
#define PE_PIN		PD6
#define SELECT_PIN	PD5
#define INPUT_PORT  PIND
#define INPUT_DDR	DDRD

// Output shift register
#define SHCP_PIN		PB2		// clock pin
#define STCP_PIN		PB1		// latch pin
#define DATA_PIN		PB0		// data pin
#define SHIFTREG_DDR	DDRB

/********************************************************************************
*****   GLOBALS                                                             *****
*********************************************************************************/

SMD_I2C_Device lcd = SMD_I2C_Device(LCD_ADDRESS, I2C_BUS_SPEED_STD);
SMD_AVR_Serial serialport = SMD_AVR_Serial(SERIAL_BAUD_RATE);	// default baudrate 19200
ShiftReg74hc595 DataRegister = ShiftReg74hc595(SHCP_PIN, STCP_PIN, DATA_PIN, &PORTB, &DDRB);

char printBuf[PRINT_BUF_LEN];	// line buffer of which last element will always be null terminator
uint8_t buf_index = 0;
bool buf_ready = false;
ack_state ackstate = NO_ACK;
ack_state useAck = ACK;
error_state errorState = NO_ERR;
bool autoLF = false;				// is printer set to use auto linefeed?
printer_state curr_state = READY;
bool lcd_present = false;			// assume not

/********************************************************************************
*****   FORWARD DECLARATIONS                                                *****
*********************************************************************************/
// DISPLAY FUNCTIONS
void displayBuffer();
void displayInit();
void displayMsg(const char msg[], lcd_msg_type msg_type);
void writeCharToLCD(uint8_t chr);
// SERIAL FUNCTIONS
void sendStateMsg();
void setCTS(uint8_t hilo);
// GENERAL FUNCTIONS
uint8_t readPin(volatile uint8_t * portreg, uint8_t pin);
void setPin(volatile uint8_t * portreg, uint8_t pin, uint8_t hilo);
void setLED(uint8_t pin, uint8_t onOff);
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
	ackstate = ACK;
}

ISR(INT1_vect)		// for /ERROR line
{
	errorState = ERR;
	clearBuffer();
}

/********************************************************************************
*****   DISPLAY FUNCTIONS                                                   *****
*********************************************************************************/

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
					writeCharToLCD(printBuf[index]);
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
			writeCharToLCD(msg_prefix[msg_type][i]);
			//lcd.sendByte(char(msg_prefix[msg_type][i]));
		}
		for(uint8_t i = 0; i < strlen(msg); i++) {
			writeCharToLCD(msg[i]);
			//lcd.sendByte(char(msg[i]));
		}
		// clear rest of line with spaces
		for(uint8_t i = strlen(msg); i < 12; i++) {
			writeCharToLCD(0x20);
			//lcd.sendByte(0x20);	// space
		}
		lcd.stop();
	}
}

void writeCharToLCD(uint8_t chr)
{
	// need to limit the range of characters that are sent to LCD
	if( (31 < chr && chr < 127) || (159 < chr && chr < 255) ) {
		lcd.sendByte(chr);
	}
}

/********************************************************************************
*****   GENERAL FUNCTIONS                                                   *****
*********************************************************************************/
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
	setPin(&LED_PORT, pin, onOff);
}

/********************************************************************************
*****   SERIAL FUNCTIONS                                                    *****
*********************************************************************************/
void sendStateMsg()
{
	serialport.writeln(prt_state_comm_msg[curr_state]);
	displayMsg(prt_state_disp_msg[curr_state], PRINTER);
}

void setCTS(uint8_t hilo)
{
	setPin(&CTS_PORT, CTS_PIN, hilo);
}

/********************************************************************************
*****   PRINTING FUNCTIONS                                                  *****
*********************************************************************************/

void clearBuffer()
{
	for(uint8_t i = 0; i < PRINT_BUF_LEN; i++) {
		printBuf[i] = 0;
	}
	buf_index = 0;
	buf_ready = false;
	//serialport.writeln("PRT_BUF_CLEARED");
	serialport.clearBuffer();
	//serialport.writeln(serial_comm_msg[SER_BUF_CLEARED]);
	//serialport.writeln(serial_comm_msg[SER_OK]);
}

printer_state initialisePrinter()
{
	setCTS(LOW);							// deter serial input
	resetAck();								// probably not necessary, but what the hell
	curr_state = INIT;
	sendStateMsg();						    // show we're initialising
	clearBuffer();							// clear the print data buffer
	serialport.clearBuffer();				// clear serial port's input buffer
	DataRegister.shiftOut(0);				// set data lines to all zeroes
	setPin(&OUTPUT_PORT, INIT_PIN, LOW);	// send /INIT pulse
	_delay_us(INIT_PULSE_LENGTH);
	setPin(&OUTPUT_PORT, INIT_PIN, HIGH);
	_delay_ms(POST_INIT_DELAY);				// let the printer settle down
	updatePrinterState();					// let's see how the printer's doing
	if(curr_state == READY) setCTS(HIGH);	// tell remote machine we're ready to receive
	return curr_state;
}

printer_state printBuffer()
{
	setCTS(LOW);							// deter further incoming data
	curr_state = PRINTING;
	sendStateMsg();
	setLED(LED_PRINTING_PIN, ON);
	uint8_t buf_index = 0;
	bool done = false;
	while(!done && errorState == NO_ERR) {
		if(printBuf[buf_index] != 0) {
			curr_state = printChar(printBuf[buf_index], useAck);
			if(curr_state != DONE) {
				done = true;	// because we've encountered an error
				serialport.writeln(prt_state_disp_msg[curr_state]);
			}
		} else {
			done = true;	// we've encountered a 0, hence end of data
		}
		buf_index++;
		if(buf_index == PRINT_BUF_LEN - 1) done = true;	// we're at the penultimate byte of the buffer, so finished
	}
	//if(TESTING) displayBuffer();
	clearBuffer();
	setCTS(HIGH);
	updatePrinterState();
	setLED(LED_PRINTING_PIN, OFF);
	return curr_state;
}

printer_state printChar(uint8_t chardata, ack_state waitForACK)
{
	updatePrinterState();
	if( !(curr_state == ERROR || curr_state == OFFLINE || curr_state == PAPER_END || errorState == ERR) ) {
		bool done = false;
		uint8_t waitTimeoutCounter = 0;
		while(!done) {
			if(curr_state == READY) {				// Print character
				resetAck();						// an excess of caution
				DataRegister.shiftOut(chardata, MSBFIRST); // put data on shift reg
				_delay_us(PRE_STROBE_DELAY);			// min 0.5us
				setPin(&OUTPUT_PORT, STROBE_PIN, LOW);	// start strobe pulse
				_delay_us(STROBE_PULSE_LENGTH);			// pulse time
				setPin(&OUTPUT_PORT, STROBE_PIN, HIGH);	// end pulse
				if(waitForACK == ACK) { 
					uint8_t timeout_count = 0;	// the printer will hold ACK low for around 5us
					while(timeout_count <= ACK_TIMEOUT_LOOP_COUNTER && ackstate == NO_ACK) {
						// the ACK line is set up on interrupt INT0 so we need to wait
						// for the interrupt to trigger
						_delay_us(ACK_TIMEOUT_LOOP_DELAY);
						timeout_count++;
					}
					if(ackstate == NO_ACK) {
						curr_state = ACK_TIMEOUT;
					} else {
						curr_state = DONE;
						ackstate = NO_ACK;	// reset
					}
				} else {
					curr_state = DONE;
				}
				done = true;
			} else {
				_delay_ms(BUSY_TIMEOUT_LOOP_DELAY);
				updatePrinterState();
				waitTimeoutCounter++;
				if(waitTimeoutCounter == BUSY_TIMEOUT_LOOP_COUNTER) {
					curr_state = BUSY_TIMEOUT;
					done = true;
				}
			}
		}
	}
	return curr_state;
}

void resetAck()
{
	ackstate = NO_ACK;
	EIFR = (1 << INTF0);	// clear the interrupt flag to be sure
}

printer_state updatePrinterState()
{
	curr_state = READY;						// let's assume the best
	if(!readPin(&INPUT_PORT, ERR_PIN)) curr_state = ERROR;	// active low
	if(readPin(&INPUT_PORT, BUSY_PIN)) curr_state = BUSY;	// active high
	if(!readPin(&INPUT_PORT, SELECT_PIN)) curr_state = OFFLINE;
	if(readPin(&INPUT_PORT, PE_PIN)) curr_state = PAPER_END;	// active high - /ERROR and BUSY will also be set
	if(curr_state == READY) errorState = NO_ERR;
	return curr_state;
}


int main(void)
{
	/********************************************************************************
	*****   SETUP                                                               *****
	*********************************************************************************/

	SHIFTREG_DDR |= (1 << SHCP_PIN | 1 << STCP_PIN | 1 << DATA_PIN);
	OUTPUT_DDR |= (1 << STROBE_PIN | 1 << INIT_PIN | 1 << AUTOFEED_PIN | 1 << SELIN_PIN);
	INPUT_DDR &= ~(1 << ACK_PIN	| 1 << ERR_PIN | 1 << BUSY_PIN | 1 << PE_PIN | 1 << SELECT_PIN);
	LED_DDR |= (1 << LED_PRINTING_PIN | 1 << LED_ERROR_PIN | 1 << LED_SERIAL_PIN);
	SERIAL_DDR |= (1 << CTS_PIN);

	setCTS(LOW);						// refuse serial data while getting set up

	// Set outputs at initial values
	setPin(&OUTPUT_PORT, STROBE_PIN, HIGH);		// active low
	setPin(&OUTPUT_PORT, INIT_PIN, HIGH);		// active low
	setPin(&OUTPUT_PORT, AUTOFEED_PIN, LOW);	// active high
	//setPin(&OUTPUT_PORT, SELIN_PIN, LOW);		// active high
	setLED(LED_PRINTING_PIN, OFF);
	setLED(LED_ERROR_PIN, OFF);
	setLED(LED_SERIAL_PIN, OFF);
	
	updatePrinterState();
	printer_state prev_state = curr_state;

	serialport.begin();							// initialise serial port

	_delay_ms(250);								// pause to allow everything to stabilise - DO WE NEED THIS?

	serialport.clearBuffer();
	serialport.writeln("--");
	serialport.writeln("PRT_START");

	// do we have an LCD panel?
	lcd_present = !(1 & lcd.checkAlive());
	//serialport.write("I2C_ERR:"); serialport.writeln(lcd_present);
	if (lcd_present) {
		serialport.writeln("LCD_OK");
		displayInit();								// initialise LCD panel
		uint8_t lcd_version = lcd.readRegister(3);
		serialport.write("LCD_VERSION_"); serialport.writeln(lcd_version);
	} else {
		serialport.writeln("LCD_NONE");
	}

	displayMsg(serial_disp_msg[SER_OK], SERIAL);

	bool runloop = true;
	//bool update_serial_state = true;
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
		setLED(LED_PRINTING_PIN, ON);
		_delay_ms(150);
		setLED(LED_PRINTING_PIN, OFF);
		setLED(LED_ERROR_PIN, ON);
		_delay_ms(150);
		setLED(LED_ERROR_PIN, OFF);
		setLED(LED_SERIAL_PIN, ON);
		_delay_ms(150);
		setLED(LED_SERIAL_PIN, OFF);
	//}

	EICRA = 0b00001010;		// INT0 & INT1 to trigger on falling edge
	EIMSK |= ((1 << INT0) | (1 << INT1));	// enable INT0 & INT1
	EIFR |= ((1 << INTF0) | (1 << INTF1));	// clear the interrupt flags
	sei();					// enable global interrupts

	/********************************************************************************
	*****   MAIN LOOP                                                           *****
	*********************************************************************************/
    while (runloop)
    {
		if(serialport.inWaiting()) {
			// we have serial data waiting. We're assuming a string terminated with a null,
			// a linefeed or a carriage return. Any characters after any one of thoe three
			// will be ignored.
			//update_serial_state = true;
			//if(TESTING) serialport.write(":");
			setLED(LED_SERIAL_PIN, ON);
			bool ser_err = false;
			while(!buf_ready && !ser_err && buf_index < PRINT_BUF_LEN) {
				//serialport.write("'");
				bool gotByte = false;
				uint8_t byte = 0;
				uint32_t no_data_count = 0;		// for timeout
				while(!gotByte && !ser_err) {
					byte = serialport.readByte();
					if(byte != NO_DATA) {		// NO_DATA = 128, the high-order version of null
						gotByte = true;
					} else {
						no_data_count++;
						if(no_data_count >= SERIAL_TIMEOUT_LOOP_COUNTER) {
							ser_err = true;
							serialport.writeln(serial_comm_msg[SER_READ_TO]);
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
						case 10:							// newline 0x0A - also a string termination
							printBuf[buf_index] = byte;		// keep newline in string
							if(buf_index < PRINT_BUF_LEN - 2) {
								// there are still at least 2 bytes left in buffer
								printBuf[buf_index + 1] = 13;	// add a carriage return
								printBuf[buf_index + 2] = 0;	// also add null termination
							} else {
								printBuf[buf_index + 1] = 0;	// we're at penultimate byte - last one has to be null
							}
							buf_ready = true;
							break;
						case 13:							// carriage return 0x0D - also a string termination
							printBuf[buf_index] = byte;		// keep CR in string
							if(buf_index < PRINT_BUF_LEN - 2) {
								// there are still at least 2 bytes left in buffer
								if(autoLF) {
									// the newline will be added by the printer
									printBuf[buf_index + 1] = 0;
								} else {
									printBuf[buf_index + 1] = 10;	// add a newline
									printBuf[buf_index + 2] = 0;	// also add null termination
								}
							} else {
								printBuf[buf_index + 1] = 0;	// we're at penultimate byte - last one has to be null
							}
							buf_ready = true;
							break;
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
					if(TESTING) serialport.writeln("*** serial error ***");
				}
			}
			setLED(LED_SERIAL_PIN, OFF);
		}

		if(buf_ready && buf_index > 0 && errorState == NO_ERR) {
			if(TESTING) serialport.writeln("Buffer ready");
			//serialstate = SER_OK;			// must have been okay because we've got decent buffer
			//update_serial_state = true;
			if(printBuf[0] == 1) {			// SPECIAL COMMAND	
				switch(printBuf[1]) {		// what's the next byte?
					case 1:					// report printer interface settings
						if(useAck) {
							serialport.writeln("PIF_ACK_ENABLED");
							} else {
							serialport.writeln("PIF_ACK_DISABLED");
						}
						if(autoLF) {
							serialport.writeln("PIF_LF_ENABLED");
							} else {
							serialport.writeln("PIF_LF_DISABLED");
						}
						break;
					case 2:					// switch print mode to use ACK
						useAck = ACK;
						serialport.writeln("PIF_ACK_ENABLED");
						break;
					case 3:					// switch print mode to use NO_ACK
						useAck = NO_ACK;
						serialport.writeln("PIF_ACK_DISABLED");
						break;
					case 4:
						autoLF = true;
						serialport.writeln("PIF_LF_ENABLED");
						break;
					case 5:
						autoLF = false;
						serialport.writeln("PIF_LF_DISABLED");
						break;
					case 32:
						updatePrinterState();
						sendStateMsg();
					case 64:				// SERIAL STUFF
						serialport.writeln("SER_PONG");
						break;
				}
				clearBuffer();
			} else {
				printBuffer();
				sendStateMsg();
				displayMsg(serial_disp_msg[SER_OK], SERIAL);
			}
		}

		//if(errorState == ERR) {
			//updatePrinterState();
			//sendStateMsg();
			//serialport.writeln("STOPPED");
			//runloop = false; // crash out
		//}

		updatePrinterState();
		if(curr_state != prev_state) {
			sendStateMsg();
			prev_state = curr_state;
		}
    }
}

