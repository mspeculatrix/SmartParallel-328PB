/*
 * SmartParallel328PB_includes.h
 *
 * Macros and enums for SmartParallel.
 *
 */

#ifndef SMARTPARALLEL328PB_DEFINES_H_
#define SMARTPARALLEL328PB_DEFINES_H_

#define PRINT_BUF_LEN 255
#define SERIAL_BAUD_RATE 19200

#define BOOT_LED_DELAY 50
#define DEFAULT_TAB_SIZE 4

#define SERIAL_COMMAND_CHAR 1 // ASCII for command mode
// Commands in command mode
#define CMD_PING				1
#define CMD_ACK_DISABLE			2
#define CMD_ACK_ENABLE			3
#define CMD_AUTOFEED_DISABLE	4
#define CMD_AUTOFEED_ENABLE		5
#define CMD_PRT_MODE_NORMAL		8 	// 80-column mode
#define CMD_PRT_MODE_COND		9 	// Condensed mode - 132-column
#define CMD_PRT_MODE_DBL		10 	// Double-width mode - 40-column
#define CMD_LINEEND_NORMAL		16 	// Don't add anything to ends of lines (def)
#define CMD_LINEEND_LF			17
#define CMD_LINEEND_CR			18
#define CMD_LINEEND_CRLF		19
#define CMD_REPORT_STATE		32
#define CMD_REPORT_ACK			33
#define CMD_REPORT_AUTOFEED		34

// TIME-OUTS
// while the ACK pulse lasts around 5us on the Epson, it might take as long as 1ms after
// the strobe pulse is finished for the ACK pulse to be sent.
#define ACK_TIMEOUT_LOOP_DELAY 1	  // us - microseconds
#define ACK_TIMEOUT_LOOP_COUNTER 1000 // xACK_TIMEOUT_LOOP_DELAY

#define BUSY_TIMEOUT_LOOP_DELAY 10	  // milliseconds
#define BUSY_TIMEOUT_LOOP_COUNTER 200 // xBUSY_TIMEOUT_LOOP_DELAY

#define SERIAL_TIMEOUT_LOOP_COUNTER 1710000 // 570000 = 1 sec (approx)

// TIMINGS
#define INIT_PULSE_LENGTH 75  // us - time to hold init low - min 50us
#define POST_INIT_DELAY 2000  // ms - time for printer to settle down after init
#define PRE_STROBE_DELAY 1	// us - time between setting data & strobe pulse, min 0.5us
#define STROBE_PULSE_LENGTH 1 // us - min 0.5us

// ----- PRINTER OUTPUTS ----------------------------------
#define STROBE_PIN PC0
#define INIT_PIN PC1
#define AUTOFEED_PIN PC2
#define SELIN_PIN PC3 // probably not using
#define OUTPUT_PORT PORTC
#define OUTPUT_DDR DDRC

// Output shift register
#define SHCP_PIN PB2 // clock pin
#define STCP_PIN PB1 // latch pin
#define DATA_PIN PB0 // data pin
#define SHIFTREG_DDR DDRB
#define SHIFTREG_REG PORTB

// ----- PRINTER INPUTS -----------------------------------
// Input pins that trigger interrupts
#define ACK_PIN PD2  // INT0 interrupt pin
#define ERR_PIN PD3  // INT1 interrupt pin
#define ERR_REG PIND // don't need the register for ACK
#define INPUT_INT_DDR DDRD
// Other input pins
#define BUSY_PIN PE3
#define PE_PIN PE1
#define SELECT_PIN PE2
#define INPUT_REG PINE
#define INPUT_DDR DDRE

// ----- SERIAL PORT ------------------------------
#define SERIAL_REG PORTD
#define SERIAL_DDR DDRD
// most of the serial port is on Port D, but CTS is on Port E.
#define CTS_PIN		PE0
#define CTS_REG		PORTE
#define CTS_DDR		DDRE
#define CTS_ONLINE  0	// CTS is active low
#define CTS_OFFLINE 1

// ----- DISPLAY ----------------------------------
#define LCD_ADDRESS 0xC6

// ----- LEDS -------------------------------------
#define STAT_PRINTING    PD5	// indicates SP is printing
#define STAT_SERIAL_RECV PD6	// SP is receiving serial data
#define STAT_CTS_OFFLINE  PD7
#define LED_REG PORTD
#define LED_DDR DDRD

enum printer_state
{
	READY,
	DONE,
	INIT,
	OFFLINE,
	PRINTING,
	BUSY,
	ERROR,
	PAPER_END,
	ACK_TIMEOUT,
	BUSY_TIMEOUT
};

enum serial_state
{
	SER_OK,
	SER_READ_TO,
	SER_BUF_CLEARED
};

enum ack_state
{
	ACK,
	NO_ACK
};

enum autofeed_state {
	AF_ENABLED,
	AF_DISABLED
};

enum lcd_msg_type
{
	PRINTER,
	SERIAL
};

struct printerState
{
	// inputs
	uint8_t busy;
	uint8_t error;
	uint8_t pe;
	uint8_t select;
	uint8_t tabsize;

	// outputs
	uint8_t selectIn;
	autofeed_state autofeed;

	// settings
	ack_state useAck;
	bool addCR;
	bool addLF;

	// current state - for reporting via serial & display
	ack_state ackstate;
	printer_state state;
	printer_state prev_state;
	bool state_changed;
} printer;

#endif /* SMARTPARALLEL328PB_DEFINES_H_ */
