/*
 * SmartParallel328PB_includes.h
 *
 * Created: 6/10/2019 11:14:19 AM
 *  Author: Steve
 */ 


#ifndef SMARTPARALLEL328PB_DEFINES_H_
#define SMARTPARALLEL328PB_DEFINES_H_

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

// ----- PRINTER OUTPUTS ----------------------------------
#define STROBE_PIN		PC0
#define INIT_PIN		PC1
#define AUTOFEED_PIN	PC2
#define SELIN_PIN		PC3		// probably not using
#define OUTPUT_PORT		PORTC
#define OUTPUT_DDR		DDRC

// Output shift register
#define SHCP_PIN		PB2		// clock pin
#define STCP_PIN		PB1		// latch pin
#define DATA_PIN		PB0		// data pin
#define SHIFTREG_DDR	DDRB
#define SHIFTREG_REG	PORTB

// ----- PRINTER INPUTS -----------------------------------
// Input pins that trigger interrupts
#define ACK_PIN			PD2
#define ERR_PIN			PD3
#define ERR_REG			PIND
#define INPUT_INT_DDR	DDRD
// Other input pins
#define BUSY_PIN		PE3
#define PE_PIN			PE1
#define SELECT_PIN		PE2
#define INPUT_REG		PINE
#define INPUT_DDR		DDRE

// ----- Serial Port ------------------------------
#define CTS_PIN			PD4
#define SERIAL_REG		PORTD
#define SERIAL_DDR		DDRD
// most of the serial port is on Port D, but RTS is on Port E.
// That said, we probably don't need it.
//#define RTS_PIN			PE0
//#define RTS_REG			PINE
//#define RTS_DDR			DDRE

// ----- LEDS -------------------------------------
#define STAT_LED1_PIN	PD5
#define STAT_LED2_PIN	PD6
#define STAT_LED3_PIN	PD7
#define LED_REG			PORTD
#define LED_DDR			DDRD


#endif /* SMARTPARALLEL328PB_DEFINES_H_ */