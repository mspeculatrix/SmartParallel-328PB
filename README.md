# SmartParallel-328PB
Atmel ATMEGA328PB code for SmartParallel project.

https://mansfield-devine.com/speculatrix/category/projects/smartparallel/

This is a work in progress.

It requires a number of [my libraries](https://github.com/mspeculatrix/avr-lib), so you'll need to ensure you set these up appropriately. 

The main function of this code is to sit and wait for incoming messages on the serial port. It also sends updates and status messages as appropriate via serial to the host.

Messages sent from the SP board have one of three prefixes:
* SER_ for messages relating to the serial port/connection.
* PRT_ relating to printer functions/actions.
* PIF_ relating to printer interface settings or status.

Incoming messages are assumed to be one of two types:

* A command – to define a setting on the SmartParallel, ask for information about printer status etc.
* A line of text to print.

Either way, the message must be terminated with a null character (ASCII 0). If bytes are received on the serial port but no null terminator arrives within a given time, the program times out and dumps the characters received. Also, if the incoming buffer fills without having received a null byte, the final char is replaced with the null byte and we go with what we've got.

Commands are sent by making the first byte ASCII 1. Any incoming bytes where the first one isn't a 1 are assumed to be text to print.

### Commands

Commands are sent by making the first byte ASCII 1. The next byte is the command.

Cmd byte 	Function

(Dec)

1 > PING - prompts the SP to send back the string SER_PONG.

2			DISABLE ACK - tell SP not to use the ACK signal when talking to printer. Responds with message: PIF_ACK_DISABLED

3 			ENABLE ACK - tell SP to use the ACK signal when taling to printer. Responds with message: PIF_ACK_ENABLED

4			AUTOFEED DISABLE - turn off Autofeed on the printer. Responds with message: PIF_AF_DISABLED

5			AUTOFEED ENABLE - turn on Autofeed on the printer. Responds with message: PIF_AF_DISABLED

8			PRINT MODE NORMAL - set standard 80-column mode

9			PRINT MODE CONDENSED - set condesned 132-column mode

10			PRINT MODE DOUBLE - set enlarged 40-column mode

16			LINE END NORMAL - don't add anything to the lines of text received

17			LINE END LF - add linefeed (ASCII 10) to lines received

18			LINE FEED CR - add carriage return (ASCII 13) to lines received

19			LINE FEED CR-LF - add linefeed & carriage return to lines received

32 			REPORT STATE - request status report on printer

33 			REPORT ACK - request state of ACK/NO ACK setting. Responds with message: PIF_ACK_ENABLED or PIF_ACK_DISABLED

34 			REPORT AF - request state of Autofeed setting. Responds with message: PIF_AF_ENABLED or PIF_AF_DISABLED

