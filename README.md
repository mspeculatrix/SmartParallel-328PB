# SmartParallel-328PB
Atmel ATMEGA328PB code for SmartParallel project.

https://mansfield-devine.com/speculatrix/category/projects/smartparallel/

This is a work in progress.

It requires a number of [my libraries](https://github.com/mspeculatrix/avr-lib), so you'll need to ensure you set these up appropriately. 

The main function of this code is to sit and wait for incoming messages on the serial port. It also sends updates and status messages as appropriate via serial to the host.

Incoming messages are assumed to be one of two types:

* A command – to define a setting on the SmartParallel, ask for information about printer status etc.
* A line of text to print.

Either way, the message must be terminated with a null character (ASCII 0). If bytes are received on the serial port but no null terminator arrives within a given time, the program times out and dumps the characters received. Also, if the incoming buffer fills without having received a null byte, the final char is replaced with the null byte and we go with what we've got.

Commands are sent by making the first byte ASCII 1. The next byte is the command. The details of the commands are (or will be) on the GitHub page.

Any incoming bytes where the first one isn't a 1 are assumed to be text to print.
