
This project allows the use of PS/2 keyboard and mouse with SUN stations fitted with proprietary 8 pins mini-DIN connector.

It is based on a PIC24FJ48GA002 microcontroller, an absolute overkill, and quite inappropriate as it cannot run under 5V power input - but I got this part in stock. I used a 74HC00 NAND gates to invert serial link signals (SUN serial protocol has negative logic) as well as 3.3V to 5V level shifter. PIC's inputs are 5V tolerant.

Resource usage in the microcontroller:
 * 2 externals interrupts
 * 1 internal timer with interrupt
 * 2 UARTs
 * a very few bytes of RAM and ~1k instructions

features and limitations:
 * keyboard layout is configured at compilation (see SUN_LAYOUT #define in main.c)
 * keyboard LED driving is not implemented
 * F12 key is mapped to SUN STOP key - handy to enter openboot prompt with STOP-A
