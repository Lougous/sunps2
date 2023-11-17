
// PIC24FJ48GA002 Configuration Bit Settings

// 'C' source line config statements

// CONFIG2
#pragma config POSCMOD = NONE           // Primary Oscillator Select (Primary oscillator disabled)
#pragma config I2C1SEL = PRI            // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = ON             // IOLOCK Protection (Once IOLOCK is set, cannot be changed)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSC2/CLKO/RC15 functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Clock switching and Fail-Safe Clock Monitor are disabled)
#pragma config FNOSC = FRC              // Oscillator Select (Fast RC Oscillator (FRC))
#pragma config SOSCSEL = SOSC           // Sec Oscillator Select (Default Secondary Oscillator (SOSC))
#pragma config WUTSEL = LEG             // Wake-up timer Select (Legacy Wake-up Timer)
#pragma config IESO = ON                // Internal External Switch Over Mode (IESO mode (Two-Speed Start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is enabled)
#pragma config ICS = PGx1               // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include "keymap.h"

//#define SUN_LAYOUT  0x21  // USA type 5
#define SUN_LAYOUT  0x23  // french type 5


#define FREQ       8000000UL  // 8 MHz (default configuration)
#define PS2_BAUDS  12000UL    // 10 to 16.6 kHz

volatile unsigned short kbd_dat;
volatile unsigned short kbd_sin;

// interrupt handler for external interrupt #1
void __attribute__((__interrupt__, auto_psv)) _INT1Interrupt(void)
{
	if (IFS1bits.INT1IF == 1) {
		// interrupt 1: keyboard
		kbd_sin = (((unsigned short)PORTBbits.RB13) << 15) | (kbd_sin >> 1);

        if (!(kbd_sin & 32)) {
			kbd_dat = kbd_sin;
			kbd_sin = 0xffff;
		}

		// clear interrupt
		IFS1bits.INT1IF = 0;
	}
}

volatile unsigned short ms_dat;
volatile unsigned short ms_sin;
volatile unsigned short ms_recv_mode;

// interrupt handler for external interrupt #2
void __attribute__((__interrupt__, auto_psv)) _INT2Interrupt(void)
{
	if (IFS1bits.INT2IF == 1) {
		// interrupt 2: mouse
		if (ms_recv_mode == 1) {
			// mouse => microcontroller
			ms_sin = (((unsigned short)PORTBbits.RB9) << 15) | (ms_sin >> 1);

     	    if (!(ms_sin & 32)) {
				ms_dat = ms_sin;
				ms_sin = 0xffff;
			}

		} else {
			// microcontroller => mouse
			LATBbits.LATB9 = ms_sin & 1;
			ms_sin = 0x8000 | (ms_sin >> 1);

			if (ms_sin == 0xfffe) {
				// start of STOP bit
                // data just driven to 1 for STOP bit, can now be left open
                // as it will pulled up by external resistor. the ACK bit is
                // driven by the mouse
                TRISBbits.TRISB9 = 1;  // data = input
            } else if (ms_sin == 0xffff) {
                // last clock low pulse in the transmission: ACK bit
                // ACK value could be checked here ...
                // back to receive mode
				ms_recv_mode = 1;
			}
		}

		// clear interrupt
		IFS1bits.INT2IF = 0;
	}
}

// interrupt handler for timer 1 (100 us pulse on mouse PS/2 clock))
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    if (IFS0bits.T1IF) {
        // drive data low (start bit)
        LATBbits.LATB9   = 0;
        TRISBbits.TRISB9 = 0;  // data = output

        // clear interrupt pending due to rts
        IFS1bits.INT2IF = 0;
    
        // release the clock
        TRISBbits.TRISB8 = 1;  // clock = input

        // enable mouse PS/2 clock interrupt
        IEC1bits.INT2IE = 1;

        // disable timer 1
        T1CON = 0x0000;
    
        // disable timer 1 interrupt
        IEC0bits.T1IE = 0;
    
        // clear timer 1 interrupt
        IFS0bits.T1IF = 0;
    }
}

// send as keyboard
void uart1_send(unsigned short txc) {
	while (U1STAbits.UTXBF == 1);

	U1TXREG = txc;
}

// send command to mouse
// bit 8 is odd parity of bits 7-0
void mouse_send(unsigned short c) {
    // disable mouse PS/2 clock interrupt, so that it won't trig when driving
    // clock low
	IEC1bits.INT2IE = 0;
    
    // prepare shift register with data to send
	ms_recv_mode = 0;
	ms_sin = (c & 0x1ff) | 0xfa00;
	
	// drive clock low (rts)
	TRISBbits.TRISB8 = 0;  // clock = output

    // prepare timer to trig an interrupt in 100 us
    PR1   = 100*(unsigned short)(FREQ/2000000UL);
    TMR1  = 0;
    IEC0bits.T1IE = 1;
	T1CON = 0x8000;  // enable, Internal clock (FOSC/2)

//
//	TRISBbits.TRISB9   = 1;  // RB9 (pin 18) is input (PS/2 data)
}

void mouse_decode(unsigned short ms_byte1,
				  unsigned short ms_byte2,
				  unsigned short ms_byte3) {
	signed short sdx, sdy;
	unsigned short dx, dy;

	// last char not fully send, abort
	if (U2STAbits.UTXBF == 1) return;

	// first char
    //     d7    d6     d5     d4     d3     d2     d1     d0
    //     1     0      0      0      0      lb     mb     rb
	//  lb (mb, rb) are cleared when the left (middle, right) button is
    //  pressed
	U2TXREG = 0x80 | ((((ms_byte1 & 1) << 2) | ((ms_byte1 & 6) >> 1)) ^ 0x07);

	// second char
    //     d7    d6     d5     d4     d3     d2     d1     d0
    //      0    dxa6   dxa5   dxa4   dxa3   dxa2   dxa1   dxa0
	// compute dx
	if (ms_byte1 & 0x40) {
		// X overflow
		if (ms_byte1 & 0x10) {
			// negative
			dx = 0x80;   // -128
		} else {
			// positive
			dx = 0x7F;   // 127
		}
	} else {
		if (ms_byte1 & 0x10) {
			// negative
			sdx = 0xff00 | ms_byte2;

			if (sdx < -128) {
				// overflow
				dx = 0x80;   // -128
			} else {
				dx = ms_byte2 & 0xff;
			}
		} else {
			// positive
			if (ms_byte2 & 0xff80) {
				// overflow
				dx = 0x7F;   // 127
			} else {
				dx = ms_byte2;
			}
		}
	}
	
	while (U2STAbits.UTXBF == 1);
	U2TXREG = dx;

	// third char
    //     d7    d6     d5     d4     d3     d2     d1     d0
    //      0    dya6   dya5   dya4   dya3   dya2   dya1   dya0
	// compute dy
	if (ms_byte1 & 0x80) {
		// X overflow
		if (ms_byte1 & 0x20) {
			// negative
			dx = 0x7F;   // 127
		} else {
			// positive
			dx = 0x80;   // -128
		}
	} else {
		if (ms_byte1 & 0x20) {
			// negative
			sdy = (0xff00 | ms_byte3);

			if (sdy < -128) {
				// overflow
				dy = 0x80;   // -128
			} else {
				dy = sdy & 0xff;
			}
		} else {
			// positive
			sdy = (0x00ff & ms_byte3);

			if (sdy & 0xff80) {
				// overflow
				dy = 0x7F;   // 127
			} else {
				dy = sdy;
			}
		}
	}
	
	while (U2STAbits.UTXBF == 1);
	U2TXREG = dy;

    // forth and fifth
	while (U2STAbits.UTXBF == 1);
	U2TXREG = 0;

	while (U2STAbits.UTXBF == 1);
	U2TXREG = 0;
}

int main (int argc, char *argv []) {
    unsigned long led_hit = 0;
    unsigned long mouse_timeout = 1000000;
	int break_mode = 0;
	int ext_mode = 0;
	unsigned short ms_byte1, ms_byte2, ms_byte3;
	typedef enum {	MS_SELF_TEST, MS_ERROR, MS_MOUSE_ID,
					MS_ACK_SAMPLE_SET, MS_ACK_SAMPLE_RATE, MS_ACK,
					MS_BYTE1, MS_BYTE2, MS_BYTE3 } ms_state_type;
	ms_state_type ms_state = MS_SELF_TEST;
    char kbd_led_mode = 0;

    ///////////////////////////////////////////////////////////////////////////
    // onchip & IO configuration
    ///////////////////////////////////////////////////////////////////////////
	// oscillator config
//    OSCCONbits.SOSCEN = 0;
	OSCCONbits.NOSC = 0; // Fast RC Oscillator

	// unlock io mapping
	// 1. Write 46h to OSCCON<7:0>.
	// 2. Write 57h to OSCCON<7:0>.
	// 3. Clear (or set) IOLOCK as a single operation

	// pinout mapping
	// disable all analog inputs
	AD1PCFG = 0xffff;

	// disable parallel master port
	PMCON = 0x0080;

	// LED: RB3 (pin 7)
	TRISBbits.TRISB3 = 0;   // output
	PORTBbits.RB3    = 0;   // GND = LED on

	// keyboard communication (bidir) with SUN
	// UART1 configuration
	RPINR18bits.U1RXR = 7;  // RX => RP7 (pin 16)
	RPOR3bits.RP6R    = 3;  // TX => RP6 (pin 15)
	U1BRG  = (((FREQ/2)/1200UL)/16) - 1;   // 1200 bauds
	U1MODE = 0x8000;                   // enable UART1
	U1STA  = 0x0400;                   // enable TX

	// mouse communication (TX only) with SUN
	// UART2 configuration
	RPINR19bits.U2RXR = 10;  // RX => RP10 (pin 21), not used !
	RPOR5bits.RP11R   = 5;   // TX => RP11 (pin 22)
	U2BRG  = (((FREQ/2)/1200UL)/16) - 1;   // 1200 bauds
	U2MODE = 0x8000;                   // enable UART2
	U2STA  = 0x0400;                   // enable TX

    // PS/2 keyboard capture
	// external interrupt #1 configuration (PS/2 clock)
	kbd_dat = 0xffff;
	kbd_sin = 0xffff;

    RPINR0bits.INT1R   = 12; // RP12 (pin 23)
	INTCON2bits.INT1EP = 1;  // interrupt on negative edge
	IPC5bits.INT1IP    = 1;  // priority 1
	IEC1bits.INT1IE    = 1;  // interrupt enabled

	TRISBbits.TRISB13  = 1;  // RB13 (pin 24) is input (PS/2 data)

    // PS/2 mouse capture
	// external interrupt #2 configuration (PS/2 clock)
	ms_dat = 0xffff;
	ms_sin = 0xffff;
    ms_recv_mode = 1;

    RPINR1bits.INT2R   = 8;  // RP8 (pin 17)
	INTCON2bits.INT2EP = 1;  // interrupt on negative edge
	IPC7bits.INT2IP    = 2;  // priority 2
	IEC1bits.INT2IE    = 1;  // interrupt enabled
	LATBbits.LATB8     = 0;  // always driven low (open-drain)

	TRISBbits.TRISB9   = 1;  // RB9 (pin 18) is input (PS/2 data)

    ///////////////////////////////////////////////////////////////////////////
    // Main loop
    ///////////////////////////////////////////////////////////////////////////
	while (1) {
		if (U1STAbits.URXDA) {
			// word received from SUN as keyboard command
			unsigned short rxc = U1RXREG;

			// turn LED off for a while
			PORTBbits.RB3 = 1;     // VCC = LED off
			led_hit       = 100000;

            if (kbd_led_mode) {
                kbd_led_mode = 0;
                
                // TODO
            } else {
                switch (rxc) {
                case 0x01:
                    // reset command
                    uart1_send(0xff);
                    uart1_send(0x04);
                    uart1_send(0x7f);
                    break;

                case 0x02:  // bell ON
                case 0x03:  // bell OFF
                case 0x0A:  // click ON
                case 0x0B:  // click OFF
                    // ignore
                    break;

                case 0x0E:  // LED
                    kbd_led_mode = 1;
                    break;

                case 0x0F:
                    // layout command
                    uart1_send(0xfe);
                    uart1_send(SUN_LAYOUT);
                    break;

                default:
                    led_hit = 1000000;
                    break;
                }
            }
		}

        if (!(kbd_dat & 32)) {
			// 11 bits read from PS/2 keyboard
			// ignore parity and stop bit
			unsigned short rxc = (kbd_dat >> 6) & 255;

			// reset receiving buffer
			kbd_dat = 0xffff;

			// turn LED off for a while
			PORTBbits.RB3 = 1;     // VCC = LED off
			led_hit       = 5000;
            unsigned char to_send = 0;
            
			if (break_mode) {
				if (ext_mode) {
					if (ps2_ext_to_sun_tbl[rxc & 0x7f]) {
						to_send = 0x80 | ps2_ext_to_sun_tbl[rxc & 0x7f];
					}
					
					break_mode = 0;
					ext_mode = 0;
				} else {
					if (ps2_to_sun_tbl[rxc & 0x7f]) {
						to_send = 0x80 | ps2_to_sun_tbl[rxc & 0x7f];
					}
				}

				break_mode = 0;
				ext_mode = 0;
		 	} else if (ext_mode) {
				if (rxc == 0xF0) {
					break_mode = 1;
				} else {
					if (ps2_ext_to_sun_tbl[rxc & 0x7f]) {
						to_send = ps2_ext_to_sun_tbl[rxc & 0x7f];
					}
					
					break_mode = 0;
					ext_mode = 0;
				}
			} else {
				if (rxc == 0xE0) {
					ext_mode = 1;
				} else if (rxc == 0xF0) {
					break_mode = 1;
				} else {
					if (ps2_to_sun_tbl[rxc & 0x7f]) {
						to_send = ps2_to_sun_tbl[rxc & 0x7f];
					}
				}
			}
            
            if (to_send) {
                uart1_send(to_send);
            }
		}

		// mouse handler
        if (!(ms_dat & 32)) {
            
            mouse_timeout = 1000000;
            
			// 11 bits read from PS/2 mouse
			// ignore parity and stop bit
			unsigned short rxc = (ms_dat >> 6) & 255;

			// reset receiving buffer
			ms_dat = 0xffff;

			switch (ms_state) {
			case MS_SELF_TEST:
			case MS_ERROR:
				if (rxc == 0xAA) {
					ms_state = MS_MOUSE_ID;
				} else {
					ms_state = MS_ERROR;
				}
				break;

			case MS_MOUSE_ID:
				// mouse ID received
				// send "enable device"
				mouse_send(0x1F3);
				ms_state = MS_ACK_SAMPLE_SET;
				break;

			case MS_ACK_SAMPLE_SET:
				if (rxc == 0xFA) {
					mouse_send(0x114);   // 20 samples/s
					ms_state = MS_ACK_SAMPLE_RATE;
				} else {
					ms_state = MS_ERROR;
				}
				break;

			case MS_ACK_SAMPLE_RATE:
				if (rxc == 0xFA) {
					mouse_send(0x0F4);   // Enable data reporting
					ms_state = MS_ACK;
				} else {
					ms_state = MS_ERROR;
				}
				break;

			case MS_ACK:
				if (rxc == 0xFA) {
					ms_state = MS_BYTE1;
				} else {
					ms_state = MS_ERROR;
				}
				break;

			case MS_BYTE1:
				ms_byte1 = rxc;
				ms_state = MS_BYTE2;
				break;

			case MS_BYTE2:
				ms_byte2 = rxc;
				ms_state = MS_BYTE3;
				break;

			case MS_BYTE3:
				ms_byte3 = rxc;
				ms_state = MS_BYTE1;

				mouse_decode(ms_byte1, ms_byte2, ms_byte3);
				break;

			default:
				break;
			}
		}

		if (led_hit == 0) {
			PORTBbits.RB3    = 0;  // GND = LED on
		} else {
			led_hit--;
		}
        
        if (mouse_timeout == 0) {
            mouse_timeout = 200000;
            
            ms_state = MS_SELF_TEST;
            mouse_send(0x1FF);   // self test
        } else {
            mouse_timeout--;
        }

		// mouse error: half bright
		if (ms_state == MS_ERROR && led_hit == 0) {
			led_hit = 1;
		}
	}

	return 0;
}
