/*!
 * PIC18F47Q43/PIC18F47Q83/PIC18F47Q84 ROM image uploader and UART emulation firmware
 * This single source file contains all code
 *
 * Target: EMUZ80 with TD4
 * Compiler: MPLAB XC8 v2.41
 *
 * Modified by Satoshi Okue https://twitter.com/S_Okue
 * Version 0.1 2023/07/16
 */

/*
	PIC18F47Q43 ROM RAM and UART emulation firmware
	This single source file contains all code

	Target: EMUZ80 - The computer with only Z80 and PIC18F47Q43
	Compiler: MPLAB XC8 v2.36
	Written by Tetsuya Suzuki
*/

// CONFIG1
#pragma config FEXTOSC = OFF	// External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG2
#pragma config CLKOUTEN = OFF	// Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON		// PRLOCKED One-Way Set Enable bit (PRLOCKED bit can be cleared and set only once)
#pragma config CSWEN = ON		// Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON		// Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#ifndef _18F47Q43
#pragma config JTAGEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF
#endif

// CONFIG3
#pragma config MCLRE = EXTMCLR	// MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON		// Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON		// IVTLOCK bit One-way set enable bit (IVTLOCKED bit can be cleared and set only once)
#pragma config LPBOREN = OFF	// Low Power BOR Enable bit (Low-Power BOR disabled)
#pragma config BOREN = SBORDIS	// Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG4
#pragma config BORV = VBOR_1P9	// Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF		// ZCD Disable bit (ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF	// PPSLOCK bit One-Way Set Enable bit (PPSLOCKED bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON		// Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON			// Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)
#pragma config XINST = OFF		// Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG5
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF		// WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG6
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC		// WDT input clock selector (Software Control)

// CONFIG7
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF		// Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF		// Storage Area Flash enable bit (SAF disabled)
#ifdef _18F47Q43
#pragma config DEBUG = OFF		// Background Debugger (Background Debugger disabled)
#endif

// CONFIG8
#pragma config WRTB = OFF		// Boot Block Write Protection bit (Boot Block not Write protected)
#pragma config WRTC = OFF		// Configuration Register Write Protection bit (Configuration registers not Write protected)
#pragma config WRTD = OFF		// Data EEPROM Write Protection bit (Data EEPROM not Write protected)
#pragma config WRTSAF = OFF		// SAF Write protection bit (SAF not Write Protected)
#pragma config WRTAPP = OFF		// Application Block write protection bit (Application Block not write protected)

// CONFIG10
#pragma config CP = OFF		 	// PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

#ifndef _18F47Q43
// CONFIG9
#pragma config BOOTPINSEL = RC5	// CRC on boot output pin selection (CRC on boot output pin is RC5)
#pragma config BPEN = OFF		// CRC on boot output pin enable bit (CRC on boot output pin disabled)
#pragma config ODCON = OFF		// CRC on boot output pin open drain bit (Pin drives both high-going and low-going signals)

// CONFIG11
#pragma config BOOTSCEN = OFF	// CRC on boot scan enable for boot area (CRC on boot will not include the boot area of program memory in its calculation)
#pragma config BOOTCOE = HALT	// CRC on boot Continue on Error for boot areas bit (CRC on boot will stop device if error is detected in boot areas)
#pragma config APPSCEN = OFF	// CRC on boot application code scan enable (CRC on boot will not include the application area of program memory in its calculation)
#pragma config SAFSCEN = OFF	// CRC on boot SAF area scan enable (CRC on boot will not include the SAF area of program memory in its calculation)
#pragma config DATASCEN = OFF	// CRC on boot Data EEPROM scan enable (CRC on boot will not include data EEPROM in its calculation)
#pragma config CFGSCEN = OFF	// CRC on boot Config fuses scan enable (CRC on boot will not include the configuration fuses in its calculation)
#pragma config COE = HALT		// CRC on boot Continue on Error for non-boot areas bit (CRC on boot will stop device if error is detected in non-boot areas)
#pragma config BOOTPOR = OFF	// Boot on CRC Enable bit (CRC on boot will not run)

// CONFIG12
#pragma config BCRCPOLT = 0xFF	// Boot Sector Polynomial for CRC on boot bits 31-24 (Bits 31:24 of BCRCPOL are 0xFF)

// CONFIG13
#pragma config BCRCPOLU = 0xFF	// Boot Sector Polynomial for CRC on boot bits 23-16 (Bits 23:16 of BCRCPOL are 0xFF)

// CONFIG14
#pragma config BCRCPOLH = 0xFF	// Boot Sector Polynomial for CRC on boot bits 15-8 (Bits 15:8 of BCRCPOL are 0xFF)

// CONFIG15
#pragma config BCRCPOLL = 0xFF	// Boot Sector Polynomial for CRC on boot bits 7-0 (Bits 7:0 of BCRCPOL are 0xFF)

// CONFIG16
#pragma config BCRCSEEDT = 0xFF	// Boot Sector Seed for CRC on boot bits 31-24 (Bits 31:24 of BCRCSEED are 0xFF)

// CONFIG17
#pragma config BCRCSEEDU = 0xFF	// Boot Sector Seed for CRC on boot bits 23-16 (Bits 23:16 of BCRCSEED are 0xFF)

// CONFIG18
#pragma config BCRCSEEDH = 0xFF	// Boot Sector Seed for CRC on boot bits 15-8 (Bits 15:8 of BCRCSEED are 0xFF)

// CONFIG19
#pragma config BCRCSEEDL = 0xFF	// Boot Sector Seed for CRC on boot bits 7-0 (Bits 7:0 of BCRCSEED are 0xFF)

// CONFIG20
#pragma config BCRCEREST = 0xFF	// Boot Sector Expected Result for CRC on boot bits 31-24 (Bits 31:24 of BCRCERES are 0xFF)

// CONFIG21
#pragma config BCRCERESU = 0xFF	// Boot Sector Expected Result for CRC on boot bits 23-16 (Bits 23:16 of BCRCERES are 0xFF)

// CONFIG22
#pragma config BCRCERESH = 0xFF	// Boot Sector Expected Result for CRC on boot bits 15-8 (Bits 15:8 of BCRCERES are 0xFF)

// CONFIG23
#pragma config BCRCERESL = 0xFF	// Boot Sector Expected Result for CRC on boot bits 7-0 (Bits 7:0 of BCRCERES are 0xFF)

// CONFIG24
#pragma config CRCPOLT = 0xFF	// Non-Boot Sector Polynomial for CRC on boot bits 31-24 (Bits 31:24 of CRCPOL are 0xFF)

// CONFIG25
#pragma config CRCPOLU = 0xFF	// Non-Boot Sector Polynomial for CRC on boot bits 23-16 (Bits 23:16 of CRCPOL are 0xFF)

// CONFIG26
#pragma config CRCPOLH = 0xFF	// Non-Boot Sector Polynomial for CRC on boot bits 15-8 (Bits 15:8 of CRCPOL are 0xFF)

// CONFIG27
#pragma config CRCPOLL = 0xFF	// Non-Boot Sector Polynomial for CRC on boot bits 7-0 (Bits 7:0 of CRCPOL are 0xFF)

// CONFIG28
#pragma config CRCSEEDT = 0xFF	// Non-Boot Sector Seed for CRC on boot bits 31-24 (Bits 31:24 of CRCSEED are 0xFF)

// CONFIG29
#pragma config CRCSEEDU = 0xFF	// Non-Boot Sector Seed for CRC on boot bits 23-16 (Bits 23:16 of CRCSEED are 0xFF)

// CONFIG30
#pragma config CRCSEEDH = 0xFF	// Non-Boot Sector Seed for CRC on boot bits 15-8 (Bits 15:8 of CRCSEED are 0xFF)

// CONFIG31
#pragma config CRCSEEDL = 0xFF	// Non-Boot Sector Seed for CRC on boot bits 7-0 (Bits 7:0 of CRCSEED are 0xFF)

// CONFIG32
#pragma config CRCEREST = 0xFF	// Non-Boot Sector Expected Result for CRC on boot bits 31-24 (Bits 31:24 of CRCERES are 0xFF)

// CONFIG33
#pragma config CRCERESU = 0xFF	// Non-Boot Sector Expected Result for CRC on boot bits 23-16 (Bits 23:16 of CRCERES are 0xFF)

// CONFIG34
#pragma config CRCERESH = 0xFF	// Non-Boot Sector Expected Result for CRC on boot bits 15-8 (Bits 15:8 of CRCERES are 0xFF)

// CONFIG35
#pragma config CRCERESL = 0xFF	// Non-Boot Sector Expected Result for CRC on boot bits 7-0 (Bits 7:0 of CRCERES are 0xFF)
#endif
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>

#define TD4_CLK 15UL 		// TD4 clock frequency
#define ROM_SIZE 0x0010		// 16 bytes

#define _XTAL_FREQ 64000000UL

#define CLK			RA5
#define CLK_OUT		LATA5
#define CLK_SOURCE	RA5PPS
#define CLR_OUT		LATE0
#define CARRY		CLC1OUT
#define	D4			RB4
#define	D5			RB5
#define	D6			RB6
#define	D7			RB7
#define	LOAD0		LATC0
#define	LOAD1		LATC1
#define	LOAD2		LATC2
#define	LOAD3		LATC3
#define M_CLK		RE1
#define M_CLK_SW	RA4
#define	ROM			RE2


//Z80 ROM equivalent, see end of this file
extern const unsigned char rom_data[];

// UART3 Transmit
void putch(char c) {
	while(!U3TXIF); // Wait or Tx interrupt flag set
	U3TXB = c; // Write data
}

/*
// UART3 Recive
int getch(void) {
	while(!U3RXIF); // Wait for Rx interrupt flag set
	return U3RXB; // Read data
}
*/

// Never called, logically
void __interrupt(irq(default),base(8)) Default_ISR(){}

// main routine
void main(void) {

	// System initialize
	OSCFRQ = 0x08; // 64MHz internal OSC

	// /CLR (RE0) output pin
	ANSELE0 = 0;	// Disable analog function
	LATE0 = 0;		// /CLR
	TRISE0 = 0;		// Set as output

	// ROM (RE2) input pin
	ANSELE2 = 0;	// Disable analog function
	WPUE2 = 1;		// Week pull up
	TRISE2 = 1;		// Set as input

	// Data bus D7-D0 pin
	ANSELB = 0x00;	// Disable analog function
	WPUB = 0xff;	// Week pull up
	TRISB = 0xff;	// Set as input

	// S3 (RD3) input pin
	ANSELD3 = 0;	// Disable analog function
	WPUD3 = 1;		// Week pull up
	TRISD3 = 1;		// Set as input

	// S2 (RD2) input pin
	ANSELD2 = 0;	// Disable analog function
	WPUD2 = 1;		// Week pull up
	TRISD2 = 1;		// Set as input

	// S1 (RD1) input pin
	ANSELD1 = 0;	// Disable analog function
	WPUD1 = 1;		// Week pull up
	TRISD1 = 1;		// Set as input

	// S0 (RD0) input pin
	ANSELD0 = 0;	// Disable analog function
	WPUD0 = 1;		// Week pull up
	TRISD0 = 1;		// Set as input

	// CARRY (RD7) input pin
	ANSELD7 = 0;	// Disable analog function
	WPUD7 = 1;		// Week pull up
	TRISD7 = 1;		// Set as input

	// Za1 (RC4) input pin
	ANSELC4 = 0;	// Disable analog function
	WPUC4 = 1;		// Week pull up
	TRISC4 = 1;		// Set as input

	// Za2 (RC6) input pin
	ANSELC6 = 0;	// Disable analog function
	WPUC6 = 1;		// Week pull up
	TRISC6 = 1;		// Set as input

	// Zb1 (RC5) input pin
	ANSELC5 = 0;	// Disable analog function
	WPUC5 = 1;		// Week pull up
	TRISC5 = 1;		// Set as input

	// Zb2 (RC7) input pin
	ANSELC5 = 0;	// Disable analog function
	WPUC5 = 1;		// Week pull up
	TRISC5 = 1;		// Set as input

	// A3 (RA3) input pin
	ANSELA3 = 0;	// Disable analog function
	WPUA3 = 1;		// Week pull up
	TRISA3 = 1;		// Set as input

	// A2 (RA2) input pin
	ANSELA2 = 0;	// Disable analog function
	WPUA2 = 1;		// Week pull up
	TRISA2 = 1;		// Set as input

	// A1 (RA1) input pin
	ANSELA1 = 0;	// Disable analog function
	WPUA1 = 1;		// Week pull up
	TRISA1 = 1;		// Set as input

	// A0 (RA0) input pin
	ANSELA0 = 0;	// Disable analog function
	WPUA0 = 1;		// Week pull up
	TRISA0 = 1;		// Set as input

	// M_CLK (RE1) output pin
	ANSELE1 = 0;	// Disable analog function
	WPUE1 = 1;		// Week pull up
	TRISE1 = 1;		// Set as input

	// M_CLK_SW (RA4) input pin
	ANSELA4 = 0;	// Disable analog function
	WPUA4 = 1;		// Week pull up
	TRISA4 = 1;		// Set as input

	// SEL1 (RD5) output pin
	ANSELD5 = 0;	// Disable analog function
	LATD5 = 0;		// SEL1=0
	TRISD5 = 0;		// Set as output

	// SEL0 (RD4) output pin
	ANSELD4 = 0;	// Disable analog function
	LATD4 = 0;		// SEL0=0
	TRISD4 = 0;		// Set as output

	// /LOAD3 (RC3) output pin
	ANSELC3 = 0;	// Disable analog function
	LATC3 = 1;		// LOAD3=1
	TRISC3 = 0;		// Set as output

	// /LOAD2 (RC2) output pin
	ANSELC2 = 0;	// Disable analog function
	LATC2 = 1;		// LOAD2
	TRISC2 = 0;		// Set as output

	// /LOAD1 (RC1) output pin
	ANSELC1 = 0;	// Disable analog function
	LATC1 = 1;		// LOAD1
	TRISC1 = 0;		// Set as output

	// /LOAD0 (RC0) output pin
	ANSELC0 = 0;	// Disable analog function
	LATC0 = 1;		// LOAD0
	TRISC0 = 0;		// Set as output

	// TD4 clock(RA5) by NCO FDC mode
	RA5PPS = 0x00;	// RA5 asined LATA5
	ANSELA5 = 0;	// Disable analog function
	TRISA5 = 0;		// NCO output pin
	NCO1INC = TD4_CLK * 67;
	NCO1CLK = 0x04;	// Clock source 31.25kHz
	NCO1PFM = 0; 	// FDC mode
	NCO1OUT = 1; 	// NCO output enable
	NCO1EN = 1;		// NCO enable

	// UART3 initialize
	U3BRG = 416;	// 9600bps @ 64MHz
	U3RXEN = 1;		// Receiver enable
	U3TXEN = 1;		// Transmitter enable

	// UART3 Receiver
	ANSELA7 = 0;	// Disable analog function
	TRISA7 = 1;		// RX set as input
	U3RXPPS = 0x07;	//RA7->UART3:RX3;

	// UART3 Transmitter
	ANSELA6 = 0;	// Disable analog function
	LATA6 = 1;		// Default level
	TRISA6 = 0;		// TX set as output
	RA6PPS = 0x26;	//RA6->UART3:TX3;

	U3ON = 1;		// Serial port enable


	//========== CLC input pin assign ===========
	// 0,1,4,5 = Port A, C
	// 2,3,6,7 = Port B, D
	CLCIN0PPS = 0x05;	// RA5 <- CLOCK
	CLCIN2PPS = 0x1f;	// RD7 <- CARRY
	CLCIN3PPS = 0x0c;	// RB4 <- D4
	CLCIN6PPS = 0x0d;	// RB5 <- D5
	CLCIN7PPS = 0x0f;	// RB7 <- D7

	//========== CLC3 SEL0 ==========
	CLCSELECT = 2;		// CLC3 select
	CLCnCON = 0x00;		// Disable CLC

	CLCnSEL0 = 3;		// CLCIN3PPS (D4)
	CLCnSEL1 = 127;		// NC
	CLCnSEL2 = 7;		// CLCIN7PPS (D7)
	CLCnSEL3 = 127;		// NC

	CLCnGLS0 = 0x02;	// noninverted D4
	CLCnGLS1 = 0x04;	// 1(0 invert) for AND gate
	CLCnGLS2 = 0x20;	// noninverted D7
	CLCnGLS3 = 0x40;	// 1(0 invert) for AND gate

	CLCnPOL = 0x00;		// noninverted the CLC3 output
	CLCnCON = 0x80;		// AND-OR

	//========== CLC4 SEL1 ==========
	CLCSELECT = 3;		// CLC4 select
	CLCnCON = 0x00;		// Disable CLC

	CLCnSEL0 = 6;		// CLCIN6PPS (D5)
	CLCnSEL1 = 127;		// NC
	CLCnSEL2 = 127;		// NC
	CLCnSEL3 = 127;		// NC

	CLCnGLS0 = 0x02;	// noninverted D5
	CLCnGLS1 = 0x04;	// 1(0 invert) for AND gate
	CLCnGLS2 = 0x10;	// 1(0 invert) for AND gate
	CLCnGLS3 = 0x40;	// 1(0 invert) for AND gate

	CLCnPOL = 0x00;		// noninverted the CLC3 output
	CLCnCON = 0x82;		// 4 input AND

	//========== CLC1 CARRY Flag ==========
	CLCSELECT = 0;		// CLC1 select

	CLCnSEL0 = 0;		// D-FF CLK <- CLOCK
	CLCnSEL1 = 2;		// D-FF D <- CARRY
	CLCnSEL2 = 127;		// D-FF S NC
	CLCnSEL3 = 127;		// D-FF R NC

	CLCnGLS0 = 0x2;		// G1D1T
	CLCnGLS1 = 0x8;		// G2D1T
	CLCnGLS2 = 0x0;		// Connect none
	CLCnGLS3 = 0x0;		// Connect none

	CLCnPOL = 0x80;		// inverted the CLC1 output
	CLCnCON = 0x84;		// Select D-FF

	CLCDATA = 0x0;		// Clear all CLC outs

	//========== CLC output pin assign ===========
	// 1,2,5,6 = Port A, C
	// 3,4,7,8 = Port B, D
	RD4PPS = 0x03;		// CLC3OUT -> RD4 -> SEL0
	RD5PPS = 0x04;		// CLC4OUT -> RD5 -> SEL1

    printf("\r\nMEZTD4 %3uHz\r\n",(unsigned int)NCO1INC / 67);

	// Unlock IVT
	IVTLOCK = 0x55;
	IVTLOCK = 0xAA;
	IVTLOCKbits.IVTLOCKED = 0x00;

	// Default IVT base address
	IVTBASE = 0x000008;

	// Lock IVT
	IVTLOCK = 0x55;
	IVTLOCK = 0xAA;
	IVTLOCKbits.IVTLOCKED = 0x01;

	// TD4 start
	GIE = 1;			// Global interrupt enable

	if (ROM) {			// ROM Board not found
		TRISB = 0x00; 	// Set as output
		LATB = rom_data[0];
	}

	__delay_ms(500);	// Dummy wait

	if (M_CLK) {
		CLK_SOURCE = 0x3f;	// NCO1
		while(CLK);			// Detect rising edge of clock
	} else {
		CLK_SOURCE = 0x00;	// LATA5
		CLK_OUT = 0;
	}
	CLR_OUT = 1;			// /CLR=1

	while(1){
		if (M_CLK) {
			CLK_SOURCE = 0x3f;		// NCO1
			while(CLK);
		} else {
			CLK_OUT = 0;
			CLK_SOURCE = 0x00;		// LATA5
		}

		if (ROM) {			// Detect ROM Board
			LATB = rom_data[PORTA & 0x0F];
		}

		LOAD0 = D6 || D7;
		LOAD1 = !D6 || D7;
		LOAD2 = !(!D6 && D7);
		LOAD3 = !((D4 || CARRY) && D6 && D7);

		if (M_CLK) {
			CLK_SOURCE = 0x3f;		// NCO1
			while(!CLK);
		} else {
			CLK_SOURCE = 0x00;		// LATA5
			while (M_CLK_SW);		// Detect MANUAL CLOCK Switch ON
			__delay_ms(20);
			CLK_OUT = 1;
			while(!M_CLK_SW);		// Detect MANUAL CLOCK Switch OFF
			__delay_ms(20);
			while(!NCO1OUT);
			while(NCO1OUT);
			CLK_OUT = 0;
		}
	}
}

const unsigned char rom_data[ROM_SIZE] = {
// Ramen timer (LOOP)
	0b10110111,		// 0x00		OUT 0b0111
	0b00000001,		// 0x01		ADD A,0b0001
	0b11100001,		// 0x02		JNC 0x01
	0b01010001,		// 0x03		ADD B,0b0001
	0b11100011,		// 0x04		JNC 0x03
	0b10110110,		// 0x05		OUT 0b0110
	0b00000001,		// 0x06		ADD A,0b0001
	0b11100110,		// 0x07		JNC 0x06
	0b01010001,		// 0x08		ADD B,0b0001
	0b11101000,		// 0x09		JNC 0x08
	0b10110000,		// 0x0A		OUT 0b0000
	0b10110100,		// 0x0B		OUT 0b0100
	0b00000001,		// 0x0C		ADD A,0b0001
	0b11101010,		// 0x0D		JNC 0x0A
	0b10111000,		// 0x0E		OUT 0b1000
	0b11110000		// 0x0F		JMP 0x00
};

// ADD A,Im		0b0000
// MOV A,B		0b0001
// IN  A		0b0010
// MOV A,Im		0b0011
// MOV B,A		0b0100
// ADD B,Im		0b0101
// IN  B		0b0110
// MOV B,Im		0b0111
//				0b1000
// OUT B		0b1001
//				0b1010
// OUT Im		0b1011
//				0b1100
//				0b1101
// JNC Im		0b1110
// JMP Im		0b1111
