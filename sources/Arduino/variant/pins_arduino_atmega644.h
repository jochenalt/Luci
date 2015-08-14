/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_AtMega644_h
#define Pins_Arduino_AtMega644_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS			32
#define NUM_ANALOG_INPUTS           8

#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 14 : -1)

// PWM pins are  PD4, PD5, PD6, PD7, PB3, PB4,
#define digitalPinHasPWM(p)         ((p) == 4 || (p) == 5 || (p) == 6 || (p) == 7 || (p) == 11 || (p) == 12))

static const uint8_t SS   = 12;
static const uint8_t MOSI = 13;
static const uint8_t MISO = 20;
static const uint8_t SCK  = 21;


static const uint8_t SDA = 15;
static const uint8_t SCL = 14;

static const uint8_t PIN_D0 = 0;
static const uint8_t PIN_D1 = 1;
static const uint8_t PIN_D2 = 2;
static const uint8_t PIN_D3 = 3;
static const uint8_t PIN_D4 = 4;
static const uint8_t PIN_D5 = 5;
static const uint8_t PIN_D6 = 6;
static const uint8_t PIN_D7 = 7;

static const uint8_t PIN_B0 = 8;
static const uint8_t PIN_B1 = 9;
static const uint8_t PIN_B2 = 10;
static const uint8_t PIN_B3 = 11;
static const uint8_t PIN_B4 = 12;
static const uint8_t PIN_B5 = 13;

static const uint8_t PIN_C0 = 14;
static const uint8_t PIN_C1 = 15;
static const uint8_t PIN_C2 = 16;
static const uint8_t PIN_C3 = 17;
static const uint8_t PIN_C4 = 18;
static const uint8_t PIN_C5 = 19;

static const uint8_t PIN_B6 = 20;
static const uint8_t PIN_B7 = 21;
static const uint8_t PIN_C6 = 22;
static const uint8_t PIN_C7 = 23;

static const uint8_t PIN_A0 = 24;
static const uint8_t PIN_A1 = 25;
static const uint8_t PIN_A2 = 26;
static const uint8_t PIN_A3 = 27;
static const uint8_t PIN_A4 = 28;
static const uint8_t PIN_A5 = 29;
static const uint8_t PIN_A6 = 30;
static const uint8_t PIN_A7 = 31;

static const uint8_t A0 = 24;
static const uint8_t A1 = 25;
static const uint8_t A2 = 26;
static const uint8_t A3 = 27;
static const uint8_t A4 = 28;
static const uint8_t A5 = 29;
static const uint8_t A6 = 30;
static const uint8_t A7 = 31;

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= PIN_D7) ? 3 : ((((p) <= PIN_B5)) ? 1 : (((p) <= PIN_C5)?2:(((p) <= PIN_B7)? 1:2))))
// #define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK3) : (((p) <= 13) ? (&PCMSK1) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
// #define digitalPinToPCMSKbit(p) (((p) <= 7) ? (p) : (((p) <= 13) ? ((p) - 8) : ((p) - 14)))

#define digitalPinToPCMSK(p)    ( ((p) <= PIN_D7) ? (&PCMSK3) : (((p) <= PIN_B5) ? (&PCMSK1) : (((p) <= PIN_C5) ? (&PCMSK2) : ((p) <= PIN_B7) ? (&PCMSK1) :  ((p) <= PIN_C7) ? (&PCMSK2) : (&PCMSK0))))
#define digitalPinToPCMSKbit(p) ( ((p) <= PIN_D7) ? (p) :       (((p) <= PIN_B5) ? ((p) - 8) : (((p) <= PIN_C5) ? ((p)-14) :  ((p) <= PIN_B7) ? ((p)-20+6) : ((p) <= PIN_C7) ? ((p)-22+6) :((p)-24))))


#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA644 
//
//                   +--\/--+
//       XCK0/T0 PB0  1|      |40  PA0 ADC0  
//       CLK0/T1 PB1  2|      |39  PA1 ADC1  
//     INT2/AIN0 PB2  3|      |38  PA2 ADC2   
//     OC0A/AIN1 PB3  4|      |37  PA3 ADC3   
//       OC0B/SS PB4  5|      |36  PA4 ADC4   
//          MOSI PB5  6|      |35  PA5 ADC5  
//          MISO PB6  7|      |34  PA6 ADC6
//           SCK PB7  8|      |33  PA7 ADC7
//             RESET  9|      |32  AVCC
//               VCC 10|      |31  GND       
//               GND 11|      |30  AVCC       
//             XTAL2 12|      |29  PC7 TOSC2     
//             XTAL1 13|      |28  PC6 TOSC1     
//           RXD PD0 14|      |27  PC5 TDI      
//         T  XD PD1 15|      |26  PC4 TDO      
//          INT0 PD2 16|      |25  PC3 TMS       
//          INT1 PD3 17|      |24  PC2 TCK      
//          OC1B PD4 18|      |23  PC1 SDA      
//          OC1A PD5 19|      |22  PC0 SCL      
//     OC2B/ICP1 PD6 20|      |21  PD7 OC2A     
//                     +------+    
// ATMEL ATMEGA1280 / ARDUINO
//
// 0-7 PE0-PE7   works
// 8-13 PB0-PB5  works
// 14-21 PA0-PA7 works 
// 22-29 PH0-PH7 works
// 30-35 PG5-PG0 works
// 36-43 PC7-PC0 works
// 44-51 PJ7-PJ0 works
// 52-59 PL7-PL0 works
// 60-67 PD7-PD0 works
// A0-A7 PF0-PF7
// A8-A15 PK0-PK7


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PD, /* 0 */
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PB, /* 8 */
	PB,
	PB,
	PB,
	PB,
	PB,
	PC, /* 14 */
	PC,
	PC,
	PC,
	PC,
	PC,
	PB, /* 20 */
	PB,
	PC,
	PC,
	PA, /* 24 */
	PA, 
	PA, 
	PA, 
	PA, 
	PA, 
	PA, 
	PA, /*  31 */
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(0), /* 0, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 8, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(0), /* 14, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6), /* 20, port B */
	_BV(7),
	_BV(6), /* 22, port C */
	_BV(7),
	_BV(0), /* 24, port A */
	_BV(1), 
	_BV(2), 
	_BV(3), 
	_BV(4), 
	_BV(5), 
	_BV(6), 
	_BV(7), /* 31*/
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,       // PD0 /* 0 - port D */
	NOT_ON_TIMER,		// PD1
	NOT_ON_TIMER,		// PD2
	NOT_ON_TIMER,		// PD3
	TIMER1B,			// PD4
	TIMER1A,			// PD5
	TIMER2B,			// PD6
	TIMER2A,			// PD7
	NOT_ON_TIMER,		// PB0 /* 8 - port B */
	NOT_ON_TIMER,		// PB1
	NOT_ON_TIMER,		// PB2
	TIMER0A,			// PB3
	TIMER0B,			// PB4
	NOT_ON_TIMER,		// PB5
	NOT_ON_TIMER,		// PB6
	NOT_ON_TIMER,		// PC0 /* 14 - port C */
	NOT_ON_TIMER,		// PC1 
	NOT_ON_TIMER,		// PC2 
	NOT_ON_TIMER,		// PC3 
	NOT_ON_TIMER,		// PC4 
	NOT_ON_TIMER,		// PC5 
	NOT_ON_TIMER,		// PB6 /* 20 - port B */
	NOT_ON_TIMER,		// PB7 
	NOT_ON_TIMER,		// PC6 /* 22 - port C */
	NOT_ON_TIMER,		// PA0 
	NOT_ON_TIMER,		// PA1 /* 24 - port A */
	NOT_ON_TIMER,		// PA1
	NOT_ON_TIMER,		// PA2 
	NOT_ON_TIMER,		// PA3 
	NOT_ON_TIMER,		// PA4 
	NOT_ON_TIMER,		// PA5 
	NOT_ON_TIMER,		// PA6 
	NOT_ON_TIMER,		// PA7 
	NOT_ON_TIMER,		// PA8 /* 31 */
};

#endif // ifdef ARDUINO_MAIN

#endif // Ifdef Pins_Arduino_h
