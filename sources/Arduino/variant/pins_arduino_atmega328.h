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

#ifndef Pins_Arduino_AtMega328_h
#define Pins_Arduino_AtMega328_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            20
#define NUM_ANALOG_INPUTS           6

#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 14 : -1)

// PWM Pins: PIN_D3, PIN_D5, PIN_D6, PIN_B1, PIN_B2, PIN_B3
#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11)

static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

static const uint8_t SDA = 18;
static const uint8_t SCL = 19;

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

static const uint8_t A0 = 14;
static const uint8_t A1 = 15;
static const uint8_t A2 = 16;
static const uint8_t A3 = 17;
static const uint8_t A4 = 18;
static const uint8_t A5 = 19;

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 2 : (((p) <= 13) ? 0 : 1))
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK2) : (((p) <= 13) ? (&PCMSK0) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
#define digitalPinToPCMSKbit(p) (((p) <= 7) ? (p) : (((p) <= 13) ? ((p) - 8) : ((p) - 14)))

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEGA 328
//    
//                  +-\/-+
//     RESET/ PC6  1|    |28  PC5 ADC5/SCL
//        RXD PD0  2|    |27  PC4 ADC4/SDA
//        TXD PD1  3|    |26  PC3 ADC3  
//       INT0 PD2  4|    |25  PC2 ADC2  
//  OC2B/INT1 PD3  5|    |24  PC1 ADC1 )
//        XCK PD4  6|    |23  PC0 ADC0  
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//      XTAL1 PB6  9|    |20  AVCC
//      XTAL2 PB7 10|    |19  PB5 SCK   
//      OC0B  PD5 11|    |18  PB4 MISO  
//      OC0A  PD6 12|    |17  PB3 MOSI/OC2A 
//            PD7 13|    |16  PB2 SS/OC1B   
//      ICP1  PB0 14|    |15  PB1 OC1A     
//                  +----+
//
// (PWM+ indicates the additional PWM pins on the ATmega168.)

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
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

};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,       // PD0 /* 0 - port D */
	NOT_ON_TIMER,		// PD1
	NOT_ON_TIMER,		// PD2
	TIMER2B,			// PD3
	NOT_ON_TIMER,		// PD4
	TIMER0B,			// PD5
	TIMER0A,			// PD6
	NOT_ON_TIMER,		// PD7
	NOT_ON_TIMER,		// PB0 /* 8 - port B */
	TIMER1A,			// PB1
	TIMER1B,			// PB2
	TIMER2A,			// PB3
	NOT_ON_TIMER,		// PB4
	NOT_ON_TIMER,		// PB5
	NOT_ON_TIMER,		// PB6
	NOT_ON_TIMER,		// PC0 /* 14 - port C */
	NOT_ON_TIMER,		// PC1 
	NOT_ON_TIMER,		// PC2 
	NOT_ON_TIMER,		// PC3 
	NOT_ON_TIMER,		// PC4 
	NOT_ON_TIMER,		// PC5 
};

#endif // ifdef ARDUINO_MAIN

#endif // Ifdef Pins_Arduino_h
