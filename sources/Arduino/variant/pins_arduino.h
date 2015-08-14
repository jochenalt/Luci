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

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__)
#include "pins_arduino_atmega8_168.h"
#elif defined (__AVR_ATmega644__) || defined (__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) 
#include "pins_arduino_atmega644.h"
#elif defined(__AVR_ATmega328__)  || defined(__AVR_ATmega328P__) 
#include "pins_arduino_atmega328.h"
#else
#fatal "AVR type unknown, it is not AtMega8, 168, 644PA, 328, 328P"
#endif

#endif
