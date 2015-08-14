/*
  twi.h - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef twi_h
#define twi_h

  #include <inttypes.h>

  //#define ATMEGA8

// a mpu6050 packet is 14 byte, with 625000 Hz it takes 174us to fetch it
  #ifndef TWI_FREQ
// #define TWI_FREQ 100000L
// #define TWI_FREQ 200000L
// #define TWI_FREQ 555555L // gives TWBR of 10
// #define TWI_FREQ 588236L // gives TWBR of 9
// #define TWI_FREQ 625000L // gives TWBR of 8 / 11
// #define TWI_FREQ 666666L // gives TWBR of 7
// #define TWI_FREQ 714285L // gives TWBR of 6
// #define TWI_FREQ 750000L // gives TWBR of 5 / 8
// #define TWI_FREQ 769230L // gives TWBR of 5
// #define TWI_FREQ 833333L // gives TWBR of 4

  #endif

  #ifndef TWI_BUFFER_LENGTH
  #define TWI_BUFFER_LENGTH 32
  #endif

  #define TWI_READY 0
  #define TWI_MRX   1
  #define TWI_MTX   2
  #define TWI_SRX   3
  #define TWI_STX   4
  
  void twi_init(void);
  void twi_setAddress(uint8_t);
  uint8_t twi_readFrom(uint8_t, uint8_t*, uint8_t, uint8_t);
  uint8_t twi_writeTo(uint8_t, uint8_t*, uint8_t, uint8_t, uint8_t);
  uint8_t twi_transmit(const uint8_t*, uint8_t);
  void twi_attachSlaveRxEvent( void (*)(uint8_t*, int) );
  void twi_attachSlaveTxEvent( void (*)(void) );
  void twi_reply(uint8_t);
  void twi_stop(void);
  void twi_releaseBus(void);

#endif

