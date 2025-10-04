/*
  Servo.h - Library for servo motors.
  Copyright (c) 2008-2009 David A. Mellis.  All right reserved.

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

#ifndef Servo_h
#define Servo_h

#include <inttypes.h>

/*
 * Defines for 16-bit timers used with Servo library
 *
 * If _useTimerX is defined then TimerX is used by the Servo library.
 * The DO NOT DEFINE signals are unused in this case, and can be used for other purposes
 * The DEFINE signals are used by the library and should not be used for other purposes
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define _useTimer5
#define _useTimer1
#define _useTimer3
#define _useTimer4
typedef enum { _timer5, _timer1, _timer3, _timer4, _Nbr_16_bit_timers } timer16_Sequence_t;

#elif defined(__AVR_ATmega32U4__)
#define _useTimer1
#define _useTimer3
typedef enum { _timer1, _timer3, _Nbr_16_bit_timers } timer16_Sequence_t;

#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#define _useTimer1
#define _useTimer3
typedef enum { _timer1, _timer3, _Nbr_16_bit_timers } timer16_Sequence_t;

#elif defined(__AVR_ATmega128__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
#define _useTimer1
#define _useTimer3
typedef enum { _timer1, _timer3, _Nbr_16_bit_timers } timer16_Sequence_t;

#else  // everything else
#define _useTimer1
typedef enum { _timer1, _Nbr_16_bit_timers } timer16_Sequence_t