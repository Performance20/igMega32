/*
 * tools.h
 *
 * Created: 20.09.2019 20:55:23
 *  Author: helge
 */ 


#ifndef __TOOLS_H__
#define __TOOLS_H__
#include "definitions.h"

#define _SET(type,name,bit)          type ## name  |= _BV(bit)
#define _CLEAR(type,name,bit)        type ## name  &= ~ _BV(bit)
#define _TOGGLE(type,name,bit)       type ## name  ^= _BV(bit)
#define _GET(type,name,bit)          ((type ## name >> bit) &  1)
#define _PUT(type,name,bit,value)    type ## name = ( type ## name & ( ~ _BV(bit)) ) | ( ( 1 & (unsigned char)value ) << bit )
#define _BIT(name,bit)               (bit)

//these macros are used by end user
#define OUTPUT_SET(pin)         _SET(DDR,pin)
#define INPUT_SET(pin)          _CLEAR(DDR,pin)
#define HIGH_SET(pin)           _SET(PORT,pin)
#define LOW_SET(pin)            _CLEAR(PORT,pin)
#define TOGGLE_SET(pin)         _TOGGLE(PORT,pin)
#define READ_PIN(pin)           _GET(PIN,pin)
#define BIT_GET(pin)            _BIT(pin)

#define setbit(port, bit) (port) |= _BV(bit)  // setbit(PORTA, PA0)
#define clearbit(port, bit) (port) &= ~_BV(bit)

#define BT(x) ( \
(x[0] == '1') << 7 | \
(x[1] == '1') << 6 | \
(x[2] == '1') << 5 | \
(x[3] == '1') << 4 | \
(x[4] == '1') << 3 | \
(x[5] == '1') << 2 | \
(x[6] == '1') << 1 | \
(x[7] == '1') << 0 )

#define BL(x) ( \
0##x >>  0 & 0001 | \
0##x >>  2 & 0002 | \
0##x >>  4 & 0004 | \
0##x >>  6 & 0010 | \
0##x >>  8 & 0020 | \
0##x >> 10 & 0040 | \
0##x >> 12 & 0100 | \
0##x >> 14 & 0200 )

#define B(x) ( \
0##x /        01 % 010 << 0 | \
0##x /       010 % 010 << 1 | \
0##x /      0100 % 010 << 2 | \
0##x /     01000 % 010 << 3 | \
0##x /    010000 % 010 << 4 | \
0##x /   0100000 % 010 << 5 | \
0##x /  01000000 % 010 << 6 | \
0##x / 010000000 % 010 << 7 )

//#define FIREBIT             BIT_GET(FIRE)
//#define SIGNALBIT           BIT_GET(SIGNAL)

inline void BlinkLED(bool led_state) {
   if (led_state == true) 
	TOGGLE_SET(LED_BUILTIN);
}

inline void BlinkLEDD(void) {
	TOGGLE_SET(LED_BUILTIN);
}

inline void SetLED_On(void) {
	HIGH_SET(LED_BUILTIN);
}

inline void SetLED_Off(void) {
	LOW_SET(LED_BUILTIN);
}

#endif /* __TOOLS_H__ */