/*
 * defintions.h
 *
 * Created: 11.10.2019 14:29:23
 *  Author: helge
 */ 


#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <avr/eeprom.h>
#include "definitions.h"
#include "protocol.h"

#define LED_BUILTIN			B,5 // on board LED
//#define LED_BUILTIN			PB5 // on board LED
#define SIG_INPUT			D,2	// Input signal from light barrier
#define FIRMWARE_REV_MAIN	0  	// 0-255
#define FIRMWARE_REV_SUB	0	//0-255

volatile static uint8_t firmwareMain = FIRMWARE_REV_MAIN;  // main.rev  = 1-255
volatile static uint8_t firmwareSub = FIRMWARE_REV_SUB;  // sub.rev  = 1-255

typedef struct {
	uint8_t  LOG_state;
	uint8_t  LED_state;
	uint8_t ignition_mode;
	int8_t ithelper_startpoint;
	uint16_t ithelper_RPM;
	int8_t ignition_fix_startpoint;
	int16_t dwell_angle_fix;
	uint8_t active_ip_tbl;
} EE_PARAM;



#endif /* __DEFINITIONS_H__ */