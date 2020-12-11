#pragma once

/*
 * protocol.h
 *
 * Created: 26.09.2019 21:24:50
 *  Author: helge

 * define the different messages to read/write spark/ignition values
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#ifndef uchar
#define uchar   unsigned char
#endif

#ifndef uint
#define uint   unsigned int
#endif

//general states
//#define VAL_STATE_OFF						0
//#define VAL_STATE_ON						1

//caused bay performance problems, USB and real time handling in parallel
#define REQ_USB_MODE						0  // wants to have the current log
#define VAL_Mode_OFF						0
#define VAL_Mode_ON							1
#define VAL_Mode_REAL						2  // working mode + real time data
#define VAL_Mode_CONFIG						3  // no working mode, only configuration
#define MAX_Mode_state					VAL_Mode_ON
#define MIN_Mode_state					VAL_Mode_OFF
#define DEF_Mode_state					VAL_Mode_CONFIG


 // logging to host handling
#define REQ_LOGGING							REQ_USB_MODE + 1  // wants to have the current log
#define REQ_LOGGING_SET						REQ_LOGGING + 1  // start logging
#define REQ_LOGGING_GET						REQ_LOGGING_SET + 1  // stopp logging
#define VAL_LOG_OFF							0
#define VAL_LOG_ON							1
#define MAX_LOG_state					VAL_LOG_ON
#define MIN_LOG_state					VAL_LOG_OFF
#define DEF_LOG_state					VAL_LOG_OFF

// onboard LED handling
#define REQ_ONBOARD_LED_SET					REQ_LOGGING_GET + 1  // set onboard LED
#define REQ_ONBOARD_LED_GET					REQ_ONBOARD_LED_SET + 1
#define VAL_LED_STATE_OFF					0
#define VAL_LED_STATE_ON					1
#define VAL_LED_STATE_BLINK					2
#define VAL_LED_STATE_INPUT					3
#define VAL_LED_STATE_DIAG					4
#define MAX_LED_state					VAL_LED_STATE_DIAG
#define MIN_LED_state					VAL_LED_STATE_OFF
#define DEF_LED_state					VAL_LED_STATE_INPUT

// define mode of operation
#define REQ_ignition_mode_GET				REQ_ONBOARD_LED_GET + 1  
#define REQ_ignition_mode_SET				REQ_ignition_mode_GET + 1
#define VAL_ignition_mode_M0				0  // manuell
#define VAL_ignition_mode_M1				1  // fix
#define VAL_ignition_mode_M2				2  // fix with start helper
#define VAL_ignition_mode_M3				3  // dyn via table
#define MAX_ignition_mode					VAL_ignition_mode_M3
#define MIN_ignition_mode					VAL_ignition_mode_M0
#define DEF_ignition_mode					VAL_ignition_mode_M2

// On mode M2, define the ignition timing (IT) point as start support, in round per minutes 
#define REQ_ITH_startpoint_GET				REQ_ignition_mode_SET + 1 
#define REQ_ITH_startpoint_SET				REQ_ITH_startpoint_GET + 1
#define MAX_ithelper_startpoint				20
#define MIN_ithelper_startpoint				(-30)
#define DEF_ithelper_startpoint				0 

// On mode M2, define the max RPM for support starter help
#define REQ_starthelp_RPM_GET				REQ_ITH_startpoint_SET + 1 
#define REQ_starthelp_RPM_SET				REQ_starthelp_RPM_GET + 1
#define MAX_ithelper_RPM					2000
#define MIN_ithelper_RPM					0
#define DEF_ithelper_RPM					500 

// On fixed config = mode M1 + M2, define the shift from TDC
#define REQ_ignition_fix_startpoint_GET		REQ_starthelp_RPM_SET + 1
#define REQ_ignition_fix_startpoint_SET		REQ_ignition_fix_startpoint_GET + 1
#define MAX_ignition_fix_startpoint			20
#define MIN_ignition_fix_startpoint			(-30)
#define DEF_ignition_fix_startpoint			(-22)

// Dwell angle
#define REQ_Dwell_Angle_GET					REQ_ignition_fix_startpoint_SET + 1
#define REQ_Dwell_Angle_SET					REQ_Dwell_Angle_GET + 1
#define MAX_dwell_angle_fix					(-140)
#define MIN_dwell_angle_fix					(-240)
#define DEF_dwell_angle_fix					(-180)  

// which table is active
#define REQ_active_ip_tbl_GET						REQ_Dwell_Angle_SET + 1 
#define REQ_active_ip_tbl_SET						REQ_active_ip_tbl_GET + 1
#define VAL_active_ip_table_1						1  // 
#define VAL_active_ip_table_2						2  // 
#define VAL_active_ip_table_3						3  // 
#define MAX_active_ip_tbl					VAL_active_ip_table_3
#define MIN_active_ip_tbl					VAL_active_ip_table_1
#define DEF_active_ip_tbl					VAL_active_ip_table_1

// set get a value in the tablex, 
// Format: 
// tbl number (0-2) - Byte (wValue.bytes[0]) Host -> Client  
// tbl position (0 - size) - Byte (wValue.bytes[1]) Host -> Client
// ignition_point rpm (0 - 20000) - 2 Bytes (wIndex.bytes[0+1] Host -> Client
// ignition_point degree (+/- 35) - 1 Byte, use etra data byte Host -> Client
// Client -> Host use serial bytes (5 bytes) 
#define ignition_point_tbl_SIZE					10	

#define REQ_ip_tbl_GET					REQ_active_ip_tbl_SET + 1  
#define REQ_ip_tbl_SET					REQ_ip_tbl_GET + 1
#define VAL_ip_table_1					1  //
#define VAL_ip_table_2					2  //
#define VAL_ip_table_3					3  //
#define MAX_ip_tbl					    VAL_ip_table_3
#define MIN_ip_tbl						VAL_ip_table_1
#define DEF_ip_tbl						VAL_ip_table_1
#define MAX_ip_tbl_entry				ignition_point_tbl_SIZE - 1
#define MIN_ip_tbl_entry						0


typedef struct ignition_point {
	int8_t		pos;
	int8_t		degree;   // shift to degree (-30/20)
	uint16_t 	rpm;     // if rpm > actl rpm
	int16_t 	dwa;     // dwell angle -240/-140
} ignition_point_t;

#define DEF_tbl_rpm					0  
#define DEF_tbl_degree				(-22) 
#define DEF_tbl_dwa					(-180)  
#define DEF_tbl_pos					0  
	
/**********************************************************************************/
//
// actual measured rpm

// actual measured rpm
#define REQ_rpm_GET								REQ_ip_tbl_SET + 1 
#define REQ_rps_GET								REQ_rpm_GET + 1
#define REQ_act_IP_GET							REQ_rps_GET + 1
#define REQ_act_DWA_GET							REQ_act_IP_GET + 1
// time to next ignition in ms make it sense ?
#define REQ_next_ip_time_in_ms_GET				REQ_act_DWA_GET + 1

// Overal running time
#define REQ_operation_sec_GET		REQ_next_ip_time_in_ms_GET + 1  // running in seconds or better minutes?
#define REQ_operation_sec_SET		REQ_operation_sec_GET + 1  //

#define	REQ_firmware_version_GET	REQ_operation_sec_SET + 1 

#define REQ_eeprom_SET		REQ_firmware_version_GET + 1  //store all parameter values in the eeprom. Could be initiate by the host to avoid to much writing to the eeprom
#define REQ_eeprom_parameter_SET			0
#define REQ_eeprom_table1_SET				1
#define REQ_eeprom_table2_SET				2
#define REQ_eeprom_table3_SET				3
#define REQ_eeprom_INIT_SET					4 // clean the used Byte (addr 0) to load default values

#define REQ_modul_BOOT					    REQ_eeprom_SET + 1

#define DATA_NUMBER_SIZE_IN_BYTE			4						// size in Byte of maximum Number = 4 Byte = uint32_t
#define DATA_STRING_SIZE_IN_BYTE			15						// size in Byte of one transfered String = 15 Byte
#define DATA_TABLE_SIZE_IN_BYTE				(sizeof(ignition_point_t)*ignition_point_tbl_SIZE)  // size in Byte of one transfered table

#endif /* PROTOCOL_H_ */