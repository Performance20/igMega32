/*
 * igMega32.c
 *
 * Created: 02.12.2020 14:01:07
 * Author : Helge
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <limits.h>


#include "usart.h"
#include "protocol.h"
#include "tools.h"
#include "definitions.h"
#include "eeprom_circular_buffer.h"


#define BUFF_SIZE   25   // buffer - 1 char

#define MAX_TIME_BETWEEN (UINT_MAX + 1) * 256   // round about 1 second
#define MAIN_LOOP_DELAY		1					// 1 sec
//#define MAIN_LOOP_DELAY_in_ms	MAIN_LOOP_DELAY * 1000  // 1 sec
#define MAIN_LOOP_DELAY_in_ms	500  //
#define OP_CNT_POINT		10  // each 10 sec write to the eeprom the counter

// EEprom parameter handling
uint8_t  EEMEM eeprom_start;
uint8_t  EEMEM eeprom_optm_start;
EE_PARAM EEMEM eeprom;
EE_PARAM parameter;
ignition_point_t EEMEM eeignition_point_tbl1[ignition_point_tbl_SIZE];
ignition_point_t EEMEM eeignition_point_tbl2[ignition_point_tbl_SIZE];
ignition_point_t EEMEM eeignition_point_tbl3[ignition_point_tbl_SIZE];
ignition_point_t ignition_point_tbl[ignition_point_tbl_SIZE];

uint8_t restartflag = 0;

#define DATA_SIZE sizeof(uint32_t)
#define OP_MAX_ENTRIES  100 // 100k per entry writing possible until eeprom die - = 100 * (4+1) = 500 Byte
#define MEM_SIZE ((DATA_SIZE) + 1) * OP_MAX_ENTRIES

uint8_t  EEMEM eeprom_operationTime[MEM_SIZE];
volatile uint32_t operationTime;
struct ee_cb cb;

void ee_read(uint8_t *addr, uint8_t *dst, size_t size) {
	eeprom_read_block(dst, addr, size);

}

void ee_write(uint8_t *addr, const uint8_t *src, size_t size) {
	eeprom_update_block(src, addr, size);
}

volatile  uint16_t act_rpm = 0;	                  // revolutions per minute
volatile  uint32_t act_rps = 0;	                  // revolutions per second
volatile  int8_t act_IP = 0;	                      // current injection point in degree
volatile  int16_t act_DWA = 0;	                      // current dwell angle in degree
// time to next ignition in ms make it sense ?
volatile  uint16_t next_ip_ms = 0;	           // in ms


volatile int32_t fire_cnt = 0;
volatile uint16_t ticks_cnt = 0;
volatile uint8_t ticksTCNT0_cnt = 0;
volatile uint16_t pre_ticks_cnt = 0;
volatile uint8_t pre_ticksTCNT0_cnt = 0;


ISR (TIMER1_OVF_vect) //  overflow count between the signals to get the current rpm value
{
	if (ticks_cnt < UINT_MAX) ticks_cnt++; // count min 1 round per second
	//ticks_cnt++; // count min 1 round per second
}

ISR (INT0_vect)
{
	// raw ticks count per second between 2 sparkle fires
	
	pre_ticks_cnt = TCNT1;
	TCNT1 = 0;
	pre_ticks_cnt = ticks_cnt;
	ticks_cnt = 0;
	
}

void init_device()
{
	uint8_t i;
	uint8_t status;
	
	cli();
	ee_cb_init(&cb, eeprom_operationTime, DATA_SIZE, OP_MAX_ENTRIES, ee_write, ee_read); // init the eeprom ring buffer
	// load eeprom data parameter data
	status = eeprom_read_byte(&eeprom_start);
	if (status != 1) // init the eeprom with default value
	{
		parameter.LOG_state = DEF_LOG_state;
		parameter.LED_state = DEF_LED_state;
		parameter.ignition_mode = DEF_ignition_mode;
		parameter.ithelper_startpoint = DEF_ithelper_startpoint;
		parameter.ithelper_RPM = DEF_ithelper_RPM;
		parameter.ignition_fix_startpoint = DEF_ignition_fix_startpoint;
		parameter.dwell_angle_fix = DEF_dwell_angle_fix;
		parameter.active_ip_tbl = DEF_active_ip_tbl;
		eeprom_update_block(&parameter, &eeprom, sizeof(eeprom));
		for (i=0; i<ignition_point_tbl_SIZE; i++)
		{
			ignition_point_tbl[i].rpm = DEF_tbl_pos;
			ignition_point_tbl[i].rpm = DEF_tbl_rpm;
			ignition_point_tbl[i].degree = DEF_tbl_degree;
			ignition_point_tbl[i].dwa = DEF_tbl_dwa;
		}
		eeprom_update_block(&ignition_point_tbl, &eeignition_point_tbl1, sizeof(eeignition_point_tbl1));
		eeprom_update_block(&ignition_point_tbl, &eeignition_point_tbl2, sizeof(eeignition_point_tbl2));
		eeprom_update_block(&ignition_point_tbl, &eeignition_point_tbl3, sizeof(eeignition_point_tbl3));
		eeprom_update_byte(&eeprom_start, 1);
	}
	else
	{
		eeprom_read_block(&parameter, &eeprom, sizeof(parameter));
		if (parameter.active_ip_tbl == VAL_active_ip_table_2)
		eeprom_read_block(&ignition_point_tbl, &eeignition_point_tbl2, sizeof(ignition_point_tbl));
		else if (parameter.active_ip_tbl == VAL_active_ip_table_3)
		eeprom_read_block(&ignition_point_tbl, &eeignition_point_tbl3, sizeof(ignition_point_tbl));
		else
		eeprom_read_block(&ignition_point_tbl, &eeignition_point_tbl1, sizeof(ignition_point_tbl));
	}
	// operation counter
	status = eeprom_read_byte(&eeprom_optm_start);
	if (status != 1) // set operation time back to zero
	{
		operationTime = 0;
		ee_cb_write(&cb, (uint8_t*) &operationTime);
		eeprom_update_byte(&eeprom_optm_start, 1);
	}
	else
	{
		ee_cb_read(&cb, (uint8_t*) &operationTime);
		operationTime += OP_CNT_POINT / 2; // max. wrong value could be 10 sec, therefore I add 5 sec to middle the error
	}
	
	// Timer 0 config
	//GTCCR |= _BV(TSM); // Timer in config mode
	//TCCR0B |=  _BV(CS00); // Prescaler = 0
	//TCCR0B &= ~ _BV(WGM02);
	//TCCR0A &= ~(_BV(WGM00) | _BV(WGM01));
	//TIMSK |= _BV(TOIE0);      // do Overflow Interrupt
	//GTCCR &= ~ _BV(TSM); //timer in run mode

	// Timer 1 config
	//TCCR1|= (_BV(CS12) | _BV(CS10));     // use clock speed = F_CPU
	//TCCR1|= (_BV(CS13));
	//TCCR1|= (_BV(CS13) | _BV(CS11)| _BV(CS10));
	//TCCR1|= (_BV(CS13) | _BV(CS12) | _BV(CS11) | _BV(CS10));
	//TIMSK |= _BV(TOIE1);  //enable overflow interrupt
	
	
	
	// Pin setting is missing PD2 = INT0, PD3 = INT1	
	INPUT_SET(SIG_INPUT); //DDRD &= ~(1 << DDD2);     // Clear the PD2 pin - PD2 (PCINT0 pin) is now an input
	HIGH_SET(SIG_INPUT); //PORTD |= (1 << PORTD2);    //turn On the Pull-up - PD2 is now an input with pull-up enabled	
	
	OUTPUT_SET(LED_BUILTIN);  //LED PB5 as output
	
	// set INT0
	EICRA |= (_BV(ISC01)); //	The falling edge of INTx generates an interrupt request 
	EICRA |= (1 << ISC00);    // set INT0 to trigger on ANY logic change
	EIMSK |= (1 << INT0);
	
	uart_init(BAUD_CALC(57600)); // 8n1 transmission is set as default
	//wdt_enable(WDTO_1S); // enable 1s watchdog timer
	sei(); // Enable interrupts after re-enumeration
	
}

int main(void)
{
	char buffer[BUFF_SIZE];
	
	init_device();
	OUTPUT_SET(LED_BUILTIN);  //LED PB5 as output
    
	/* Replace with your application code */
	bool a = false;
    while (1) 
    {	
		if (uart_AvailableBytes())
		{
			uart_gets(buffer, BUFF_SIZE); // read 24 bytes or one line from usart buffer 
			//uart_putstr(buffer);
			TOGGLE_SET(LED_BUILTIN); /*PORTB = (1<<PORTB5);*/ a = 0; 
			//TOGGLE_SET(LED_BUILTIN);
			uart_puts("hello world\n"); // write const string to usart buffer // C++ restriction, in C its the same as uart_putstr()
			_delay_ms(100);	
		}
    }
}

