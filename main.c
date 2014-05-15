// TAB WIDTH 7

// Generic includes
#include <io.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

// Project includes
#include "clock.h"		// clocks
#include "uart0.h"		// serial port
#include "timerA.h"		// timerA (kHz range)
#include "timerB.h"		// timerB (MHz range, used for radio communication)
#include "mac_clocksync.h"	// radio mac protocol 
#include "ds2411.h"		// external chip with unique serial number

// UART callback function
uint16_t char_rx(uint8_t c);

// printf's putchar on the serial port
int16_t putchar(int16_t c) {
    return uart0_putchar(c);
}

// TIMER A overflow callback function 
uint16_t timer_overflow(void);

// TIMER A callback function for the algorithm
uint16_t run_algorithm(void);

// TIMER A callback function for skew correction
uint16_t skew_correction(void);

// MAC callback functions
uint16_t frame_rx(uint8_t packet[], uint16_t length, uint16_t src_addr, uint16_t time_1w, uint16_t time_0w);
uint16_t frame_error(void);
uint16_t frame_sent(void);

// Sends the current (corrected) time to other nodes, with mode information 
void send_time(char mode);

// Given two words, apply (possibly negative) offset correction
uint16_t add_offset(int16_t d, uint16_t *w1, uint16_t *w0);

// OFFSET correction
void increase_offset(uint16_t my_time_1w, uint16_t my_time_0w, uint16_t other_time_1w, uint16_t other_time_0w);

// SKEW correction
void adjust_skew(uint16_t my_time_1w, uint16_t my_time_0w, uint16_t other_time_1w, uint16_t other_time_0w);

// PSEUDO RANDOM UPDATE
void update_rnd(void);


/* Global variables */

// time counter most significant word
// (experiment needs to be less than approx. 36 hours)
// least significant word is timerA_time()
volatile uint16_t time_1w;

// pseudorandom number
volatile uint16_t rnd;

// monitoring_node: if 1, broadcasts time every second
volatile uint8_t monitoring_node = 0;

// offset
volatile int16_t offset = 0;

// skew correction period
volatile uint16_t skew = 20000;

// sync algorithm
// n = none
// o = offset correction
// s = offset and skew correction
volatile char sync_algorithm = 'n';

/**
 * The main function.
 */

int main( void )
{

	// Stop the watchdog timer.
	WDTCTL = WDTPW + WDTHOLD;
    
	// Clock settings
	set_mcu_speed_xt2_mclk_8MHz_smclk_8MHz();	// used by CDMA
	set_aclk_div(1); // ACKL is at 32768Hz		// used for clock synchronization
    
	// Initialize the UART0
	uart0_init(UART0_CONFIG_8MHZ_115200);	// 115kbaud, SMCLK is at 8MHz
	uart0_register_callback(char_rx);	// Set the UART callback function

	// Initialize random number
	ds2411_init();
	rnd = (((uint16_t)ds2411_id.serial0) << 8) + (uint16_t)ds2411_id.serial1;

	// Timer settings
	time_1w = 0;
	timerA_init();
	timerA_start_ACLK_div(TIMERA_DIV_1);			// timerA period = 2s
	timerA_register_cb(TIMERA_ALARM_OVER, timer_overflow);	// timerA overflow event
	timerA_register_cb(TIMERA_ALARM_CCR0, run_algorithm);	// run algorithm at CCR0
	timerA_register_cb(TIMERA_ALARM_CCR1, skew_correction); // compensate skew error at CCR1
	timerA_set_alarm_from_now(TIMERA_ALARM_CCR0, rnd, 54983);	// same period 1.678s, different phase
	timerA_set_alarm_from_now(TIMERA_ALARM_CCR1, 35000, skew);	// skew compensation happens every 'skew' ticks

	// Initialize the MAC layer (radio)
	mac_init(11);
	mac_set_rx_cb(frame_rx);
	mac_set_error_cb(frame_error);
	mac_set_sent_cb(frame_sent);

	// Enable Interrupts
	eint();
    
 	while (1) {
	}

	return 0;
}


// By connecting to each node via the serial interface
//   nc wsn430-<nodeid> 20000
// it is possible to send commands, and receive data

// Function called when a char is received on the serial interface

uint16_t char_rx(uint8_t c) {

	switch (c) {
		case 'm':	// send time for monitoring, once
			printf(" Sending my time to others, for monitoring\n");	
			send_time('m');
			break;
		case 'l':	// logging
			printf(" I will send monitoring messages every second\n");	
			monitoring_node = 1;
			break;
		case 'n':	// set sync algorithm to 'none'
			printf(" Sync algorithm: none (broadcasted to all nodes)\n");	
			sync_algorithm = 'n';
			send_time('n');
			break;
		case 'o':	// set sync algorithm to 'offset correction'
			printf(" Sync algorithm: offset correction (broadcasted to all nodes)\n");
			sync_algorithm = 'o';
			send_time('o');
			break;
		case 's':	// set sync algorithm to 'skew correction'
			printf(" Sync algorithm: skew correction (broadcasted to all nodes)\n");
			sync_algorithm = 's';
			send_time('s');
			break;
	}

	return 1;

}

// When timerA overflows (approx. any second), the 1st word is increased by 1,
// and time reading is broadcasted, if the node is in monitoring mode.

uint16_t timer_overflow(void) {

	// increase msw by 1
	time_1w++;

	// if (monitoring_node && ((time_1w & 0x0003) == 0x0002)) send_time('m');
	// if the node is in monitoring mode (command l), send time for monitoring
	if (monitoring_node) send_time('m');

	return 1;

}

// When timerA hits CCR0, the synchronization algorithm is executed only if a 
// (p-weighted) coin flip returns true. This implements geometrically
// distributed (approx. exponentially distributed) waiting times.
// Expected waiting time is 1 / p * (CCR0 period)
// If p = 0x004F/0xFFFF = 1.2e-3, then 
// Ewt is 829.55 * 1.678 = 1391.9s = 23.2 min

uint16_t run_algorithm(void) {

	if (rnd < 0x004F) send_time(sync_algorithm); 
	update_rnd();

	return 1;

}


// SEND_TIME sends the current (corrected) time in broadcast to other nodes

void send_time(char mode) {

	uint16_t ctime_1w;
	uint16_t ctime_0w;

	uint8_t mymessage[6];
	mymessage[0] = 0;		// unused
	mymessage[1] = mode;

	ctime_1w = time_1w;
	ctime_0w = timerA_time();
	add_offset(offset, &ctime_1w, &ctime_0w);

	mymessage[2] = ctime_1w & 0xff;
	mymessage[3] = ctime_1w >> 8;

	mymessage[4] = ctime_0w & 0xff;
	mymessage[5] = ctime_0w >> 8;

	mac_send_once(mymessage, 6, MAC_BROADCAST);

}

// Function called when a frame is received

uint16_t frame_rx(uint8_t packet[], uint16_t length, uint16_t src_addr, uint16_t freeze_time_1w, uint16_t freeze_time_0w) {

	// get other node's time from message
	uint16_t other_time_1w = (uint16_t)(packet[2] | packet[3] << 8);
	uint16_t other_time_0w = (uint16_t)(packet[4] | packet[5] << 8);

	// apply local offset
	add_offset(offset, &freeze_time_1w, &freeze_time_0w);

	// remove transmission delay 
	add_offset(-35, &freeze_time_1w, &freeze_time_0w);

	// check packet purpose
	switch (packet[1]) {
		case 's':
			adjust_skew(freeze_time_1w, freeze_time_0w, other_time_1w, other_time_0w);
		case 'o':
			increase_offset(freeze_time_1w, freeze_time_0w, other_time_1w, other_time_0w);
			printf("RUN, %u, %u, ", src_addr, node_addr);
			printf("%u, %u, ", other_time_1w, other_time_0w);
			printf("%u, %u\n", freeze_time_1w, freeze_time_0w);
		case 'n':
			sync_algorithm = packet[1];
			break;
		case 'm':
			printf("MON, %u, %u, ", src_addr, node_addr);
			printf("%u, %u, ", other_time_1w, other_time_0w);
			printf("%u, %u, %i\n", freeze_time_1w, freeze_time_0w, freeze_time_0w-other_time_0w);
	}

	return 1;

}


uint16_t frame_error(void) {
	return 0;
}

uint16_t frame_sent(void) {
	return 0;
}

uint16_t add_offset(int16_t d, uint16_t *w1, uint16_t *w0) {

	uint16_t abs_offset;

	if (d >= 0) {
		if (d > (65535 - *w0)) {
			*w1 = *w1 + 1;
		}
		*w0 = *w0 + d;
	} else {
		abs_offset = ~d + 1;
		if (abs_offset > *w0) {
			*w1 = *w1 - 1;
		}
		*w0 = *w0 - abs_offset;
	}

	return 1;

}

void increase_offset(uint16_t my_time_1w, uint16_t my_time_0w, uint16_t other_time_1w, uint16_t other_time_0w) {

	uint16_t d;

	while (my_time_1w < other_time_1w) {
		my_time_1w++;
		time_1w++;
	}

	while (my_time_1w > other_time_1w) {
		my_time_1w--;
		time_1w--;
	}
	
	if (my_time_0w < other_time_0w) {
		d = (other_time_0w - my_time_0w)>>1;
		if ((uint16_t)(32767-offset) < d) time_1w++;
		offset = offset + d;
	} else {
		d = (my_time_0w - other_time_0w)>>1;
		if ((uint16_t)(offset+32768) < d) time_1w--;
		offset = offset - d;
	}

}

void adjust_skew(uint16_t my_time_1w, uint16_t my_time_0w, uint16_t other_time_1w, uint16_t other_time_0w) {

	int16_t d;
	uint16_t delta;

	d = other_time_0w - my_time_0w;

	printf("SKE was %u\n", skew);

	printf("D is, %i\n", d);

	if (d>0) {
		delta = (skew>>9) * (d>>1);
		if (delta > (65535-skew)) {
			skew = 65535;
		} else {
			skew = skew + delta;
		}
	} else {
		d = ~d + 1;
		delta = (skew>>9) * (d>>1);
		if (delta > (skew - 200)) {
			skew = 200;
		} else {
			skew = skew - delta;
		}
	}

	timerA_update_alarm_period(TIMERA_ALARM_CCR1, skew);

	printf("SKE becomes %u\n", skew);

}

void update_rnd(void) {

	rnd ^= (rnd << 13);
    	rnd ^= (rnd >> 9);
    	rnd ^= (rnd << 7);

}

uint16_t skew_correction(void) {

	if (offset==-32768) time_1w--;
	offset--;
	return 1;

}

