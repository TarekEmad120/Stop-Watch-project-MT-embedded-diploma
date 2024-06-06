/*
 * mini_project2.c
 *
 *  Created on: Sep 13, 2023
 *      Author: Tarek Emad
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include"typedefining.h"

// Define pins for 7-segment displays, 7447 decoder, and push buttons
#define SEGMENT_PORT    PORTA
#define DECODER_PORT    PORTC
#define SEGMENT_DDR     DDRA
#define DECODER_DDR     DDRC

// Define pins for 7-segment enable/disable
#define ENABLE_PINS     0x3F  // 6 bits for 6 displays
// Global variables
uint16_t stopwatch_time = 0; // Time in SECS
uint8_t stopwatch_running = 1; // Flag to indicate if stopwatch is running
uint16_t sec=0,min=0,h=0;


void displaysegment(void){
	h=stopwatch_time/3600;
	min=(stopwatch_time/60)%60;
	sec=stopwatch_time%60;

	SEGMENT_PORT &= (0xC0);
	SEGMENT_PORT |= (1<<0) ;
	DECODER_PORT &= (0xF0);
	DECODER_PORT|= (sec%10);

	_delay_us(5);

	SEGMENT_PORT &= (0xC0);
	SEGMENT_PORT |= (1<<1) ;
	DECODER_PORT &= (0xF0);
	DECODER_PORT|= (sec/10);

	_delay_us(5);

	SEGMENT_PORT &= (0xC0);
	SEGMENT_PORT |= (1<<2) ;
	DECODER_PORT &= (0xF0);
	DECODER_PORT|= (min%10);

	_delay_us(5);

	SEGMENT_PORT &= (0xC0);
	SEGMENT_PORT |= (1<<3) ;
	DECODER_PORT &= (0xF0);
	DECODER_PORT|= (min/10);

	_delay_us(5);

	SEGMENT_PORT &= (0xC0);
	SEGMENT_PORT |= (1<<4) ;
	DECODER_PORT &= (0xF0);
	DECODER_PORT|= (h%10);

	_delay_us(5);

	SEGMENT_PORT &= (0xC0);
	SEGMENT_PORT |= (1<<5) ;
	DECODER_PORT &= (0xF0);
	DECODER_PORT|= (h/10);

	_delay_us(5);

}


// Initialize external interrupts for buttons
void interrupt_init() {
	MCUCR |= (1 << ISC01); // INT0 falling edge trigger
	MCUCR |= (1 << ISC11) |(1<<ISC10); // INT1 rising edge trigger
	MCUCSR &=~(1 << ISC2); // INT2 falling edge trigger
	GICR |= (1 << INT0) | (1 << INT1) | (1 << INT2); // Enable INT0, INT1, INT2

}

void timer1_init() {
	TCCR1A = (1<<FOC1A);
	TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10);
	TCNT1 = 0;				// Bottom = 0
	OCR1A = 976;			// Clock for 1 second
	TIMSK |= (1<<OCIE1A);	// Set Module flag

}


ISR(INT0_vect) {
	stopwatch_time = 0;

}

// External interrupt 1 ISR (pause stopwatch)
ISR(INT1_vect) {
	if (stopwatch_running) {
		TIMSK &= ~(1 << OCIE1A); // Disable Timer1 compare interrupt
		stopwatch_running = 0;
	}
}

// External interrupt 2 ISR (resume stopwatch)
ISR(INT2_vect) {
	if (!stopwatch_running) {
		TIMSK |= (1 << OCIE1A); // Enable Timer1 compare interrupt
		stopwatch_running = 1;
	}
}

// Timer1 compare interrupt (1ms timer)
ISR(TIMER1_COMPA_vect) {

	if (stopwatch_running) {
		stopwatch_time++; // Increment stopwatch time
		// Implement logic to update and display the stopwatch time


	}
}

int main() {
	// Initialize DDR and PORT register
	SEGMENT_DDR = ENABLE_PINS ;
	DECODER_DDR = 0x0F;
	DDRD &= ~((1<<PD2) | (1<<PD3));
	DDRB &= ~(1<<PB2);
	PORTD |= 1<<PD2;
	PORTB |= 1<<PB2;
	SREG |= (1<<7);
	DECODER_PORT = 0;
	interrupt_init();
	timer1_init();


	while (1) {
		displaysegment();

	}

	return 0;
}

