/*
  encoder.c - Response when encoder interrupt is triggered
*/
#include "encoder.h"
#include <avr/interrupt.h>
#include <avr/io.h>

volatile int threshold= 50;
volatile unsigned char a, b;
volatile unsigned char new_state, old_state;
volatile unsigned char changed;

/** INTERRUPT FOR HANDLING ROTARY ENCODER **/
ISR(PCINT2_vect){ 
	unsigned char C = PIND;
    // Read the input bits and determine A and B.
	a = C & (1<<2);
	b = C & (1<<3);

	if (old_state == 0) {
		// Handle A and B inputs for state 0
		if (a){
			new_state = 1;
			threshold++;
		}
		else if(b){
			new_state = 2;
			threshold--;
		}
	}
	else if (old_state == 1) {
	    // Handle A and B inputs for state 1
		if (!a){
			new_state = 0;
			threshold--;
		}
		else if (b){
			new_state = 3;
			threshold++;
		}
	}
	else if (old_state == 2) {
	    // Handle A and B inputs for state 2
		if (a){
			new_state = 3;
			threshold--;
		}
		else if (!b){
			new_state=0;
			threshold++;
		}
	}
	else{   // old_state = 3
	    // Handle A and B inputs for state 3
		if (!a){
			new_state=2;
			threshold++;
		}
		else if(b){
			new_state=1;
			threshold--;
		}
	}

    if (new_state != old_state) {
    	changed = 1;
    	old_state = new_state;
	}


	if (threshold <50){
		threshold = 50;
	}
	else if (threshold >90){
		threshold = 90;
	}
}