/*
  serial.c - Code to parse received data
*/
#include "serial.h"
#include <avr/interrupt.h>
#include <avr/io.h>

volatile unsigned char data_start = 0;
volatile unsigned char data_end=0;
volatile int buff_count=0;
volatile unsigned char temp_buff[6];

/** INTERRUPT FOR HANDLING RECEIVED DATA FROM SERIAL INTERFACE **/
ISR(USART_RX_vect){ // triggered if data was received
	char x = UDR0;
	if (x =='@'){ // start if receive @
		data_start = 1;
		buff_count =0;
		data_end = 0;	
	}
	else if ((x=='+') || (x =='-')){ // if +/- not in first spot, restart
		if (data_start){
			if (buff_count ==0){
				temp_buff[buff_count]=x;
				buff_count++;
			}
			else{
				data_start = 0;
			}
		}
	}
	else if ((x >= '0') && (x <='9')){ // if int value is not in right spot, restart
		if (data_start){
			if ((buff_count >=1) && (buff_count <= 3)){
				temp_buff[buff_count] = x;
				buff_count++;
			}
			else if ((buff_count==0) || (buff_count >3)){
				data_start = 0;
			}
		}
	}
	else if ((x == '#') && (buff_count>2)){ // if received # and in right spot, end 
		data_end = 1;
		data_start = 0;
	}
	if (data_start == 0){ // count resets when restarts
		buff_count = 0;
	}
}