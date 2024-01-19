/********************************************
 *
 *  Name: Carrie Lei
 *  Email: cnlei@usc.edu
 *  Section: Friday 11 AM
 *  Assignment: Final Project - Thermometer
 *
 ********************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "ds18b20.h"
#include "lcd.h"
#include "serial.h"
#include "encoder.h"

/** DEFINE MYUBRR **/
#define FOSC 16000000 // Clock frequency
#define BAUD 9600 // Baud rate used
#define MYUBRR (FOSC/16/BAUD-1)

/** NON-GLOBAL VARIABLES USED **/
volatile int whole_num;
volatile int buzzer_timer=0;
volatile unsigned char flag_buzzed = 0;
volatile unsigned char remote, local;

int main(void){
   
    /** INITIALIZE INPUTS **/
	DDRC &= ~(1<<3); // PC3 = temp sensor
	PORTC |= (1<<3); // Enable pullup resistor on PC3

	DDRD &= ~(1<<2); // PD2, PD3 = encoder
  	DDRD &= ~(1<<3);
	PORTD |= (1<<2); // Enable pull-up resistors on PD2, PD3
	PORTD |= (1<<3);

	DDRC &= ~(1<<1); // PC1 = local display button
    PORTC |= (1<<1); // Enable pullup resistor on PC1
	DDRC &= ~(1<<2); // PC2 = remote display button
	PORTC |= (1<<2); // Enable pullup resistor on PC2
	PCICR |= (1<<PCIE1); // Enable pin change interrupts on buttons
	PCICR |= (1<< PCIE2);
	PCMSK1 |= (1<<PCINT9);
	PCMSK1 |= (1<<PCINT10);

	DDRD &= ~(1<<0); // PD0 = RX input
	PORTD |= (1<<0); // Enable pullup resistor on PD0

    /** INITIALIZE OUTPUTS **/
    DDRB |= (1<<3); // PB3 = PWM output to servo motor
    DDRB |= (1<<4); // PB4 = Red LED
	DDRB |= (1<<5); // PB5 = Green LED
    DDRC |= (1<<5); // PC5 = buzzer
    DDRC |= (1<<4); // PC4 = TX line output
	PORTC &= ~(1<<4); // PC4 enable line is logical zero, received data will pass through buffer to RX

	/** CONFIGURE USART0 MODULE **/
	UBRR0 = MYUBRR; // set BAUD rate
	UCSR0B |= (1 << TXEN0 | 1 << RXEN0);  // enable RX/TX bits
	UCSR0C = (3 << UCSZ00); //asynch, no parity, 1 stop bit, 8 data bits
	UCSR0B |= (1<<RXCIE0);
	
    /** INITIALIZE TIMERS **/
	timer0_init(); // controls buzzer
    timer1_init(); // controls 'warm' LED
    timer2_init(); // controls PWM/servo dial 
	
    /** ENABLE INTERRUPTS **/
    PCICR |= (1<<PCIE2);
    PCMSK2 |= (1<< PCINT18);
    PCMSK2 |= (1<< PCINT19);
    sei();

	lcd_init(); // Initialize LCD
	ds_init();	// Initialize temperature sensor

    /** LCD SPLASH SCREEN **/
	lcd_writecommand(1);
	lcd_moveto(0,0);
	lcd_stringout("EE109 Project");
	lcd_moveto(1,0);
	lcd_stringout("Carrie Lei");
	_delay_ms(2000);
	lcd_writecommand(1);

	/** DETERMINE INITIAL ROTARY ENCODER VALUES & STATE **/
	unsigned char C = PIND;
	a = C & (1<<2);
	b = C & (1<<3);
    if (!b && !a){
		old_state = 0;
	}
		else if (!b && a){
		old_state = 1;
	}
    else if (b && !a){
		old_state = 2;
	}
    else{
		old_state = 3;
	}

	/** TERMPERATURE CONVERSION **/
    unsigned char t[2];
    if (ds_init() == 0) {    // Initialize the DS18B20
         lcd_stringout("Sensor not working"); // Sensor not responding
    }
    ds_convert();    // Start first temperature conversion

	/** INITIAL EEPROM VALUE **/
	unsigned char x;
	x = eeprom_read_byte((void *) 1000);
	if ((x<50) || (x>90)){
		x = 50;
	}
	threshold = x;
   
    while (1) {
        if (ds_temp(t)) { 
            /** CONVERT & PRINT TEMPERATURE FROM DS18B20 **/
			lcd_writecommand(1);
			unsigned short temperature;
			temperature = t[0];
			temperature |= (unsigned short)t[1]<<8;

			whole_num = (((((temperature * 10) / 16) * 90) / 5) + 3200) / 100;
			int decimal = ((((((temperature * 10) / 16) * 90) / 5) + 3200) /10) % 10;

			unsigned char buff[3];
			snprintf(buff, 3, "%d", whole_num);
			lcd_moveto(0,0);
			lcd_stringout(buff);

			lcd_moveto(0,2);
			lcd_stringout(".");
			
			unsigned char buff1[3];
			snprintf(buff1, 3, "%d", decimal);
			lcd_moveto(0,3);
			lcd_stringout(buff1);

            /** ADJUST SERVO MOTOR ACCORDINGLY **/
			OCR2A = 50 -((3*whole_num)/8);

			/** TRANSMIT TEMP SENSOR VALUE TO SERIAL **/
			unsigned char send_temp[6];
			snprintf(send_temp, 6, "@+%d#", whole_num);
			int i;
			for (i=0; i<6; i++){
				tx_char(send_temp[i]);
			}

			/** CONTROL LEDS ACCORDINGLY **/
			if (whole_num <= threshold){ // cool
				PORTB |= (1<<5);
				PORTB &= ~(1<<4); 
				lcd_moveto(0, 8);
				lcd_stringout("COOL");
				TCCR1B &= ~(1<<CS12);
				flag_buzzed = 0;
			}
			else if ((whole_num> threshold) && (whole_num<threshold+3)){ // warm
				lcd_moveto(0,8);
				lcd_stringout("WARM");		
				TCCR1B |= (1<<CS12); 
				PORTB &= ~(1<<5);
				flag_buzzed = 0;
			}
			else if (whole_num >= threshold+3){ // hot
				PORTB |= (1<<4); 
				lcd_moveto(0, 8);
				lcd_stringout("HOT!");
				PORTC |= (1<<5);
				TCCR1B &= ~(1<<CS12); 
				if (flag_buzzed == 0){
					TCCR0B |= (1<<CS02); // turn on 'warm' timer
				}				
			}
            ds_convert();   // Start next conversion  
        }

        /** CHANGE THRESHOLD ACCORDING TO ENCODER VALUES **/
		if (changed){ 
			changed = 0;
			lcd_moveto(1,0);
			unsigned char buff2[3];
			snprintf(buff2,3,"%d", threshold);
			lcd_stringout(buff2);
			_delay_us(100);
			eeprom_update_byte((void *) 1000, threshold);
		}

        /** CHANGE REMOTE/LOCAL STATE ACCORDING TO BUTTON VALUES **/
		if (local){
			lcd_moveto(1,10);
			lcd_stringout("local");
			OCR2A = 50 -((3*whole_num)/8); // move servo motor/dial according to local temp
		}
		if (remote){
			lcd_moveto(1,10);
			lcd_stringout("remote");
			int remote_num;
			sscanf(temp_buff, "%3d", remote_num);
			OCR2A = 50 -((3*remote_num)/8); // move servo motor/dial according to remote temp
		}

        /** STRINGOUT TEMPERATURE RECEIVED FROM SERIAL **/
		if (data_end){
			data_end = 0;
			lcd_moveto(1,3);
			lcd_stringout(temp_buff);
		}
    }
}

ISR(PCINT1_vect){ // interrupt for button presses 
	if ((PINC&(1<<1))==0){ 
        local = 1;
		remote = 0;
    }	
	if ((PINC&(1<<2))==0){ 
    	remote = 1;
		local =0;
    } 
}

ISR(TIMER0_COMPA_vect){ // interrupt for buzzer
	buzzer_timer++;
	PINC |= (1<<5);
	if (buzzer_timer == 250){
		flag_buzzed = 1;
		PINC &= ~(1<<5);
		TCCR0B &= ~(1 << CS00); 
		TCCR0B &= ~(1 << CS01);
		TCCR0B &= ~(1 << CS02);
		buzzer_timer =0;
	}
}

ISR(TIMER1_COMPA_vect){ // interrupt for flashing red LED
	PORTB ^= (1<<4); 
}

void timer0_init(void){ // timer for buzzer
	TCCR0A |= (1<<WGM02);  
	TIMSK0 |= (1<<OCIE0A);
	OCR0A = 250;
}

void timer1_init(void){ // timer for 'warm' flashing
    TCCR1B |= (1<<3); // Set mode for "Clear Timer on Compare" (CTC)
    TIMSK1 |= (1<<1); // Enable "Output Compare A Match Interrupt"
    OCR1A = 31250; 
}

void timer2_init(void) // timer for servo dial
{
    TCCR2A |= (0b11 << WGM00);  // Fast PWM mode, modulus = 256
    TCCR2A |= (0b10 << COM0A0); // Turn D11 on at 0x00 and off at OCR2A
    OCR2A = 12; // Initial pulse duty cycle of 50%            
    TCCR2B |= (0b111 << CS20);  
}

void tx_char(char ch) { // function to send data
	while ((UCSR0A & (1<<UDRE0)) == 0) {}
	UDR0 = ch;
}