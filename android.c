#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define FOSC 10000000
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
#define BUFFER_SIZE 9

//function prototypes
void uart_init(void); //initiating usart communication, setting the correct bits
void pwm_init(void); //initiating PWM-timer
static int uart_putchar(char c, FILE *stream); //the putchar function, sends data
void servo_parse(void);
void pin_irq_enable(void);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

//variables
volatile char buff[BUFFER_SIZE];
//volatile char ReceivedByte;
volatile int indx, flag;



int main(void){
	_delay_ms(500);

  

  
//initiating uart and pwm
uart_init();
pwm_init();
pin_irq_enable();


//forever-running loop
while(1){


	servo_parse(); //servo-parsing function
	
	

} 

}
//end of main




//ISR and functions
ISR(USART_RX_vect){
	
	
	
	if(buff[0] != '%') //check start char, if not equal, reset for another try
		indx = 0;
	
	buff[indx++] = UDR0;//filling buffer, one byte at a time.
	
	if(indx>= BUFFER_SIZE){ //when buffer is filled (index bigger or equal to buffer size defined in header)
		flag = 1;				//set finish flag to 1 to allow reading of datastring, and reset indx to start over
		indx = 0;				//for new transfer
	}
	
}

ISR(INT0_vect){
	printf("0xFF");
}


void uart_init(void){

	UCSR0B |= (1<<RXEN0); //enable rx (recieve)
	UCSR0B |= (1<<TXEN0); //enable tx (transmit)

	UCSR0C |= (1<<UCSZ01); //enables 8bit-transfer
	UCSR0C |= (1<<UCSZ00);

	UBRR0L = MYUBRR; //loads the baud_prescale-value for 9600 baud
	UBRR0H = (MYUBRR >> 8);

	UCSR0B |= (1<<RXCIE0); //enable the usart recieve interrupt

	sei();

	DDRD = 0b00111110; //setting RXD to input, all others output
	stdout = &mystdout; //setting printf init

}

void pwm_init(void){

	ICR1 = 12500;
	TCCR1B |= (1<<WGM13);
	
	TCCR1B |= (1<<CS11);
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1); 
	
	//TOP = 16,000,000/(8*50)-1 = 39999
	

	//setting all PORTB to output
	DDRB = 0b11111111;

}

void servo_parse(void){
	uint8_t k = 0;
	uint8_t xDeg, yDeg = 0;
	char xVal[4], yVal[4];
	
		if(buff[4] != '$') //check the data in buffer
			flag = 0;
		
		
		if(flag==1){ // if new data is available, proceed. 
			k = 0; 
		cli(); //disable uart-interrupt, dont want to be interrupted with new data when parsing
		
		
		while(buff[k+1] != '$'){ //parse through the data until we see the first separator
			xVal[k] = buff[k+1];
			k++;
		}
	
		k=0; //reset the counter
		
		
		while(buff[k+5] != '#'){ //parse through the data until we see the second separator
			yVal[k] = buff[k+5];
			k++;
		}
		
	
		xDeg = atoi(xVal); 
		yDeg = atoi(yVal);
		//printf("%c - %c", xDeg, yDeg); // for debug purposes
		
		OCR1A = 625 + xDeg*3;
		OCR1B = 625 + yDeg*3;
	
		flag = 0; //set flag to false, ready to read new data
		}
	
		
		sei(); //enable interrupt, ready for new data 
	
	
}

void pin_irq_enable(void){
	
	//enable pullup
	PORTD |= (1<<7) | (1<<6);
	
	//enable interrupt on PD6 and PD7
	PCMSK1 |= (1<<4) | (1<<5);
	
	//enable interrupt on falling edge
	EICRA |= (1<<ISC01);
	//turn on interrupts
	EIMSK |= (1<<INT0);
}


int uart_putchar(char c, FILE *stream){
    if (c == '\n') uart_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    
    return 0;
}



