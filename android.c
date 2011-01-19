#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define FOSC 16000000
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1



//function prototypes
void uart_init(void); //initiating usart communication, setting the correct bits
void pwm_init(void); //initiating PWM-timer
static int uart_putchar(char c, FILE *stream); //the putchar function, sends data
uint8_t uart_getchar(void); //getchar function, collects data

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

//variables
volatile uint16_t ReceivedByte; 






int main(void){


//initiating uart and pwm
uart_init();
pwm_init();



//forever-running loop
while(1){
	OCR1A = ICR1*1.5/20;
	





}

}
//end of main











//ISR and functions

ISR(USART_RX_vect){

   
   ReceivedByte = UDR; // Fetch the recieved byte value into the variable "ByteReceived" 
    
}


void uart_init(void){

	UCSRB |= (1<<RXEN); //enable rx (recieve)
	UCSRB |= (1<<TXEN); //enable tx (transmit)

	UCSRC |= (1<<UCSZ1); //enables 8bit-transfer
	UCSRC |= (1<<UCSZ0);

	UBRRL = MYUBRR; //loads the baud_prescale-value for 9600 baud
	UBRRH = (MYUBRR >> 8);

	UCSRB |= (1<<RXCIE); //enable the 

	sei();

	DDRD = 0b11111110; //setting RXD to input, all others output
	stdout = &mystdout; //setting printf init

}

void pwm_init(void){

	
	TCCR1B |= (1<<WGM13) | (1<<WGM12);
	TCCR1A |= (1<<WGM11);
	
	TCCR1B |= (1<<CS11);
	TCCR1A |= (1<<COM1A1);
	
	//TOP = 16,000,000/(8*50)-1 = 39999
	ICR1 = 39999;

	//setting all PORTB to output
	DDRB = 0b11111111;

}


int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = c;
    
    return 0;
}

uint8_t uart_getchar(void)
{
    while( !(UCSRA & (1<<RXC)) );
    return(UDR);
}


