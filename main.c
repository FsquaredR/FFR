/*---------------------------------------------------------------
/   _______             _______
/  |@|@|@|@|           |@|@|@|@|        RobotCode
/  |@|@|@|@|   _____   |@|@|@|@|        Started:      2/26/2012
/  |@|@|@|@| /\_T_T_/\ |@|@|@|@|        Last Updated: 
/  |@|@|@|@||/\ T T /\||@|@|@|@|        
/   ~/T~~T~||~\/~T~\/~||~T~~T\~         By: Us
/    \|__|_| \(-(O)-)/ |_|__|/
/    _| _|    \\8_8//    |_ |_
/  |(@)]   /~~[_____]~~\   [(@)|
/    ~    (  |       |  )    ~
/        [~` ]       [ '~]
/        |~~|         |~~|
/        |  |         |  |
/       _<\/>_       _<\/>_
/      /_====_\     /_====_\
/
/ COMPILE: avr-gcc main.c -o main.elf -Os -DF_CPU=8000000 -mmcu=atmega644 -std=gnu99
/ CONVERT: avr-objcopy -O ihex main.elf main.hex
/ PROGRAM: avrdude -P usb -c avrisp2 -p atmega644P -U flash:w:main.hex
/----------------------------------------------------------------*/

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <math.h>
#include <avr/interrupt.h>
#include "RobotIO.h"

#define F_CPU 8000000UL
#define FOSC 8000000
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

/* Encoders */
short dirL = 0;
long countLF = 0;
long countLR = 0;
short dirR = 0;
long countRF = 0;
long countRR = 0;
																
void init(void){
	/* Init Pins */
	DDRA = 0x00;				// PORTA 0-3 -input (IR Inputs)
	DDRC = 0xAF;				// PORTC 0-3 -output (Motor Outputs) 7 (PING) -output
	DDRD = 0x33;					  // PORTD 0,1 -output(serial) 2,3,6,7 -input (Encoder) 4[R]5[L]-output (PWM)
    DDRB = 0xFF;
	
	/* Init INT */
	//EICRA |= ISC00 || ISC01;		  // Sets INT0 @Rising Edge
	//EICRA |= ISC11 || ISC10;		  // Sets INT1 @Rising Edge
    EIMSK  |= (1 << INT0);
    EICRA |= (1 << ISC00);
    EICRA |= (0 << ISC01);
	
	/* Init PWM */
	TCCR1A = 0x81;              // 8-bit, Non-Inverted PWM
    //TCCR1B = 1;                 // Starts PWM
	
	/* Init ADC */	
    ADCSRA = 0xEB;				// changed to allow interrupt
    ADMUX |= (1 << REFS0);		// AVCC with external capacitor at AREF pin
    ADCSRA |= 0x10;				// clear flag
}

void USART_TransmitString(const char* str)
{
	for (;*str;str++)
	{
		USART_Transmit(*str);
	}
	
}

unsigned char USART_Receive(void)
{
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

void USART_Transmit(unsigned char data)
{
    
	//WAIT FOR EMPTY BUFFER
	while (!(UCSR0A & (1<<UDRE0)));
	
	//Put data on buffer
	UDR0 = data;
    
}

void USART_Init(unsigned int ubrr)
{
	//SET BAUD RATE
	UBRR0H = (unsigned char) (ubrr>>8);
	UBRR0L = (unsigned char) ubrr;
	
	//ENABLE REC - TRA
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	
	//SET Frame Format
	UCSR0C = (3<<1);
	
}

int main(void)
{
    init();
    
    USART_Init(MYUBRR);
    
    LM0(0);
    LM1(0);
    RM0(0);
    RM1(0);
    sei();
    USART_TransmitString((char*)"Starting MIRCO\nPush Button!!\n");
    while(!PBS)
    {
        _delay_ms(1);
    }
    USART_TransmitString((char*)"Button Pushed...\n");
    
	while(1){

        
        // AYERS MOTOR DEMO
        motorFWD();
        _delay_ms(500);
        motorSTP();
        PORTB = 0xFF;
        _delay_ms(2000);
        motorRVS();
        _delay_ms(500);
        motorSTP();
        while(!PBS)
        {
            _delay_ms(1);
        }
    }
}

/* Motor Control Functions */
void setMotorSpeed(char c, int p){		// Set speed with a percentage
	int s = 0;
	
	if(p >= 0 && p <= 100){ 
		s = (p * 255)/100;
	}	
	
	if(c == 'L') OCR1A = s;				// PORTD 5
	if(c == 'R') OCR1B = s;				// PORTD 4
}

void motorFWD(){				
	LM0(1);
	LM1(0);
	RM0(0);
	RM1(1);	
}

void motorRVS(){				
	LM0(0);
	LM1(1);
	RM0(1);
	RM1(0);	
}

void motorSTP(){
	LM0(0);
	LM1(0);
	RM0(0);
	RM1(0);
}

/* PING Functions */ // Needs to be tested
#define SAMPLERATE 40

int ping_cm(){
  int avg = 0;
  int sum = 0;
  
  for(int i = 0; i < (SAMPLERATE/2); i++){
    sum += pingRead();
    _delay_ms(5);	
  }
  avg = (sum/(SAMPLERATE/2));
  
  return (avg /29 /2);		// Needed to be recalabrated... maybe... probably not though
}

int pingRead(){
  int onTime = 0;
  
  FLIPBIT(PORTC,8);             
  ULTRAW(0);				    // Pulse The Ping Sensor 
  _delay_us(2);
  ULTRAW(1);
  _delay_us(5);
  ULTRAW(0);
  FLIPBIT(PORTC,8);
  
  onTime = pulseInHighUltra();	// Collect Data 
  
  return onTime;  
}

int pulseInHighUltra(){
	int count = 0;
	
	while(ULTRAR){ 					//Pulse is high
		count++;
		_delay_us(1);
	}
	
	 return count;
}

/*
ISR(INT0_vect) {
    PORTB = 0xFF;
    
    //TODO: Read interrupt
    if(LENC1){
        PORTB = 0xFF;
        //LED(1);
    }else{
        PORTB = 0x00;
        
        //LED(0);
    }
}

*/
ISR(INT0_vect){
    
    if(LENC0){
        if(!LENC1){          //Checks other right encoder
            dirL = 1;
            countLF++;
            PORTB = 0xFF;
        }else{
            dirL = -1;
            countLR++;
            PORTB = 0x00;
        }
    }
    
}

ISR(INT1_vect){
    if(RENC0){
        if(!RENC1){          //Checks other right encoder
            dirR = 1;
            countRF++;
        }else{
            dirR = -1;
            countRR++;
        }
    }
}

void go(int d, short dirR, short dirL){			// Assuming overall encoders get cleared ( d in increments )
	unsigned int dismvR = 0;     // incs.
	unsigned int dismvL = 0;
    
	if(dirR = 1){
		unsigned int ENCR_p = countRF;		// countR0 increments on forward movment
	}else if(dirR = -1){
		unsigned int ENCR_p = countRR;		// countR1 increments on reverse movment
	}
	
	if(dirL = 1){
		unsigned int ENCL_p = countLF;
	}else if(dirL = -1){
		unsigned int ENCL_p = countLR;
	}
	
	while(dismvR < dirR && dismvL < dirL){
		if(dismvR < dirR){
			if(dirR = 1) motorR(F); else if(dirR = -1) motorR(R);
		}else{
			motorR(S);
		}
		if(dismvL < dirL){
			if(dirL = 1) motorL(F); else if(dirL = -1) motorL(R);
		}else{
			motorL(S);
		}
		
		dismvR = countRF - ENCR_p;
		dismvL = countLF - ENCL_p;
	}
}