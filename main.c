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
#include <util/atomic.h>
#include <string.h>
#include <math.h>
#include <avr/interrupt.h>
#include "RobotIO.h"
//#include "IRdata.h"

#define FOSC 8000000
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
#define SAMPLERATE 10

/* Encoder Varriables*/
short leftDirection = 0;	//1 = forward : -1 = Reverse
int count_LeftFWD = 0;
int count_LeftRVS = 0;
short rightDirection = 0;	//1 = forward : -1 = Reverse
int count_RightFWD = 0;
int count_RightRVS = 0;

/* Prototypes*/
void motorSTP(void);

int ping_cm(void);
long pingRead(void);

long readIR(int);
int readADC(int);


unsigned char USART_Receive(void);
void USART_TransmitString(const char* str);
void USART_Transmit(unsigned char data);
void USART_Init(unsigned int ubrr);

volatile uint16_t adc_value;
volatile uint8_t  bool_adc_done;
volatile uint8_t  bool_led;

																
void init(void){
	/* Init Pins */
	DDRA = 0x00;		// PORTA 0-3 -input (IR Inputs)
	DDRC = 0x7F;		// PORTC 0-3 -output (Motor Outputs) 7 (PING) -output
	DDRD = 0x32;		// PORTD 0,1 -output(serial) 2,3,6,7 -input (Encoder) 4[R]5[L]-output (PWM)
    DDRB = 0xFF;
	
	/* Init INT */
    EIMSK  = (1 << INT0)  | (1 << INT1);
    EICRA |= (1 << ISC00) | (1 << ISC01);
    EICRA |= (1 << ISC10) | (1 << ISC11);
	
	/* Init PWM */
    TCCR0A = (1<<COM0A1)|(1<<COM0B1)|(1<<WGM00)|(1<<WGM01);
    TCCR0B = (1<<CS00);
	OCR0A = 10;
    OCR0B = 10;
    
    /* Init ADC */
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz 
   	ADMUX  |= (1 << REFS0); // Set ADC reference to AVCC 
   	ADCSRA |= (1 << ADEN);  // Enable ADC 
   	ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt/**/

   	/* Init timer1 */
   	TIMSK1 |= (1 << TOIE1); // Enable overflow interrupt 
	TCNT1 = 49911; // Preload timer with precalculated value 
	TCCR1B |= ((1 << CS10) | (1 << CS11)); // Set up timer at Fcpu/64 
	bool_led = 1;
}

int main(void)
{
    init();
    
    USART_Init(MYUBRR);

    sei();
    LED(0);
    USART_TransmitString((char*)"Micro Started: Push Button\n");
    while(PBS)
    {
        _delay_ms(1);
    }
    LED(1);
    USART_TransmitString((char*)"Button Pushed\n");
    uint8_t i = 0;
    char buffer[10];
	while(1){
		memset(buffer,0,10);
		itoa(readADC(0), buffer, 10);
		OCR0A = 10;
		OCR0B = 10;

		USART_TransmitString(buffer);
		/*if ((readADC(0) > 300)||(readADC(1) > 300)||(readADC(2) > 300)||(readADC(3) > 300)) {
			motorFWD();
		} else {
			motorSTP();
		}*/
    }
}

/* Motor Control Functions */ //Funky needs checking
void setMotorSpeed(char c, int p){		// Set speed with a percentage
	int s = 0;
	
	if(p >= 0 && p <= 100){ 
		s = (p * 255)/100;
	}	
	
	if(c == 'L') OCR1A = s;				// PORTD 5
	if(c == 'R') OCR1B = s;				// PORTD 4
}

/*----------PING Sensor-----------*/
int ping_cm(){
   long avg = 0;
   long sum = 0;
  
  for(int i = 0; i < (SAMPLERATE); i++){
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		sum += pingRead();
		_delay_ms(1);	
	}    
  }
  avg = (sum/(SAMPLERATE));

  return (avg /29 /2);		// Needed to be recalibrated... maybe... probably not though
}

 long pingRead(){	
	long count = 0;

	ULTRA_OUT;             
	ULTRA_WRITE(0);				    // Pulse The Ping Sensor 
	_delay_us(2);
	ULTRA_WRITE(1);
	_delay_us(5);
	ULTRA_WRITE(0);
	ULTRA_IN;

	while(!ULTRA_READ){}
	while(ULTRA_READ){
		count++;
		_delay_us(1);
	}
	return count;  
}

/*----------------ADC Functions-------------*/
int readADC(int channel){		// PORT A
	bool_adc_done = 0;
	ADMUX  &= 0xE0;
	ADMUX  |= channel;
	ADCSRA |= (1 << ADIE);
	ADCSRA |= (1 << ADSC);
	while(bool_adc_done==0) asm("nop"); //Block till conversion's done
	return adc_value;
}

/*----------Interrupts------------*/
ISR(INT0_vect){
    if(LENC0){
        if(LENC1){          //Checks other right encoder
            leftDirection = 1;
            count_LeftFWD++;
        }else{
            leftDirection = -1;
            count_LeftRVS++;
        }
    }
    
}

ISR(INT1_vect){
    if(RENC0){
        if(!RENC1){          //Checks other right encoder
            rightDirection = 1;
            count_RightFWD++;
        }else{
            rightDirection = -1;
            count_RightRVS++;
        }
    }
}

ISR(ADC_vect){ 
	adc_value = ADC;
	ADCSRA &= ~(1<<ADSC); // TURN OFF ADC
	ADCSRA &= ~(1<<ADIE);
	bool_adc_done = 1;
} 

ISR(TIMER1_OVF_vect){ 
	if(bool_led) {
		LED(1);
		bool_led = 0;
	} else {
		LED(0);
		bool_led = 1;
	}
   TCNT1  = 49911; // Reload timer with precalculated value 
}

/*----------Serial Comm------------*/
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

/*----------Motor Control------------*/
void motorFWD(void){				
	LM0(1);
	LM1(0);
	LM1(0);
	RM0(0);
	RM1(1);	
}

void motorRVS(void){				
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
/*
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
}*/
/* IR Equations
 LF: cm=33097.0 * (x)^(-1.322)
 RF: cm=19799.0 * (x)^(-1.243)
 LR: cm=03440.3 * (x)^(-1.078)
 RR: cm=03790.6 * (x)^(-1.088)
 */

 // Returned value in (10^-4)m
/*long readIR(int x){
    int adc_data = 0;
    for(unsigned char i=0; i<SAMPLERATE; i++)
    {
        adc_data += readADC(x);
    }
    adc_data = adc_data/SAMPLERATE;
    
    switch (x) {
        case LF_IR:
            return table_LF_IR[adc_data];
            break;
        case RF_IR:
            return table_RF_IR[adc_data];
            break;
        case LR_IR:
            return table_LR_IR[adc_data];
            break;
        case RR_IR:
            return table_RR_IR[adc_data];;
            break;
        default:
            break;
    }
    return -1; // RETURN WITH ERROR
}/**/