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
//#include "IRdata.h"

#define FOSC 8000000
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
#define SAMPLERATE 40

/* Encoders */
volatile short dirL = 0;
volatile long countLF = 0;
volatile long countLR = 0;
volatile short dirR = 0;
volatile long countRF = 0;
volatile long countRR = 0;

/* PROTOTYPES */
void motorSTP(void);
long readIR(int);
int readADC(int);

volatile uint16_t adc_value;
volatile uint8_t  bool_adc_done;

																
void init(void){
	/* Init Pins */
	DDRA = 0x00;				// PORTA 0-3 -input (IR Inputs)
	DDRC = 0x7F;				// PORTC 0-3 -output (Motor Outputs) 7 (PING) -output
	DDRD = 0x32;				// PORTD 0,1 -output(serial) 2,3,6,7 -input (Encoder) 4[R]5[L]-output (PWM)
    DDRB = 0xFF;
	
	/* Init INT */
	EIMSK = (1 << INT0)|(1 << INT1);
    EICRA |= (1 << ISC00)|(1 << ISC01);
    EICRA |= (1 << ISC10)|(1 << ISC11);

	/* Init PWM */
    TCCR0A = (1<<COM0A1)|(1<<COM0B1)|(1<<WGM00)|(1<<WGM01);
    TCCR0B = (1<<CS00);
	OCR0A = 10;
    OCR0B = 200;
    
    
	/* Init ADC */	
    //ADCSRA = 0xEB;				// changed to allow interrupt
    //ADMUX |= (1 << REFS0);		// AVCC with external capacitor at AREF pin
    //ADCSRA |= 0x10;				// clear flag
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz 
   	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC 
   	ADCSRA |= (1 << ADEN);  // Enable ADC 
   	ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt 


   





}
/*
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
*/

int main(void)
{
    init();
    /*
    USART_Init(MYUBRR);
    */
    //motorSTP();
    sei();
	readADC(1);
	while(1)
	{
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		readADC(2);
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

/* PING Functions */ // Needs to be tested
/*
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
*/
ISR(INT0_vect){
    
    if(LENC0){
        if(!LENC1){          //Checks other right encoder
            dirL = 1;
            countLF++;
        }else{
            dirL = -1;
            countLR++;
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

ISR(ADC_vect) 
{ 
	adc_value = ADC;
	ADCSRA &= ~(1<<ADSC); // TURN OFF ADC
	ADCSRA &= ~(1<<ADIE);
	bool_adc_done = 1;
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
}*/

/* ADC Functions */
int readADC(int channel){		// PORT A
	bool_adc_done = 0;
	ADMUX  |= channel;
	ADCSRA |= (1 << ADIE);
	ADCSRA |= (1 << ADSC);
	while(bool_adc_done==0) asm("nop"); //Block till conversion's done
}

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
