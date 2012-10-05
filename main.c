<<<<<<< HEAD
/*---------------------------------------------------------------
/   _______             _______
/  |@|@|@|@|           |@|@|@|@|        RobotCode
/  |@|@|@|@|   _____   |@|@|@|@|        Started:      2/26/2012
/  |@|@|@|@| /\_T_T_/\ |@|@|@|@|        Last Updated: 9/24/2012
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
/----------------------------------------------------------------*/

/*
 * FFR_1-31-12.c
 *
 * Created: 2/12/2012 2:09:57 PM
 *  Author: Andrew
 */ 

#include <avr/io.h>
#include <util/delay.h>
/*	Notes:
*		Robot Motor controls:
*			    LM0  LM1  |  RM0  RM1
*			FWD: 0    1      1     1
*			RVS: 1    0      0     0
*			LFT: 0    0      1     1
*			RGT: 1    1      0     0
*			STP: 0    1      1     0
*/

/* Macros For Bitsetting  */
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT)) 
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT)) 
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT)) 
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

/* Pin Declairations */
	// Sensors
#define ULTRAW(x) (x) ? SETBIT(PORTC, 7) : CLEARBIT(PORTC, 7);
#define ULTRAR CHECKBIT(PINC, PC7)
#define RF_IR 0
#define RR_IR 1
#define LF_IR 2
#define LR_IR 3
	// Fire I/O
#define FAN(x) (x) ? SETBIT(PORTC, 4) : CLEARBIT(PORTC, 4);
#define LED(x) (x) ? SETBIT(PORTC, 5) : CLEARBIT(PORTC, 5);
#define PBS CHECKBIT(PINC, PC6)
	// Motors
#define LM0(x) (x) ? SETBIT(PORTC, 0) : CLEARBIT(PORTC, 0);		
#define LM1(x) (x) ? SETBIT(PORTC, 1) : CLEARBIT(PORTC, 1);		 
#define RM0(x) (x) ? SETBIT(PORTD, 7) : CLEARBIT(PORTD, 7);
#define RM1(x) (x) ? SETBIT(PORTD, 6) : CLEARBIT(PORTD, 6);
																
void init(void){
	/* Init Pins */
	DDRA = 0x00;				// PORTA 0-3 -input (IR Inputs)
	DDRC = 0x8F;				// PORTC 0-3 -output (Motor Outputs) 7 (PING) -output
	DDRD = 0xF0;				// PORTD 0-3 -input (Encoder Feedback) 4[Right] 5[Left] -output (PWM)
	
	/* Init INT */
	// Add external Interupts
	
	/* Init PWM */
	TCCR1A = 0x81;              // 8-bit, Non-Inverted PWM
    TCCR1B = 1;                 // Starts PWM
	
	/* Init ADC */	
    ADCSRA = 0xEB;				// changed to allow interrupt
    ADMUX |= (1 << REFS0);		// AVCC with external capacitor at AREF pin
    ADCSRA |= 0x10;				// clear flag
}

int main(void)
{
    init();
    
    LM0(0);
    LM1(0);
    RM0(0);
    RM1(0);
    
    while(!PBS)
    {
        _delay_ms(1);
    }
    
    
	while(1){
        motorFWD();
        _delay_ms(1000);
        motorSTP();
        _delay_ms(1000);
        motorRVS();
        _delay_ms(1000);
        motorSTP();
        _delay_ms(1000);
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

/* ADC Functions */
int readADC(int channel){		// PORT A

	ADMUX |= channel;			// read from specified channel
    ADMUX |= (1 << REFS0);		// AVCC with external capacitor at AREF pin
	ADCSRA = 0xEB;				// changed to allow interrupt
    while(!(ADCSRA & 0x10)) asm volatile ("nop"::); //if still converting, wait
    ADCSRA |= 0x10;				// clear flag
		
    return ADC;
}

/* Encoder Functions */
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
=======
/*---------------------------------------------------------------
/   _______             _______
/  |@|@|@|@|           |@|@|@|@|        RobotCode
/  |@|@|@|@|   _____   |@|@|@|@|        Started:      2/26/2012
/  |@|@|@|@| /\_T_T_/\ |@|@|@|@|        Last Updated: 9/24/2012
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
/----------------------------------------------------------------*/

/*
 * FFR_1-31-12.c
 *
 * Created: 2/12/2012 2:09:57 PM
 *  Author: Andrew
 */ 

#include <avr/io.h>
#include <util/delay.h>
/*	Notes:
*		Robot Motor controls:
*			    LM0  LM1  |  RM0  RM1
*			FWD: 0    1      1     1
*			RVS: 1    0      0     0
*			LFT: 0    0      1     1
*			RGT: 1    1      0     0
*			STP: 0    1      1     0
*/

/* Macros For Bitsetting  */
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT)) 
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT)) 
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT)) 
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

#define FOSC 1000000
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

/* Pin Declairations */
	// Sensors
#define ULTRA(x) (x) ? SETBIT(PORTC, 7) : CLEARBIT(PORTC, 7);
#define RF_IR 0
#define RR_IR 1
#define LF_IR 2
#define LR_IR 3
	// Fire I/O
#define FAN(x) (x) ? SETBIT(PORTC, 4) : CLEARBIT(PORTC, 4);
#define LED(x) (x) ? SETBIT(PORTC, 5) : CLEARBIT(PORTC, 5);
#define PBS CHECKBIT(PINC, PC6)
	// Motors
#define LM0(x) (x) ? SETBIT(PORTC, 0) : CLEARBIT(PORTC, 0);		
#define LM1(x) (x) ? SETBIT(PORTC, 1) : CLEARBIT(PORTC, 1);		 
#define RM0(x) (x) ? SETBIT(PORTD, 7) : CLEARBIT(PORTD, 7);
#define RM1(x) (x) ? SETBIT(PORTD, 6) : CLEARBIT(PORTD, 6);
																
void init(void){
	/* Init Pins */
	DDRA = 0x00;				// PORTA 0-3 -input (IR Inputs)
	DDRC = 0x0F;				// PORTC 0-3 -output (Motor Outputs)
	DDRD = 0xF0;				// PORTD 0-3 -input (Encoder Feedback) 4[Right] 5[Left] -output (PWM)
	
	/* Init INT */
	// Add external Interupts
	
	/* Init PWM */
	TCCR1A = 0x81;              // 8-bit, Non-Inverted PWM
    TCCR1B = 1;                 // Starts PWM
	
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

void initUSART(unsigned int ubrr)
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
    
    initUSART(MYUBRR);
    
    LM0(0);
    LM1(0);
    RM0(0);
    RM1(0);
    
    while(!PBS)
    {
        _delay_ms(1);
    }
    
    
	while(1){
        motorFWD();
        _delay_ms(1000);
        motorSTP();
        _delay_ms(1000);
        motorRVS();
        _delay_ms(1000);
        motorSTP();
        _delay_ms(1000);
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

/* ADC Functions */
int readADC(int channel){		// PORT A

	ADMUX |= channel;			// read from specified channel
    ADMUX |= (1 << REFS0);		// AVCC with external capacitor at AREF pin
	ADCSRA = 0xEB;				// changed to allow interrupt
    while(!(ADCSRA & 0x10)) asm volatile ("nop"::); //if still converting, wait
    ADCSRA |= 0x10;				// clear flag
		
    return ADC;
}

/* Encoder Functions */
/*
int ping_cm(){
  long avg = 0;
  long sum = 0;
  
  for(int i = 0; i < (SAMPLERATE/2); i++){
    sum += pingRead();
    _delay_ms(5);	
  }
  avg = (sum/(SAMPLERATE/2));
  
  return (avg /29 /2);
}

int pingRead(){
  long onTime = 0;
  
  DDRC = 0x8F;                // Pulse The Ping Sensor 
  ULTRA(0);
  _delay_us(2);
  ULTRA(1);
  _delay_us(5);
  ULTRA(0);
  
  DDRC = 0x0F;               // Collect Data 
  onTime = pulseInH(0x80);
  
  return onTime;  
}

long pulseInH(int p){
	long count = 0;
	
	while(CHECKBIT(PORTC, 7)){ //Pulse is high
		count++;
		_delay_us(1);
	}
	
	 return count;
}
*/
>>>>>>> Added serial
