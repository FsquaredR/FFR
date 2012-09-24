/*---------------------------------------------------------------
/   _______             _______
/  |@|@|@|@|           |@|@|@|@|        RobotCode
/  |@|@|@|@|   _____   |@|@|@|@|        Started:      2/26/2012
/  |@|@|@|@| /\_T_T_/\ |@|@|@|@|        Last Updated: 9/24/2012
/  |@|@|@|@||/\ T T /\||@|@|@|@|        
/   ~/T~~T~||~\/~T~\/~||~T~~T\~         By: Andrew & Mike
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
*			FWD: 1    1      1     1
*			RVS: 0    0      0     0
*			LFT: 0    0      1     1
*			RGT: 1    1      0     0
*			STP: 0    1      1     0
*
*
*/

/* Macros For Bitsetting  */
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT)) 
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT)) 
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT)) 
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

/* Pin Declairations */
	// Sensors
#define ULTRA(x) (x) ? SETBIT(PORTC, 7) : CLEARBIT(PORTC, 7);
#define RF_IR 0
	// Fire I/O
#define FAN(x) (x) ? SETBIT(PORTC, 4) : CLEARBIT(PORTC, 4);
	// Motors
#define LM0(x) (x) ? SETBIT(PORTA, 0) : CLEARBIT(PORTA, 0);		
#define LM1(x) (x) ? SETBIT(PORTA, 1) : CLEARBIT(PORTA, 1);		 
#define RM0(x) (x) ? SETBIT(PORTA, 2) : CLEARBIT(PORTA, 2);		
#define RM1(x) (x) ? SETBIT(PORTA, 3) : CLEARBIT(PORTA, 3);		
																
void init(void){
	/* Init Pins */
	DDRA = 0x0F;				// PORTA 0-3 -output (Motor Controls)
	DDRC = 0x00;				// PORTC 0-3 -input (Encoder Feedback)
	
	/* Init INT */
	PCMSK1 = 0x80;				// Setting interrupts to pins 22-25
	PCMSK2 = 0x03;
	EICRA = 0x05;				// Setting interrupts
	
	/* Init PWM */
	DDRD = 0x30;			   	// PORTD 4[Right] 5[Left] -output (PWM)	
	TCCR1A = 0x81;              // 8-bit, Non-Inverted PWM
    TCCR1B = 1;                 // Starts PWM
	
	/* Init ADC */
    ADMUX |= (1 << REFS1);      // AVCC with external capacitor at AREF pin
    ADCSRA = 0xEB;				// changed to allow interrupt
    while(!(ADCSRA & 0x10)) asm volatile ("nop"::); // if still converting, wait
    ADCSRA |= 0x10;				// clear flag
}
	
int adcTest = 0;

int main(void)
{
    init();
	setMotorSpeed(100);
	motorSTP();
	while(1){
		adcTest = readADC(RF_IR);
		setMotorSpeed('L', ((adcTest * 100) / 1024));
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
	LM1(1);
	RM0(1);
	RM1(1);	
}

void motorRVS(){				
	LM0(0);
	LM1(0);
	RM0(0);
	RM1(0);	
}

void motorSTP(){
	LM0(0);
	LM1(1);
	RM0(0);
	RM1(1);
}

/* ADC Functions */
int readADC(int channel){		// PORT A

    ADMUX |= channel;			// read from specified channel
    ADMUX |= (1 << REFS1);		// DIV: 1.1v
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