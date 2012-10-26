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
#include "IRdata.h"

#define FOSC 8000000
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
<<<<<<< HEAD
#define SAMPLERATE 40
=======

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
#define LED(x) (x) ? SETBIT(PORTC, 6) : CLEARBIT(PORTC, 6);
#define PBS CHECKBIT(PINC, PC7)
	// Motors
#define LM0(x) (x) ? SETBIT(PORTC, 0) : CLEARBIT(PORTC, 0);		
#define LM1(x) (x) ? SETBIT(PORTC, 1) : CLEARBIT(PORTC, 1);		 
#define RM0(x) (x) ? SETBIT(PORTC, 2) : CLEARBIT(PORTC, 2);
#define RM1(x) (x) ? SETBIT(PORTC, 3) : CLEARBIT(PORTC, 3);
	// Encoders
#define RENC0 CHECKBIT(PIND, PC3) // gets interupt INT1
#define RENC1 CHECKBIT(PIND, PC7)
#define LENC0 CHECKBIT(PIND, PC2) // gets interupt INT0
#define LENC1 CHECKBIT(PIND, PC6)
>>>>>>> Added PWM

/* Encoders */
short dirL = 0;
long countLF = 0;
long countLR = 0;
short dirR = 0;
long countRF = 0;
long countRR = 0;
																
void init(void){
	/* Init Pins */
	DDRA = 0x00;
    DDRB = 0xFF;
	DDRC = 0x7F;
	DDRD = 0x32;
	
	/* Init INT */
	//EICRA |= ISC00 || ISC01;		  // Sets INT0 @Rising Edge
	//EICRA |= ISC11 || ISC10;		  // Sets INT1 @Rising Edge
    EIMSK |= (1 << INT0);
    EICRA |= (1 << ISC00);
    EICRA |= (0 << ISC01);
	
	/* Init PWM */
    // FIX!!
	
	/* Init ADC */	
    ADCSRA  = 0xEB;				// changed to allow interrupt
    ADMUX  |= (1 << REFS0);		// AVCC with external capacitor at AREF pin
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

<<<<<<< HEAD
=======
void setPWM1(uint16_t c)
{
    OCR1AH = ((0xFF00 & c) >> 0x08);
    OCR1AL = (0x00FF & c);
}

void setPWM2(uint16_t c)
{
    OCR1BH = (0xFF00 & c) >> 0x08;
    OCR1BL = (0x00FF & c);
}
>>>>>>> Added PWM

int main(void)
{
    init();
    
    USART_Init(MYUBRR);
    
<<<<<<< HEAD
    motorSTP();
    sei();
=======
    uint16_t pwmV1 = 0xFFFF;
    uint16_t pwmV2 = 0xFFFF;
    
    // PWM 
    TCCR1A = ( (1<<COM1B1)  | (1<<COM1A1) | (1<<WGM10) | (1<<WGM11));
    TCCR1B = ((1<<CS11) | (1<<CS10));
    OCR1AH = (0xFF00 & pwmV1) >> 0x08;
    OCR1AL = (0x00FF & pwmV1); //75% PWM
    OCR1BH = (0xFF00 & pwmV2) >> 0x08;
    OCR1BL = (0x00FF & pwmV2); //75% PWM

    
    LM0(0);
    LM1(0);
    RM0(0);
    RM1(0);
    //sei();
>>>>>>> Added PWM
    USART_TransmitString((char*)"Starting MIRCO\nPush Button!!\n");
    while(PBS)
    {
        _delay_ms(1);
    }
    USART_TransmitString((char*)"Button Pushed...\n");
    
	while(1){
<<<<<<< HEAD
        while(!PBS){
            _delay_ms(1);
        }
        double temp = readIR(LF_IR);
        USART_Transmit(temp);
    }
}

/* Motor Control Functions */ //Funky needs checking
=======
        
        setPWM1((uint16_t)32000); // 16 bit: Only values 0xFFFF and 32000 make sense?
        setPWM2((uint16_t)32000); // 0xFFFF = Full Speed  :  32000 = Half-ish Speed

        motorFWD();
        _delay_ms(2000);
        motorSTP();
        _delay_ms(1000);
        motorRVS();
        _delay_ms(400);
        motorSTP();
        _delay_ms(1000);
        while(PBS)
        {
            _delay_ms(1);
        }

    }
}





/* Motor Control Functions // DO NOT USE
>>>>>>> Added PWM
void setMotorSpeed(char c, int p){		// Set speed with a percentage
	int s = 0;
	
	if(p >= 0 && p <= 100){ 
		s = (p * 255)/100;
	}	
	
	if(c == 'L') OCR1A = s;				// PORTD 5
	if(c == 'R') OCR1B = s;				// PORTD 4
} */

<<<<<<< HEAD
/* PING Functions */ // Needs to be tested
=======
void motorFWD(){				
	LM0(0);
	LM1(1);
	RM0(1);
	RM1(0);
}

void motorRVS(){				
    LM0(1);
	LM1(0);
	RM0(0);
	RM1(1);
}

void motorSTP(){
	LM0(0);
	LM1(0);
	RM0(0);
	RM1(0);
}

/* IR Equations
 LF: cm=33097.0 * (x)^(-1.322)
 RF: cm=19799.0 * (x)^(-1.243)
 LR: cm=03440.3 * (x)^(-1.078)
 RR: cm=03790.6 * (x)^(-1.088)
 */

double readIR(int x){
    double adc_data = 0;
    for(unsigned char i=0; i<40; i++)
    {
        adc_data += (double)readADC(x);
    }
    adc_data = adc_data/40;
    
    switch (x) {
        case LF_IR:
            return (double)(33097.0 * pow( adc_data, (double)-1.322));
            break;
        case RF_IR:
            return (double)(19799.0 * pow( adc_data, (double)-1.243));
            break;
        case LR_IR:
            return (double)(03440.3 * pow( adc_data, (double)-1.078));
            break;
        case RR_IR:
            return (double)(03790.6 * pow( adc_data, (double)-1.088));
            break;
        default:
            break;
    }
    return -1; // RETURN WITH ERROR
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

/* PIMG Functions */
#define SAMPLERATE 40
>>>>>>> Added PWM

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
<<<<<<< HEAD
/*
void go(int d, short dirR, short dirL){			// Assuming overall encoders get cleared ( d in increments )
=======

/*void go(int d, short dirR, short dirL){			// Assuming overall encoders get cleared ( d in increments )
>>>>>>> Added PWM
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
<<<<<<< HEAD
}*/
/* IR Equations
 LF: cm=33097.0 * (x)^(-1.322)
 RF: cm=19799.0 * (x)^(-1.243)
 LR: cm=03440.3 * (x)^(-1.078)
 RR: cm=03790.6 * (x)^(-1.088)
 */

 // Returned value in mm
int readIR(int x){
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

void motorFWD(void){				
	LM0(1);
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

void motorSTP(void){
	LM0(0);
	LM1(0);
	RM0(0);
	RM1(0);
}
=======
}*/
>>>>>>> Added PWM
