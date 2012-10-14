//
//  portc_test.c
//  
//
//  Created by Tomasz Walczak on 10/13/12.
//
//

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <math.h>
#include <avr/interrupt.h>

void main(void)
{
    DDRA = 0xFF;
    DDRB = 0xFF;
    DDRC = 0xFF;
    DDRD = 0xFF;
    
    uint16_t pwmV1 = 0xFFFF;
    uint16_t pwmV2 = 0xFFFF;
    
    // 10 BIT PHASE CORRECT PWM, PRESCALER 64, CLEAR OC1A PIN ON COPARE MATCH
    
    TCCR1A = ( (1<<COM1B1)  | (1<<COM1A1) | (1<<WGM10) | (1<<WGM11));
    TCCR1B = ((1<<CS11) | (1<<CS10));
    //TCCR0A = (1<<COM0A1) | (1<<WGM00);
    //TCCR0B = (1<<WGM02) | (1<<CS00)| (1<<CS01);
    //OCR0A   = 0xC0; //75% PWM
    //OCR0B   = 0xC0; //75% PWM
    OCR1AH = (0xFF00 & pwmV1) >> 0x08;
    OCR1AL = (0x00FF & pwmV1); //75% PWM
    OCR1BH = (0xFF00 & pwmV2) >> 0x08;
    OCR1BL = (0x00FF & pwmV2); //75% PWM

    setPWM1(0xFFFF);
    setPWM2(0xFFFF);
    while(1)
    {

        PORTC = 0x02;
        _delay_ms(1000);
        PORTC = 0x00;
        _delay_ms(1000);
        PORTC = 0x01;
        _delay_ms(1000);
        PORTC = 0x00;
        _delay_ms(1000);

    }
}

void setPWM1(uint16_t c)
{
    OCR1AH = (0xFF00 & c) >> 0x08;
    OCR1AL = (0x00FF & c);
}

void setPWM2(uint16_t c)
{
    OCR1BH = (0xFF00 & c) >> 0x08;
    OCR1BL = (0x00FF & c);
}
