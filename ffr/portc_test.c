//
//  portc_test.c
//  
//
//  Created by Tomasz Walczak on 10/13/12.
//
//

#include <avr/io.h>
#include <util/delay.h>

void main(void)
{
    DDRA = 0xFF;
    DDRB = 0xFF;
    DDRC = 0xFF;
    DDRD = 0xFF;
    

    while(1)
    {

        PORTC = 0xFF;

    }
}


