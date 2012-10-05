#include <avr/interrupt.h> 

#define RENC0 CHECKBIT(PIND, PC3) // gets interupt
#define RENC1 CHECKBIT(PIND, PC7)

#define LENC0 CHECKBIT(PIND, PC2) // gets interupt
#define LENC1 CHECKBIT(PIND, PC6)

DDRD = 0x33;					  // PORTD 0,1 -output(serial) 2,3,6,7 -input (Encoder) 4[R] 5[L] -output (PWM)

EICRA |= ISC00 || ISC01;		  // Sets INT0 @Rising Edge
EICRA |= ISC11 || ISC10;		  // Sets INT1 @Rising Edge

ISR(INT0_vect)
{
//your  code
}

ISR(INT1_vect)
{
//your  code
}
