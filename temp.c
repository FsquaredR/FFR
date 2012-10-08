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

/* 9.5(pi) = 29.845 cm Wheel cir.
   1200/ 29.845 = 40.208 incs./cm
*/
void go(int d, short dirR, short dirL){			// Assuming overall encoders get cleared ( d in increments )
	unsigned int dismvR = 0;     // incs.
	unsigned int dismvL = 0;

	if(dirR = 1){
		unsigned int ENCR_p = countR0;		// countR0 increments on forward movment
	}else if(dirR = -1){
		unsigned int ENCR_p = countR1;		// countR1 increments on reverse movment
	}
	
	if(dirL = 1){
		unsigned int ENCL_p = countL0;
	}else if(dirL = -1){
		unsigned int ENCL_p = countL1;
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
		
		dismvR = countR0 - ENCR_p;
		dismvL = countL0 - ENCL_p;
	}	
}

void motorR(char d){
	if(d = F){
		RM0(0);
		RM1(1);	
	}else if(d = R){
		RM0(1);
		RM1(0);
	}else if(d = S){
		RM0(0);
		RM1(0);
	}
}

void motorL(char d){
	if(d = F){
		LM0(1);
		LM1(0);	
	}else if(d = R){
		LM0(0);
		LM1(1);
	}else if(d = S){
		LM0(0);
		LM1(0);
	}
}

/* Encoders */
short dirL = 0;
long countL0 = 0;
long countL1 = 0;
short dirR = 0;
long countR0 = 0;
long countR1 = 0;

ISR(INT0_vect){

  if(!LENC1){          //Checks other right encoder
    dirL = 1;
    countL0++;
  }else{
    dirL = -1;
    countL1++;
  }
}

ISR(INT1_vect)
{

  if(!RENC1){          //Checks other right encoder
    dirR = 1;
    countR0++;
  }else{
    dirR = -1;
    countR1++;
  }
}
	