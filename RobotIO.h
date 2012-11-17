#ifndef RobotIO_H
#define RobotIO_H

/* Macros For Bitsetting  */
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT)) 
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

/* Pin Declairations */
	// Sensors
#define ULTRA_WRITE(x) (x) ? SETBIT(PORTC, 5) : CLEARBIT(PORTC, 5);
#define ULTRA_IN CLEARBIT(DDRC, 5)
#define ULTRA_OUT SETBIT(DDRC, 5)
#define ULTRA_READ CHECKBIT(PINC, PC5)
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

#endif 