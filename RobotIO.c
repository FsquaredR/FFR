/* IR Equations
 LF: cm=33097.0 * (x)^(-1.322)
 RF: cm=19799.0 * (x)^(-1.243)
 LR: cm=03440.3 * (x)^(-1.078)
 RR: cm=03790.6 * (x)^(-1.088)
 */

double readIR(int x){
    double adc_data = 0;
    for(unsigned char i=0; i<SAMPLERATE; i++)
    {
        adc_data += (double)readADC(x);
    }
    adc_data = adc_data/SAMPLERATE;
    
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