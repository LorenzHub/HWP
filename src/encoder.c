#include <avr/io.h>
#include "encoder.h"
#include <avr/interrupt.h>
#include <communication/communication.h>
#include <inttypes.h>
#include <tools/variablesAccess.h>

#define PIN_CHANGED(new, old, bit) (((new) ^ (old)) & (1 << (bit)))
#define PIN_IS_HIGH(pin, bit) ((pin) & (1 << (bit)))

static uint8_t pinOld;
static int16_t rightEncoderCounter;
static int16_t LeftEncoderCounter;

void encoder_init(){
    DDRB &= ~((1<<DDB7) | (1<<DDB6) | (1<<DDB5) | (1<<DDB4));                    // PB7 to PB4 as Inputs
    PCICR |= (1<<PCIE0);                                                        // Pin-Change-Interrupt for each Pin in PCINT7:0
    PCMSK0 |= ((1<<PCINT7) | (1<<PCINT6))| (1<<PCINT5) | (1<<PCINT4);           // PB7 to PB4 (PCINT7/6/5/4) enable
    pinOld = PINB & ((1<<PCINT7) | (1<<PCINT6) | (1<<PCINT5) | (1<<PCINT4));
}


ISR(PCINT0_vect){
	uint8_t pinNew = PINB & ((1<<PCINT7) | (1<<PCINT6) | (1<<PCINT5) | (1<<PCINT4));
	
	//Right Encoder
	if( PIN_CHANGED(pinNew, pinOld, PCINT7) ) { // flank change on channel a
		if (PIN_IS_HIGH(pinNew, PCINT7)) {	    // flank change from LOW to HIGH
			if (PIN_IS_HIGH(pinNew, PCINT6))	// B is HIGH
				rightEncoderCounter--;
			else								// B is LOW
				rightEncoderCounter++;
		} 
		else {							        // flank change from HIGH to LOW
			if (PIN_IS_HIGH(pinNew, PCINT6))	// B is HIGH
				rightEncoderCounter++;
			else								// B is LOW
				rightEncoderCounter--;
		}
	}

	if( PIN_CHANGED(pinNew, pinOld, PCINT6) ) { // flank change on channel B
		if (PIN_IS_HIGH(pinNew, PCINT6)) {	    // flank change from LOW to HIGH
			if (PIN_IS_HIGH(pinNew, PCINT7)) 	// A is HIGH
				rightEncoderCounter++;
			else 						        // A is LOW
				rightEncoderCounter--;
			
		} 
		else {							        // flank change from HIGH to LOW
			if (PIN_IS_HIGH(pinNew, PCINT7)) 	// A is HIGH
				rightEncoderCounter--;
			else 								// A is LOW
				rightEncoderCounter++;
		}
	}
   
	
	//Left Encoder
	if( PIN_CHANGED(pinNew, pinOld, PCINT5) ) { // flank change on channel A
		if (PIN_IS_HIGH(pinNew, PCINT5)) {	    // flank change from LOW to HIGH
			if (PIN_IS_HIGH(pinNew, PCINT4)) 	// B is HIGH
				LeftEncoderCounter++;
			else 								// B is LOW
				LeftEncoderCounter--;
			
		} 
		else {								    // flank change from HIGH to LOW
			if (PIN_IS_HIGH(pinNew, PCINT4)) 	// B is HIGH
				LeftEncoderCounter--;
			else 								// B is LOW
				LeftEncoderCounter++;
		}
	}

	if(PIN_CHANGED(pinNew, pinOld, PCINT4)) {   // flank change on channel B
		if (PIN_IS_HIGH(pinNew, PCINT4)) {	    // flank change from LOW to HIGH
			if (PIN_IS_HIGH(pinNew, PCINT5)) 	// A is HIGH
				LeftEncoderCounter--;
			else 								// A is LOW
				LeftEncoderCounter++;
		} 
		else {								    // flank change from HIGH to LOW
			if (PIN_IS_HIGH(pinNew, PCINT5)) 	// A is HIGH
				LeftEncoderCounter++;
			else 								// A is LOW
				LeftEncoderCounter--;
		}
	}

    pinOld = pinNew;    
 }

 int16_t encoder_getCountR(){
  return rightEncoderCounter;
 }

 int16_t encoder_getCountL(){
  return LeftEncoderCounter;
 }
