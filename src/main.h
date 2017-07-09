/*
 * main.h
 *
 *  Created on: Jun 10, 2017
 *      Author: jjank
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <stdlib.h>


// *****************************DEFINITIONS******************************

#define	MYTIM3_PERIOD ((uint16_t)0x20D0)				// button limiting timer runs for 100ms
#define myTIM3_PRESCALER ((uint32_t)0x7D0)				// so need to count to 1680000/2000=8400=0x20D0
#define	MYTIM4_PERIOD ((uint16_t)0x1068)				// debouncing timer runs for 50ms
#define myTIM4_PRESCALER ((uint32_t)0x7D0)				// so need to count to 8400000/2000=4200=0x1068
#define myTIM2_PRESCALER ((uint16_t)0x03E8)				//want ADC to run every 75ms =13.3Hz board is at 168MHz prescale by 1000
#define myTIM2_PERIOD ((uint32_t)0x3138)				//so need to count to count to 12600=0x3138 gona change for testing
#define NUM_CHANNELS 13
#define ACTIVE 1
#define NOT_ACTIVE 0

typedef enum selector_state
{
	sine,
	sawtooth,
	square,
	triangle,
	other2
} selector_state;

typedef enum button_pushed
{
	up,
	down,
	back,
	enter,
}button_pushed;

typedef struct
{
	button_pushed button;
	int button_state;
}button_state;

/******************************************************Global Variables***********************/


volatile uint16_t ADCBuffer[NUM_CHANNELS];	//DMA buffer for ADC values
selector_state lfo_state, vfo_state;		//state variables for selectors
button_state menubutton;


#endif /* MAIN_H_ */
