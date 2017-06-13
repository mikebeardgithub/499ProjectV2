/*
 * main.h
 *
 *  Created on: Jun 10, 2017
 *      Author: jjank
 */

#include <stdint.h>

#ifndef MAIN_H_
#define MAIN_H_

// *****************************DEFINITIONS******************************


#define myTIM2_PRESCALER ((uint16_t)0x03E8)				//want ADC to run every 75ms =13.3Hz board is at 168MHz prescale by 1000
#define myTIM2_PERIOD ((uint32_t)0x3138)				//so need to count to count to 12600=0x3138 gona change for testing
// #define NUM_CHANNELS 13
#define NUM_CHANNELS 1


typedef enum selector_state
{
	sine,
	sawtooth,
	square,
	triangle,
	other2
} selector_state;


/******************************************************Global Variables***********************/


selector_state lfo_state, vfo_state;		//state variables for selectors
volatile uint16_t ADCBuffer[NUM_CHANNELS];	//DMA buffer for ADC values


#endif /* MAIN_H_ */
