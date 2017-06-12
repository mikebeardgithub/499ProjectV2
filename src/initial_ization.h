/*
 * initial_ization.h
 *
 *  Created on: May 31, 2017
 *      Author: jjank
 */


#include "main.h"

/**************************************Global Variables****************************/




// Initializes ADC pins and Calibrates ADC Do DMA transfer
void init_adc(volatile uint16_t ADCBuffer[NUM_CHANNELS]);

void init_gpios();

