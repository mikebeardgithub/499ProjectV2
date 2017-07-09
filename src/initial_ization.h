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

// Initializes the selector pins and EXTi, also tim4 used for debouncing
void init_gpios();

// Initializes the push buttons for screen navigation and exti also sets up tim5 for debouncing
void  init_push_buttons();

// Initializes the LCD screen SPI communications
void init_spi();

// Initializes all the GPIOs for parallel communication to LCD screen
void init_parallel();
