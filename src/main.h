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

#define	MYTIM3_PERIOD ((uint16_t)0x396C)				// button limiting timer runs for 175ms
#define myTIM3_PRESCALER ((uint32_t)0x7D0)				// so need to count to 29400000/2000=14700=0x396C
#define	MYTIM4_PERIOD ((uint16_t)0x396C)				// debouncing timer runs for 150ms
#define myTIM4_PRESCALER ((uint32_t)0x7D0)				// so need to count to 25200000/2000=12600=0x3138
#define myTIM2_PRESCALER ((uint16_t)0x03E8)				//want ADC to run every 75ms =13.3Hz board is at 168MHz prescale by 1000
#define myTIM2_PERIOD ((uint32_t)0x3138)				//so need to count to count to 12600=0x3138 gona change for testing
#define NUM_CHANNELS 12
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
	enter
}button_pushed;

typedef enum primary_menu
{
	startup,
	filter,
	ADSR,
	LFO,
	secondaryVCO,
	ADSR_Mod,
	LFO_Mod
}primary_menu;

typedef enum filter_state
{
	no_filter,
	highpass,
	lowpass,
	bandpass
}filter_state;

typedef enum modulation_state
{
	NO_MOD,
	VCOfreq,
	VCOamp,
	LFOfreq,			// MB: this is not needed.
	LFOamp,				// MB: this is not needed.
	FILTER_freq,
	DualMode_LFO,		// MB: this is not needed.
	DualMode_VCO
}modulation_state;

/*
 * Button state used for button debouncing
 */
typedef struct
{
	button_pushed button;
	int button_state;
}button_state;

/*
 * Struct for menu state machine keeps track of what menu and where the cursor is
 */
typedef struct
{
	primary_menu menu_state;					//which menu
	unsigned int cursor_option;					//where is cursor pointing
	modulation_state lfo_mod;					//What is LFO modulating
	modulation_state adsr_mod;					//what is ADSR modulating
	filter_state filterst8;						//what is filter modulating
	unsigned int secondary_vco;					//is the secondary VCO on
}menu_state;

/******************************************************Global Variables***********************/


volatile uint16_t ADCBuffer[NUM_CHANNELS];	//DMA buffer for ADC values
selector_state lfo_state, vfo_state;		//state variables for selectors
button_state menubutton;
menu_state current_menu_state;

#endif /* MAIN_H_ */
