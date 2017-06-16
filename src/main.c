/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 7.1.2   2017-05-18

The MIT License (MIT)
Copyright (c) 2009-2017 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/

/*
 * This file was downloaded and heavily modified from the project found here:
 * https://github.com/MrBlueXav/horrorophone-eclipse-with-makefile
 *
 * There may be very little of the original file left.
 *
 */


/* Includes */
#include "initial_ization.h"
#include "user_interface.h"
#include "main.h"
#include "osc.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio_codec.h"

#include <stdio.h>

/* Globals */
uint8_t                 state = OFF;					// From horrorophone

extern volatile uint16_t buffer_output[BUFF_LEN];
// extern uint16_t wav_vco = WAVE_SINE;

// ADC
volatile uint16_t adc_value;


/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	// ADC
	adc_value = ADCBuffer[0];

	/**
	*  IMPORTANT NOTE!
	*  The symbol VECT_TAB_SRAM needs to be defined when building the project
	*  if code has been located to RAM and interrupts are used.
	*  Otherwise the interrupt table located in flash will be used.
	*  See also the <system_*.c> file and how the SystemInit() function updates
	*  SCB->VTOR register.
	*  E.g.  SCB->VTOR = 0x20000000;
	*/

	/* Initialize LEDS */
	STM_EVAL_LEDInit(LED3); // orange LED		// From Horrorophone
	STM_EVAL_LEDInit(LED4); // green LED		// From Horrorophone
	STM_EVAL_LEDInit(LED5); // red LED			// From Horrorophone
	STM_EVAL_LEDInit(LED6); // blue LED			// From Horrorophone

	/* Green Led On: start of application */
	STM_EVAL_LEDOn(LED4);						// From Horrorophone

	// John's
	init_adc(ADCBuffer);						//initialize ADC, do this last because it starts the timer

	/* Initialize User Button */
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);		// From Horrorophone

	EVAL_AUDIO_Init( OUTPUT_DEVICE_AUTO, VOL, SAMPLERATE);
	EVAL_AUDIO_Play(buffer_output, BUFF_LEN);

	// Main loop.
	while (1)
	{
		adc_value = ADCBuffer[0];

		// From Horrorophone
		if (STM_EVAL_PBGetState(BUTTON_USER) && (state == OFF))
		{
			state = ON;			// From Horrorophone
			STM_EVAL_LEDOn(LED6); // blue LED ON	// From Horrorophone
			// wav_vco = (wav_vco + 1)%4; // Count up to 3 and then roll over to 0.
			// freq_lfo =   ( (float32_t)( ADC3ConvertedValue & 0xffb )/10);
		}
		else
		{
			if (! STM_EVAL_PBGetState(BUTTON_USER))
			{
				state = OFF;		// From Horrorophone
				STM_EVAL_LEDOff(LED6); // blue LED OFF
			}
		}
	}
}


/**
  * @brief  Basic management of the timeout situation.
  * @param  None
  * @retval None
  */
uint32_t Codec_TIMEOUT_UserCallback(void)
{
	// TODO: See instructions in function declaration.  I've seen this LED turn on, which may signal an issue.
	STM_EVAL_LEDOn(LED5); 				/*  alert : red LED !  */
	return (0);
}

/**
* @brief  Manages the DMA Half Transfer complete interrupt.
* @param  None
* @retval None
*/
void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size)
{
	/*
	Generally this interrupt routine is used to load the buffer when
	a streaming scheme is used: When first Half buffer is already transferred load
	the new data to the first half of buffer while DMA is transferring data from
	the second half. And when Transfer complete occurs, load the second half of
	the buffer while the DMA is transferring from the first half ...
	 */

	generate_waveforms(0, BUFF_LEN_DIV2);
	return;
}

/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
	generate_waveforms(BUFF_LEN_DIV2, BUFF_LEN);
	return;
}


/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */

  // return -1;
	return 0;
}

void EVAL_AUDIO_Error_CallBack(void* pData)
{
	STM_EVAL_LEDOn(LED3);
}
