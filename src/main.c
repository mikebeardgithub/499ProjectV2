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

void ADC3_CH12_DMA_Config(void);

/* Globals */
// int sample;
float           		pass = 1.f ;					// From horrorophone
// float           		phase2 = 0.0f , phase2Step;		// From horrorophone
// float           		f1 = FREQ1 , f2 = FREQ2 , freq;	// From horrorophone
// __IO uint16_t 			ADC3ConvertedValue = 0;			// From horrorophone
// RCC_ClocksTypeDef    RCC_Clocks;						// From horrorophone -turned off - might be needed.
GPIO_InitTypeDef        GPIO_InitStructure;				// From horrorophone
uint8_t                 state = OFF;					// From horrorophone
// __IO uint32_t 			TimingDelay = 50;				// From horrorophone



extern volatile uint16_t buffer_output[BUFF_LEN];
extern uint16_t wav_vco;


/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
  int i = 0;

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
   STM_EVAL_LEDInit(LED4); // green LED			// From Horrorophone
   STM_EVAL_LEDInit(LED5); // red LED			// From Horrorophone
   STM_EVAL_LEDInit(LED6); // blue LED			// From Horrorophone

   /* Green Led On: start of application */
   STM_EVAL_LEDOn(LED4);						// From Horrorophone

   /* ADC3 configuration *******************************************************/
    /*  - Enable peripheral clocks                                              */
    /*  - DMA2_Stream0 channel2 configuration                                   */
    /*  - Configure ADC Channel12 pin as analog input  : PC2                    */
    /*  - Configure ADC3 Channel12                                              */
    ADC3_CH12_DMA_Config();						// From Horrorophone

    /* Start ADC3 Software Conversion */
    ADC_SoftwareStartConv(ADC3);				// From Horrorophone

	/* Initialize User Button */
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);		// From Horrorophone

	EVAL_AUDIO_Init( OUTPUT_DEVICE_AUTO, VOL, SAMPLERATE);
	EVAL_AUDIO_Play(buffer_output, BUFF_LEN);

  /* Infinite loop */
  while (1)
  {
	i++;

	// From Horrorophone
    if (STM_EVAL_PBGetState(BUTTON_USER) && (state == OFF))
    {
      state = ON;			// From Horrorophone
      STM_EVAL_LEDOn(LED6); // blue LED ON	// From Horrorophone
      pass = 0.5f;			// From Horrorophone

      wav_vco = (wav_vco + 1)%4; // Count up to 3 and then roll over to 0.

      // freq_lfo =   ( (float32_t)( ADC3ConvertedValue & 0xffb )/10);
    }
    else
    {
      if (! STM_EVAL_PBGetState(BUTTON_USER))
      {
        STM_EVAL_LEDOff(LED6); // blue LED OFF
        pass = 0.0f;		// From Horrorophone
        state = OFF;		// From Horrorophone
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
	STM_EVAL_LEDOn(LED5); /*  alert : red LED !  */
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

/**
  * @brief  ADC3 channel12 with DMA configuration
  * @param  None
  * @retval None
  */

// TODO: See what can be removed fro this function.
void ADC3_CH12_DMA_Config(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel0 configuration **************************************/
//  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//  DMA_InitStructure.DMA_BufferSize = 1;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
//  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
//  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
//  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC3 Channel12 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_8b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel12 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
}




// TODO: Delete these functions.
//---------------------------------------------------------------------------
/**************
* returns a random float between 0 and 1
*****************/
/*
float randomNum(void)
  {
		return 0.5;
  }
*/
/*
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}
*/
