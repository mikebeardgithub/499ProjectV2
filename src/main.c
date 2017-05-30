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
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio_codec.h"

#include <math.h>
#include "arm_math.h"

void ADC3_CH12_DMA_Config(void);

/* Globals */
int sample;
float           		pass = 1.f ;
float           		phase2 = 0.0f , phase2Step;
// float           		f1 = FREQ1 , f2 = FREQ2 , freq;
__IO uint16_t 			ADC3ConvertedValue = 0;
RCC_ClocksTypeDef       RCC_Clocks;
GPIO_InitTypeDef        GPIO_InitStructure;
uint8_t                 state = OFF;
__IO uint32_t 			TimingDelay = 50;


volatile uint16_t buffer_vco[BUFF_LEN] = {0};
volatile uint16_t buffer_lfo[BUFF_LEN] = {0};
volatile float32_t buffer_lfo_float[BUFF_LEN] = {0};
volatile uint16_t buffer_output[BUFF_LEN] = {0};

volatile uint16_t mov_avg [MOV_AVG_BUFF_LEN] = {0};
volatile uint16_t mov_avg_index = 0;
volatile uint16_t mov_avg_sum;

// Good test frequency: freq_vco = 410
volatile float32_t freq_vco = 500.0;
volatile float32_t freq_lfo = 2.7;
// float32_t freq_vco = 375.0;					// Pure sine if BUFF_LEN is 128
volatile uint16_t angle_mem = 0.0;


uint16_t wav_vco_sin = 0;
uint16_t wav_vco_square = 1;
uint16_t wav_lfo_sin = 1;
uint16_t wav_lfo_square = 0;
uint16_t mod_am = 0;
uint16_t mod_fm = 0;


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
  // int datasize = sizeof(data)/2;
  int retVal = -1;

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
   STM_EVAL_LEDInit(LED3); // orange LED
   STM_EVAL_LEDInit(LED4); // green LED
   STM_EVAL_LEDInit(LED5); // red LED
   STM_EVAL_LEDInit(LED6); // blue LED

   /* Green Led On: start of application */
   STM_EVAL_LEDOn(LED4);

   /* ADC3 configuration *******************************************************/
    /*  - Enable peripheral clocks                                              */
    /*  - DMA2_Stream0 channel2 configuration                                   */
    /*  - Configure ADC Channel12 pin as analog input  : PC2                    */
    /*  - Configure ADC3 Channel12                                              */
    ADC3_CH12_DMA_Config();

    /* Start ADC3 Software Conversion */
    ADC_SoftwareStartConv(ADC3);

	/* Initialize User Button */
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);

	retVal = EVAL_AUDIO_Init( OUTPUT_DEVICE_AUTO, VOL, SAMPLERATE);
	retVal = EVAL_AUDIO_Play(buffer_output, BUFF_LEN);

  /* Infinite loop */
  while (1)
  {
	i++;


    if (STM_EVAL_PBGetState(BUTTON_USER) && (state == OFF))
    {
      state = ON;
      STM_EVAL_LEDOn(LED6); // blue LED ON
      pass = 0.5f;
    }
    else
    {
      if (! STM_EVAL_PBGetState(BUTTON_USER))
      {
        STM_EVAL_LEDOff(LED6); // blue LED OFF
        pass = 0.0f;
        state = OFF;
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
	/* Generally this interrupt routine is used to load the buffer when
	a streaming scheme is used: When first Half buffer is already transferred load
	the new data to the first half of buffer while DMA is transferring data from
	the second half. And when Transfer complete occurs, load the second half of
	the buffer while the DMA is transferring from the first half ... */

	// Moving average filter for ADC3 (PC3)
//	mov_avg[mov_avg_index] = ADC3ConvertedValue;			// Get newest value
//	mov_avg_sum += ADC3ConvertedValue;						// Accumulate
//	mov_avg_sum -= mov_avg[(mov_avg_index + 1) % MOV_AVG_BUFF_LEN];	// Remove oldest
//	mov_avg_index = (mov_avg_index + 1) % MOV_AVG_BUFF_LEN;							// Increment index
//	freq_vco = 4.0 * ( (float32_t)  mov_avg_sum)/MOV_AVG_BUFF_LEN;

	// freq_vco = (float32_t) ADC3ConvertedValue;
	// freq_lfo = (float32_t) ADC3ConvertedValue/100;
	volatile float32_t angle_vco = freq_vco*2*PI/SAMPLERATE;
	volatile float32_t angle_lfo = freq_lfo*2*PI/SAMPLERATE;

	// Fill buffer from 0 to BUFF_LEN/2
	volatile int i = 0;
	volatile int samples_cycle = 0;
	volatile int samples_half_cycle = 0;

	// VCO Waveform
	if(wav_vco_sin == 1)
	{
		for(i = 0; i < BUFF_LEN_DIV2; i++)
		{
			buffer_vco[i] = 2000 + 2000*arm_sin_f32((angle_mem+i)*angle_vco);
		}
	}
	else if(wav_vco_square == 1)
	{
		/*
		 * In a single square pulse cycle, there are n samples, each of which is 1/48000s long.
		 * Therefore,  T = n/48000
		 * --> n = 48000*T
		 * --> n = 48000/f
		 * Therefore, duration of positive (one) half is n/2 = 48000/2f.  Same for negative (zero) half.
		 *
		 */

		samples_cycle = SAMPLERATE/freq_vco;
		samples_half_cycle = samples_cycle/2;

		for(i = 0; i < BUFF_LEN_DIV2; i++)
		{

			if((angle_mem+i)%samples_cycle < samples_half_cycle)
			{
				buffer_vco[i] = 4000;
			}
			else
			{
				buffer_vco[i] = 0000;
			}
			//buffer_vco[i] = 2000 + 2000*arm_sin_f32((angle_mem+i)*angle_vco);
		}
	}

	// LFO Waveform
	if(wav_lfo_sin == 1)
	{
		for(i = 0; i < BUFF_LEN_DIV2; i++)
		{
			buffer_lfo_float[i] = 40.0 + 40.0*arm_sin_f32((angle_mem+i)*angle_lfo);
		}
	}

	// VCO-LFO modulation
	if(mod_am == 1)
	{
		for(i = 0; i < BUFF_LEN_DIV2; i++)
		{
			buffer_output[i] = buffer_vco[i] * buffer_lfo_float[i];
		}
	}
	else if(mod_fm == 1)
	{
		for(i = 0; i < BUFF_LEN_DIV2; i++)
		{
			buffer_vco[i] = 2000 + 2000*arm_sin_f32((angle_mem+i)*angle_vco + buffer_lfo_float[i]);
			buffer_output[i] = buffer_vco[i];
		}
	}
	// No modulation
	else
	{
		for(i = 0; i < BUFF_LEN_DIV2; i++)
		{
			buffer_output[i] = buffer_vco[i];
		}
	}


	// Remember lfo phase and resume next run of callback.
	angle_mem = (angle_mem + i) % SAMPLERATE;
	return;
}

/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
	// Turns off yellow LED -- indicates no error occurred.
	STM_EVAL_LEDOff(LED3);

	// freq_vco = (float32_t) ADC3ConvertedValue;
	// freq_lfo = (float32_t) ADC3ConvertedValue/100;
	volatile float32_t angle_vco = freq_vco*2*PI/SAMPLERATE;
	volatile float32_t angle_lfo = freq_lfo*2*PI/SAMPLERATE;

	// float32_t  sinOutput;
	volatile int i = 0;
	volatile int samples_cycle = 0;
	volatile int samples_half_cycle = 0;

	// TODO: remove after testing.
	// memset(buffer_vco, 0, sizeof(buffer_vco));
	// memset(buffer_output, 0, sizeof(buffer_output));

	// VCO Waveform
	if(wav_vco_sin == 1)
	{
		for(i = BUFF_LEN_DIV2; i < BUFF_LEN; i++)
		{
			buffer_vco[i] = 2000 + 2000*arm_sin_f32((angle_mem+(i-BUFF_LEN_DIV2))*angle_vco);
		}
	}
	else if(wav_vco_square == 1)
	{
		/*
		 * In a single square pulse cycle, there are n samples, each of which is 1/48000s long.
		 * Therefore,  T = n/48000
		 * --> n = 48000*T
		 * --> n = 48000/f
		 * Therefore, duration of positive (one) half is n/2 = 48000/2f.  Same for negative (zero) half.
		 *
		 */

		samples_cycle = SAMPLERATE/freq_vco;
		samples_half_cycle = samples_cycle/2;

		for(i = BUFF_LEN_DIV2; i < BUFF_LEN; i++)
		{

			if((angle_mem+i)%samples_cycle < samples_half_cycle)
			{
				buffer_vco[i] = 4000;
			}
			else
			{
				buffer_vco[i] = 0000;
			}
			//buffer_vco[i] = 2000 + 2000*arm_sin_f32((angle_mem+i)*angle_vco);
		}
	}

	// LFO Waveform
	if(wav_lfo_sin == 1)
	{
		for(i = BUFF_LEN_DIV2; i < BUFF_LEN; i++)
		{
			// buffer_lfo_float[i] = 0.4 + 0.4*arm_sin_f32((angle_mem+(i-BUFF_LEN_DIV2))*angle_lfo);
			buffer_lfo_float[i] = 40.0 + 40.0*arm_sin_f32((angle_mem+(i-BUFF_LEN_DIV2))*angle_lfo);
		}
	}

	// VCO-LFO modulation
	if(mod_am == 1)
	{
		for(i = BUFF_LEN_DIV2; i < BUFF_LEN; i++)
		{
			buffer_output[i] = buffer_vco[i] * buffer_lfo_float[i];
		}
	}
	else if(mod_fm == 1)
	{
		for(i = BUFF_LEN_DIV2; i < BUFF_LEN; i++)
		{
			buffer_vco[i] = 2000 + 2000*arm_sin_f32((angle_mem+(i-BUFF_LEN_DIV2))*angle_vco + buffer_lfo_float[i]);
			buffer_output[i] = buffer_vco[i];
		}
	}

	// No modulation
	else
	{
		for(i = BUFF_LEN_DIV2; i < BUFF_LEN; i++)
		{
			buffer_output[i] = buffer_vco[i];
		}
	}

	// Remember lfo phase and resume next run of callback.
	angle_mem = (angle_mem + i - BUFF_LEN_DIV2) % SAMPLERATE;
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
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

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


//---------------------------------------------------------------------------
/**************
* returns a random float between 0 and 1
*****************/
float randomNum(void)
  {
		return 0.5;
  }


void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}
