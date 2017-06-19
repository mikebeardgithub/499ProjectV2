/*
 * initial_ization.c
 *
 *  Created on: May 29, 2017
 *      Author: jjank
 */


#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "initial_ization.h"
#include "main.h"

/*********************************************************Globlal Variables***************************************
 *
 */






/*
 * Initializes the ADC to work in scan mode with 13 channels for pots
 * Uses DMA to transfer data and a timer to throttle the ADC conversion
 * The conversion cycle occurs every 75ms this function starts the timer.
 */
void init_adc(volatile uint16_t ADCBuffer[NUM_CHANNELS]){

	/* Define ADC init structures */
	ADC_InitTypeDef       adc_init_struct;
	ADC_CommonInitTypeDef adc_com_init_struct;
	DMA_InitTypeDef DMA_Init_struct;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef time_base_struct;




	/* Enable timer (timer runs at 13.3 Hz)*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseStructInit(&time_base_struct);
	time_base_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	time_base_struct.TIM_CounterMode = TIM_CounterMode_Up;
	time_base_struct.TIM_Period = myTIM2_PERIOD;
	time_base_struct.TIM_Prescaler = myTIM2_PRESCALER;
	TIM_TimeBaseInit(TIM2, &time_base_struct);
	TIM_SelectOutputTrigger(TIM2,TIM_TRGOSource_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	/* Enable clock on DMA1 & GPIO's */
	/* Enable DMA2, thats where ADC peripheral is used */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

	/*Initialize GPIO's for ADC*/
	//A bank pins
	//GPIO_Pin_0	VFO-Amplitude
	//GPIO_Pin_1	VFO-Frequency
	//GPIO_Pin_2	LFO-Amplitude
	//GPIO_Pin_3	LFO-Frequency
	//GPIO_Pin_4	VCO-Volume
	//GPIO_Pin_5	ENVELOPE-Attack
	//GPIO_Pin_6	ENVELOPE-decay
	//GPIO_Pin_7	ENVELOPE-Sustain
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//B bank pins
	//GPIO_Pin_0	ENVELOPE-Release
	//GPIO_Pin_1	FILTER-FreqLow
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//C bank pins//GPIO_Pin_0	FILTER-FreqHigh
	//GPIO_Pin_1	FILTER-FreqResonance
	//GPIO_Pin_4	FILTER-FreqGain
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	/*  Initialise DMA */
	DMA_StructInit(&DMA_Init_struct);							// reset struct

	/* config of DMA */
	DMA_Init_struct.DMA_Channel = DMA_Channel_0; 				/* See Tab 43 */
	DMA_Init_struct.DMA_BufferSize = NUM_CHANNELS;				/*  13adc channels */
	DMA_Init_struct.DMA_DIR = DMA_DIR_PeripheralToMemory; 		/* ADC to mem */
	DMA_Init_struct.DMA_FIFOMode = DMA_FIFOMode_Disable; 		/* no FIFO */
	DMA_Init_struct.DMA_FIFOThreshold = 0;
	DMA_Init_struct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_Init_struct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init_struct.DMA_Mode = DMA_Mode_Circular; 				/* circular buffer */
	DMA_Init_struct.DMA_Priority = DMA_Priority_High; 			/* high priority */
	DMA_Init_struct.DMA_Memory0BaseAddr = (uint32_t)ADCBuffer; 	/* target addr */
	DMA_Init_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; /* 16 bit */
	DMA_Init_struct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_Init_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_Init_struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_Init_struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Init(DMA2_Stream0, &DMA_Init_struct); 					/* See Table 43 for mapping */
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/*Initialize ADC*/
	ADC_StructInit(&adc_init_struct);						//populates structs with reset defaults
	ADC_CommonStructInit(&adc_com_init_struct);
	ADC_Cmd(ADC1, DISABLE);
	ADC_DeInit();

	/* init ADC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/*Initialize Common ADC */
	adc_com_init_struct.ADC_Mode = ADC_Mode_Independent;
	adc_com_init_struct.ADC_Prescaler = ADC_Prescaler_Div2;
	adc_com_init_struct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	adc_com_init_struct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&adc_com_init_struct);

	/* Initialize ADC1 */
	adc_init_struct.ADC_Resolution = ADC_Resolution_12b;
	adc_init_struct.ADC_ScanConvMode = ENABLE;
	adc_init_struct.ADC_ContinuousConvMode = DISABLE;
	adc_init_struct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	adc_init_struct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	adc_init_struct.ADC_DataAlign = ADC_DataAlign_Right;
	adc_init_struct.ADC_NbrOfConversion = NUM_CHANNELS; /* 5 channels in total */
	ADC_Init(ADC1, &adc_init_struct);

	/* Configure channels */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);		//VFO-Amplitude
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_480Cycles);		//VFO-Frequency
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_480Cycles);		//LFO-Amplitude
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_480Cycles);		//LFO-Frequency
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_480Cycles);		//VCO-Volume
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_480Cycles);		//ENVELOPE-Attack
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_480Cycles);		//ENVELOPE-decay
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_480Cycles);		//ENVELOPE-Sustain
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 9, ADC_SampleTime_480Cycles);		//ENVELOPE-Release
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 10, ADC_SampleTime_480Cycles);	//FILTER-FreqLow
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 11, ADC_SampleTime_480Cycles);	//FILTER-FreqHigh
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 12, ADC_SampleTime_480Cycles);	//FILTER-FreqResonance
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 13, ADC_SampleTime_480Cycles);	//FILTER-FreqGain


	/* Enable ADC1 DMA */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);			//enables DMA request after all adc conversions
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 **************************************************************/
	ADC_Cmd(ADC1, ENABLE);

	TIM_Cmd(TIM2, ENABLE);		//This could be dine in the main however gonna leave it here



}








/*
 * Sets up the 5 position selectors

 */

void init_gpios(){

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef tim4_base_struct;
	EXTI_InitTypeDef EXTI_init_struct;
	NVIC_InitTypeDef EXTI_NVIC_init_struct;
	NVIC_InitTypeDef TIM4_NVIC_init_struct;


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 			//This is already turned on in ADC Init function
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

	/*
	 * E Bank pins
	 * PE7 		vco_sine
	 * PE8		vco_sawtooth
	 * PE9		vco_square
	 * PE10		vco_triangle
	 * PE11 	vco_other 2 talk to mike
	 * PE12 	lfo_sine
	 * PE13		lfo_sawtooth
	 * PE14		lfo_square
	 * PE15		lfo_triangle
	 */
	GPIO_StructInit(&GPIO_InitStructure);							// Default values
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;					//input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;				//slow
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;				//no
	GPIO_Init(GPIOE, &GPIO_InitStructure);


	/*
	 * C bank pins
	 * PC12		lfo_other2
	 */
	GPIO_StructInit(&GPIO_InitStructure);							//default values
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;					//input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;				//slow
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;				//pull down
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	/*Configure Tim4 for debouncing	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseStructInit(&tim4_base_struct);
	tim4_base_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	tim4_base_struct.TIM_CounterMode = TIM_CounterMode_Up;
	tim4_base_struct.TIM_Period = MYTIM4_PERIOD;
	tim4_base_struct.TIM_Prescaler = myTIM4_PRESCALER;
	TIM_TimeBaseInit(TIM2, &tim4_base_struct);

	TIM4_NVIC_init_struct.NVIC_IRQChannel = TIM4_IRQn;
	TIM4_NVIC_init_struct.NVIC_IRQChannelCmd = ENABLE;
	TIM4_NVIC_init_struct.NVIC_IRQChannelPreemptionPriority = 0x00;
	TIM4_NVIC_init_struct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&TIM4_NVIC_init_struct);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);


	/*Configure pins as EXTI*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource7);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource8);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource9);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource10);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource11);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource12);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource13);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource14);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource15);


	//init EXTI
	EXTI_init_struct.EXTI_Line = EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9 | EXTI_Line10 | EXTI_Line11 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15;
	EXTI_init_struct.EXTI_LineCmd = ENABLE;
	EXTI_init_struct.EXTI_Mode =  EXTI_Mode_Interrupt;
	EXTI_init_struct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_init_struct);

	EXTI_NVIC_init_struct.NVIC_IRQChannel = EXTI9_5_IRQn;
	EXTI_NVIC_init_struct.NVIC_IRQChannelPreemptionPriority = 0x0F;
	EXTI_NVIC_init_struct.NVIC_IRQChannelSubPriority = 0x0F;
	EXTI_NVIC_init_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&EXTI_NVIC_init_struct);


	EXTI_NVIC_init_struct.NVIC_IRQChannel = EXTI15_10_IRQn;;
	EXTI_NVIC_init_struct.NVIC_IRQChannelPreemptionPriority = 0x0F;
	EXTI_NVIC_init_struct.NVIC_IRQChannelSubPriority = 0x0F;
	EXTI_NVIC_init_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&EXTI_NVIC_init_struct);




}
