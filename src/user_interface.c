/*
 * user_interface.c
 *
 *  Created on: Jun 10, 2017
 *      Author: jjank
 */

#include "user_interface.h"
#include "stm32f4xx_gpio.h"
#include "main.h"


/*
 * Gets and updated the state of both the selector rotary switches
 * This funtion is called by the tim2 interrupt handler
 */
void update_selector_state (){

	if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) == 1){
		vfo_state = sine;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8) == 1){
		vfo_state = sawtooth;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == 1){
		vfo_state = square;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10) == 1){
		vfo_state = triangle;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11) == 1){
		vfo_state = other2;
	}
//could put some fault detection at the end of these if statements come back later if theres time

	if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12) == 1){
		lfo_state = sine;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13) == 1){
		lfo_state = sawtooth;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14) == 1){
		lfo_state = square;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15) == 1){
		lfo_state = triangle;
	}else if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 1){
		lfo_state = other2;
	}
}
