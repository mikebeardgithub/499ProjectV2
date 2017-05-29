/*
 * main.h
 *
 *  Created on: May 19, 2017
 *      Author: admin
 */


/*
 * This file was downloaded and adapted from the project found here:
 * https://github.com/MrBlueXav/horrorophone-eclipse-with-makefile
 */



#ifndef MAIN_H_
#define MAIN_H_

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio_codec.h"

#include <stdio.h>
#include "stm32f4xx_it.h"

#define _2PI                    6.283185307f
// #define _PI						3.14159265f
// #define _INVPI					0.3183098861f
// #define SAMPLERATE              48000
#define SAMPLERATE              48000

#define VOL                     80
#define BUFF_LEN_DIV4           16
#define BUFF_LEN_DIV2           32
#define BUFF_LEN                64

#define MOV_AVG_BUFF_LEN		128
#define ON                      1
#define OFF                     0

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)


#endif /* MAIN_H_ */
