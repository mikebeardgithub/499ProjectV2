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
#include "arm_math.h"
#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio_codec.h"

#include <stdio.h>
#include "stm32f4xx_it.h"

/* TODO: Used by Horrorophone project -- remove and test */
#define _2PI                    6.283185307f
// #define _PI						3.14159265f
// #define _INVPI					0.3183098861f
// #define SAMPLERATE              48000
#define SAMPLERATE              48000

#define VOL                     80
#define BUFF_LEN                64

/* TODO: Used by Horrorophone project -- remove and test */
#define BUFF_LEN_DIV4           16
#define BUFF_LEN_DIV2           32


#define MOV_AVG_BUFF_LEN		128
#define ON                      1
#define OFF                     0


#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)

/* Defines for wave shapes */
#define WAVE_NONE				0
#define WAVE_SINE				1
#define WAVE_SQUARE				2
#define WAVE_SAWTOOTH			3
#define WAVE_NOISE				4

/* Defines for modulation types */
#define MOD_NONE				0
#define MOD_AM					1
#define MOD_FM					2

#define VCO_AMP					4000
#define LFO_AMP_AM				0.4
#define LFO_AMP_FM				0.5
#define LFO_AMP_FM2				10

float32_t square(uint16_t current_sample, uint16_t samples_half_cycle);
float32_t sawtooth(uint16_t current_sample, uint16_t samples_cycle);
void generate_waveforms(uint16_t start, uint16_t end);

#endif /* MAIN_H_ */
