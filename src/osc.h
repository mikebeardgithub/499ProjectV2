/*
 * osc.h
 *
 *  Created on: May 19, 2017
 *      Author: admin
 */

/*
 * This file was downloaded and adapted from the project found here:
 * https://github.com/MrBlueXav/horrorophone-eclipse-with-makefile
 * Almost none of the original code remains.
 */

#ifndef OSC_H_
#define OSC_H_

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "stm32f4xx_it.h"

#define SAMPLERATE              48000
#define ONE_SECOND				96000			// Two seconds worth of samples (per channel).
#define TWO_SECOND				192000			// Two seconds worth of samples (per channel).
#define FIVE_SECOND				480000			// Four seconds worth of samples (per channel).
#define TEN_SECOND				960000			// Four seconds worth of samples (per channel).

#define VOL                     80
#define BUFF_LEN                64

/* TODO: Used by Horrorophone project -- remove and test */
#define BUFF_LEN_DIV2           32
#define ON                      1
#define OFF                     0

/* Defines for wave shapes */
#define WAVE_NONE				0
#define WAVE_SINE				1
#define WAVE_SQUARE				2
#define WAVE_SAWTOOTH			3
#define WAVE_TRIANGLE			4
#define WAVE_NOISE				5

/* Defines for modulation types */
#define MOD_NONE				0
#define MOD_AM					1
#define MOD_FM					2

#define VCO_AMP					4000
#define LFO_AMP_AM				0.4
#define LFO_AMP_FM				0.5
#define LFO_AMP_FM2				10

void generate_waveforms(uint16_t start, uint16_t end);
float32_t gen_square(uint16_t current_sample, uint16_t samples_half_cycle);
float32_t gen_sawtooth(uint32_t current_sample, uint32_t samples_cycle, float32_t min, float32_t max);
float32_t gen_rampdown(uint32_t current_sample, uint32_t samples_cycle, float32_t min, float32_t max);
float32_t gen_triangle(uint32_t current_sample, uint32_t samples_half_cycle, float32_t amp);
float32_t gen_triangle_integral(uint32_t current_sample, uint32_t samples_half_cycle, float32_t amp);

float32_t gen_square_angle(float32_t angle);
float32_t gen_sawtooth_angle(float32_t angle);
float32_t gen_sawtooth_integral_angle(float32_t angle);
float32_t gen_rampdown_angle(float32_t angle);
float32_t gen_triangle_angle(float32_t angle);
float32_t gen_triangle_integral_angle(float32_t angle);

#endif /* OSC_H_ */
