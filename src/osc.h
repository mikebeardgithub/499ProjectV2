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
#define HALF_SECOND				48000			// Half second worth of samples (per channel).
#define ONE_SECOND				96000			// One second worth of samples (per channel).
#define TWO_SECOND				192000			// Two seconds worth of samples (per channel).
#define FIVE_SECOND				480000			// Four seconds worth of samples (per channel).
#define TEN_SECOND				960000			// Four seconds worth of samples (per channel).

#define TWO_PI					6.28318530718
#define ONE_DIV_PI				0.31830988618
#define TWO_DIV_PI				0.63661977236
#define PI_DIV_2				1.57079632679
#define ONE_DIV_2_PI			0.15915494309
#define ONE_DIV_4_PI			0.07957747155

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


typedef struct osc_setting
{
	volatile float32_t freq_vco;
	volatile float32_t freq_vco2;
	volatile float32_t freq_lfo;

	uint16_t wav_vco;
	uint16_t wav_lfo;
	uint16_t mod_type;

	float32_t vco_amp;
	float32_t vco_amp2;
	float32_t lfo_amp;
	float32_t lfo_offset;

	float32_t square_min;
	float32_t square_max;

	float32_t sawtooth_vco_min;
	float32_t sawtooth_vco_max;

	float32_t sawtooth_lfo_min;
	float32_t sawtooth_lfo_max;

	float32_t fm_mod_level;

} osc_setting;

typedef struct adsr_setting
{
	float32_t sustain_amp;

	uint32_t attack_len;
	uint32_t decay_len;
	uint32_t sustain_len;
	uint32_t release_len;
	uint32_t blank_len;			// Blank time between 'note'.  Can be zero.
} adsr_setting;


void generate_waveforms(uint16_t start, uint16_t end);
float32_t gen_square(uint16_t current_sample, uint16_t samples_half_cycle);
float32_t gen_sawtooth(uint32_t current_sample, uint32_t samples_cycle, float32_t min, float32_t max);
float32_t gen_rampdown(uint32_t current_sample, uint32_t samples_cycle, float32_t min, float32_t max);
float32_t gen_triangle(uint32_t current_sample, uint32_t samples_half_cycle, float32_t amp);
float32_t gen_triangle_integral(uint32_t current_sample, uint32_t samples_half_cycle, float32_t amp);

float32_t gen_square_angle(float32_t angle);
float32_t gen_sawtooth_angle(float32_t angle);

// TODO: test then rename
float32_t gen_sawtooth_angle2( float32_t angle, float32_t delta, uint32_t len);

float32_t gen_sawtooth_integral_angle(float32_t angle);
float32_t gen_rampdown_angle(float32_t angle);
float32_t gen_triangle_angle(float32_t angle);
float32_t gen_triangle_integral_angle(float32_t angle);

float32_t integrate(float32_t value);

float32_t fast_fmod(float32_t x, float32_t y);



#endif /* OSC_H_ */
