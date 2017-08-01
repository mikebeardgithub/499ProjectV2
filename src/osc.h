/*
 * osc.h
 *
 *  Created on: May 19, 2017
 *      Author: Mike Beard
 */


#ifndef OSC_H_
#define OSC_H_

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "stm32f4xx_it.h"

#define SAMPLERATE              48000
#define QUARTER_SECOND			24000			// Half second worth of samples (per channel).
#define HALF_SECOND				48000			// Half second worth of samples (per channel).
#define ONE_SECOND				96000			// One second worth of samples (per channel).
#define TWO_SECOND				192000			// Two seconds worth of samples (per channel).
#define FIVE_SECOND				480000			// Four seconds worth of samples (per channel).
#define TEN_SECOND				960000			// Ten seconds worth of samples (per channel).
#define TWENTY_SECOND			1920000			// 20 seconds worth of samples (per channel).

#define TWO_PI					6.28318530718
#define ONE_DIV_PI				0.31830988618
#define TWO_DIV_PI				0.63661977236
#define PI_DIV_2				1.57079632679
#define PI_DIV_4				0.78539816339
#define ONE_DIV_2_PI			0.15915494309
#define ONE_DIV_4_PI			0.07957747155

#define VOL                     80
#define LENGTH_BUFFER           64
#define BUFF_LEN_HALF           32

#define OFF                     0
#define ON                      1

/* Defines for wave shapes */
#define WAVE_NONE				0
#define WAVE_SINE				1
#define WAVE_SQUARE				2
#define WAVE_SAWTOOTH			3
#define WAVE_TRIANGLE			4
#define WAVE_NOISE				5

/* Defines for modulation types */
//#define MOD_NONE				0
//#define MOD_AM					1
//#define MOD_FM					2

#define VCO_AMP					4000
#define LFO_AMP_AM				0.0001
#define LFO_AMP_FM				0.010

typedef struct osc_setting
{
	float32_t volume;

	float32_t vco_freq;
	float32_t vco2_freq;
	float32_t lfo_freq;

	selector_state vco_wav;
	selector_state lfo_wav;

	modulation_state mod;

//	uint16_t am_mod;
//	uint16_t fm_mod;

	float32_t vco_amp;
	float32_t vco_amp2;

	float32_t lfo_amp;
	float32_t lfo_amp_am;
	float32_t lfo_amp_fm;

	// float32_t lfo_offset;

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
	modulation_state mod;

	float32_t sustain_amp;

	uint32_t attack_len;
	uint32_t decay_len;
	uint32_t sustain_len;
	uint32_t release_len;
	uint32_t blank_len;			// Blank time between 'note'.  Can be zero.

	float32_t attack_len_rad;
	float32_t decay_len_rad;
	float32_t sustain_len_rad;
	float32_t release_len_rad;
	float32_t blank_len_rad;			// Blank time between 'note'.  Can be zero.

} adsr_setting;


void generate_waveforms(uint16_t start, uint16_t end);
void generate_waveforms2(uint16_t start, uint16_t end);

void adsr(uint16_t start, uint16_t end);
void adsr_rad(uint16_t start, uint16_t end);

float32_t gen_square_angle(float32_t angle);
float32_t gen_sawtooth_angle(float32_t angle);

// TODO: test then rename
float32_t gen_sawtooth_angle2( float32_t angle, float32_t delta, uint32_t len);

float32_t gen_sawtooth_integral_angle(float32_t angle);
float32_t gen_rampdown_angle(float32_t angle);
float32_t gen_rampdown_angle2( float32_t angle, float32_t min, float32_t max);

float32_t gen_triangle_angle(float32_t angle);
float32_t gen_triangle_integral_angle(float32_t angle);

float32_t fast_fmod(float32_t x, float32_t y);
uint32_t moving_avg(uint32_t *ptrArrNumbers, uint32_t *ptrSum, uint32_t pos, uint32_t len, uint16_t nextNum);
uint16_t pseudo_log(uint16_t x);
#endif /* OSC_H_ */
