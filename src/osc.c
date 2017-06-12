/*
 * osc.c
 *
 *  Created on: Jun 12, 2017
 *      Author: admin
 */

#include "osc.h"
#include <stdint.h>
#include <stdlib.h>


volatile uint16_t buffer_vco[BUFF_LEN] = {0};
// volatile uint16_t buffer_lfo[BUFF_LEN] = {0};
volatile float32_t buffer_lfo_float[BUFF_LEN] = {0};
volatile uint16_t buffer_output[BUFF_LEN] = {0};
volatile float32_t buffer_adsr[BUFF_LEN] = {0};			// Attack, sustain, decay, release

volatile uint16_t mov_avg [MOV_AVG_BUFF_LEN] = {0};
volatile uint16_t mov_avg_index = 0;
volatile uint16_t mov_avg_sum;

// Good test frequency: freq_vco = 410
volatile uint16_t freq_vco = 700;
// volatile uint16_t freq_vco = 750.0;
volatile float32_t freq_lfo = 2.0;						// Moderate LFO frequency
// volatile uint16_t freq_lfo = 501;				// For testing LFO
// volatile uint16_t freq_vco = 375.0;				// Pure sine if BUFF_LEN is 128

// volatile uint16_t sample_count = 0;
volatile uint32_t sample_count = 0;


uint16_t adsr = 0;

uint16_t wav_vco = WAVE_SINE;
uint16_t wav_lfo = WAVE_SAWTOOTH;
uint16_t mod_type = MOD_FM;

float32_t vco_amp = VCO_AMP;
float32_t lfo_amp = 1.0;
float32_t lfo_offset = 2.0;

float32_t square_min = 0.4;
float32_t square_max = 1.0;

float32_t sawtooth_vco_min = 0.0;
float32_t sawtooth_vco_max = 1.0;

float32_t sawtooth_lfo_min = 0.0;
float32_t sawtooth_lfo_max = 1.0;

float32_t fm_mod_level = 0.6;

// ADSR - Attack Decay Sustain Release
uint32_t a_start = 0;
uint32_t d_start = 12000;		// So attack is 12000 samples
uint32_t s_start = 18000;		// So decay is 6000 samples
uint32_t r_start = 54000;		// So sustain is 12000 samples
uint32_t r_end =   60000;		// release is ...

uint32_t attack_len =  12000;
uint32_t decay_len =   6000;
uint32_t sustain_len = 36000;
uint32_t release_len = 6000;



// For first half of buffer, start = 0; end = buff_len/2
// For second half, start = buff_len/2; end = buff_len
void generate_waveforms(uint16_t start, uint16_t end)
{
	// Turns off yellow LED -- indicates no error occurred.
	//	STM_EVAL_LEDOff(LED3);
	// Moving average filter for ADC3 (PC3)
	//	mov_avg[mov_avg_index] = ADC3ConvertedValue;					// Get newest value
	//	mov_avg_sum += ADC3ConvertedValue;								// Accumulate
	//	mov_avg_sum -= mov_avg[(mov_avg_index + 1) % MOV_AVG_BUFF_LEN];	// Remove oldest
	//	mov_avg_index = (mov_avg_index + 1) % MOV_AVG_BUFF_LEN;							// Increment index
	//	freq_vco = 4.0 * ( (float32_t)  mov_avg_sum)/MOV_AVG_BUFF_LEN;

	// TODO: test frequency accuracy.
	//		Sinusoids are correct, but square and sawtooth aren't.
	// TODO: Consider using arm_sin_q15 instead of arm_sin_f32.  However, results might cause clipping.
	// TODO: Store vco default amplitude in global and set with a defined value.

	// Odd frequencies cause beating.
	// if (freq_vco % 2) { /* x is odd */  freq_vco +=1; }
	// if (freq_lfo % 2) { /* x is odd */  freq_lfo +=1; }

	volatile int i = 0;

	// freq_vco = 2 * ADC3ConvertedValue;

	// TODO: test this.
	// freq_lfo =   ( (float32_t)( ADC3ConvertedValue & 0xffb )/10);

	volatile float32_t angle_vco = freq_vco*PI/SAMPLERATE;
	volatile float32_t angle_lfo = freq_lfo*PI/SAMPLERATE;

	// For square
	// Fill buffer from 0 to BUFF_LEN/2
	volatile uint32_t samples_half_cycle_vco = SAMPLERATE/freq_vco;
	volatile uint32_t samples_cycle_vco = 2*samples_half_cycle_vco;

	volatile uint32_t samples_half_cycle_lfo = SAMPLERATE/freq_lfo;
	volatile uint32_t samples_cycle_lfo = 2*samples_half_cycle_lfo;

	// TODO: remove after testing.
	// memset(buffer_vco, 0, sizeof(buffer_vco));
	// memset(buffer_output, 0, sizeof(buffer_output));

	// Sine VCO
	// TODO: to save cpu cycles, consider calculating one cycle of the sine wave and then copying it into the rest of the buffer.
	if(wav_vco == WAVE_SINE && mod_type != MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp + vco_amp*arm_sin_f32((sample_count+(i-start))*angle_vco);
		}
	}

	// Square VCO
	else if(wav_vco == WAVE_SQUARE && mod_type != MOD_FM)
	{
		/*
		 * In a single square pulse cycle, there are n samples, each of which is 1/48000s long.
		 * Therefore,  T = n/48000
		 * --> n = 48000*T
		 * --> n = 48000/f
		 * Therefore, duration of positive (one) half is n/2 = 48000/2f.  Same for negative (zero) half.
		 *
		 */

		// samples_cycle = 2*(SAMPLERATE/freq_vco);
		// samples_half_cycle = samples_cycle/2;

		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp * square((sample_count+(i-start)) % samples_cycle_vco, samples_half_cycle_vco);
		}
	}

	// Sawtooth VCO
	else if(wav_vco == WAVE_SAWTOOTH && mod_type != MOD_FM)
	{
		// samples_cycle = 2*(SAMPLERATE/freq_vco);

		for(i = start; i < end; i++)
		{
			// TODO: store amplitude in a variable.
			buffer_vco[i] = vco_amp * sawtooth(samples_cycle_vco - ((sample_count+(i-start)) % samples_cycle_vco), samples_cycle_vco, sawtooth_vco_min, sawtooth_vco_max);
		}
	}

	else if(wav_vco == WAVE_TRIANGLE && mod_type != MOD_FM)
	{
		// samples_half_cycle = SAMPLERATE/freq_vco;
		// samples_cycle = 2 * samples_half_cycle;

		for(i = start; i < end; i++)
		{
			// TODO: store amplitude in a variable.
			// TODO: offset for triangle...?
			buffer_vco[i] = vco_amp * triangle( (sample_count+(i-start)) % samples_cycle_vco, samples_half_cycle_vco, 1.0);
			// buffer_vco[i] = triangle( (sample_count+(i-start)) % samples_cycle, samples_half_cycle, 1.0);

			// TODO: testing...
			// buffer_lfo_float[i] = triangle( (sample_count+(i-start)) % samples_cycle, samples_half_cycle, 1.0);
		}
	}

	// SINE LFO
	if(wav_lfo == WAVE_SINE)
	{
		if(wav_vco == WAVE_SAWTOOTH || wav_vco == WAVE_TRIANGLE)
		{
			lfo_offset = 0.5;
			lfo_amp = 0.25;
		}

		for(i = start; i < end; i++)
		{
			// buffer_lfo_float[i] = 0.4 + 0.4*arm_sin_f32((sample_count+i)*angle_lfo);		// Small amplitude for AM mod of sine
			// buffer_lfo_float[i] = 40.0 + 40.0*arm_sin_f32((sample_count+i)*angle_lfo);	// Large amplitude for FM mod of sine
			// buffer_lfo_float[i] = 10 + 10*arm_sin_f32((sample_count+i)*angle_lfo);		// Medium amplitude for FM mod of square

			// TODO: uncomment after testing.
			buffer_lfo_float[i] = lfo_offset + lfo_amp*arm_sin_f32((sample_count+(i-start))*angle_lfo);

			// TODO: testing for FM mod of sawtooth
			// buffer_lfo_float[i] = 0.5 + 0.25*arm_sin_f32((sample_count+(i-start))*angle_lfo);
		}
	}

	// Square LFO
	// TODO: amplitude adjustment -- so it's not just 000011111, but could be 0.2 0.2 0.2 0.6 0.6 0.6
	else if(wav_lfo == WAVE_SQUARE)
	{
		// samples_cycle = 2*(SAMPLERATE/freq_lfo);
		// samples_half_cycle = samples_cycle/2;

		for(i = start; i < end; i++)
		{
			buffer_lfo_float[i] = square((sample_count+(i-start)) % samples_cycle_lfo, samples_half_cycle_lfo);
		}
	}

	// Sawtooth LFO
	else if(wav_lfo == WAVE_SAWTOOTH)
	{
		// samples_cycle = 2*(SAMPLERATE/freq_lfo);

		// TODO: TEST For FM modulation, sawtooth shape LFO is one way
		// 	     For AM modulation, sawtooth shape is the other way

		if(mod_type != MOD_FM)
		{
			for(i = start; i < end; i++)
			{
				// buffer_lfo_float[i] = sawtooth(samples_cycle - (sample_count+(i-start)) % samples_cycle, samples_cycle, sawtooth_lfo_min, sawtooth_lfo_max);

				buffer_lfo_float[i] = sawtooth((sample_count+(i-start)) % samples_cycle_lfo, samples_cycle_lfo, sawtooth_lfo_min, sawtooth_lfo_max);
			}
		}

		// If FM mod, need integral of modulating signal.  Integral of ramp is right side of parabola.
		else
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = sawtooth((sample_count+(i-start)) % samples_cycle_lfo, samples_cycle_lfo, sawtooth_lfo_min, sawtooth_lfo_max);
				buffer_lfo_float[i] = buffer_lfo_float[i] * buffer_lfo_float[i];
			}
		}

	}

	else if(wav_lfo == WAVE_TRIANGLE)
	{
		// samples_half_cycle = SAMPLERATE/freq_lfo;
		// samples_cycle = 2 * samples_half_cycle;

		if(mod_type != MOD_FM)
		{
			for(i = start; i < end; i++)
			{
				// TODO: change 1.0 to variable.
				// 			Variable for min/max
				buffer_lfo_float[i] = triangle( (sample_count+(i-start)) % samples_cycle_lfo, samples_half_cycle_lfo, 1.0);
			}
		}

		// If FM mod, need integral of modulating signal.
		else
		{
			for(i = start; i < end; i++)
			{
				// TODO: change 1.0 to variable.
				// 			Variable for min/max
				buffer_lfo_float[i] = triangle_integral( (sample_count+(i-start)) % samples_cycle_lfo, samples_half_cycle_lfo, 1.0);
			}
		}
	}


	// No LFO
	else if(wav_lfo == WAVE_NONE)
	{
		// TODO: fill lfo buffer with zeros.
	}

	// AM modulation
	if(mod_type == MOD_AM)
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = buffer_vco[i] * buffer_lfo_float[i];
		}
	}

	// FM for sine wave VCO.
	else if(wav_vco == WAVE_SINE && mod_type == MOD_FM && wav_lfo == WAVE_SINE)
	{
		for(i = start; i < end; i++)
		{
			// Using 40 for sine modulated with sine
			// TODO: consider changing 40 to a variable.
			buffer_vco[i] = vco_amp + vco_amp*arm_sin_f32((sample_count+(i-start))*angle_vco + 40*buffer_lfo_float[i]);
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for sine wave VCO, square LFO.
	else if(wav_vco == WAVE_SINE && mod_type == MOD_FM && wav_lfo == WAVE_SQUARE)
	{
		for(i = start; i < end; i++)
		{
			// For modulating with square and sawtooth wave
			buffer_vco[i] = vco_amp + vco_amp*arm_sin_f32( (sample_count+(i-start))*angle_vco*buffer_lfo_float[i] );
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for sine wave VCO.
	// TODO: this is messed up.
	// Frequency keeps increasing.  I think it's a phase thing...
	// If you + buffer_lfo_float.... no change in freq.  If you mult, it climbs and falls.
	else if(wav_vco == WAVE_SINE && mod_type == MOD_FM && ( wav_lfo == WAVE_SAWTOOTH ||  wav_lfo == WAVE_TRIANGLE))
	{
		for(i = start; i < end; i++)
		{
			// For modulating with square and sawtooth wave
			// buffer_vco[i] = vco_amp + vco_amp*arm_sin_f32( (sample_count+(i-start))*angle_vco*buffer_lfo_float[i] );
			// buffer_vco[i] = vco_amp + vco_amp*arm_sin_f32( ( (sample_count+(i-start)) % samples_cycle_lfo) *angle_vco*buffer_lfo_float[i] );
			buffer_vco[i] = vco_amp + vco_amp*arm_sin_f32( ( (sample_count+(i-start)) % samples_cycle_lfo) * angle_vco + 1000*buffer_lfo_float[i] );
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for square wave VCO.
	// TODO: Fix glitchiness. I think just needs offset, fm_mod_level, etc adjusted.
	else if(wav_vco == WAVE_SQUARE && mod_type == MOD_FM)
	{
		// samples_cycle = 2*(SAMPLERATE/freq_vco);
		// samples_half_cycle = samples_cycle/2;

		for(i = start; i < end; i++)
		{
			// buffer_vco[i] = vco_amp * square( (sample_count+(i-start)) % ( (uint16_t)(samples_cycle + 20*buffer_lfo_float[i]) ), ( samples_cycle + 20*buffer_lfo_float[i])/2 );

			// Is it samples_cycle_vco or _lfo??
			buffer_vco[i] = vco_amp * square( (sample_count+(i-start)) % ( (uint16_t)(samples_cycle_vco*fm_mod_level*buffer_lfo_float[i]) ), ( samples_cycle_vco*fm_mod_level*buffer_lfo_float[i])/2 );
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for sawtooth wave VCO.
	// TODO: Works for sine LFO.  Crappy for sawtooth LFO.  Check gain, offset.
	else if(wav_vco == WAVE_SAWTOOTH && mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			// buffer_vco[i] = 40 * sawtooth(  samples_cycle - ((sample_count+i) % samples_cycle));

			// During call to sawtooth, I think do...
			//		samples_cycle - (sample_count+(i-start)) ...
			// Because, otherwise the sawtooth waveform appears backwards.
			buffer_vco[i] = vco_amp * sawtooth( samples_cycle_vco - (sample_count+(i-start)) % ( (uint16_t)(samples_cycle_vco*fm_mod_level*buffer_lfo_float[i]) ), samples_cycle_vco*fm_mod_level*buffer_lfo_float[i], sawtooth_vco_min, sawtooth_vco_max);
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for triangle wave VCO.
	// TODO: fix this...
	else if(wav_vco == WAVE_TRIANGLE && mod_type == MOD_FM)
	{
		// samples_half_cycle = SAMPLERATE/freq_vco;
		// samples_cycle = 2* samples_half_cycle;

		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp * triangle( (sample_count+(i-start)) % ( (uint16_t)(samples_cycle_vco*fm_mod_level*buffer_lfo_float[i]) ), samples_half_cycle_vco*fm_mod_level*buffer_lfo_float[i], 1.0);
			buffer_output[i] = buffer_vco[i];
		}
	}

	// No modulation
	else
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = buffer_vco[i];
		}
	}

	// ADSR: Attack decay sustain release
	// The waveform contains 4 segments.
	if(adsr)
	{
		for(i = start; i < end; i++)
		{
			if(sample_count+(i-start) < d_start)
			{
				// Attack
				// buffer_adsr[i] = 0.2;
				// sawtooth(current sample, samples per cycle, min, max)
				// buffer_lfo_float[i] = sawtooth(samples_cycle - (sample_count+(i-start)) % samples_cycle, samples_cycle, sawtooth_lfo_min, sawtooth_lfo_max);
				buffer_adsr[i] = 0.5 * sawtooth( (sample_count+(i-start)) % d_start, d_start/2, 0.0, 1.0);

			}
			else if(sample_count+(i-start) < s_start)
			{
				// Decay
				// buffer_adsr[i] = 0.4;
				buffer_adsr[i] = rampdown((sample_count+(i-start)) % s_start-d_start, s_start-d_start, 0.5, 1.0);
			}
			else if(sample_count+(i-start) < r_start)
			{
				// Sustain
				// buffer_adsr[i] = 0.6;
				// buffer_adsr[i] = sawtooth((sample_count+(i-start)) % samples_cycle, r_start-s_start, 0.5, 0.5);
				buffer_adsr[i] = rampdown((sample_count+(i-start)) % s_start-d_start, s_start-d_start, 0.5, 0.5);
			}
			else if(sample_count+(i-start) < r_end)
			{
				// Release
				// buffer_adsr[i] = 1.0;
				// rampdown(current sample, samples/cycle, min value, max value)
				buffer_adsr[i] = rampdown( ( sample_count+(i-start) ) % r_end-r_start, r_end-r_start, 0.0, 0.5);
			}
			else
			{
				buffer_adsr[i] = 0;
			}

			buffer_output[i] = buffer_output[i] * buffer_adsr[i];
		}
	}

	// Remember lfo phase and resume next run of callback.
	// TODO: This line may be causing ticking.
	//		 Might be able to rollover at end of (vfo? lfo?) waveform instead of samplerate.
	// 		However.. might need to also account for size of integer.

	// sample_count = (sample_count + (i-start)) % SAMPLERATE;
	//
	sample_count = (sample_count + (i-start)) % TEN_SECOND;

	return;
}


/*
 * square()
 * Returns 0 if 0.0 < current_sample < samples_half_cycle
 * Returns 1 if current_sample > samples_half_cycle
 *
 * Parameter angle: value from 0.0 to 1.0.
 */
float32_t square(uint16_t current_sample, uint16_t samples_half_cycle)
{
	if (current_sample < samples_half_cycle)
	{
		return square_min;
	}
	return square_max;
}

/*
 * sawtooth()
 * Returns value ranging linearly from min to max, depending on input value.
 * The function is a linear function, f(x) = mx + b.
 *		b is the min output value, set by sawtooth_min (global variable).
 *		m*(xmax) + b is the max output value set by sawtooth_max (global variable).
 *
 *		Global variables sawtooth_min and sawtooth_max correspond to delta-y and are used to calculate the
 *		slope of the sawtooth wave.

 *		Parameter samples_cycle is the number of samples in a cycle.  This corresponds to delta-x and is used to
 *		calculate the slope of the sawtooth wave.
 *
 *		The global variables and samples_cycle comprise delta-y/delta-x, which equals slope m.
 *
 *		Parameter current_sample is the n'th sample in the current cycle.  This corresponds to x in f(x) = mx+b

 *
 */
float32_t sawtooth(uint32_t current_sample, uint32_t samples_cycle, float32_t min, float32_t max)
{
	float32_t m = 0.0;
	float32_t val = 0.0;

	// y = mx + b
//	m = (max - min)/samples_cycle;
//	val = m * current_sample + min;

	m = (max - min)/(samples_cycle-1);
	val = (m * current_sample) + min;

	return val;
}

float32_t rampdown(uint32_t current_sample, uint32_t samples_cycle, float32_t min, float32_t max)
{
	float32_t m = 0.0;
	float32_t val = 0.0;

	// y = mx + b
	m = (max - min)/samples_cycle;
	val = max - m * current_sample + min;

	return val;
}

float32_t triangle(uint32_t current_sample, uint32_t samples_half_cycle, float32_t amp)
{
	float32_t m = 0.0;
	float32_t result = 0.0;
	// float32_t val = 0.0;

	// Increase from a negative value to its opposite value. Eg. -1 to 1 over 1/2 the wave's period
	// Then decrease from 1 to -1 over 1/2 the wave's period

	m = amp/(samples_half_cycle);

	if(current_sample < samples_half_cycle)
	{
		return (m * current_sample);
	}
	// Make sure difference can be negative.
	return amp + (m * (int32_t)(samples_half_cycle - current_sample));
}


float32_t triangle_integral(uint32_t current_sample, uint32_t samples_half_cycle, float32_t amp)
{
	float32_t m = 0.0;
	float32_t result = 0.0;
	// float32_t val = 0.0;

	// Increase from a negative value to its opposite value. Eg. -1 to 1 over 1/2 the wave's period
	// Then decrease from 1 to -1 over 1/2 the wave's period

	m = amp/(samples_half_cycle);

	if(current_sample < samples_half_cycle)
	{
		result = m*current_sample;
		return result*result;


	}
	// TODO: testing with integral...
	// Make sure difference can be negative.
	// return amp + (m * (int32_t)(samples_half_cycle - current_sample));

	result = amp + (m * (int32_t)(samples_half_cycle - current_sample));
	return -(result*result);
}
