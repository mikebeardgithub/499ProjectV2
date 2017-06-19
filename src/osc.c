/*
 * osc.c
 *
 *  Created on: Jun 12, 2017
 *      Author: admin
 */

#include "osc.h"
#include <stdint.h>
#include <stdlib.h>


volatile uint16_t buffer_output[BUFF_LEN] = {0};		// TODO: Can probably get rid of this.
volatile uint16_t buffer_vco[BUFF_LEN] = {0};
volatile float32_t buffer_lfo_float[BUFF_LEN] = {0};
volatile float32_t buffer_adsr[BUFF_LEN] = {0};			// Attack, sustain, decay, release

// TODO: organize these globals into struct(s)
volatile uint16_t freq_vco = 650;
volatile float32_t freq_lfo = 10.0;						// Moderate LFO frequency
volatile uint32_t sample_count = 0;

uint16_t wav_vco = WAVE_SQUARE;
uint16_t wav_lfo = WAVE_SQUARE;
uint16_t mod_type = MOD_NONE;

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
uint16_t adsr = 1;									// Enable/disable ADSR.

// Set ADSR lengths in numbers of samples.
// It's easier to think in terms of length, but it's easier to program in terms of start-end.
float32_t attack_amp = 1.0;
float32_t decay_amp = 0.5;
float32_t sustain_amp = 0.5;

uint32_t attack_len =  400;
uint32_t decay_len =   400;
uint32_t sustain_len = 6000;
uint32_t release_len = 700;
uint32_t blank_len = 8000;			// Blank time between 'note'.  Can be zero.

// Start-end samples.  These are calculated later.
// TODO: probably don't need to be global.
uint32_t attack_start = 0;
uint32_t decay_start = 0;
uint32_t sustain_start = 0;
uint32_t release_start = 0;
uint32_t blank_start = 0;
uint32_t blank_end = 0;

// extern uint16_t adc_value;


// For first half of buffer, start = 0; end = buff_len/2
// For second half, start = buff_len/2; end = buff_len
void generate_waveforms(uint16_t start, uint16_t end)
{
	// Calculate start-end boundaries for each of attack, sustain, ...
	// TODO: uncomment after testing.
//	a_start = 0;
//	d_start = attack_len;
//	s_start = d_start + decay_len;
//	r_start = s_start + sustain_len;
//	b_start = r_start + release_len;
//	b_end = b_start + blank_len;

	attack_start = 0;
	decay_start = attack_len;
	sustain_start = decay_start + decay_len;
	release_start = sustain_start + sustain_len;
	blank_start = release_start + release_len;
	blank_end = blank_start + blank_len;

	// TODO: test frequency accuracy.
	// TODO: Consider using arm_sin_q15 instead of arm_sin_f32.  However, results might cause clipping.
	// TODO: Store vco default amplitude in global and set with a defined value.

	volatile int i = 0;

	// freq_lfo = adc_value;

	// For sine waveforms.
	volatile float32_t angle_vco = freq_vco*PI/SAMPLERATE;
	volatile float32_t angle_lfo = freq_lfo*PI/SAMPLERATE;

	// For non-sine waveforms.
	volatile uint32_t samples_half_cycle_vco = SAMPLERATE/freq_vco;
	volatile uint32_t samples_cycle_vco = 2*samples_half_cycle_vco;

	volatile uint32_t samples_half_cycle_lfo = SAMPLERATE/freq_lfo;
	volatile uint32_t samples_cycle_lfo = 2*samples_half_cycle_lfo;

	// For adsr
	volatile uint32_t sample_cycle_adsr = attack_len + decay_len + sustain_len + blank_len;

	// Sine VCO
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

		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp * gen_square((sample_count+(i-start)) % samples_cycle_vco, samples_half_cycle_vco);
		}
	}

	// Sawtooth VCO
	else if(wav_vco == WAVE_SAWTOOTH && mod_type != MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp * gen_sawtooth(samples_cycle_vco - ((sample_count+(i-start)) % samples_cycle_vco), samples_cycle_vco, sawtooth_vco_min, sawtooth_vco_max);
		}
	}

	else if(wav_vco == WAVE_TRIANGLE && mod_type != MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp * gen_triangle( (sample_count+(i-start)) % samples_cycle_vco, samples_half_cycle_vco, 1.0);
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
		for(i = start; i < end; i++)
		{
			buffer_lfo_float[i] = gen_square((sample_count+(i-start)) % samples_cycle_lfo, samples_half_cycle_lfo);
		}
	}

	// Sawtooth LFO
	else if(wav_lfo == WAVE_SAWTOOTH)
	{
		// TODO: TEST For FM modulation, sawtooth shape LFO is one way
		// 	     For AM modulation, sawtooth shape is the other way

		if(mod_type != MOD_FM)
		{
			for(i = start; i < end; i++)
			{
				// buffer_lfo_float[i] = sawtooth(samples_cycle - (sample_count+(i-start)) % samples_cycle, samples_cycle, sawtooth_lfo_min, sawtooth_lfo_max);

				buffer_lfo_float[i] = gen_sawtooth((sample_count+(i-start)) % samples_cycle_lfo, samples_cycle_lfo, sawtooth_lfo_min, sawtooth_lfo_max);
			}
		}

		// If FM mod, need integral of modulating signal.  Integral of ramp is right side of parabola.
		else
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_sawtooth((sample_count+(i-start)) % samples_cycle_lfo, samples_cycle_lfo, sawtooth_lfo_min, sawtooth_lfo_max);
				buffer_lfo_float[i] = buffer_lfo_float[i] * buffer_lfo_float[i];
			}
		}
	}

	else if(wav_lfo == WAVE_TRIANGLE)
	{
		if(mod_type != MOD_FM)
		{
			for(i = start; i < end; i++)
			{
				// TODO: change 1.0 to variable.
				// 			Variable for min/max
				buffer_lfo_float[i] = gen_triangle( (sample_count+(i-start)) % samples_cycle_lfo, samples_half_cycle_lfo, 1.0);
			}
		}

		// If FM mod, need integral of modulating signal.
		else
		{
			for(i = start; i < end; i++)
			{
				// TODO: change 1.0 to variable.
				// 			Variable for min/max
				buffer_lfo_float[i] = gen_triangle_integral( (sample_count+(i-start)) % samples_cycle_lfo, samples_half_cycle_lfo, 1.0);
			}
		}
	}

	// No LFO
	else if(wav_lfo == WAVE_NONE)
	{
		// TODO: fill lfo buffer with zeros?
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
	// TODO: Use integral fm modulation formula.  See if sawtooth vco version works here.
	else if(wav_vco == WAVE_SQUARE && mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			// buffer_vco[i] = vco_amp * square( (sample_count+(i-start)) % ( (uint16_t)(samples_cycle + 20*buffer_lfo_float[i]) ), ( samples_cycle + 20*buffer_lfo_float[i])/2 );

			buffer_vco[i] = vco_amp * gen_square( (sample_count+(i-start)) % ( (uint16_t)(samples_cycle_vco*fm_mod_level*buffer_lfo_float[i]) ), ( samples_cycle_vco*fm_mod_level*buffer_lfo_float[i])/2 );
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
			buffer_vco[i] = vco_amp * gen_sawtooth( samples_cycle_vco - (sample_count+(i-start)) % ( (uint16_t)(samples_cycle_vco*fm_mod_level*buffer_lfo_float[i]) ), samples_cycle_vco*fm_mod_level*buffer_lfo_float[i], sawtooth_vco_min, sawtooth_vco_max);
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for triangle wave VCO.
	// TODO: fix this...
	else if(wav_vco == WAVE_TRIANGLE && mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp * gen_triangle( (sample_count+(i-start)) % ( (uint16_t)(samples_cycle_vco*fm_mod_level*buffer_lfo_float[i]) ), samples_half_cycle_vco*fm_mod_level*buffer_lfo_float[i], 1.0);
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
	// The waveform contains 5 segments (asdr + a blank space)
	// TODO: There is a tick every 10s... seems specific to the ADSR.
	if(adsr)
	{
		for(i = start; i < end; i++)
		{
			// First part tells us sample number into the adsr cycle: (sample_count+(i-start))%sample_cycle_adsr
			if( (sample_count+(i-start))%sample_cycle_adsr < decay_start)
			{
				// Attack
				buffer_adsr[i] = gen_sawtooth( (sample_count+(i-start)) % sample_cycle_adsr, attack_len, 0.0, attack_amp);

			}

			else if( (sample_count+(i-start))%sample_cycle_adsr < sustain_start)
			{
				// Decay
				buffer_adsr[i] = gen_rampdown((sample_count+i-start-decay_start) % sample_cycle_adsr, decay_len, decay_amp, attack_amp);
			}

			else if( (sample_count+(i-start))%sample_cycle_adsr < release_start)
			{
				// Sustain
				buffer_adsr[i] = gen_rampdown((sample_count+i-start-sustain_start) % sample_cycle_adsr, sustain_len, sustain_amp, sustain_amp);
			}

			else if( (sample_count+(i-start))%sample_cycle_adsr < blank_start)
			{
				// Release
				buffer_adsr[i] = gen_rampdown( (sample_count+i-start-release_start) % sample_cycle_adsr, release_len, 0.0, sustain_amp);

			}
			else if( (sample_count+(i-start))%sample_cycle_adsr < blank_end)
			{
				// Blank
				buffer_adsr[i] = 0;
			}

			buffer_output[i] = buffer_output[i] * buffer_adsr[i];
		}
	}

	// Remember lfo phase and resume next run of callback.
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
float32_t gen_square(uint16_t current_sample, uint16_t samples_half_cycle)
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
float32_t gen_sawtooth(uint32_t current_sample, uint32_t samples_cycle, float32_t min, float32_t max)
{
	float32_t m = 0.0;
	float32_t val = 0.0;

	// y = mx + b
	m = (max - min)/samples_cycle;
	val = (m * current_sample) + min;

	return val;
}

float32_t gen_rampdown(uint32_t current_sample, uint32_t samples_cycle, float32_t min, float32_t max)
{
	float32_t m = 0.0;
	float32_t val = 0.0;

	// y = mx + b
	// m = (max - min)/samples_cycle;
	// val = max - m * current_sample + min;

	m = (min - max)/samples_cycle;
	val = m * current_sample + max;

	return val;
}

float32_t gen_triangle(uint32_t current_sample, uint32_t samples_half_cycle, float32_t amp)
{
	float32_t m = 0.0;

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


float32_t gen_triangle_integral(uint32_t current_sample, uint32_t samples_half_cycle, float32_t amp)
{
	float32_t m = 0.0;
	float32_t result = 0.0;

	// Increase from a negative value to its opposite value. Eg. -1 to 1 over 1/2 the wave's period
	// Then decrease from 1 to -1 over 1/2 the wave's period
	m = amp/(samples_half_cycle);

	if(current_sample < samples_half_cycle)
	{
		result = m*current_sample;
		return result*result;
	}
	// Make sure difference can be negative.
	result = amp + (m * (int32_t)(samples_half_cycle - current_sample));
	return -(result*result);
}
