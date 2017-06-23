/*
 * osc.c
 *
 *  Created on: Jun 12, 2017
 *      Author: admin
 */

#include "osc.h"
#include <stdint.h>
#include <stdlib.h>


volatile uint16_t buffer_output[BUFF_LEN] = {0};
volatile uint16_t buffer_vco[BUFF_LEN] = {0};
volatile float32_t buffer_lfo_float[BUFF_LEN] = {0};
volatile float32_t buffer_adsr[BUFF_LEN] = {0};			// Attack, sustain, decay, release

// TODO: organize these globals into struct(s)
volatile uint16_t freq_vco = 650;
volatile float32_t freq_lfo = 10.0;						// Moderate LFO frequency
volatile uint32_t sample_count = 0;

uint16_t wav_vco = WAVE_SQUARE;
uint16_t wav_lfo = WAVE_SINE;
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
uint16_t adsr = 1;									// Enable/disable ADSR.

// Set ADSR lengths in numbers of samples.
// It's easier to think in terms of length, but it's easier to program in terms of start-end.
float32_t attack_amp = 1.0;
float32_t decay_amp = 0.3;
float32_t sustain_amp = 0.3;

uint32_t attack_len =  400;
uint32_t decay_len =   400;
uint32_t sustain_len = 6500;
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
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp + vco_amp * gen_square_angle((sample_count+(i-start)) * angle_vco);
		}
	}

	// Sawtooth VCO
	else if(wav_vco == WAVE_SAWTOOTH && mod_type != MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp + vco_amp * gen_sawtooth_angle((sample_count+(i-start)) * angle_vco);
		}
	}

	else if(wav_vco == WAVE_TRIANGLE && mod_type != MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp + vco_amp * gen_triangle_angle((sample_count+(i-start)) * angle_vco);
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
			buffer_lfo_float[i] = lfo_offset + lfo_amp*arm_sin_f32((sample_count+(i-start))*angle_lfo);
		}
	}

	// Square LFO
	// TODO: amplitude adjustment -- so it's not just 000011111, but could be 0.2 0.2 0.2 0.6 0.6 0.6
	else if(wav_lfo == WAVE_SQUARE)
	{
		if(mod_type != MOD_FM)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_square_angle((sample_count+(i-start))*angle_lfo);
			}
		}
		else
		{
			for(i = start; i < end; i++)
			{
				// Sawtooth is integral of triangle
				buffer_lfo_float[i] = gen_triangle_angle((sample_count+(i-start))*angle_lfo);
			}
		}
	}

	// Sawtooth LFO
	else if(wav_lfo == WAVE_SAWTOOTH)
	{
		if(mod_type != MOD_FM)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_sawtooth_angle((sample_count+(i-start))*angle_lfo);
			}
		}

		// If FM mod, need integral of modulating signal.  Integral of ramp is right side of parabola.
		else
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_sawtooth_integral_angle((sample_count+(i-start))*angle_lfo);
			}
		}
	}

	else if(wav_lfo == WAVE_TRIANGLE)
	{
		if(mod_type != MOD_FM)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_triangle_angle( (sample_count+(i-start)) * angle_lfo);
			}
		}

		// If FM mod, need integral of modulating signal.
		else
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_triangle_integral_angle( (sample_count+(i-start)) * angle_lfo);
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
	else if(wav_vco == WAVE_SINE && mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			// Using 40 for sine modulated with sine
			// TODO: consider changing 1000 to a variable.
			buffer_vco[i] = vco_amp + vco_amp*arm_sin_f32((sample_count+(i-start))*angle_vco + 1000*buffer_lfo_float[i]);
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for square wave VCO.
	else if(wav_vco == WAVE_SQUARE && mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp + vco_amp * gen_square_angle((sample_count+(i-start))*angle_vco + 50*buffer_lfo_float[i]);
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for sawtooth wave VCO.
	else if(wav_vco == WAVE_SAWTOOTH && mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp + vco_amp * gen_sawtooth_angle((sample_count+(i-start))*angle_vco + 400*buffer_lfo_float[i]);
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for triangle wave VCO.
	else if(wav_vco == WAVE_TRIANGLE && mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = vco_amp + vco_amp * gen_triangle_angle((sample_count+(i-start))*angle_vco + 100*buffer_lfo_float[i]);
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
	// TODO: There is a tick every 10s... seems specific to the ADSR.  Caused by sample_count rollover.
	// TODO: Choose some good adsr preset values.
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

	sample_count = sample_count + (i - start);
	sample_count = sample_count % TWO_SECOND;

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

/* Parameters:
 * 	angle: normalized angle between 0 and 2*PI.  Similar to sine function.
 */
float32_t gen_square_angle(float32_t angle)
{
	angle = fmod(angle, 2*PI);
	if (angle < PI)
	{
		return -1;
	}
	return 1;
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

float32_t gen_sawtooth_angle(float32_t angle)
{
	float32_t m = 0.0;
	float32_t val = 0.0;

	angle = fmod(angle, 2*PI);

	// y = mx + b
	m = 2/(2*PI);	// TODO: # define for PI, 2*PI, 1/2*PI
	val = -1+angle*m;
	return val;
}

float32_t gen_sawtooth_integral_angle(float32_t angle)
{
	float32_t val = 0.0;
	float32_t m = 0.0;

	angle = fmod(angle, 2*PI);		// TODO: pull this out into generate_waveforms().
	m = 1/(2*PI);					// TODO: # define for PI, 2*PI, 1/2*PI

	// val = -1+angle*m;

	val = m*angle;			// Generate linear value between 0 and 1
	val = val*val;			// Square it.  Produces parabola y: 0 to 1
	val = val*2;			// Double it.
	val = val - 1;			// Shift it down
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

float32_t gen_rampdown_angle(float32_t angle)
{
	float32_t m = 0.0;
	float32_t val = 0.0;

	angle = fmod(angle, 2*PI);		// TODO: pull this out into generate_waveforms().

	// y = mx + b
	m = -1/(2*PI);
	val = angle*m;
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

float32_t gen_triangle_angle(float32_t angle)
{
	float32_t val = 0.0;
	float32_t m = 0.0;

	// Increase from a negative value to its opposite value. Eg. -1 to 1 over 1/2 the wave's period
	// Then decrease from 1 to -1 over 1/2 the wave's period

	angle = fmod(angle, 2*PI);		// TODO: pull this out into generate_waveforms().
	m = 2/(PI);
	if (angle < PI)
	{
		val = -1 + m*angle;
		return val;
	}
	// Make sure difference can be negative.
	// return amp + (m * (int32_t)(samples_half_cycle - current_sample));
	val =  3 - m*angle;
	return val;
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

// Integral of triangle wave is convex parabola going up and then concave parabola going down.
float32_t gen_triangle_integral_angle(float32_t angle)
{
	float32_t val = 0.0;
	float32_t m = 0.0;

	angle = fmod(angle, 2*PI);		// TODO: pull this out into generate_waveforms().
	m = 1/(PI);
	if (angle < PI)

	if(angle < PI)
	{
		val = m*angle;			// Generate linear value between 0 and 1
		val = val*val;			// Square it.  Produces parabola y: 0 to 1
		val = val*2;			// Double it.
		val = val - 1;			// Shift it down
		return val;
	}

	angle = angle - PI;
	val = m*angle;			// Generate linear value between 0 and 1
	val = val*val;			// Square it.  Produces parabola y: 0 to 1
	val = 1 - val;			// Turn it upside down
	val = val*2;			// Double it
	val = val - 1;			// Shift it down
	return val;
}
