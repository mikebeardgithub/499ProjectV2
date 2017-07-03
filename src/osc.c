/*
 * osc.c
 *
 *  Created on: Jun 12, 2017
 *      Author: admin
 */

#include "osc.h"
#include <stdint.h>
#include <stdlib.h>

// #include <time.h>	// For testing.

volatile uint16_t buffer_output[BUFF_LEN] = {0};
volatile uint16_t buffer_vco[BUFF_LEN] = {0};
volatile uint16_t buffer_vco2[BUFF_LEN] = {0};
volatile float32_t buffer_lfo_float[BUFF_LEN] = {0};
volatile float32_t buffer_adsr[BUFF_LEN] = {0};			// Attack, sustain, decay, release

// TODO: organize these globals into struct(s)
// TODO: it might be that odd frequencies cause the ticking.  Might be able to detect with mod, fmod.
osc_setting osc =
{
	.freq_vco = 685.7,
	.freq_vco2 = 690.7,
	.freq_lfo = 1.75,						// Moderate LFO frequency

	.wav_vco = WAVE_TRIANGLE,
	.wav_lfo = WAVE_NONE,
	.mod_type = MOD_NONE,

	.vco_amp = VCO_AMP/2,
	.vco_amp2 = VCO_AMP/6,
	.lfo_offset = 2.0,

	.square_min = 0.4,
	.square_max = 1.0,

	.sawtooth_vco_min = 0.0,
	.sawtooth_vco_max = 1.0,

	.sawtooth_lfo_min = 0.0,
	.sawtooth_lfo_max = 1.0,

	.fm_mod_level = 0.6
};

volatile uint32_t sample_count = 0;						// TODO remove
volatile uint32_t sample_count_vco = 0;
volatile uint32_t sample_count_lfo = 0;
volatile uint32_t sample_count_adsr = 0;

// ADSR - Attack Decay Sustain Release
uint16_t adsr = ON;									// Enable/disable ADSR.

// Set ADSR lengths in numbers of samples.
// It's easier to think in terms of length, but it's easier to program in terms of start-end.
adsr_setting adsr_01 = {
		.attack_amp=1.0,
		.decay_amp=0.3,
		.sustain_amp=0.3,

		.attack_len=400,
		.decay_len=400,
		.sustain_len=6500,
		.release_len=700,
		.blank_len=8000
};

adsr_setting adsr_02 = {
		.attack_amp=1.0,
		.decay_amp=0.5,
		.sustain_amp=0.5,

		.attack_len=20,
		.decay_len=10000,
		.sustain_len=0,
		.release_len=10000,
		.blank_len=20000
};

adsr_setting adsr_03 = {
		.attack_amp=1.0,
		.decay_amp=0.3,
		.sustain_amp=0.5,

		.attack_len=5000,
		.decay_len=500,
		.sustain_len=30000,
		.release_len=900,
		.blank_len=40000
};


// Start-end samples.  These are calculated later.
// TODO: probably don't need to be global.
uint32_t attack_start = 0;
uint32_t decay_start = 0;
uint32_t sustain_start = 0;
uint32_t release_start = 0;
uint32_t blank_start = 0;
uint32_t blank_end = 0;

// extern uint16_t adc_value;


/*
 * For first half of buffer, start = 0; end = buff_len/2
 * For second half, start = buff_len/2; end = buff_len
 */
void generate_waveforms(uint16_t start, uint16_t end)
{
	uint32_t max_sample_count = HALF_SECOND/2;	// Good

	// freq_vco = ( (float32_t) (ADCBuffer[1] & 0xffa0) / 50);

	// TOOD: fast log
//	freq_lfo = (float)ADCBuffer[0];
//	freq_lfo = floor(freq_lfo / 50);
//	freq_lfo = freq_lfo / 10;


	adsr_setting adsr_settings = adsr_03;

	// TODO: probably don't need to recalc over and over
	// Calculate start-end boundaries for each of attack, sustain, ...
	attack_start = 0;
	decay_start = adsr_settings.attack_len;
	sustain_start = decay_start + adsr_settings.decay_len;
	release_start = sustain_start + adsr_settings.sustain_len;
	blank_start = release_start + adsr_settings.release_len;
	blank_end = blank_start + adsr_settings.blank_len;

	// TODO: test frequency accuracy.
	// TODO: Consider using arm_sin_q15 instead of arm_sin_f32.  However, results might cause clipping.
	// TODO: Store vco default amplitude in global and set with a defined value.

	volatile int i = 0;


	// TODO: probably don't need to recalc over and over
	// volatile float32_t angle_vco = freq_vco*PI/SAMPLERATE;
	volatile float32_t angle_vco = osc.freq_vco*PI_DIV_2/(max_sample_count);	// 'angle' based samples per cycle.
	volatile float32_t angle_vco2 = osc.freq_vco2*PI_DIV_2/(max_sample_count);	// 'angle' based samples per cycle.
	// volatile float32_t angle_vco2 = angle_vco;
	volatile float32_t angle_lfo = osc.freq_lfo*PI/(max_sample_count);

	// volatile float32_t samples_cycle_vco = SAMPLERATE / freq_vco;
	volatile uint32_t samples_cycle_lfo = SAMPLERATE / osc.freq_lfo;
	volatile uint32_t samples_cycle_adsr = adsr_settings.attack_len + adsr_settings.decay_len + adsr_settings.sustain_len + adsr_settings.release_len + adsr_settings.blank_len;

	// Sine VCO
	if(osc.wav_vco == WAVE_SINE && osc.mod_type != MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32((sample_count_vco+(i-start))*angle_vco);
			buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2*arm_sin_f32((sample_count_vco+(i-start))*angle_vco2);
			buffer_vco[i] = buffer_vco[i] + buffer_vco2[i];
		}
	}

	// Square VCO
	else if(osc.wav_vco == WAVE_SQUARE && osc.mod_type != MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_square_angle((sample_count_vco+(i-start)) * angle_vco);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_square_angle((sample_count_vco+(i-start)) * angle_vco2);
			// buffer_vco[i] = buffer_vco[i] + buffer_vco2[i];
		}
	}

	// Sawtooth VCO
	else if(osc.wav_vco == WAVE_SAWTOOTH && osc.mod_type != MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			// angle = (sample_count+(i-start)) * angle_vco;

			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_sawtooth_angle((sample_count_vco+(i-start)) * angle_vco);
			buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_sawtooth_angle((sample_count_vco+(i-start)) * angle_vco2);
			buffer_vco[i] = buffer_vco[i] + buffer_vco2[i];
		}
	}

	// Triangle VCO
	else if(osc.wav_vco == WAVE_TRIANGLE && osc.mod_type != MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_triangle_angle((sample_count_vco+(i-start)) * angle_vco);
			buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_triangle_angle((sample_count_vco+(i-start)) * angle_vco2);
			buffer_vco[i] = buffer_vco[i] + buffer_vco2[i];

		}
	}

	// SINE LFO
	if(osc.wav_lfo == WAVE_SINE)
	{
		for(i = start; i < end; i++)
		{
			// buffer_lfo_float[i] = arm_sin_f32((sample_count+(i-start))*angle_lfo);
			buffer_lfo_float[i] = arm_sin_f32((sample_count_lfo+(i-start))*angle_lfo);

		}
	}

	// Square LFO
	// TODO: amplitude adjustment -- so it's not just 000011111, but could be 0.2 0.2 0.2 0.6 0.6 0.6
	// 			use square_min and square_max or use amp and offset.
	else if(osc.wav_lfo == WAVE_SQUARE)
	{
		if(osc.mod_type != MOD_FM)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_square_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
		else
		{
			for(i = start; i < end; i++)
			{
				// Sawtooth is integral of triangle
				buffer_lfo_float[i] = gen_triangle_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
	}

	// Sawtooth LFO
	else if(osc.wav_lfo == WAVE_SAWTOOTH)
	{
		if(osc.mod_type != MOD_FM)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_sawtooth_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}

		// If FM mod, need integral of modulating signal.  Integral of ramp is right side of parabola.
		else
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_sawtooth_integral_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
	}

	else if(osc.wav_lfo == WAVE_TRIANGLE)
	{
		if(osc.mod_type != MOD_FM)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_triangle_angle( (sample_count_lfo+(i-start)) * angle_lfo);
			}
		}

		// If FM mod, need integral of modulating signal.
		else
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = gen_triangle_integral_angle( (sample_count_lfo+(i-start)) * angle_lfo);
			}
		}
	}

	// No LFO
	else if(osc.wav_lfo == WAVE_NONE)
	{
		// TODO: fill lfo buffer with zeros?
	}

	// AM modulation
	if(osc.mod_type == MOD_AM)
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = buffer_vco[i] * buffer_lfo_float[i];
		}
	}

	// FM for sine wave VCO.
	else if(osc.wav_vco == WAVE_SINE && osc.mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			// Using 40 for sine modulated with sine
			// TODO: consider changing 1000 to a variable.
			buffer_vco[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32((sample_count_vco+(i-start))*angle_vco + 100*buffer_lfo_float[i]);
			buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2*arm_sin_f32((sample_count_vco+(i-start))*angle_vco2 + 100*buffer_lfo_float[i]);
			buffer_vco2[i] = 0;
			buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
		}
	}

	// FM for square wave VCO.
	else if(osc.wav_vco == WAVE_SQUARE && osc.mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_square_angle((sample_count_vco+(i-start))*angle_vco + 50*buffer_lfo_float[i]);
			buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_square_angle((sample_count_vco+(i-start))*angle_vco2 + 50*buffer_lfo_float[i]);
			// buffer_vco2[i] = 0;
			buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
		}
	}

	// FM for sawtooth wave VCO.
	else if(osc.wav_vco == WAVE_SAWTOOTH && osc.mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_sawtooth_angle((sample_count_vco+(i-start))*angle_vco + 50*buffer_lfo_float[i]);
			buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_sawtooth_angle((sample_count_vco+(i-start))*angle_vco2 + 50*buffer_lfo_float[i]);
			buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
		}
	}

	// FM for triangle wave VCO.
	else if(osc.wav_vco == WAVE_TRIANGLE && osc.mod_type == MOD_FM)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_triangle_angle((sample_count_vco+(i-start))*angle_vco + 50*buffer_lfo_float[i]);
			buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_triangle_angle((sample_count_vco+(i-start))*angle_vco2 + 50*buffer_lfo_float[i]);
			buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
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
			if( (sample_count_adsr+(i-start))%samples_cycle_adsr < decay_start)
			{
				// Attack
				buffer_adsr[i] = gen_sawtooth( (sample_count_adsr+(i-start)) % samples_cycle_adsr, adsr_settings.attack_len, 0.0, adsr_settings.attack_amp);
			}

			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < sustain_start)
			{
				// Decay
				buffer_adsr[i] = gen_rampdown((sample_count_adsr+i-start-decay_start) % samples_cycle_adsr, adsr_settings.decay_len, adsr_settings.decay_amp, adsr_settings.attack_amp);
			}

			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < release_start)
			{
				// Sustain
				buffer_adsr[i] = gen_rampdown((sample_count_adsr+i-start-sustain_start) % samples_cycle_adsr, adsr_settings.sustain_len, adsr_settings.sustain_amp, adsr_settings.sustain_amp);
			}

			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < blank_start)
			{
				// Release
				buffer_adsr[i] = gen_rampdown( (sample_count_adsr+i-start-release_start) % samples_cycle_adsr, adsr_settings.release_len, 0.0, adsr_settings.sustain_amp);

			}
			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < blank_end)
			{
				// Blank
				buffer_adsr[i] = 0;
			}

			buffer_output[i] = buffer_output[i] * buffer_adsr[i];
		}
	}

	// TODO: there should be multiple sample_counts: for vco, lfo, and adsr.
	// Might need to modulus each based on wavelenth in samples.
	sample_count = sample_count + (i - start);
	sample_count = sample_count % (TEN_SECOND);

	sample_count_vco = sample_count;
//	sample_count_vco = sample_count + (i - start);
//	sample_count_vco = sample_count_vco % samples_cycle_vco;

	sample_count_lfo = sample_count_lfo + (i - start);
	sample_count_lfo = sample_count_lfo % samples_cycle_lfo;

	sample_count_adsr = sample_count_adsr + (i - start);
	sample_count_adsr = sample_count_adsr % samples_cycle_adsr;


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
		return osc.square_min;
	}
	return osc.square_max;
}

/* Parameters:
 * 	angle: normalized angle between 0 and 2*PI.  Similar to sine function.
 */
float32_t gen_square_angle(float32_t angle)
{
	angle = fast_fmod(angle, 2*PI);
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

	angle = fast_fmod(angle, 2*PI);

	// y = mx + b
	m = ONE_DIV_PI;
	val = -1+angle*m;
	return val;
}

float32_t gen_sawtooth_integral_angle(float32_t angle)
{
	float32_t val = 0.0;
	float32_t m = 0.0;

	angle = fast_fmod(angle, 2*PI);
	m = ONE_DIV_2_PI;
	val = m*angle;			// Generate linear value between 0 and 1
	val = val*val;			// Square it.  Produces parabola y: 0 to 1
	val = val*2;			// Double it.
	val = val - 1;			// Shift it down
	return val;

	// Test
	// return 2*(m*angle)*2 - 1;

	// return 0;
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

	angle = fast_fmod(angle, 2*PI);		// TODO: pull this out into generate_waveforms().

	// y = mx + b
	m = -ONE_DIV_2_PI;
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

	angle = fast_fmod(angle, 2*PI);		// TODO: pull this out into generate_waveforms().
	m = TWO_DIV_PI;
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

	angle = fast_fmod(angle, 2*PI);		// TODO: pull this out into generate_waveforms().
	m = ONE_DIV_PI;
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

/*
 * Found similar function at this address: https://cboard.cprogramming.com/c-programming/105096-fmod.html
 * Modified it to work with float32_t.
 * NOTE: Possible alternative found here: https://stackoverflow.com/questions/26342823/implementation-of-fmod-function
 * 	return (a - b * floor(a / b));
 */
float32_t fast_fmod(float32_t x,float32_t y)
{
	float32_t a;
	return ( (a = x/y ) - (uint32_t)a ) * y;
}

// TODO
void count_cycles()
{
	/* ************************************************************** */
	// TODO: time this function call to estimate processor load.
	/*
	 * CPU cycle counting to measure duration of code.
	 * This cycle-counting code was copied from here:
	 * http://embeddedb.blogspot.ca/2013/10/how-to-count-cycles-on-arm-cortex-m.html
	 */
	volatile uint32_t count = 0;

	// addresses of registers
	volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
	volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
	volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;

	// enable the use DWT
	*DEMCR = *DEMCR | 0x01000000;

	// Reset cycle counter
	*DWT_CYCCNT = 0;

	// enable cycle counter
	*DWT_CONTROL = *DWT_CONTROL | 1 ;
	/* ************************************************************** */



	/* ************************************************************* */
	// TODO: for measuring time.
	// number of cycles stored in count variable
	count = *DWT_CYCCNT;
	/* ************************************************************* */
}
