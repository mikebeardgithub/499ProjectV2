/*
 * osc.c
 *
 *  Created on: Jun 12, 2017
 *      Author: Mike Beard
 */

#include "osc.h"
#include <stdint.h>
#include <stdlib.h>

// #include <time.h>	// For testing.

volatile uint16_t buffer_output[BUFF_LEN] = {0};
volatile uint16_t buffer_vco[BUFF_LEN] = {0};
volatile uint16_t buffer_vco2[BUFF_LEN] = {0};
volatile float32_t buffer_lfo_float[BUFF_LEN] = {0};
volatile float32_t buffer_adsr_am[BUFF_LEN] = {0};			// Attack, sustain, decay, release
volatile float32_t buffer_adsr_fm[BUFF_LEN] = {0};

// TODO: organize these globals into struct(s)
// TODO: it might be that odd frequencies cause the ticking.  Might be able to detect with mod, fmod.
osc_setting osc =
{
	.vco_freq = 400.0,
	.vco2_freq = 400.0,
	.lfo_freq = 1.5,						// Moderate LFO frequency

	.vco_wav = WAVE_SINE,
	.lfo_wav = WAVE_SINE,
	.am_mod = OFF,
	.fm_mod = ON,

	.vco_amp = VCO_AMP/2,
	.vco_amp2 = VCO_AMP/5,
	.lfo_amp = 0.7,
	// .lfo_offset = 2.0,

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
uint16_t adsr_am = OFF;					// Enable/disable ADSR amplitude envelope.
uint16_t adsr_fm = OFF;					// Enable/disable ADSR frequency envelope.

/*
 *	Set ADSR lengths in numbers of samples.
 */

// Percussive
adsr_setting adsr_01 = {
		.sustain_amp=0.7,

		.attack_len=400,
		.decay_len=400,
		.sustain_len=3500,
		.release_len=700,
		.blank_len=14000
};

// Bell
adsr_setting adsr_02 = {
		.sustain_amp=0.3,

		.attack_len=300,
		.decay_len=300,
		.sustain_len=10000,
		.release_len=30000,
		.blank_len=30000
};


adsr_setting adsr_03 = {
		.sustain_amp=0.5,

		.attack_len=5000,
		.decay_len=500,
		.sustain_len=30000,
		.release_len=900,
		.blank_len=40000
};

adsr_setting adsr_04 = {
		.sustain_amp=0.5,

		.attack_len=300,
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


// TODO: for adsr frequency.  Test and remove if not needed.
volatile float32_t delta = 0.0;


/*
 * For first half of buffer, start = 0; end = buff_len/2
 * For second half, start = buff_len/2; end = buff_len
 */
void generate_waveforms(uint16_t start, uint16_t end)
{
	uint32_t max_sample_count = QUARTER_SECOND;

	osc.vco_wav = vfo_state;
	osc.lfo_wav = lfo_state;

	osc.vco_amp = ADCBuffer[0];
	osc.vco_freq = ADCBuffer[1];
	osc.lfo_amp = (float) ADCBuffer[2]/4095;		// AM: div by 4095
	osc.lfo_freq = (float) ADCBuffer[3]/20;			// AM: div by 20

	adsr_setting adsr_settings = adsr_03;
	adsr_settings.attack_len = ADCBuffer[5]*20;		// A5
	adsr_settings.decay_len = (ADCBuffer[6])*20;	// A6
	adsr_settings.sustain_len = ADCBuffer[7]*10;	// A7
	adsr_settings.release_len = ADCBuffer[8]*10;	// B0
	adsr_settings.blank_len = ADCBuffer[10]*20;		// C0
	adsr_settings.sustain_amp = (float32_t) ADCBuffer[12]/4095;		// C4

	// TODO: probably don't need to recalc over and over
	// Calculate start-end boundaries for each of attack, sustain, ...
	attack_start = 0;
	decay_start = adsr_settings.attack_len;
	sustain_start = decay_start + adsr_settings.decay_len;
	release_start = sustain_start + adsr_settings.sustain_len;
	blank_start = release_start + adsr_settings.release_len;
	blank_end = blank_start + adsr_settings.blank_len;

	// TODO: Consider using arm_sin_q15 instead of arm_sin_f32.  However, results might cause clipping.
	volatile int i = 0;

	// TODO: probably don't need to recalc over and over
	// TODO: simplify these calculations
	volatile float32_t angle_vco = osc.vco_freq*PI_DIV_2/(max_sample_count);	// 'angle' based samples per cycle.
	// volatile float32_t angle_vco2 = osc.vco2_freq*PI_DIV_2/(max_sample_count);	// 'angle' based samples per cycle.
	volatile float32_t angle_lfo = osc.lfo_freq*PI_DIV_2/(max_sample_count);

	volatile float32_t angle_attack = PI/adsr_settings.attack_len;
	volatile float32_t angle_decay = PI/adsr_settings.decay_len;
	// volatile float32_t angle_sustain = PI/adsr_settings.sustain_len;
	volatile float32_t angle_release = PI/adsr_settings.release_len;

	// volatile float32_t samples_cycle_vco = SAMPLERATE / osc.vco_freq;
	volatile uint32_t samples_cycle_lfo = 2 * SAMPLERATE / osc.lfo_freq;
	volatile uint32_t samples_cycle_adsr = adsr_settings.attack_len + adsr_settings.decay_len + adsr_settings.sustain_len + adsr_settings.release_len + adsr_settings.blank_len;

	// Sine VCO
	// if(osc.vco_wav == WAVE_SINE && osc.fm_mod == OFF)
	if(osc.vco_wav == sine && osc.fm_mod == OFF)
	{
		for(i = start; i < end; i++)
		{
			// buffer_vco[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32((sample_count_vco+(i-start))*angle_vco);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2*arm_sin_f32((sample_count_vco+(i-start))*angle_vco2);
			// buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
			// buffer_output[i] = buffer_vco[i];

			buffer_output[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32((sample_count_vco+(i-start))*angle_vco);
		}
	}

	// Square VCO
	// else if(osc.vco_wav == WAVE_SQUARE && osc.fm_mod == OFF)
	else if(osc.vco_wav == square && osc.fm_mod == OFF)
	{
		for(i = start; i < end; i++)
		{
			 buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_square_angle((sample_count_vco+(i-start)) * angle_vco);
			 // buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_square_angle((sample_count_vco+(i-start)) * angle_vco2);
			 buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
		}
	}

	// Sawtooth VCO
	// else if(osc.vco_wav == WAVE_SAWTOOTH && osc.fm_mod == OFF)
	else if(osc.vco_wav == sawtooth && osc.fm_mod == OFF)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_sawtooth_angle((sample_count_vco+(i-start)) * angle_vco);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_sawtooth_angle((sample_count_vco+(i-start)) * angle_vco2);
			buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
		}
	}

	// Triangle VCO
	// else if(osc.vco_wav == WAVE_TRIANGLE && osc.fm_mod == OFF)
	else if(osc.vco_wav == triangle && osc.fm_mod == OFF)
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_triangle_angle((sample_count_vco+(i-start)) * angle_vco);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_triangle_angle((sample_count_vco+(i-start)) * angle_vco2);
			// buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
			buffer_output[i] = buffer_vco[i];
		}
	}

	// SINE LFO
	// if(osc.lfo_wav == WAVE_SINE)
	if(osc.lfo_wav == sine)
	{
		if(osc.fm_mod == OFF)
		{
			for(i = start; i < end; i++)
			{
				// AM
				buffer_lfo_float[i] = osc.lfo_amp + osc.lfo_amp*arm_sin_f32((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
		else
		{
			for(i = start; i < end; i++)
			{
				// FM
				buffer_lfo_float[i] = osc.lfo_amp*arm_cos_f32((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
	}

	// Square LFO
	// else if(osc.lfo_wav == WAVE_SQUARE)
	else if(osc.lfo_wav == square)
	{
		if(osc.fm_mod == OFF)
		{
			// AM
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = osc.lfo_amp*gen_square_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
		else
		{
			// FM
			for(i = start; i < end; i++)
			{
				// Sawtooth is integral of triangle
				buffer_lfo_float[i] = osc.lfo_amp*gen_triangle_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
	}

	// Sawtooth LFO
	// else if(osc.lfo_wav == WAVE_SAWTOOTH)
	else if(osc.lfo_wav == sawtooth)
	{
		if(osc.fm_mod == OFF)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = osc.lfo_amp*gen_sawtooth_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}

		// If FM mod, need integral of modulating signal.  Integral of ramp is right side of parabola.
		else
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = osc.lfo_amp*gen_sawtooth_integral_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
	}

	// else if(osc.lfo_wav == WAVE_TRIANGLE)
	else if(osc.lfo_wav == triangle)
	{
		if(osc.fm_mod == OFF)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = osc.lfo_amp*gen_triangle_angle( (sample_count_lfo+(i-start)) * angle_lfo);
			}
		}

		// If FM mod, need integral of modulating signal.
		else
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = osc.lfo_amp*gen_triangle_integral_angle( (sample_count_lfo+(i-start)) * angle_lfo);
			}
		}
	}

	// Generic ADSR envelope
	// The waveform contains 5 segments (asdr + a blank space)
	if(adsr_am || adsr_fm)
	{
		for(i = start; i < end; i++)
		{
			// First part tells us sample number into the adsr cycle: (sample_count+(i-start))%sample_cycle_adsr
			if( (sample_count_adsr+(i-start))%samples_cycle_adsr < decay_start)
			{
				// Attack
				// Sine, FM --> Try 1.0
				// Square, FM --> Use 0.4
				// Triangle, FM ---> Try 2.0
				buffer_adsr_am[i] = 1.0 + 1.0 * gen_sawtooth_angle( (sample_count_adsr+(i-start)) % samples_cycle_adsr * angle_attack);
			}

			// TODO: This segment has unwanted harmonics.
			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < sustain_start)
			{
				// Decay
				// buffer_adsr_am[i] = gen_rampdown( (sample_count_adsr+i-start-decay_start) % samples_cycle_adsr, adsr_settings.decay_len, adsr_settings.sustain_amp, 1.0);
				// buffer_adsr_am[i] = gen_rampdown_angle( (sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr * angle_decay);


				//-------------------------------------
//				float32_t m = 0.0;
//				float32_t val = 0.0;
//				float32_t angle = (sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr * angle_decay;
//				angle = fast_fmod(angle, TWO_PI);
//				// y = mx + b
//				val = 1.0 - angle*(ONE_DIV_PI);
//				buffer_adsr_am[i] = adsr_settings.sustain_amp + one_min_sust_amp * val;

				//-------------------------------------
				// The (1.0 - ...) causes noise.  Not sure why...
				// buffer_adsr_am[i] = gen_rampdown_angle( (sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr * angle_decay);
				// buffer_adsr_am[i] = (1.0 - adsr_settings.sustain_amp) * buffer_adsr_am[i];
				// buffer_adsr_am[i] = adsr_settings.sustain_amp + buffer_adsr_am[i];
				//-------------------------------------

				// buffer_adsr_am[i] = adsr_settings.sustain_amp + (1.0 - adsr_settings.sustain_amp) * gen_rampdown_angle( (sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr * angle_decay);



				//-------------------------------------
				// buffer_adsr_am[i] = 1.0;
				//-------------------------------------

				//-------------------------------------
				// buffer_adsr_am[i] = adsr_settings.sustain_amp + (1.0 - adsr_settings.sustain_amp) * gen_rampdown_angle( (sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr * angle_decay);
				//-------------------------------------
				// buffer_adsr_am[i] = gen_sawtooth_angle( (sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr * angle_decay);
				// buffer_adsr_am[i] = 0.5 - 0.5 * buffer_adsr_am[i];
				// buffer_adsr_am[i] = adsr_settings.sustain_amp + (1.0 - adsr_settings.sustain_amp) * buffer_adsr_am[i];
				//-------------------------------------
				// buffer_adsr_am[i] = gen_rampdown((sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr, samples_cycle_adsr, 0.0, 1.0);
				//-------------------------------------

				// Sine, FM --> Try 1.0
				// Square, FM --> Use 0.4
				// Triangle, FM ---> Try 2.0
				buffer_adsr_am[i] = 1.0 * gen_rampdown_angle2( (sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr * angle_decay, adsr_settings.sustain_amp, 1.0);
//				if(buffer_adsr_am[i] < adsr_settings.sustain_amp + 0.01)
//				{
//					uint16_t test;
//					test = 1;
//				}
			}

			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < release_start)
			{
				// Sustain
				buffer_adsr_am[i] = adsr_settings.sustain_amp;
			}

			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < blank_start)
			{
				// Release
				buffer_adsr_am[i] = adsr_settings.sustain_amp * gen_rampdown_angle( (sample_count_adsr+(i-start-release_start)) % samples_cycle_adsr * angle_release);

			}
			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < blank_end)
			{
				// Blank
				buffer_adsr_am[i] = 0;
			}
			buffer_output[i] = buffer_output[i] * buffer_adsr_am[i];
		}
	}

	/*
	 * ADSR frequency envelope.
	 * Uses the ADSR amplitude envelope and integrates each of the shapes.
	 */
	//
	if(adsr_fm)
	{
		for(i = start; i < end; i++)
		{
			// First part tells us sample number into the adsr cycle: (sample_count+(i-start))%sample_cycle_adsr
			if( (sample_count_adsr+(i-start))%samples_cycle_adsr < decay_start)
			{
				// Attack
				// buffer_adsr_fm[i] = 0.5 * buffer_adsr_am[i] * buffer_adsr_am[i];
				buffer_adsr_fm[i] = buffer_adsr_am[i];
				if(i > 0)
				{
					buffer_adsr_fm[i] = buffer_adsr_fm[i] + buffer_adsr_fm[i-1];
				}
				else
				{
					buffer_adsr_fm[i] = buffer_adsr_fm[i] + buffer_adsr_fm[BUFF_LEN-1];
				}
			}

			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < sustain_start)
			{
				// Decay
				// buffer_adsr_fm[i] = 0.5 * buffer_adsr_am[i] * buffer_adsr_am[i];
				buffer_adsr_fm[i] = buffer_adsr_am[i];
				if(i > 0)
				{
					buffer_adsr_fm[i] = buffer_adsr_fm[i] + buffer_adsr_fm[i-1];
				}
				else
				{
					buffer_adsr_fm[i] = buffer_adsr_fm[i] + buffer_adsr_fm[BUFF_LEN-1];
				}
			}

			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < release_start)
			{
				// Sustain
				// DO this only once--get last delta from previous section.
				if( (sample_count_adsr+(i-start))%samples_cycle_adsr == sustain_start)
				{
					// buffer_adsr_fm[i] = buffer_adsr_fm[i-1];
					if(i > 1)
					{
						delta = buffer_adsr_fm[i-1] - buffer_adsr_fm[i-2];
					}
					else if(i == 0)
					{
						delta = buffer_adsr_fm[BUFF_LEN-1] - buffer_adsr_fm[BUFF_LEN-2];
					}
					else if(i == 1)
					{
						delta = buffer_adsr_fm[i-1] - buffer_adsr_fm[BUFF_LEN-1];
					}
					buffer_adsr_fm[i] = 0.0;
				}
				else
				{
					if(i > 0)
					{
						buffer_adsr_fm[i] = buffer_adsr_fm[i-1] + delta;
					}
					else
					{
						buffer_adsr_fm[i] = buffer_adsr_fm[BUFF_LEN-1] + delta;
					}
				}
			}

			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < blank_start)
			{
				// Release
				buffer_adsr_fm[i] = buffer_adsr_am[i];
				if(i > 0)
				{
					buffer_adsr_fm[i] = buffer_adsr_fm[i] + buffer_adsr_fm[i-1];
				}
				else
				{
					buffer_adsr_fm[i] = buffer_adsr_fm[i] + buffer_adsr_fm[BUFF_LEN-1];
				}
			}
			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < blank_end)
			{
				// Blank
				buffer_adsr_fm[i] = 0;
			}
		}
	}

	// TODO: add buffer_adsr_fm[i] to phase
	// FM for sine wave VCO.
	if(osc.vco_wav == sine && ( osc.fm_mod == ON || adsr_fm == ON ) )
	{
		for(i = start; i < end; i++)
		{
			// buffer_vco[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32((sample_count_vco+(i-start))*angle_vco + 100*buffer_lfo_float[i]);
			buffer_vco[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32((sample_count_vco+(i-start))*angle_vco + 100*buffer_lfo_float[i] + buffer_adsr_fm[i]);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2*arm_sin_f32((sample_count_vco+(i-start))*angle_vco2 + 100*buffer_lfo_float[i]);
			// buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
			buffer_output[i] = buffer_vco[i];

		}
	}

	// FM for square wave VCO.
	else if(osc.vco_wav == square && ( osc.fm_mod == ON || adsr_fm == ON ) )
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_square_angle((sample_count_vco+(i-start))*angle_vco + 100*buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_square_angle((sample_count_vco+(i-start))*angle_vco2 + 50*buffer_lfo_float[i]);
			// buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for sawtooth wave VCO.
	else if(osc.vco_wav == sawtooth && ( osc.fm_mod == ON || adsr_fm == ON ) )
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_sawtooth_angle((sample_count_vco+(i-start))*angle_vco + 50*buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_sawtooth_angle((sample_count_vco+(i-start))*angle_vco2 + 50*buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
			// buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
			buffer_output[i] = buffer_vco[i];
		}
	}

	// FM for triangle wave VCO.
	else if(osc.vco_wav == triangle && ( osc.fm_mod == ON || adsr_fm == ON ) )
	{
		for(i = start; i < end; i++)
		{
			buffer_vco[i] = osc.vco_amp + osc.vco_amp * gen_triangle_angle((sample_count_vco+(i-start))*angle_vco + 50*buffer_lfo_float[i] + 0.01 * buffer_adsr_fm[i]);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_triangle_angle((sample_count_vco+(i-start))*angle_vco2 + 50*buffer_lfo_float[i]);
			// buffer_output[i] = buffer_vco[i] + buffer_vco2[i];
			buffer_output[i] = buffer_vco[i];
		}
	}

	// AM modulation.
	if(osc.am_mod == ON)
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = buffer_output[i] * buffer_lfo_float[i];
		}
	}

	// ADSR amplitude envelope
	// The waveform contains 5 segments (asdr + a blank space)
	if(adsr_am)
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = buffer_output[i] * buffer_adsr_am[i];
		}
	}


	//	sample_count = sample_count + (i - start);
	//	sample_count = sample_count % (TEN_SECOND);

	/*
	 * TODO
	 * Modding sample_count_vco with TEN_SECOND... not ideal because it causes a glitch every
	 * 10 seconds.  Using samples_cycle_vco results in a non-integer value.
	 *  An ideal mod value to use would be the first integer multiple of samples_cycle_vco.
	 *  This might help:
	 *  https://math.stackexchange.com/questions/1823788/how-to-determine-lowest-integer-multiple-for-any-given-decimal-fraction
	 * OR...
	 * ** Multiple samples_cycle_vco by 100 or 1000 and mod that....  Might be close enough.
	 */
	sample_count_vco = sample_count_vco + (i - start);
	sample_count_vco = sample_count_vco % ONE_SECOND;

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

	angle = fast_fmod(angle, TWO_PI);

	// y = mx + b
	m = ONE_DIV_PI;
	val = -1+angle*m;
	return val;
}

//float32_t gen_sawtooth_angle2( float32_t angle, float32_t delta, uint32_t len)
//{
//	float32_t m = 0.0;
//	float32_t val = 0.0;
//
//	angle = fast_fmod(angle, len);
//
//	// y = mx + b
//	// m = delta * len;
//	m = delta * SAMPLERATE;
//	val = angle*m;
//	return val;
//}

float32_t gen_sawtooth_integral_angle(float32_t angle)
{
	float32_t val = 0.0;
	float32_t m = 0.0;

	angle = fast_fmod(angle, TWO_PI);
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

/*
 * Generate ramp value from +1 down to 0 based on angle.
 * Parameter angle is a radian.
 */
float32_t gen_rampdown_angle(float32_t angle)
{
	float32_t m = 0.0;
	float32_t val = 0.0;

	angle = fast_fmod(angle, TWO_PI);

	// y = mx + b
	m = -ONE_DIV_PI;
	val = 1.0 + angle*m;
	return val;
}


float32_t gen_rampdown_angle2( float32_t angle, float32_t min, float32_t max)
{
	float32_t m = 0.0;
	float32_t val = 0.0;

	angle = fast_fmod(angle, TWO_PI);

	// y = mx + b
	m = (min - max) * ONE_DIV_PI;
	val = 1.0 + angle*m;


	return val;
}

// TODO:
float32_t gen_rampdown_integral_angle(float32_t angle)
{
	return 0.0;
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
 * Param: value - sample value
 * Returns: running accumulation of values.
 * Assumes that input signal is centered around zero, so that accumulation centers around zero.
 */
float32_t integrate(float32_t value)
{
	static float32_t sum;
	sum = sum + value;
	return sum;
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

/*
 * This function was found at this address: https://gist.github.com/CAFxX/ad150f2403a0604e14cc
 */
uint32_t ilog10c(uint64_t v)
{
  static const uint64_t thr[64] = {
    10000000000000000000ULL, 0, 0, 0, 1000000000000000000ULL, 0, 0, 100000000000000000ULL, 0, 0,
       10000000000000000ULL, 0, 0, 0,    1000000000000000ULL, 0, 0,    100000000000000ULL, 0, 0,
          10000000000000ULL, 0, 0, 0,       1000000000000ULL, 0, 0,       100000000000ULL, 0, 0,
             10000000000ULL, 0, 0, 0,          1000000000ULL, 0, 0,          100000000ULL, 0, 0,
                10000000ULL, 0, 0, 0,             1000000ULL, 0, 0,             100000ULL, 0, 0,
                   10000ULL, 0, 0, 0,                1000ULL, 0, 0,                100ULL, 0, 0,
                      10ULL, 0, 0, 0
  };
  uint32_t lz = __builtin_clzll(v);
  return (63 - lz) * 3 / 10 + (v >= thr[lz]);
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
