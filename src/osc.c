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

// Defaults
osc_setting osc =
{
	.vco_freq = 1400.0,
	.vco2_freq = 413.7,
	.lfo_freq = 1.73,

	.vco_wav = sine,
	.lfo_wav = sine,		// other2 means OFF, for now.
	.mod = VCOfreq,

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

//volatile uint32_t sample_count = 0;						// TODO remove
//volatile uint32_t sample_count_vco = 0;
//volatile uint32_t sample_count_lfo = 0;
volatile uint32_t sample_count_adsr = 0;

/*
 *	Set ADSR lengths in numbers of samples.
 */

// Percussive
volatile adsr_setting adsr_01 = {
		.mod = VCOamp,

		.sustain_amp=0.7,

		.attack_len=400,
		.decay_len=400,
		.sustain_len=3500,
		.release_len=700,
		.blank_len=14000
};

// Bell
volatile adsr_setting adsr_02 = {
		.mod = VCOamp,
		.sustain_amp=0.3,

		.attack_len=300,
		.decay_len=300,
		.sustain_len=10000,
		.release_len=30000,
		.blank_len=30000
};


volatile adsr_setting adsr_03 = {
		.mod = VCOamp,
		.sustain_amp=0.5,

		.attack_len=5000,
		.decay_len=500,
		.sustain_len=30000,
		.release_len=900,
		.blank_len=40000
};

volatile adsr_setting adsr_04 = {
		.mod = VCOamp,
		.sustain_amp=0.5,

		.attack_len=300,
		.decay_len=500,
		.sustain_len=30000,
		.release_len=900,
		.blank_len=40000
};


// Start-end samples.  These are calculated later.
// TODO: probably don't need to be global.
//volatile uint32_t attack_start = 0;
//volatile uint32_t decay_start = 0;
//volatile uint32_t sustain_start = 0;
//volatile uint32_t release_start = 0;
//volatile uint32_t blank_start = 0;
//volatile uint32_t blank_end = 0;


/*
 * Moving average -- TODO: remove after testing.
 */
#define MOV_AVG_BUFF_LEN		256
volatile uint32_t mov_avg1 [MOV_AVG_BUFF_LEN] = {0};
volatile uint32_t mov_avg_index1 = 0;
volatile uint32_t mov_avg_sum1;

volatile uint32_t mov_avg2 [MOV_AVG_BUFF_LEN] = {0};
volatile uint32_t mov_avg_index2 = 0;
volatile uint32_t mov_avg_sum2;

volatile uint32_t mov_avg3 [MOV_AVG_BUFF_LEN] = {0};
volatile uint32_t mov_avg_index3 = 0;
volatile uint32_t mov_avg_sum3;

#define SPIKE_FILTER_LEN		2
float32_t spike_buff [SPIKE_FILTER_LEN] = {0};
uint16_t spike_buff_index = 0;
float32_t prev_osc_freq = 0;

// TODO: for adsr frequency.  Test and remove if not needed.
volatile float32_t delta = 0.0;


/*
 * For first half of buffer, start = 0; end = buff_len/2
 * For second half, start = buff_len/2; end = buff_len
 */

volatile float32_t theta_vco = 0.0;
volatile float32_t theta_lfo = 0.0;
volatile float32_t theta_adsr = 1.0;

volatile uint16_t testflag =  0;

// Get ADSR values.
volatile adsr_setting adsr_settings;			// Fall back on this.

void generate_waveforms(uint16_t start, uint16_t end)
{
	// Get wave shape.
	// osc.vco_wav = vfo_state;
	// osc.lfo_wav = lfo_state;

	osc.vco_wav = sine;
	osc.lfo_wav = square;
	osc.mod = VCOfreq;
	// osc.mod = VCOamp;
	//osc.mod = NO_MOD

	// Oscillators - amplitude and frequency.
	osc.vco_amp = (float) (ADCBuffer[0] & 0xffff);					// A0
	osc.vco_freq = (float) (ADCBuffer[1] & 0xfffc) * 2 * PI;		// A1
	osc.lfo_amp = (float) (ADCBuffer[2] & 0xfffc)/200;				// AM: div by 4095
	osc.lfo_freq = (float) (ADCBuffer[3] & 0xfffc)/5;				// TODO: AM: div by 20

	osc.vco_freq = movingAvg(mov_avg1, &mov_avg_sum1, mov_avg_index1, MOV_AVG_BUFF_LEN, (ADCBuffer[1] & 0xfffc)*2*PI);
	mov_avg_index1++;
	if (mov_avg_index1 >= MOV_AVG_BUFF_LEN)
	{
		mov_avg_index1 = 0;
	}

	osc.lfo_freq = movingAvg(mov_avg2, &mov_avg_sum2, mov_avg_index2, MOV_AVG_BUFF_LEN, (ADCBuffer[3] & 0xfffc)/5);
	mov_avg_index2++;
	if (mov_avg_index2 >= MOV_AVG_BUFF_LEN)
	{
		mov_avg_index2 = 0;
	}

	osc.lfo_amp = movingAvg(mov_avg3, &mov_avg_sum3, mov_avg_index3, MOV_AVG_BUFF_LEN, (ADCBuffer[2] & 0xfffc)/200);
	mov_avg_index3++;
	if (mov_avg_index3 >= MOV_AVG_BUFF_LEN)
	{
		mov_avg_index3 = 0;
	}

	volatile uint32_t i = 0;
	adsr_settings = adsr_03;

	// adsr_settings.mod = DualMode_VCO;
	adsr_settings.mod = VCOamp;						// TODO: turn this off when LCD activated.
	// adsr_settings.mod = NO_MOD;
	// adsr_settings.mod = VCOfreq;
	/*
	 * TODO: Turn this on when LCD activated.
	 * 	adsr_settings.mod = menu_state.adsr_mod;
	 */

	//	// Calculate angle amount to increment per sample.
	volatile float32_t rads_per_sample_vco = osc.vco_freq / ONE_SECOND;		// Radians to increment for each iteration.
	volatile float32_t rads_per_sample_lfo = osc.lfo_freq / ONE_SECOND;		// Radians to increment for each iteration.

	// adsr(start, end);
	adsr(start, end);


	// Sine LFO
	if(osc.lfo_wav == sine)
	{
		if(osc.mod == VCOamp || osc.mod == DualMode_VCO)
		{
			for(i = start; i < end; i++)
			{
				theta_lfo = theta_lfo + rads_per_sample_lfo;
				// AM - Requires an amplitude offset.
				buffer_lfo_float[i] = osc.lfo_amp + osc.lfo_amp*arm_sin_f32(theta_lfo);
			}
		}
		else if(osc.mod == VCOfreq)
		{
			for(i = start; i < end; i++)
			{
				theta_lfo = theta_lfo + rads_per_sample_lfo;
				// FM - No offset.
				buffer_lfo_float[i] = osc.lfo_amp*arm_cos_f32(theta_lfo);
			}
		}
	}

	// Square LFO
	else if(osc.lfo_wav == square)
	{
		if(osc.mod == VCOamp || osc.mod == DualMode_VCO)
		{
			// AM
			for(i = start; i < end; i++)
			{
				theta_lfo = theta_lfo + rads_per_sample_lfo;
				buffer_lfo_float[i] = osc.lfo_amp*gen_square_angle(theta_lfo);
			}
		}
		else if(osc.mod == VCOfreq)
		{
			// FM
			for(i = start; i < end; i++)
			{
				theta_lfo = theta_lfo + rads_per_sample_lfo;
				// Sawtooth is integral of triangle
				buffer_lfo_float[i] = osc.lfo_amp + osc.lfo_amp*gen_triangle_angle(theta_lfo);
			}
		}

	}

	// Sawtooth LFO
	else if(osc.lfo_wav == sawtooth)
	{

		if(osc.mod == VCOamp || osc.mod == DualMode_VCO)
		{
			for(i = start; i < end; i++)
			{
				theta_lfo = theta_lfo + rads_per_sample_lfo;
				buffer_lfo_float[i] = osc.lfo_amp*gen_sawtooth_angle(theta_lfo);
			}
		}

		// If FM mod, need integral of modulating signal.  Integral of ramp is right side of parabola.
		else if(osc.mod == VCOfreq)
		{
			for(i = start; i < end; i++)
			{
				theta_lfo = theta_lfo + rads_per_sample_lfo;
				buffer_lfo_float[i] = osc.lfo_amp*gen_sawtooth_integral_angle(theta_lfo);
			}
		}
	}

	// Triangle LFO
	else if(osc.lfo_wav == triangle)
	{
		if(osc.mod == VCOamp || osc.mod == DualMode_VCO)
		{
			for(i = start; i < end; i++)
			{
				theta_lfo = theta_lfo + rads_per_sample_lfo;
				buffer_lfo_float[i] = osc.lfo_amp*gen_triangle_angle(theta_lfo);
			}
		}

		// If FM mod, need integral of modulating signal.
		else if(osc.mod == VCOfreq)
		{
			for(i = start; i < end; i++)
			{
				theta_lfo = theta_lfo + rads_per_sample_lfo;
				buffer_lfo_float[i] = osc.lfo_amp*gen_triangle_integral_angle(theta_lfo);
			}
		}

	}

	// Sine VCO
	if(osc.vco_wav == sine)
	{

		for(i = start; i < end; i++)
		{
			theta_vco = theta_vco + rads_per_sample_vco;
			buffer_output[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32(theta_vco + buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
		}
	}

	// Square VCO
	else if(osc.vco_wav == square)
	{

		for(i = start; i < end; i++)
		{
			theta_vco = theta_vco + rads_per_sample_vco;
			buffer_output[i] = osc.vco_amp + osc.vco_amp*gen_square_angle(theta_vco + buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
		}
	}

	// Sawtooth VCO
	else if(osc.vco_wav == sawtooth)
	{

		for(i = start; i < end; i++)
		{
			theta_vco = theta_vco + rads_per_sample_vco;
			// buffer_output[i] = osc.vco_amp + osc.vco_amp*gen_sawtooth_angle(theta_vco + 10*buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
			buffer_output[i] = osc.vco_amp + osc.vco_amp*gen_sawtooth_angle(theta_vco + buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
		}
	}

	// Triangle VCO
	else if(osc.vco_wav == triangle)
	{

		for(i = start; i < end; i++)
		{
			theta_vco = theta_vco + rads_per_sample_vco;
			buffer_output[i] = osc.vco_amp + osc.vco_amp*gen_triangle_angle(theta_vco + buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
		}
	}

	// AM Modulate VCO with LFO
	if(osc.mod == VCOamp || osc.mod == DualMode_VCO)
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = buffer_output[i] * buffer_lfo_float[i];
		}
	}

	// AM Modulate VCO with ADSR
	if(adsr_settings.mod == VCOamp || adsr_settings.mod == DualMode_VCO)
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = buffer_output[i] * buffer_adsr_am[i];
		}
	}

	theta_vco = fast_fmod(theta_vco, TWO_PI);
	theta_lfo = fast_fmod(theta_lfo, TWO_PI);
	theta_adsr = fast_fmod(theta_adsr, TWO_PI);

//	sample_count_adsr = sample_count_adsr + (i - start);
//	sample_count_adsr = sample_count_adsr % samples_cycle_adsr;

	return;
}




/*
 * Clean version - for testing only
 */
void generate_waveforms2(uint16_t start, uint16_t end)
{
	uint32_t i = 0;
	osc.vco_wav = sine;
	// osc.vco_freq = (float) (ADCBuffer[1] & 0xffff) * 2 * PI;					// A1


//	// Trying to eliminate frequency fluctuations.
//	if (osc.vco_freq + 750.0 < prev_osc_freq || osc.vco_freq - 750.0 > prev_osc_freq &&  prev_osc_freq > 100.0)
//	{
//		osc.vco_freq = prev_osc_freq;
//	}
//	prev_osc_freq = osc.vco_freq;

	// ---------------------------------------
	// Someone else's moving average filter.
	// Found here: https://gist.github.com/bmccormack/d12f4bf0c96423d03f82
	// Note that this discards bits and then smooths it.  What if it did this the other way around?
	// osc.vco_freq = log10(ADCBuffer[1] & 0xffff)*200;
	osc.vco_freq = movingAvg(mov_avg1, &mov_avg_sum1, mov_avg_index1, MOV_AVG_BUFF_LEN, ADCBuffer[1] & 0xfffc);
	mov_avg_index1++;
	if (mov_avg_index1 >= MOV_AVG_BUFF_LEN)
	{
		mov_avg_index1 = 0;
	}



	float32_t rads_per_sample = osc.vco_freq / ONE_SECOND;		// Radians to increment for each iteration.

	for(i = start; i < end; i++)
	{
		theta_vco = theta_vco + rads_per_sample;
		// buffer_output[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32(theta_vco);
		// buffer_output[i] = osc.vco_amp + osc.vco_amp*gen_square_angle(theta_vco);
		buffer_output[i] = osc.vco_amp + osc.vco_amp*gen_sawtooth_angle(theta_vco);
	}

	theta_vco = fast_fmod(theta_vco, 2 * PI);

	return;
}


void adsr(uint16_t start, uint16_t end)
{
	uint16_t i = 0;
	// adsr_settings = adsr_03;
	adsr_settings.attack_len = ADCBuffer[5]*20;		// A5
	adsr_settings.decay_len = (ADCBuffer[6])*20;	// A6
	adsr_settings.sustain_len = ADCBuffer[7]*20;	// A7
	adsr_settings.release_len = ADCBuffer[8]*40;	// B0
	adsr_settings.blank_len = ADCBuffer[10]*40;		// C0
	adsr_settings.sustain_amp = (float32_t) ADCBuffer[12]/4095;		// C4
	volatile uint32_t samples_cycle_adsr = adsr_settings.attack_len + adsr_settings.decay_len + adsr_settings.sustain_len + adsr_settings.release_len + adsr_settings.blank_len;

	// Calculate ADSR boundaries.
	// uint32_t attack_start = 0;
	uint32_t decay_start = adsr_settings.attack_len;
	uint32_t sustain_start = decay_start + adsr_settings.decay_len;
	uint32_t release_start = sustain_start + adsr_settings.sustain_len;
	uint32_t blank_start = release_start + adsr_settings.release_len;
	uint32_t blank_end = blank_start + adsr_settings.blank_len;


	volatile float32_t angle_attack = PI/adsr_settings.attack_len;
	volatile float32_t angle_decay = PI/adsr_settings.decay_len;
	// volatile float32_t angle_sustain = PI/adsr_settings.sustain_len;
	volatile float32_t angle_release = PI/adsr_settings.release_len;

	// Generic ADSR envelope
	// The waveform contains 5 segments (asdr + a blank space)
	// if(adsr_am || adsr_fm)
	if(adsr_settings.mod == VCOamp || adsr_settings.mod == VCOfreq || adsr_settings.mod == DualMode_VCO)
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

			else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < sustain_start)
			{
				// Decay
				buffer_adsr_am[i] = 1.0 * gen_rampdown_angle2( (sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr * angle_decay, adsr_settings.sustain_amp, 1.0);
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
			// buffer_output[i] = buffer_output[i] * buffer_adsr_am[i];
		}
	}


	/*
	 * ADSR frequency envelope.
	 * Uses the ADSR amplitude envelope and integrates each of the shapes.
	 */
	// if(adsr_fm)
	if(adsr_settings.mod == VCOfreq || adsr_settings.mod == DualMode_VCO)
	{
		for(i = start; i < end; i++)
		{
			// First part tells us sample number into the adsr cycle: (sample_count+(i-start))%sample_cycle_adsr
			if( (sample_count_adsr+(i-start))%samples_cycle_adsr < decay_start)
			{
				// Attack
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

	sample_count_adsr = sample_count_adsr + (i - start);
	sample_count_adsr = sample_count_adsr % samples_cycle_adsr;
}

void adsr_rad(uint16_t start, uint16_t end)
{
	volatile uint16_t i = 0;
	adsr_settings.attack_len = ADCBuffer[5]*20;		// A5
	adsr_settings.decay_len = (ADCBuffer[6])*20;	// A6
	adsr_settings.sustain_len = ADCBuffer[7]*20;	// A7
	adsr_settings.release_len = ADCBuffer[8]*20;	// B0
	adsr_settings.blank_len = ADCBuffer[10]*20;		// C0
	adsr_settings.sustain_amp = (float32_t) ADCBuffer[12]/4095;		// C4

	// One cycle is the entire ADSR envelope plus blank space.
	volatile uint32_t samples_cycle_adsr = adsr_settings.attack_len + adsr_settings.decay_len + adsr_settings.sustain_len + adsr_settings.release_len + adsr_settings.blank_len;
	volatile float32_t rads_per_sample_adsr = 0.0;

	// Avoid division by zero.
	rads_per_sample_adsr = 2.0 * PI / (float32_t) samples_cycle_adsr;
	if (isinf(rads_per_sample_adsr))
	{
		rads_per_sample_adsr = 1000;
	}

	adsr_settings.attack_len_rad = adsr_settings.attack_len * rads_per_sample_adsr;
	adsr_settings.decay_len_rad = adsr_settings.decay_len  * rads_per_sample_adsr;
	adsr_settings.sustain_len_rad = adsr_settings.sustain_len  * rads_per_sample_adsr;
	adsr_settings.release_len_rad = adsr_settings.release_len  * rads_per_sample_adsr;
	adsr_settings.blank_len_rad = adsr_settings.blank_len  * rads_per_sample_adsr;


	// float32_t attack_start_rad = 0.0;
	volatile float32_t decay_start_rad = adsr_settings.attack_len_rad;
	volatile float32_t sustain_start_rad = decay_start_rad + adsr_settings.decay_len_rad;
	volatile float32_t release_start_rad = sustain_start_rad + adsr_settings.sustain_len_rad;
	volatile float32_t blank_start_rad = release_start_rad + adsr_settings.release_len_rad;
	volatile float32_t blank_end_rad = blank_start_rad + adsr_settings.blank_len_rad;

	// for testing - should be 6.28...
	// volatile float32_t adsr_length_rad = adsr_settings.attack_len_rad + decay_start_rad + adsr_settings.decay_len_rad + sustain_start_rad + adsr_settings.sustain_len_rad + release_start_rad + adsr_settings.release_len_rad + blank_start_rad + adsr_settings.blank_len_rad;

	// Generic ADSR envelope
	// The waveform contains 5 segments (asdr + a blank space)
	if(adsr_settings.mod == VCOamp || adsr_settings.mod == VCOfreq || adsr_settings.mod == DualMode_VCO)
	{
		for(i = start; i < end; i++)
		{
			theta_adsr = theta_adsr + rads_per_sample_adsr;

			// First part tells us sample number into the adsr cycle: (sample_count+(i-start))%sample_cycle_adsr
			// if( (sample_count_adsr+(i-start))%samples_cycle_adsr < decay_start)
			if(theta_adsr < decay_start_rad)
			{
				// Attack
				// Sine, FM --> Try 1.0
				// Square, FM --> Use 0.4
				// Triangle, FM ---> Try 2.0
				// buffer_adsr_am[i] = 1.0 + 1.0 * gen_sawtooth_angle( (sample_count_adsr+(i-start)) % samples_cycle_adsr * angle_attack);
				buffer_adsr_am[i] = 1.0 + 1.0 * gen_sawtooth_angle(theta_adsr * PI/ adsr_settings.attack_len_rad);
			}

			// else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < sustain_start)
			else if(theta_adsr < sustain_start_rad)
			{
				// Decay
				// buffer_adsr_am[i] = 1.0 * gen_rampdown_angle2( (sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr * angle_decay, adsr_settings.sustain_amp, 1.0);
				buffer_adsr_am[i] = 1.0 * gen_rampdown_angle2(theta_adsr * 2 * PI/ adsr_settings.attack_len_rad, adsr_settings.sustain_amp, 1.0);
			}

			// else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < release_start)
			else if(theta_adsr < release_start_rad)
			{
				// Sustain
				buffer_adsr_am[i] = adsr_settings.sustain_amp;
			}

			// else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < blank_start)
			else if(theta_adsr < blank_start_rad)
			{
				// Release
				// buffer_adsr_am[i] = adsr_settings.sustain_amp * gen_rampdown_angle( (sample_count_adsr+(i-start-release_start)) % samples_cycle_adsr * angle_release);
				buffer_adsr_am[i] = adsr_settings.sustain_amp * gen_rampdown_angle( theta_adsr * PI/ adsr_settings.attack_len_rad );
			}

			// else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < blank_end)
			else if(theta_adsr < blank_end_rad)
			{
				// Blank
				buffer_adsr_am[i] = 0;
			}
			// TODO: I think we do this later.
			// buffer_output[i] = buffer_output[i] * buffer_adsr_am[i];
		}
	}


	/*
	 * ADSR frequency envelope.
	 * Uses the ADSR amplitude envelope and integrates each of the shapes.
	 */
	// if(adsr_fm)
	if(adsr_settings.mod == VCOfreq || adsr_settings.mod == DualMode_VCO)
	{
		for(i = start; i < end; i++)
		{
			// First part tells us sample number into the adsr cycle: (sample_count+(i-start))%sample_cycle_adsr
			// if( (sample_count_adsr+(i-start))%samples_cycle_adsr < decay_start)
			if(theta_adsr < decay_start_rad)
			{
				// Attack
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

			// else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < sustain_start)
			else if(theta_adsr < sustain_start_rad)
			{
				// Decay
				buffer_adsr_fm[i] = buffer_adsr_am[i];
				if(i > 0)
				{
					buffer_adsr_fm[i] = buffer_adsr_fm[i] + buffer_adsr_fm[i-1];
				}
				else
				{
					buffer_adsr_fm[i] = buffer_adsr_fm[i] + buffer_adsr_fm[BUFF_LEN-1];
				}

				// Get delta for release segment
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

			}
			else if(theta_adsr < release_start_rad)
			{
				// Sustain
				// DO this only once--get last delta from previous section.
				// TODO: dangerous -- comparing floats -- could result in mayhem.
				// if( (theta_adsr >= sustain_start_rad - 0.00001) && (theta_adsr <= sustain_start_rad + 0.00001))
//				if (floatcmp(theta_adsr, sustain_start_rad, 550) && testflag == 0)
//				{
//					testflag = 1;
//					if(i > 1)
//					{
//						delta = buffer_adsr_fm[i-1] - buffer_adsr_fm[i-2];
//					}
//					else if(i == 0)
//					{
//						delta = buffer_adsr_fm[BUFF_LEN-1] - buffer_adsr_fm[BUFF_LEN-2];
//					}
//					else if(i == 1)
//					{
//						delta = buffer_adsr_fm[i-1] - buffer_adsr_fm[BUFF_LEN-1];
//					}
//					buffer_adsr_fm[i] = 0.0;
//				}
//				else
//				{
					if(i > 0)
					{
						buffer_adsr_fm[i] = buffer_adsr_fm[i-1] + delta;
					}
					else
					{
						buffer_adsr_fm[i] = buffer_adsr_fm[BUFF_LEN-1] + delta;
					}
//				}
			}
			else if(theta_adsr < blank_start_rad)
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
			else if(theta_adsr < blank_end_rad)
			{
				// Blank
				buffer_adsr_fm[i] = 0;
			}
		}
	} // end of adsr fm

	testflag = 0;

}


///*
// * square()
// * Returns 0 if 0.0 < current_sample < samples_half_cycle
// * Returns 1 if current_sample > samples_half_cycle
// *
// * Parameter angle: value from 0.0 to 1.0.
// */
//float32_t gen_square(uint16_t current_sample, uint16_t samples_half_cycle)
//{
//	if (current_sample < samples_half_cycle)
//	{
//		return osc.square_min;
//	}
//	return osc.square_max;
//}

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
//float32_t gen_sawtooth(uint32_t current_sample, uint32_t samples_cycle, float32_t min, float32_t max)
//{
//	float32_t m = 0.0;
//	float32_t val = 0.0;
//
//	// y = mx + b
//	m = (max - min)/samples_cycle;
//	val = (m * current_sample) + min;
//
//	return val;
//}

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

//float32_t gen_rampdown(uint32_t current_sample, uint32_t samples_cycle, float32_t min, float32_t max)
//{
//	float32_t m = 0.0;
//	float32_t val = 0.0;
//
//	// y = mx + b
//	// m = (max - min)/samples_cycle;
//	// val = max - m * current_sample + min;
//
//	m = (min - max)/samples_cycle;
//	val = m * current_sample + max;
//
//	return val;
//}

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

//// TODO:
//float32_t gen_rampdown_integral_angle(float32_t angle)
//{
//	return 0.0;
//}


//float32_t gen_triangle(uint32_t current_sample, uint32_t samples_half_cycle, float32_t amp)
//{
//	float32_t m = 0.0;
//
//	// Increase from a negative value to its opposite value. Eg. -1 to 1 over 1/2 the wave's period
//	// Then decrease from 1 to -1 over 1/2 the wave's period
//
//	m = amp/(samples_half_cycle);
//
//	if(current_sample < samples_half_cycle)
//	{
//		return (m * current_sample);
//	}
//	// Make sure difference can be negative.
//	return amp + (m * (int32_t)(samples_half_cycle - current_sample));
//}

float32_t gen_triangle_angle(float32_t angle)
{
	float32_t val = 0.0;
	float32_t m = 0.0;

	// Increase from a negative value to its opposite value. Eg. -1 to 1 over 1/2 the wave's period
	// Then decrease from 1 to -1 over 1/2 the wave's period

	angle = fast_fmod(angle, 2*PI);
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



//float32_t gen_triangle_integral(uint32_t current_sample, uint32_t samples_half_cycle, float32_t amp)
//{
//	float32_t m = 0.0;
//	float32_t result = 0.0;
//
//	// Increase from a negative value to its opposite value. Eg. -1 to 1 over 1/2 the wave's period
//	// Then decrease from 1 to -1 over 1/2 the wave's period
//	m = amp/(samples_half_cycle);
//
//	if(current_sample < samples_half_cycle)
//	{
//		result = m*current_sample;
//		return result*result;
//	}
//	// Make sure difference can be negative.
//	result = amp + (m * (int32_t)(samples_half_cycle - current_sample));
//	return -(result*result);
//}

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

/*
 * Found here: https://gist.github.com/bmccormack/d12f4bf0c96423d03f82
 */
uint32_t movingAvg(uint32_t *ptrArrNumbers, uint32_t *ptrSum, uint32_t pos, uint32_t len, uint16_t nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return (uint32_t) *ptrSum / len;
}


///*
// * Param: value - sample value
// * Returns: running accumulation of values.
// * Assumes that input signal is centered around zero, so that accumulation centers around zero.
// */
//float32_t integrate(float32_t value)
//{
//	static float32_t sum;
//	sum = sum + value;
//	return sum;
//}

///*
// * This function was found at this address: https://gist.github.com/CAFxX/ad150f2403a0604e14cc
// */
//uint32_t ilog10c(uint64_t v)
//{
//  static const uint64_t thr[64] = {
//    10000000000000000000ULL, 0, 0, 0, 1000000000000000000ULL, 0, 0, 100000000000000000ULL, 0, 0,
//       10000000000000000ULL, 0, 0, 0,    1000000000000000ULL, 0, 0,    100000000000000ULL, 0, 0,
//          10000000000000ULL, 0, 0, 0,       1000000000000ULL, 0, 0,       100000000000ULL, 0, 0,
//             10000000000ULL, 0, 0, 0,          1000000000ULL, 0, 0,          100000000ULL, 0, 0,
//                10000000ULL, 0, 0, 0,             1000000ULL, 0, 0,             100000ULL, 0, 0,
//                   10000ULL, 0, 0, 0,                1000ULL, 0, 0,                100ULL, 0, 0,
//                      10ULL, 0, 0, 0
//  };
//  uint32_t lz = __builtin_clzll(v);
//  return (63 - lz) * 3 / 10 + (v >= thr[lz]);
//}


//void count_cycles()
//{
//	/* ************************************************************** */
//	// TODO: time this function call to estimate processor load.
//	/*
//	 * CPU cycle counting to measure duration of code.
//	 * This cycle-counting code was copied from here:
//	 * http://embeddedb.blogspot.ca/2013/10/how-to-count-cycles-on-arm-cortex-m.html
//	 */
////	volatile uint32_t count = 0;
////
////	// addresses of registers
////	volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
////	volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
////	volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;
////
////	// enable the use DWT
////	*DEMCR = *DEMCR | 0x01000000;
////
////	// Reset cycle counter
////	*DWT_CYCCNT = 0;
////
////	// enable cycle counter
////	*DWT_CONTROL = *DWT_CONTROL | 1 ;
//	/* ************************************************************** */
//
//
//	/* ************************************************************* */
//	// TODO: for measuring time.
//	// number of cycles stored in count variable
////	count = *DWT_CYCCNT;
//	/* ************************************************************* */
//}


/*
 * Attempt to remove noise spikes from adc inputs.
 * Param: buffer.  Buffer of last few samples.  Could possibly contain just 2 samples.
 * Param: i.  Index of current sample in buffer.
 * Param: max_diff.  Sets max allowable difference.
 */
//float32_t spike_filter(float32_t buffer[], uint16_t i, float32_t max_diff)
//{
//	// Choose previous index.
//	uint16_t prev_i = 0;
//	if (i == 0)
//	{
//		prev_i = SPIKE_FILTER_LEN - 1;
//	}
//	else
//	{
//		prev_i = i - 1;
//	}
//
//	// If new value exceeds difference, then ignore it.
////	if (buffer[i] + max_diff < buffer[prev_i] || buffer[i] - max_diff > buffer[prev_i])
////	{
////		buffer[i] = buffer[prev_i];
////	}
//
//
//	// If new value exceeds difference, then ignore it.
//	if (buffer[i] + max_diff < buffer[prev_i] || buffer[i] - max_diff > buffer[prev_i])
//	{
//		return buffer[prev_i];
//	}
//
//	return buffer[i];
//}

/*
 *
 * Found here: https://stackoverflow.com/questions/5989191/compare-two-floats
 */
//uint16_t floatcmp(float32_t float1, float32_t float2, uint16_t precision)
//{
//	uint16_t int1, int2;
//
//   if (float1 > 0)
//      int1 = (uint16_t)(float1 * precision + .5);
//   else
//      int1 = (uint16_t)(float1 * precision - .5);
//
//   if (float2 > 0)
//      int2 = (uint16_t)(float2 * precision + .5);
//   else
//      int2 = (uint16_t)(float2 * precision - .5);
//
//   return (int1 == int2);
//}
