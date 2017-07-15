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
	.vco_freq = 0.000001,
	.vco2_freq = 413.7,
	.lfo_freq = 1.73,

	.vco_wav = square,
	.lfo_wav = other2,		// other2 means OFF, for now.
	.mod = NO_MOD,

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

/*
 *	Set ADSR lengths in numbers of samples.
 */

// Percussive
adsr_setting adsr_01 = {
		.mod = VCOamp,

		.sustain_amp=0.7,

		.attack_len=400,
		.decay_len=400,
		.sustain_len=3500,
		.release_len=700,
		.blank_len=14000
};

// Bell
adsr_setting adsr_02 = {
		.mod = VCOamp,
		.sustain_amp=0.3,

		.attack_len=300,
		.decay_len=300,
		.sustain_len=10000,
		.release_len=30000,
		.blank_len=30000
};


adsr_setting adsr_03 = {
		.mod = VCOamp,
		.sustain_amp=0.5,

		.attack_len=5000,
		.decay_len=500,
		.sustain_len=30000,
		.release_len=900,
		.blank_len=40000
};

adsr_setting adsr_04 = {
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
uint32_t attack_start = 0;
uint32_t decay_start = 0;
uint32_t sustain_start = 0;
uint32_t release_start = 0;
uint32_t blank_start = 0;
uint32_t blank_end = 0;


/*
 * Moving average -- TODO: remove after testing.
 */
#define MOV_AVG_BUFF_LEN		256
volatile uint32_t mov_avg [MOV_AVG_BUFF_LEN] = {0};
volatile uint32_t mov_avg_index = 0;
volatile uint32_t mov_avg_sum;

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

float32_t theta_vco = 0;
float32_t theta_lfo = 0;
float32_t theta_adsr = 0;

void generate_waveforms(uint16_t start, uint16_t end)
{
	// Get wave shape.
	osc.vco_wav = vfo_state;
	osc.lfo_wav = lfo_state;

	// osc.vco_wav = sine;
	// osc.vco_wav = sine;

	// Oscillators - amplitude and frequency.
	osc.vco_amp = (float) (ADCBuffer[0] & 0xffff);					// A0
	osc.vco_freq = (float) (ADCBuffer[1] & 0xffff) * 2 * PI;		// A1
	osc.lfo_amp = (float) (ADCBuffer[2] & 0xffff)/200;				// AM: div by 4095
	osc.lfo_freq = (float) (ADCBuffer[3] & 0xffff)/20;				// TODO: AM: div by 20

	uint32_t i = 0;

	// Get ADSR values.
	// adsr_setting adsr_settings = adsr_03;			// Fall back on this.
	adsr_setting adsr_settings;


	// adsr_settings.mod = DualMode_VCO;
	adsr_settings.mod = NO_MOD;						// TODO: turn this off when LCD activated.
	/*
	 * TODO: Turn this on when LCD activated.
	 * 	adsr_settings.mod = menu_state.adsr_mod;
	 */

	adsr_settings.attack_len = ADCBuffer[5]*20;		// A5
	adsr_settings.decay_len = (ADCBuffer[6])*20;	// A6
	adsr_settings.sustain_len = ADCBuffer[7]*10;	// A7
	adsr_settings.release_len = ADCBuffer[8]*20;	// B0
	adsr_settings.blank_len = ADCBuffer[10]*20;		// C0
	adsr_settings.sustain_amp = (float32_t) ADCBuffer[12]/4095;		// C4

	// Calculate ADSR boundaries.
	attack_start = 0;
	decay_start = adsr_settings.attack_len;
	sustain_start = decay_start + adsr_settings.decay_len;
	release_start = sustain_start + adsr_settings.sustain_len;
	blank_start = release_start + adsr_settings.release_len;
	blank_end = blank_start + adsr_settings.blank_len;

	// Calculate angle amount to increment per sample.
	float32_t rads_per_sample_vco = osc.vco_freq / ONE_SECOND;		// Radians to increment for each iteration.
	float32_t rads_per_sample_lfo = osc.lfo_freq / ONE_SECOND;		// Radians to increment for each iteration.
	// TODO: float32_t rads_per_sample_adsr = 1 * TWO_PI / ONE_SECOND;

	// One cycle is the entire ADSR envelope plus blank space.
	uint32_t samples_cycle_adsr = adsr_settings.attack_len + adsr_settings.decay_len + adsr_settings.sustain_len + adsr_settings.release_len + adsr_settings.blank_len;
	// float32_t rads_per_sample_adsr = TWO_PI / samples_cycle_adsr;

	adsr_settings.attack_len_rad = adsr_settings.attack_len  * TWO_PI / samples_cycle_adsr;
	adsr_settings.decay_len_rad = adsr_settings.decay_len  * TWO_PI / samples_cycle_adsr;
	adsr_settings.sustain_len_rad = adsr_settings.sustain_len  * TWO_PI / samples_cycle_adsr;
	adsr_settings.release_len_rad = adsr_settings.release_len  * TWO_PI / samples_cycle_adsr;
	adsr_settings.blank_len_rad = adsr_settings.blank_len  * TWO_PI / samples_cycle_adsr;

	// float32_t attack_start_rad = 0.0;
	float32_t decay_start_rad = adsr_settings.attack_len_rad;
	float32_t sustain_start_rad = decay_start_rad + adsr_settings.decay_len_rad;
	float32_t release_start_rad = sustain_start_rad + adsr_settings.sustain_len_rad;
	float32_t blank_start_rad = release_start_rad + adsr_settings.release_len_rad;
	float32_t blank_end_rad = blank_start_rad + adsr_settings.blank_len_rad;


//	// Generic ADSR envelope
//	// The waveform contains 5 segments (asdr + a blank space)
//	// if(adsr_am || adsr_fm)
	if(adsr_settings.mod == VCOamp || adsr_settings.mod == VCOfreq || adsr_settings.mod == DualMode_VCO)
	{
		for(i = start; i < end; i++)
		{
			// First part tells us sample number into the adsr cycle: (sample_count+(i-start))%sample_cycle_adsr
			// if( (sample_count_adsr+(i-start))%samples_cycle_adsr < decay_start)
			if(theta_adsr < decay_start_rad)
			{
				// Attack
				// Sine, FM --> Try 1.0
				// Square, FM --> Use 0.4
				// Triangle, FM ---> Try 2.0
				// buffer_adsr_am[i] = 1.0 + 1.0 * gen_sawtooth_angle( (sample_count_adsr+(i-start)) % samples_cycle_adsr * angle_attack);
				buffer_adsr_am[i] = 1.0 + 1.0 * gen_sawtooth_angle( theta_adsr);
			}

			// else if( (sample_count_adsr+(i-start))%samples_cycle_adsr < sustain_start)
			else if(theta_adsr < sustain_start_rad)
			{
				// Decay
				// buffer_adsr_am[i] = 1.0 * gen_rampdown_angle2( (sample_count_adsr+(i-start-decay_start)) % samples_cycle_adsr * angle_decay, adsr_settings.sustain_amp, 1.0);
				buffer_adsr_am[i] = 1.0 * gen_rampdown_angle2(theta_adsr, adsr_settings.sustain_amp, 1.0);
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
				buffer_adsr_am[i] = adsr_settings.sustain_amp * gen_rampdown_angle( theta_adsr );
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


	// Sine LFO
	if(osc.lfo_wav == sine)
	{

		for(i = start; i < end; i++)
		{
			theta_lfo = theta_lfo + rads_per_sample_lfo;
			buffer_lfo_float[i] = osc.lfo_amp + osc.lfo_amp*arm_sin_f32(theta_lfo);
		}
	}

	// Square LFO
	if(osc.lfo_wav == square)
	{

		for(i = start; i < end; i++)
		{
			theta_lfo = theta_lfo + rads_per_sample_lfo;
			buffer_lfo_float[i] = osc.lfo_amp + osc.lfo_amp*gen_square_angle(theta_lfo);
		}
	}

	// Sawtooth LFO
	if(osc.lfo_wav == sawtooth)
	{

		for(i = start; i < end; i++)
		{
			theta_lfo = theta_lfo + rads_per_sample_lfo;
			buffer_lfo_float[i] = osc.lfo_amp + osc.lfo_amp*gen_sawtooth_angle(theta_lfo);
		}
	}

	// Triangle LFO
	if(osc.lfo_wav == triangle)
	{

		for(i = start; i < end; i++)
		{
			theta_lfo = theta_lfo + rads_per_sample_lfo;
			buffer_lfo_float[i] = osc.lfo_amp + osc.lfo_amp*gen_triangle_angle(theta_lfo);
		}
	}

	// Sine VCO
	if(osc.vco_wav == sine)
	{

		for(i = start; i < end; i++)
		{
			theta_vco = theta_vco + rads_per_sample_vco;
			buffer_output[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32(theta_vco + buffer_lfo_float[i] + buffer_adsr_fm[i]);
		}
	}

	// Square VCO
	if(osc.vco_wav == square)
	{

		for(i = start; i < end; i++)
		{
			theta_vco = theta_vco + rads_per_sample_vco;
			buffer_output[i] = osc.vco_amp + osc.vco_amp*gen_square_angle(theta_vco + 100*buffer_lfo_float[i] + buffer_adsr_fm[i]);
		}
	}

	// Sawtooth VCO
	if(osc.vco_wav == sawtooth)
	{

		for(i = start; i < end; i++)
		{
			theta_vco = theta_vco + rads_per_sample_vco;
			buffer_output[i] = osc.vco_amp + osc.vco_amp*gen_sawtooth_angle(theta_vco + 100*buffer_lfo_float[i] + buffer_adsr_fm[i]);
		}
	}

	// Triangle VCO
	if(osc.vco_wav == triangle)
	{

		for(i = start; i < end; i++)
		{
			theta_vco = theta_vco + rads_per_sample_vco;
			buffer_output[i] = osc.vco_amp + osc.vco_amp*gen_triangle_angle(theta_vco + 100*buffer_lfo_float[i] + buffer_adsr_fm[i]);
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

	return;
}



/*
 * Clean version - for testing only
 */
void generate_waveforms2(uint16_t start, uint16_t end)
{
	uint32_t i = 0;
	osc.vco_wav = square;
	osc.vco_freq = (float) (ADCBuffer[1] & 0xffff) * 2 * PI;					// A1


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
	osc.vco_freq = movingAvg(mov_avg, &mov_avg_sum, mov_avg_index, MOV_AVG_BUFF_LEN, ADCBuffer[1] & 0xfffc);
	mov_avg_index++;
	if (mov_avg_index >= MOV_AVG_BUFF_LEN)
	{
		mov_avg_index = 0;
	}

	float32_t rads_per_sample = osc.vco_freq / ONE_SECOND;		// Radians to increment for each iteration.

	for(i = start; i < end; i++)
	{
		theta_vco = theta_vco + rads_per_sample;
		// buffer_output[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32(theta_vco);
		buffer_output[i] = osc.vco_amp + osc.vco_amp*gen_square_angle(theta_vco);
	}



	theta_vco = fast_fmod(theta_vco, 2 * PI);

	return;
}





void generate_waveforms1(uint16_t start, uint16_t end)
{
	uint32_t max_sample_count = QUARTER_SECOND;


	osc.vco_wav = vfo_state;
	osc.lfo_wav = lfo_state;

	// osc.vco_amp = (float) (ADCBuffer[0] & 0xfffe);			// A0
	osc.vco_freq = (float) (ADCBuffer[1]);			// A1
	// osc.lfo_amp = (float) (ADCBuffer[2] & 0xfffe)/4095;		// AM: div by 4095
	// osc.lfo_freq = (float) (ADCBuffer[3] & 0xfff0)/20;			// AM: div by 20

	// My moving average filter.
	// Moving average filter osc.vco_freq
//	mov_avg[mov_avg_index] = ADCBuffer[1] & 0xff00;								// Get most recent value.
//	mov_avg_sum += ADCBuffer[1] & 0xff00;										// Accumulate.
//	mov_avg_sum -= mov_avg[(mov_avg_index + 1) % MOV_AVG_BUFF_LEN];				// Subtract oldest.
//	mov_avg_index++;
//	if (mov_avg_index >= MOV_AVG_BUFF_LEN)
//	{
//		mov_avg_index = 0;
//	}
//	osc.vco_freq = ( (float32_t)  mov_avg_sum)/MOV_AVG_BUFF_LEN;

	// ---------------------------------------
	// Someone else's moving average filter.
	// Found here: https://gist.github.com/bmccormack/d12f4bf0c96423d03f82
	// Note that this discards bits and then smooths it.  What if it did this the other way around?
	// osc.vco_freq = log10(ADCBuffer[1] & 0xffff)*200;
//	osc.vco_freq = movingAvg(mov_avg, &mov_avg_sum, mov_avg_index, MOV_AVG_BUFF_LEN, ADCBuffer[1] & 0xfff0);
//	mov_avg_index++;
//	if (mov_avg_index >= MOV_AVG_BUFF_LEN)
//	{
//		mov_avg_index = 0;
//	}


	/*
	 * Noise from ADC seems to be very fast spikes.
	 * Attempt to remove them by ignoring samples that differ greatly from previous samples.
	 */
//	spike_buff[spike_buff_index] = ADCBuffer[1];
//	spike_buff_index++;
//	if (spike_buff_index >= SPIKE_FILTER_LEN)
//	{
//		spike_buff_index = 0;
//	}
//	osc.vco_freq = spike_filter(spike_buff, spike_buff_index, 2000);



	/*
	 * Additional ADC values.
	 */
	// volume = ADCBuffer[4];
	// fc_low = ADCBuffer[9];
	// fc_high = ADCBuffer[10];
	// fc_resonance = ADCBuffer[11];
	// gain = ADCBuffer[12];


	// Get ADSR values.
	adsr_setting adsr_settings = adsr_03;			// Fall back on this.

	// TODO: turn this off...
	// adsr_settings.mod = DualMode_VCO;
	adsr_settings.mod = NO_MOD;
	/*
	 * TODO: Turn this on...
	 * 	adsr_settings.mod = menu_state.adsr_mod;
	 */

	adsr_settings.attack_len = ADCBuffer[5]*20;		// A5
	adsr_settings.decay_len = (ADCBuffer[6])*20;	// A6
	adsr_settings.sustain_len = ADCBuffer[7]*10;	// A7
	adsr_settings.release_len = ADCBuffer[8]*20;	// B0
	adsr_settings.blank_len = ADCBuffer[10]*20;		// C0
	adsr_settings.sustain_amp = (float32_t) ADCBuffer[12]/4095;		// C4

	// Calculate ADSR boundaries.
	attack_start = 0;
	decay_start = adsr_settings.attack_len;
	sustain_start = decay_start + adsr_settings.decay_len;
	release_start = sustain_start + adsr_settings.sustain_len;
	blank_start = release_start + adsr_settings.release_len;
	blank_end = blank_start + adsr_settings.blank_len;

	volatile int i = 0;

	// Calculate "angles".  This is the sample position in a cycle.
	volatile float32_t angle_vco = osc.vco_freq*PI_DIV_2/(max_sample_count);	// 'angle' based samples per cycle.
	// volatile float32_t angle_vco2 = osc.vco2_freq*PI_DIV_2/(max_sample_count);	// 'angle' based samples per cycle.
	volatile float32_t angle_lfo = osc.lfo_freq*PI_DIV_2/(max_sample_count);

	volatile float32_t angle_attack = PI/adsr_settings.attack_len;
	volatile float32_t angle_decay = PI/adsr_settings.decay_len;
	// volatile float32_t angle_sustain = PI/adsr_settings.sustain_len;
	volatile float32_t angle_release = PI/adsr_settings.release_len;

	// Calculate number of cycles.
	// volatile float32_t samples_cycle_vco = SAMPLERATE / osc.vco_freq;
	volatile uint32_t samples_cycle_lfo = 2 * SAMPLERATE / osc.lfo_freq;
	volatile uint32_t samples_cycle_adsr = adsr_settings.attack_len + adsr_settings.decay_len + adsr_settings.sustain_len + adsr_settings.release_len + adsr_settings.blank_len;

	/*
	 * TODO: these VCO calculations can be combined with the FM ones -- for AM no modulation, just set phase to 0.
	 */

	// Sine VCO
	// if(osc.vco_wav == sine && osc.fm_mod == OFF)
	if(osc.vco_wav == sine && (osc.mod == NO_MOD || osc.mod == VCOamp ))
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32((sample_count_vco+(i-start))*angle_vco);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2*arm_sin_f32((sample_count_vco+(i-start))*angle_vco2);
		}
	}

	// Square VCO
	// else if(osc.vco_wav == square && osc.fm_mod == OFF)
	if(osc.vco_wav == square && (osc.mod == NO_MOD || osc.mod == VCOamp ) )
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = osc.vco_amp + osc.vco_amp * gen_square_angle((sample_count_vco+(i-start)) * angle_vco);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_square_angle((sample_count_vco+(i-start)) * angle_vco2);
		}
	}

	// Sawtooth VCO
	// else if(osc.vco_wav == sawtooth && osc.fm_mod == OFF)
	if(osc.vco_wav == sawtooth && (osc.mod == NO_MOD || osc.mod == VCOamp ) )
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = osc.vco_amp + osc.vco_amp * gen_sawtooth_angle((sample_count_vco+(i-start)) * angle_vco);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_sawtooth_angle((sample_count_vco+(i-start)) * angle_vco2);
		}
	}

	// Triangle VCO
	// else if(osc.vco_wav == triangle && osc.fm_mod == OFF)
	if(osc.vco_wav == triangle && (osc.mod == NO_MOD || osc.mod == VCOamp ) )
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = osc.vco_amp + osc.vco_amp * gen_triangle_angle((sample_count_vco+(i-start)) * angle_vco);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_triangle_angle((sample_count_vco+(i-start)) * angle_vco2);
		}
	}

	// SINE LFO
	if(osc.lfo_wav == sine)
	{
		// if(osc.fm_mod == OFF)
		if(osc.mod == VCOamp || osc.mod == DualMode_VCO)
		{
			for(i = start; i < end; i++)
			{
				// AM - Requires an amplitude offset.
				buffer_lfo_float[i] = osc.lfo_amp + osc.lfo_amp*arm_sin_f32((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
		else if(osc.mod == VCOfreq)
		{
			for(i = start; i < end; i++)
			{
				// FM - No offset.
				buffer_lfo_float[i] = osc.lfo_amp*arm_cos_f32((sample_count_lfo+(i-start))*angle_lfo);
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
				buffer_lfo_float[i] = osc.lfo_amp*gen_square_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
		else if(osc.mod == VCOfreq)
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
	else if(osc.lfo_wav == sawtooth)
	{
		if(osc.mod == VCOamp || osc.mod == DualMode_VCO)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = osc.lfo_amp*gen_sawtooth_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}

		// If FM mod, need integral of modulating signal.  Integral of ramp is right side of parabola.
		else if(osc.mod == VCOfreq)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = osc.lfo_amp*gen_sawtooth_integral_angle((sample_count_lfo+(i-start))*angle_lfo);
			}
		}
	}

	else if(osc.lfo_wav == triangle)
	{
		if(osc.mod == VCOamp || osc.mod == DualMode_VCO)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = osc.lfo_amp*gen_triangle_angle( (sample_count_lfo+(i-start)) * angle_lfo);
			}
		}

		// If FM mod, need integral of modulating signal.
		else if(osc.mod == VCOfreq)
		{
			for(i = start; i < end; i++)
			{
				buffer_lfo_float[i] = osc.lfo_amp*gen_triangle_integral_angle( (sample_count_lfo+(i-start)) * angle_lfo);
			}
		}
	}

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
			buffer_output[i] = buffer_output[i] * buffer_adsr_am[i];
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

	// FM for sine wave VCO.
	if(osc.vco_wav == sine && ( osc.mod == VCOfreq || osc.mod == DualMode_VCO || adsr_settings.mod == VCOfreq || adsr_settings.mod == DualMode_VCO ) )
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = osc.vco_amp + osc.vco_amp*arm_sin_f32((sample_count_vco+(i-start))*angle_vco + 100*buffer_lfo_float[i] + buffer_adsr_fm[i]);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2*arm_sin_f32((sample_count_vco+(i-start))*angle_vco2 + 100*buffer_lfo_float[i]);
		}
	}

	// FM for square wave VCO.
	// else if(osc.vco_wav == square && ( osc.mod == ON || adsr_fm == ON ) )
	else if(osc.vco_wav == square && ( osc.mod == VCOfreq || osc.mod == DualMode_VCO || adsr_settings.mod == VCOfreq || adsr_settings.mod == DualMode_VCO ) )
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = osc.vco_amp + osc.vco_amp * gen_square_angle((sample_count_vco+(i-start))*angle_vco + 100*buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_square_angle((sample_count_vco+(i-start))*angle_vco2 + 50*buffer_lfo_float[i]);
		}
	}

	// FM for sawtooth wave VCO.
	// else if(osc.vco_wav == sawtooth && ( osc.mod == ON || adsr_fm == ON ) )
	else if(osc.vco_wav == sawtooth && ( osc.mod == VCOfreq || osc.mod == DualMode_VCO || adsr_settings.mod == VCOfreq || adsr_settings.mod == DualMode_VCO ) )
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = osc.vco_amp + osc.vco_amp * gen_sawtooth_angle((sample_count_vco+(i-start))*angle_vco + 50*buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_sawtooth_angle((sample_count_vco+(i-start))*angle_vco2 + 50*buffer_lfo_float[i] + 0.3 * buffer_adsr_fm[i]);
		}
	}

	// FM for triangle wave VCO.
	// else if(osc.vco_wav == triangle && ( osc.mod == ON || adsr_fm == ON ) )
	else if(osc.vco_wav == triangle && ( osc.mod == VCOfreq || osc.mod == DualMode_VCO || adsr_settings.mod == VCOfreq || adsr_settings.mod == DualMode_VCO ) )
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = osc.vco_amp + osc.vco_amp * gen_triangle_angle((sample_count_vco+(i-start))*angle_vco + 50*buffer_lfo_float[i] + 0.01 * buffer_adsr_fm[i]);
			// buffer_vco2[i] = osc.vco_amp2 + osc.vco_amp2 * gen_triangle_angle((sample_count_vco+(i-start))*angle_vco2 + 50*buffer_lfo_float[i]);
		}
	}

	// AM modulation.
	// if(osc.am_mod == ON)
	if(osc.mod == VCOamp || osc.mod == DualMode_VCO)
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = buffer_output[i] * buffer_lfo_float[i];
		}
	}

	// ADSR amplitude envelope
	// The waveform contains 5 segments (asdr + a blank space)
	// if(adsr_am)
	if(adsr_settings.mod == VCOamp || adsr_settings.mod == DualMode_VCO)
	{
		for(i = start; i < end; i++)
		{
			buffer_output[i] = buffer_output[i] * buffer_adsr_am[i];
		}
	}

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
	sample_count_vco = sample_count_vco % TWENTY_SECOND;

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


/*
 * TODO: Remove after testing
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


/*
 * Attempt to remove noise spikes from adc inputs.
 * Param: buffer.  Buffer of last few samples.  Could possibly contain just 2 samples.
 * Param: i.  Index of current sample in buffer.
 * Param: max_diff.  Sets max allowable difference.
 */
float32_t spike_filter(float32_t buffer[], uint16_t i, float32_t max_diff)
{
	// Choose previous index.
	uint16_t prev_i = 0;
	if (i == 0)
	{
		prev_i = SPIKE_FILTER_LEN - 1;
	}
	else
	{
		prev_i = i - 1;
	}

	// If new value exceeds difference, then ignore it.
//	if (buffer[i] + max_diff < buffer[prev_i] || buffer[i] - max_diff > buffer[prev_i])
//	{
//		buffer[i] = buffer[prev_i];
//	}


	// If new value exceeds difference, then ignore it.
	if (buffer[i] + max_diff < buffer[prev_i] || buffer[i] - max_diff > buffer[prev_i])
	{
		return buffer[prev_i];
	}

	return buffer[i];
}

