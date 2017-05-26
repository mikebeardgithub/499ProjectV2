/*
 * main.h
 *
 *  Created on: May 19, 2017
 *      Author: admin
 */


/*
 * This file was downloaded and adapted from the project found here:
 * https://github.com/MrBlueXav/horrorophone-eclipse-with-makefile
 */



#ifndef MAIN_H_
#define MAIN_H_

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio_codec.h"

#include <stdio.h>
#include "stm32f4xx_it.h"

// TODO: remove these (test first).  They're set elsewhere.
#undef AUDIO_MAL_MODE_NORMAL
#define AUDIO_MAL_MODE_CIRCULAR

#define _2PI                    6.283185307f
#define _PI						3.14159265f
#define _INVPI					0.3183098861f
#define SAMPLERATE              48000
#define FREQ1                   440.0f   // default carrier frequency
#define FREQ2                   8.0f     // default modulation frequency


#define VOL                     80
#define BUFF_LEN_DIV4           32 // 2ms latency at 48kHz
#define BUFF_LEN_DIV2           64
#define BUFF_LEN                128
// #define BUFF_LEN                97
// #define BUFF_LEN                769  /* Audio buffer length : count in 16bits half-words */


#define DELAYLINE_LEN           14000  // max delay in samples
#define DELAY                   13000  // actual delay (in samples)
#define DELAY_VOLUME            0.1f   // 0.3f
#define FEEDB                   0.4f   //0.4f
#define ON                      1
#define OFF                     0

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)


#endif /* MAIN_H_ */
