/*
 * user_interface.c
 *
 *  Created on: Jun 10, 2017
 *      Author: jjank
 */

#include "user_interface.h"
#include "stm32f4xx_gpio.h"
#include "lcd.h"
#include "main.h"


/*
 * Gets and updated the state of both the selector rotary switches
 * This funtion is called by the tim2 interrupt handler
 */
void update_selector_state (){

	if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) == 1){
		vfo_state = sine;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8) == 1){
		vfo_state = sawtooth;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == 1){
		vfo_state = square;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10) == 1){
		vfo_state = triangle;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11) == 1){
		vfo_state = other2;
	}
//could put some fault detection at the end of these if statements come back later if theres time

	if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12) == 1){
		lfo_state = sine;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13) == 1){
		lfo_state = sawtooth;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14) == 1){
		lfo_state = square;
	}else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15) == 1){
		lfo_state = triangle;
	}else if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6) == 1){
		lfo_state = other2;
	}
}

void init_state(){

	current_menu_state.menu_state = startup;
	current_menu_state.cursor_option = 0;
	current_menu_state.adsr_mod = NO_MOD;
	current_menu_state.lfo_mod = NO_MOD;
	current_menu_state.filterst8 = no_filter;
	current_menu_state.secondary_vco = 0;
	display_new_menu();

}



void handle_enter(){
	if(current_menu_state.menu_state == startup){
		switch(current_menu_state.cursor_option){
			case 0:
				current_menu_state.menu_state = ADSR;
				current_menu_state.cursor_option = 0;
				display_new_menu();
				break;

			case 1:
				current_menu_state.menu_state = filter;
				current_menu_state.cursor_option = 0;
				display_new_menu();
				break;

			case 2:
				current_menu_state.menu_state = LFO;
				current_menu_state.cursor_option = 0;
				display_new_menu();
				break;

			case 3:
				current_menu_state.menu_state = secondaryVCO;
				current_menu_state.cursor_option = 0;
				display_new_menu();
				break;
		}

	}else if(current_menu_state.menu_state == filter){
		switch(current_menu_state.cursor_option){
			case 0:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.filterst8 = no_filter;
				display_new_menu();
				break;

			case 1:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.filterst8 = highpass;
				display_new_menu();
				break;

			case 2:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.filterst8 = lowpass;
				display_new_menu();
				break;

			case 3:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.filterst8 = bandpass;
				display_new_menu();
				break;
		}

	}else if(current_menu_state.menu_state == ADSR){
		switch(current_menu_state.cursor_option){
			case 0:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.adsr_mod = NO_MOD;
				display_new_menu();
				break;

			case 1:
				current_menu_state.menu_state = ADSR_Mod;
				current_menu_state.cursor_option = 0;
				display_new_menu();
				break;

			case 2:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.adsr_mod = DualMode_VCO;
				display_new_menu();
				break;

			case 3:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.adsr_mod = DualMode_LFO;
				display_new_menu();
				break;
		}

	}else if(current_menu_state.menu_state == LFO){
		switch(current_menu_state.cursor_option){
			case 0:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.lfo_mod = NO_MOD;
				display_new_menu();
				break;

			case 1:
				current_menu_state.menu_state = LFO_Mod;
				current_menu_state.cursor_option = 0;
				display_new_menu();
				break;

			case 2:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.lfo_mod = DualMode_VCO;
				display_new_menu();
				break;

			case 3:														// should never be here so this is error handling
				current_menu_state.menu_state = LFO;
				current_menu_state.cursor_option = 0;
				display_new_menu();
				break;
		}

	}else if(current_menu_state.menu_state == secondaryVCO){
		switch(current_menu_state.cursor_option){
			case 0:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.secondary_vco = 0;
				display_new_menu();
				break;

			case 1:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.secondary_vco = 1;
				display_new_menu();
				break;

			case 2:														// should never be here so this is error handling
				current_menu_state.menu_state = secondaryVCO;
				current_menu_state.cursor_option = 0;
				display_new_menu();
				break;

			case 3:														// should never be here so this is error handling
				current_menu_state.menu_state = secondaryVCO;
				current_menu_state.cursor_option = 0;
				display_new_menu();
				break;
		}

	}else if(current_menu_state.menu_state == ADSR_Mod){
		switch(current_menu_state.cursor_option){
			case 0:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.adsr_mod = VCOamp;
				display_new_menu();
				break;

			case 1:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.adsr_mod = VCOfreq;
				display_new_menu();
				break;

			case 2:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.adsr_mod = FILTER_freq;
				display_new_menu();
				break;
		}

	}else if(current_menu_state.menu_state == LFO_Mod){
		switch(current_menu_state.cursor_option){
			case 0:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.lfo_mod = VCOamp;
				display_new_menu();
				break;

			case 1:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.lfo_mod = VCOfreq;
				display_new_menu();
				break;

			case 2:
				current_menu_state.menu_state = startup;
				current_menu_state.cursor_option = 0;
				current_menu_state.lfo_mod = FILTER_freq;
				display_new_menu();
				break;


			case 3:														// should never be here so this is error handling
				current_menu_state.menu_state = LFO_Mod;
				current_menu_state.cursor_option = 0;
				display_new_menu();
				break;
		}
	}
}

/*
 * not gonna use this button
 */
void handle_back(){


	if(current_menu_state.menu_state == startup){
		LCD_WRITE(32);
		current_menu_state.cursor_option = 0;
		curser_start();

	}else if(current_menu_state.menu_state == ADSR_Mod){
		current_menu_state.menu_state = ADSR;
		current_menu_state.cursor_option = 0;
		display_new_menu();

	}else if(current_menu_state.menu_state == LFO_Mod){
		current_menu_state.menu_state = LFO;
		current_menu_state.cursor_option = 0;
		display_new_menu();

	}else{
		current_menu_state.menu_state = startup;
		current_menu_state.cursor_option = 0;
		display_new_menu();

	}
}



void handle_up(){
	if(current_menu_state.menu_state == LFO || current_menu_state.menu_state == LFO_Mod || current_menu_state.menu_state == ADSR_Mod || current_menu_state.menu_state == ADSR ){
		switch(current_menu_state.cursor_option){
			case 0:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 1;
				LCD_SETDDRAM(0x1D);
				LCD_WRITE(199);
				LCD_SETDDRAM(0x1D);
				break;

			case 1:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 2;
				LCD_SETDDRAM(0x54);
				LCD_WRITE(199);
				LCD_SETDDRAM(0x54);
				break;

			case 2:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 0;
				curser_start();
				break;
		}

	}else if(current_menu_state.menu_state == secondaryVCO){
		switch(current_menu_state.cursor_option){
			case 0:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 1;
				LCD_SETDDRAM(0x1D);
				LCD_WRITE(199);
				LCD_SETDDRAM(0x1D);
				break;

			case 1:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 0;
				curser_start();
				break;
		}

	}else{
		switch(current_menu_state.cursor_option){
			case 0:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 1;
				LCD_SETDDRAM(0x1D);
				LCD_WRITE(199);
				LCD_SETDDRAM(0x1D);
				break;

			case 1:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 2;
				LCD_SETDDRAM(0x54);
				LCD_WRITE(199);
				LCD_SETDDRAM(0x54);
				break;

			case 2:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 3;
				LCD_SETDDRAM(0x5D);
				LCD_WRITE(199);
				LCD_SETDDRAM(0x5D);
				break;

			case 3:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 0;
				curser_start();
				break;
		}
	}
}


/*
 * not gonna use this button
 */
void handle_down(){
	if(current_menu_state.menu_state == LFO || current_menu_state.menu_state == LFO_Mod || current_menu_state.menu_state == ADSR_Mod ){
		switch(current_menu_state.cursor_option){
			case 0:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 2;
				LCD_SETDDRAM(0x54);
				LCD_WRITE(199);
				LCD_SETDDRAM(0x54);
				break;

			case 1:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 0;
				curser_start();
				break;

			case 2:
				LCD_WRITE(32);
				current_menu_state.cursor_option = 1;
				LCD_SETDDRAM(0x1D);
				LCD_WRITE(199);
				LCD_SETDDRAM(0x1D);
				break;
			}

	}else if(current_menu_state.menu_state == secondaryVCO){
			switch(current_menu_state.cursor_option){
				case 0:
					LCD_WRITE(32);
					current_menu_state.cursor_option = 1;
					LCD_SETDDRAM(0x1D);
					LCD_WRITE(199);
					LCD_SETDDRAM(0x1D);
					break;

				case 1:
					LCD_WRITE(32);
					current_menu_state.cursor_option = 0;
					curser_start();
					break;
			}

	}else{
		switch(current_menu_state.cursor_option){
		case 0:
			LCD_WRITE(32);
			current_menu_state.cursor_option = 3;
			LCD_SETDDRAM(0x5D);
			LCD_WRITE(199);
			LCD_SETDDRAM(0x5D);
			break;

		case 1:
			LCD_WRITE(32);
			current_menu_state.cursor_option = 0;
			curser_start();
			break;

		case 2:
			LCD_WRITE(32);
			current_menu_state.cursor_option = 1;
			LCD_SETDDRAM(0x1D);
			LCD_WRITE(199);
			LCD_SETDDRAM(0x1D);
			break;

		case 3:
			LCD_WRITE(32);
			current_menu_state.cursor_option = 2;
			LCD_SETDDRAM(0x54);
			LCD_WRITE(199);
			LCD_SETDDRAM(0x54);
			break;
		}
	}
}

void display_new_menu(){

	switch(current_menu_state.menu_state){

		case startup:
			lcd_writeline("     Main  Menu     ", 1);
			lcd_writeline(" ADSR     Filter    ", 2);
			lcd_writeline(" LFO      Sec VCO   ", 3);
			break;

		case filter:
			lcd_writeline("    Filter  Menu    ", 1);
			lcd_writeline(" Off      HighPass  ", 2);
			lcd_writeline(" LowPass  BandPass  ", 3);
			break;

		case ADSR:
			lcd_writeline("     ADSR  Menu     ", 1);
			lcd_writeline(" Off      ON        ", 2);
			lcd_writeline(" AM+FM_VCO_Mode     ", 3);
			break;

		case ADSR_Mod:
			lcd_writeline("ADSR Modulation Menu", 1);
			lcd_writeline(" VCO_AMP  VCO_FREQ  ", 2);
			lcd_writeline(" Filter_Freq        ", 3);
			break;

		case LFO:

			lcd_writeline("     LFO   Menu     ", 1);
			lcd_writeline(" OFF      ON        ", 2);
			lcd_writeline(" AM+FM_VCO_Mode     ", 3);
			break;

		case LFO_Mod:
			lcd_writeline("LFO Modulation Menu ", 1);
			lcd_writeline(" VCO_AMP  VCO_FREQ  ", 2);
			lcd_writeline(" Filter_Freq        ", 3);
			break;

		case secondaryVCO:
			lcd_writeline(" Secondary VCO Menu ", 1);
			lcd_writeline(" OFF      ON        ", 2);
			lcd_writeline("                    ", 3);
			break;
		}
	curser_start();
	}


void curser_start(){
	LCD_SETDDRAM(0x14);
	LCD_WRITE(199);
	LCD_SETDDRAM(0x14);
}
