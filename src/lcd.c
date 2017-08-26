/*
 * lcd.c
 *
 *  Created on: Jul 1, 2017
 *      Author: jjank
 */

#include "lcd.h"
#include "stm32f4xx_spi.h"

int g_lcd_pos;

void delay(int delay){
	for(int i=delay;i>0;i--){

	}
	return;
}



void SPI_TX(int tx){

	//RS
	if(tx & 0x0200){
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
	}else{
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	}

	//R/W
	if(tx & 0x0100){
		GPIO_SetBits(GPIOD, GPIO_Pin_6);
	}else{
		GPIO_ResetBits(GPIOD, GPIO_Pin_6);
	}

	GPIO_SetBits(GPIOD, GPIO_Pin_12);

	//DB0
	if(tx & 0x01){
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
	}else{
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
	}
	//DB1
	if(tx & 0x02){
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
	}else{
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	}
	//DB2
	if(tx & 0x04){
		GPIO_SetBits(GPIOC, GPIO_Pin_11);
	}else{
		GPIO_ResetBits(GPIOC, GPIO_Pin_11);
	}
	//DB3
	if(tx & 0x08){
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	}else{
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	}
	//DB4
	if(tx & 0x10){
		GPIO_SetBits(GPIOC, GPIO_Pin_14);
	}else{
		GPIO_ResetBits(GPIOC, GPIO_Pin_14);
	}
	//DB5
	if(tx & 0x20){
		GPIO_SetBits(GPIOC, GPIO_Pin_15);
	}else{
		GPIO_ResetBits(GPIOC, GPIO_Pin_15);
	}
	//DB6
	if(tx & 0x40){
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
	}else{
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	}
	//DB7
	if(tx & 0x80){
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
	}else{
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	}

	//cycle enable bits
	GPIO_ResetBits(GPIOD, GPIO_Pin_12);

	//not using SPI any more gona configure in parallel and use this function to send the instructions/data


//	tx=tx<<6;
//	//while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
//	if ((SPI2->SR & SPI_SR_TXE) == SPI_SR_TXE){			 /* Test Tx empty */
//
//
//		SPI2->DR = tx;									/* Will inititiate 8-bit transmission if TXE */
//		while(SPI2->SR & SPI_SR_BSY);					//wait for SPI2 to be done
//		GPIO_SetBits(GPIOC, GPIO_Pin_5);				//chip select high
//	}
//
//	return;
//	GPIO_ResetBits(GPIOC, GPIO_Pin_5);				//chip select low
//	while(SPI2->SR & SPI_SR_BSY);					//wait for SPI2 to be ready
//	SPI_I2S_SendData(SPI2, tx);
//	while(SPI2->SR & SPI_SR_BSY);					//wait for SPI2 to be done
//	GPIO_SetBits(GPIOC, GPIO_Pin_5);				//chip select high

}

int LCD_TX(unsigned int RS,unsigned int RW, unsigned int data)
{
  int temp = data;
  temp += (RW<<8);
  temp+= (RS<<9);
  //GPIO_ResetBits(GPIOC, GPIO_Pin_5);				//chip select low no more chip select with parallel comms
  SPI_TX(temp);
  //GPIO_SetBits(GPIOC, GPIO_Pin_5);				//chip select high no more chip select with parallel comms
  return temp;

}

void LCD_CLEAR(void)
{
    int rs = 0;
    int rw = 0;
    int data = 0b1;
    LCD_TX(rs,rw,data);
}

void LCD_HOME(void)
{
    int rs = 0;
    int rw = 0;
    int data = 0b10;
    LCD_TX(rs,rw,data);
    g_lcd_pos = 0;
}

void LCD_MODE(int incr,int shift)
{
    int rs = 0;
    int rw = 0;
    int data = (incr&0b1)<<1;
    data += (shift & 0b1);
    data += (0b1<<2);
    LCD_TX(rs,rw,data);
}

void LCD_DISPLAY(int on,int cursor, int blink)
{
    int rs = 0;
    int rw = 0;
    int data = (0b1<<3);
    data += (on&0b1)<<2;
    data += (cursor&0b1)<<1;
    data += (blink &0b1);
    LCD_TX(rs,rw,data);
}

void LCD_SHIFT(int shift, int direction)
{
    int rs = 0;
    int rw = 0;
    int data = (0b1<<4);
    data += (shift&0b1)<<3;
    data += (direction&0b1)<<2;
    LCD_TX(rs,rw,data);
}

void LCD_FUNCTION(int datalength, int fonttable)
{
    int rs = 0;
    int rw = 0;
    int data = (0b1<<5);
    data += (datalength&0b1)<<4;
    data += 0b1<<3;
    data += (fonttable&0b111);
    LCD_TX(rs,rw,data);
}

void LCD_SETCGRAM(int ADdressCG)
{
    int rs = 0;
    int rw = 0;
    int data = (0b1<<6);
    data += (ADdressCG & 0b111111);
    LCD_TX(rs,rw,data);
}

void LCD_SETDDRAM(int DDADress)
{
    int rs = 0;
    int rw = 0;
    int data = (0b1<<7);
    data += (DDADress & 0b1111111);
    LCD_TX(rs,rw,data);
}

void LCD_READSTATUS(int *busy, int *currentaddress)
{
//    int rs = 0;
//    int rw = 1;
//    int data = 0;
//    SPI_RX();
//    LCD_TX(rs,rw,data);
//    int tempa;
//    int leaders;
//    int dummy;
//    int real;
//    int tempb;
//    int tempc;
//    //delay(0xff);
//    tempa = SPI_RX();
//    //delay(0xff);
//   // LCD_TX(0,0,0);
//    //delay(0xff);
//    //tempa = SPI_RX();
//    leaders = tempa &(3<<14);
//    dummy   = tempa &(0xff<<5);
//    real = tempa &(0x3f);
//    tempb = real &(1<<5);
//    tempc = real & 0b11111;
//    tempc = tempc<<2;
//    busy = tempb;
//    currentaddress = tempc;
//    //READ LCD DATA
//    //Bit 7 = Busy flag
//    //Bits 0-6 are address
}


void LCD_WRITE(int data)
{
    int rs = 1;
    int rw = 0;
    LCD_TX(rs,rw,data);
}

void lcd_init(void)
{

  //SPI_TX(0b0000000001);//Clear Display
  LCD_CLEAR();
  delay(0xffff);
  //LCD_TX(0,0,0b00111011);  //Function Set: 8 Bit, European 2
  LCD_FUNCTION(LCD_8BIT,LCD_EUROPE2);
  delay(0xffff);
  //SPI_TX(0b0000001011);//Display off
  LCD_DISPLAY(LCD_OFF,LCD_NOCURSOR,LCD_BLINK);
  delay(0xffff);
  //SPI_TX(0b0000000001);//Clear Display
  LCD_CLEAR();
  delay(0xffff);
  //SPI_TX(0b0000000110);
  LCD_MODE(LCD_ENTRY_INCR,LCD_ENTRY_NOSHIFT);
  delay(0xffff);
  //SPI_TX(0b0000000010); // Return home
  LCD_HOME();
  delay(0xffff);
  //SPI_TX(0b0000001100);//Display on
  LCD_DISPLAY(LCD_ON,LCD_NOCURSOR,LCD_BLINK);
  delay(0xffff);
  lcd_writeline("      GUMMYBOX      ", 0);
}

void lcd_writechar(int char_write,int row,int col)
{
      int addr;
//      int realaddr;
//      int busy;
      switch(row){
          case 0: addr = 0x0; break;
          case 1: addr = 0x40;break;
          case 2: addr = 0x14;break;
          case 3: addr = 0x54;break;
          default: addr=0x0;  break;}
      addr += col;
      LCD_SETDDRAM(addr);
      g_lcd_pos = addr;
      LCD_WRITE(char_write);
      //LCD_READSTATUS(busy,realaddr);
      addr++;
}

void lcd_writeline(char * str, int row)
{
      int addr;
      switch(row){
          case 0: addr = 0x0; break;
          case 1: addr = 0x40;break;
          case 2: addr = 0x14;break;
          case 3: addr = 0x54;break;
          default: addr=0x0;  break;}
      LCD_SETDDRAM(addr);
      delay(0xfff);
      g_lcd_pos = addr;
      for(int i=0;i<strlen(str);i++)
      {
          LCD_WRITE(str[i]);
          delay(0xfff);
          //LCD_WRITE(str[i]);
          g_lcd_pos++;
          if(i==20) i = strlen(str);
      }
}


void lcd_update(int waveform,double dcoffset, int zout, double zload, double vout,double freq)
{
    char buffer[50];
    //function = 1 Sine
    //function = 2 Triangle
    //function = 3 Square
  //lcd_writeline("01234567890123456789",0);
  //lcd_writeline("VOLTWORKS SOUP. CAN.",0);
//

// if(waveform == 1) sprintf(buffer,"Wave:Sine%c DC:%+1.2lfV",0b00011101,0b00011101,dcoffset);
// if(waveform == 2) sprintf(buffer,"W:Triangle DC:%+1.2lfV",dcoffset);
// if(waveform == 3) sprintf(buffer,"W:Square…À DC:%+1.2lfV",0b00011101,0b00011101,dcoffset);
//  lcd_writeline(buffer,0);

//  if(zload < 0 &&zout <100 ) sprintf(buffer,"Zout:%dﬁ Zload:HiZ ﬁ",zout);
//  else if(zout >= 100) sprintf(buffer,"Zout:%dkﬁ Zload:HiZ ﬁ",zout/1000);
//  else sprintf(buffer,"Zout:%dﬁ Zload:%4d ﬁ",zout,zload);
//  lcd_writeline(buffer,1);
//  if(zload <0) sprintf(buffer,"Output:%2.2lfVpp HiZ   ",vout);
//  else sprintf(buffer,"Output:%2.2lfVpp Load    ",vout);
//  lcd_writeline(buffer,2);
// sprintf(buffer,"Freq (Hz): %7.2lf           ",freq);
//  lcd_writeline(buffer,3);
}

void lcd_scaled_update(int waveform, int DC_offset, int zo, double zl, int gain_control, double freq)
{
        double real_DC =0;
        if (DC_offset == 128) real_DC = 0.0;
        else if(DC_offset > 128) real_DC = (DC_offset - 128)*(-1.4246/128);
        else if(DC_offset < 128) real_DC = (DC_offset-128)*(-1.3945/128);
        double rf = 3291.0;
        double rg = 1792.0;
        double k  = 1.71;
        double vin = 2.15;
        double real_output = vin*(rf/rg)*k*(((2*((double)gain_control/256)-1)+1)/2);
        real_output = 1.98;
        //0V to 2V
        if(gain_control < 80) real_output = real_output + (0.024519)*(gain_control-80);
        //2V to 4V
        if(gain_control > 80) real_output = real_output + (0.026667)*(gain_control-80);
        //4V to 6V
        if(gain_control > 155) real_output = real_output + (0.0026315)*(gain_control-155);
        //6V to 6.5V
        if(gain_control > 224) real_output = real_output - (0.00015072)*(gain_control-155);
        //if(gain_control > 225) real_output = 6.070;
        lcd_update(waveform,real_DC,zo,zl,real_output,freq);
}

int reverse_DC(double setpoint)
{
    int returnval;
    double temp;
    if(setpoint == 0.0) returnval=128;
    if(setpoint >0)
    {
    temp = (128.0/-1.4246)* setpoint;
    returnval = (temp+0.5) + 128;
    }
    else if (setpoint <0)
    {
    temp = (128.0/-1.3945)* setpoint;
    returnval = (temp+0.5) + 128;
    }
    else returnval=128;
    return returnval;
}

int reverse_Av(double setpoint)
{
    int returnval;
    double temp;
    if(setpoint == 1.98) returnval=80;
    if(setpoint < 1.98)
    {
        temp = setpoint - 1.98 + (80.0 *0.024519);
        temp = temp/0.024519;
        returnval = (temp+0.5);
    }
    if(setpoint > 1.98)
    {
        temp = setpoint - 1.98 + (80.0 *0.026667);
        temp = temp/0.026667;
        returnval = (temp+0.5);
    }
    if(setpoint > 4.00)
    {
       //temp = setpoint - 1.98 + (80.0 *0.026667)+(155.0*0.0026315);
       //temp = temp/(0.0026315+0026315);
       returnval = (temp+0.5);
    }
    if(setpoint > 6.00)
    {
      // temp = setpoint - 1.98 + (80.0 *0.026667)+(155.0*0.0026315)-(224.0 *0.00015072);
      // temp = temp/(0.0026315+0026315+0.00015072);
       returnval = (temp+0.5);
    }
    return returnval;
}

