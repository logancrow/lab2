//fixed.c
//Function definitions for functions prototyped in fixed.h
//Logan Crow and Samantha Flaim
//Date Created: 9/4/2018
//Last Modified: 9/5/2018

#include <stdio.h>
#include <stdint.h>
#include "string.h"
#include "ST7735.h"
#include "PLL.h"
#include "fixed.h"
#include "../inc/tm4c123gh6pm.h"
#include <stdlib.h>

/****************ST7735_sDecOut2***************
 converts fixed point number to LCD
 format signed 32-bit with resolution 0.01
 range -99.99 to +99.99
 Inputs:  signed 32-bit integer part of fixed-point number
 Outputs: none
 send exactly 6 characters to the LCD 
Parameter LCD display
 12345    " **.**"
  2345    " 23.45"  
 -8100    "-81.00"
  -102    " -1.02" 
    31    "  0.31" 
-12345    "-**.**"
 */ 
void ST7735_sDecOut2(int32_t n){
	if(n > 9999){                 //if value is too large, send " **.**" to LCD
		ST7735_OutString(" **.**");
	}else{
	  if(n < -9999){                //if value is too small, send "-**.**" to LCD
		  ST7735_OutString("-**.**");
	  }else{
		  char number[7];             //char array to store string repreenting number
		  int16_t x = n/100;             //divide by 100
			int8_t y = abs(n%100);         //decimal places
			if((n < 0) && (n > -100)){    //if between -1 and 0, add " -0" to string
				number[0] = ' ';
				number[1] = '-';               
				number[2] = '0';
			}else{
			  sprintf(number,"%3d",x);         //convert int to string, 3 characters, leading spaces
			}
			number[3] = '.';                      //place decimal in string
		  sprintf(number + 4,"%02d",y);         //put decimal places in, leading 0s, 2 characters
		  ST7735_OutString(number);            //send to LCD
	  }
  }
}

/**************ST7735_uBinOut6***************
 unsigned 32-bit binary fixed-point with a resolution of 1/64. 
 The full-scale range is from 0 to 999.99. 
 If the integer part is larger than 63999, it signifies an error. 
 The ST7735_uBinOut6 function takes an unsigned 32-bit integer part 
 of the binary fixed-point number and outputs the fixed-point value on the LCD
 Inputs:  unsigned 32-bit integer part of binary fixed-point number
 Outputs: none
 send exactly 6 characters to the LCD 
Parameter LCD display
     0	  "  0.00"
     1	  "  0.01"
    16    "  0.25"
    25	  "  0.39"
   125	  "  1.95"
   128	  "  2.00"
  1250	  " 19.53"
  7500	  "117.19"
 63999	  "999.99"
 64000	  "***.**"
*/
void ST7735_uBinOut6(uint32_t n){
	if(n > 63999){                     //if too large, print "***.**"
		ST7735_OutString("***.**");
	}else{
		char number[7];                         //create char array
		uint16_t x = n/64;                      //non decimal part of the number
		uint8_t y = ((n%64)*100)/64;            //deciaml part of the number
		if((((n%64)*100)%64) >= 5) y += 1;      //round up if necessary
		sprintf(number,"%3d",x);                //print first half, 3 digits, spaces leading	
		sprintf(number + 4,"%02d",y);           //print decimals, 2 digits, 0s leading
		number[3] = '.';                        //place decimal
		ST7735_OutString(number);               //send to lcd
	}
}

int16_t save_minX, save_maxX, save_minY, save_maxY;

/**************ST7735_XYplotInit***************
 Specify the X and Y axes for an x-y scatter plot
 Draw the title and clear the plot area
 Inputs:  title  ASCII string to label the plot, null-termination
          minX   smallest X data value allowed, resolution= 0.001
          maxX   largest X data value allowed, resolution= 0.001
          minY   smallest Y data value allowed, resolution= 0.001
          maxY   largest Y data value allowed, resolution= 0.001
 Outputs: none
 assumes minX < maxX, and miny < maxY
*/
void ST7735_XYplotInit(char *title, int32_t minX, int32_t maxX, int32_t minY, int32_t maxY) {
	ST7735_FillScreen(ST7735_WHITE); //clear entire screen
	
  ST7735_FillRect(0, 0, 129, 32, ST7735_BLACK);
	ST7735_DrawString(0, 0, title, ST7735_WHITE);


	//save min/max for X/Y
	save_minX = minX;
	save_maxX = maxX;
	save_minY = minY;
	save_maxY = maxY;
}

/**************ST7735_XYplot***************
 Plot an array of (x,y) data
 Inputs:  num    number of data points in the two arrays
          bufX   array of 32-bit fixed-point data, resolution= 0.001
          bufY   array of 32-bit fixed-point data, resolution= 0.001
 Outputs: none
 assumes ST7735_XYplotInit has been previously called
 neglect any points outside the minX maxY minY maxY bounds
*/
void ST7735_XYplot(uint32_t num, int32_t bufX[], int32_t bufY[]) {
	for (int i = 0; i < num; i++) {
		if ((bufX[i] >= save_minX) && (bufX[i] <= save_maxX) && (bufY[i] >= save_minY) && (bufY[i] <= save_maxY)) {
			ST7735_DrawPixel((127 * (bufX[i] - save_minX)) / (save_maxX - save_minX), 32 + (127 * (save_maxY - bufY[i])) / (save_maxY - save_minY), ST7735_BLUE);
		}
	}
}
