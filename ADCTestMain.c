// ADCTestMain.c
// Runs on TM4C123
// This program periodically samples ADC channel 0 and stores the
// result to a global variable that can be accessed with the JTAG
// debugger and viewed with the variable watch feature.
// Daniel Valvano
// September 5, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// center of X-ohm potentiometer connected to PE3/AIN0
// bottom of X-ohm potentiometer connected to ground
// top of X-ohm potentiometer connected to +3.3V 
#include <stdint.h>
#include "ADCSWTrigger.h"
#include "../inc/tm4c123gh6pm.h"
#include "PLL.h"

#define PF2             (*((volatile uint32_t *)0x40025010))
#define PF1             (*((volatile uint32_t *)0x40025008))
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
 
volatile uint32_t ADCvalue;
// This debug function initializes Timer0A to request interrupts
// at a 100 Hz frequency.  It is similar to FreqMeasure.c.
/*
//function that calculates jitter for 1000 unit array of timestamps
int main(void){
  //variables to store minimum and maximum differences
  uint32_t maxdif = 0x00;
  uint32_t mindif = 0xFF;
  for(int i = 1;i < 1000;i++){           //for every element in the array
    //if difference is greater than max or less than min, update them
    if((time[i] - time[i - 1]) > maxdif)  
      maxdif = time[i] - time[i - 1];
    if((time[i] - time[i - 1]) < mindif) 
      mindif = time[i] - time[i - 1];   
  }
  uint32_t jitter = maxdif - mindif;     //definition of jitter
}
*/
void Timer0A_Init100HzInt(void){
  volatile uint32_t delay;
  DisableInterrupts();
  // **** general initialization ****
  SYSCTL_RCGCTIMER_R |= 0x01;      // activate timer0
  delay = SYSCTL_RCGCTIMER_R;      // allow time to finish activating
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
  TIMER0_CFG_R = 0;                // configure for 32-bit timer mode
  // **** timer0A initialization ****
                                   // configure for periodic mode
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER0_TAILR_R = 799999;         // start value for 100 Hz interrupts
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;// enable timeout (rollover) interrupt
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// clear timer0A timeout flag
  TIMER0_CTL_R |= TIMER_CTL_TAEN;  // enable timer0A 32-b, periodic, interrupts
  // **** interrupt initialization ****
                                   // Timer0A=priority 2
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_EN0_R = 1<<19;              // enable interrupt 19 in NVIC
}

//arrays to store data at interrupts
uint32_t time[1000];
uint32_t data[1000];
uint16_t count = 0;

void Timer0A_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;    // acknowledge timer0A timeout
  PF2 ^= 0x04;                   // profile
  PF2 ^= 0x04;                   // profile
  ADCvalue = ADC0_InSeq3();
	if(count < 1000){       //if arrays not full
		time[count] = 0xFFFFFFFF - TIMER1_TAR_R;  //record time and data
		data[count] = ADCvalue;
		count++;         
	}
  PF2 ^= 0x04;                   // profile
}
int main(void){
  PLL_Init(Bus80MHz);                   // 80 MHz
  SYSCTL_RCGCGPIO_R |= 0x20;            // activate port F
  ADC0_InitSWTriggerSeq3_Ch9();         // allow time to finish activating
  Timer0A_Init100HzInt();               // set up Timer0A for 100 Hz interrupts
  GPIO_PORTF_DIR_R |= 0x06;             // make PF2, PF1 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x06;          // disable alt funct on PF2, PF1
  GPIO_PORTF_DEN_R |= 0x06;             // enable digital I/O on PF2, PF1
                                        // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF00F)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
  PF2 = 0;                      // turn off LED
  EnableInterrupts();
  while(1){
    PF1 ^= 0x02;  // toggles when running in main
  }
}
void process_data(void){
  //sort the array of 1000 inputs from potentiometer
  uint16_t x, j, temp;
  for (int i = 1; i < 1000; i++){
    for (j = i; (j > 0 && data[j-1] > data[j]); j--){
      temp = data[j];
      data[j] = data[j-1];
      data[j-1] = temp;
    }
  }

  //find how many unique numbers in the 1000 inputs
  uint32_t n = 0; //number of unique numbers in the 1000
  for (int i = 1; i < 1000; i++) {
    if (data[i] < data[i - 1]) {
      n++;
    }
  }
  //create arrays to store the number of occurances for each
    //unique number and the number itself
  uint32_t data_count[n];
  uint32_t data_num[n];
  uint32_t num_count; 
  uint32_t ind = 0;
  uint32_t k = 0;
  while ((k < 1000) && (ind < n)) {
    data_num[ind] = data[k];
    num_count = 0;
    while (data[k] == data[k+1]) {
      num_count++;
      k++;
    }
    num_count++;
    k++;
    data_count[ind] = num_count;
    ind++;
  }

  //print graph to screen
  double res = 127 / (data_num[ind - 1] - data_num[0]);
  for (int l = ind - 1; l >= 0; l--){
    ST7735_Line((int)(data_num[l] * res), (int)(data_num[l] * res), 159, (data_count[l] / 6.5), ST7735_BLACK);
  }
}
//************* ST7735_Line********************************************
//  Draws one line on the ST7735 color LCD
//  Inputs: (x1,y1) is the start point
//          (x2,y2) is the end point
// x1,x2 are horizontal positions, columns from the left edge
//               must be less than 128
//               0 is on the left, 126 is near the right
// y1,y2 are vertical positions, rows from the top edge
//               must be less than 160
//               159 is near the wires, 0 is the side opposite the wires
//        color 16-bit color, which can be produced by ST7735_Color565() 
// Output: none
void ST7735_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color){
  int32_t A, B, P;
  int16_t current_y;

  //make sure pixels are within area of screen
  if ((x1 < 128) && (y1 < 160)){
    //horizontal line
    if (x1 == x2){
      if (y1 < y2){
        for (int y = y1; y <= y2; y++){
          ST7735_DrawPixel(x1, y, color);
        }
      }
      else if (y1 > y2){
        for (int y = y2; y >= y1; y--){
          ST7735_DrawPixel(x1, y, color);
        }
      }
    }
    //vertical line
    else if (y1 == y2){
      if (x1 < x2){
        for (int x = x1; x <= x2; x++){
          ST7735_DrawPixel(x, y1, color);
        }
      }
      else if (x1 > x2){
        for (int x = x2; x >= x1; x--){
          ST7735_DrawPixel(x, y1, color);
        }
      }
    }
    //diagonal line
    else {
      ST7735_DrawPixel(x1, y1, color);
      current_y = y1;
      for (int x = x1 + 1; x < x2; x++) {
        A = 2 * ((int16_t)y2 - current_y);
        B = A - (2 * ((int16_t)x2 - x);
        P = A - ((int16_t)x2 - x);
        if (P < 0) {
          P += A;
        }
        else {
          current_y++;
          P += B;
        }
        ST7735_DrawPixel(x, current_y, color);
      }
      ST7735_DrawPixel(x2, y2, color);
    }



    //diagonal line with positive slope (goes down and to the right)
    /*else if ((x1 < x2) && (y1 < y2) && ((x2 - x1) > (y2 - y1))) {
      for (int x = x1; x <= x2; x++) {
        slope = (y2 - y1)/(x2 - x1);
        while (current_x <= x2) {
          current_y = slope * (x - x1) + y1;
          ST7735_DrawPixel(x, current_y, color);
        }
      }
    }
    //diagonal line with positive slope (goes up and to the left)
    else if ((x2 < x1) && (y2 < y1) && (x1 - x2) > (y1 - y2))) {
      for (int x = x2; x <= x1; x++) {
        slope = (y1 - y2)/(x1 - x2);
        while (current_x <= x1) {
          current_y = slope * (x - x2) + y2;
          ST7735_DrawPixel(x, current_y, color);
        }
      }
    }*/

  }
}

