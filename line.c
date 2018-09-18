//line.c
//function definition for ST7735_Line
//Logan Crow and Samantha Flaim
//Date Created: 9-17-18
//Date Last Modified: 9-17-18
/*
#include "line.h"
#include "ST7735.h"
#include "PLL.h"
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
        B = A - (2 * ((int16_t)x2 - x));
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
      else if ((x1 < x2) && (y1 < y2) && ((x2 - x1) > (y2 - y1))) {
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
    }

  }
}
*/