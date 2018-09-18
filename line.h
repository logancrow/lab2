//line.h
//function decleration for ST7735_Line
//Logan Crow and Samantha Flaim
//Date Created: 9-17-18
//Date Last Modified: 9-17-18





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
//void ST7735_Line(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t);