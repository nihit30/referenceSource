

#ifndef INC_LCDDRIVER_H_
#define INC_LCDDRIVER_H_

#include <stdint.h>



#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))  //PE2
#define CS_NOT       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4))) //PB1

#define RED_BL_LED   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4))) // PB5
#define GREEN_BL_LED (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4))) // PE5
#define BLUE_BL_LED  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // PE4

// Set pixel arguments
#define CLEAR  0
#define SET    1
#define INVERT 2




void sendGraphicsLcdCommand(uint8_t command);
void sendGraphicsLcdData(uint8_t data);
void setGraphicsLcdPage(uint8_t page);
void setGraphicsLcdColumn(uint8_t x);
void refreshGraphicsLcd();
void clearGraphicsLcd();
void initGraphicsLcd();
void drawGraphicsLcdPixel(uint8_t x, uint8_t y, uint8_t op);
void drawGraphicsLcdRectangle(uint8_t xul, uint8_t yul, uint8_t dx, uint8_t dy, uint8_t op);
void setGraphicsLcdTextPosition(uint8_t x, uint8_t page);
void putcGraphicsLcd(char c);
void putsGraphicsLcd(char str[]);


#endif /* INC_LCDDRIVER_H_ */
