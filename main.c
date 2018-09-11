// Reference source is responsible to transmit 'synchronization signal' to beacons and mobile receiver using RF modules.
// RF modules are connected to the controller on UART serial interface.
// Packet is transmitted every 1sec using a timer interrupt
// Information in packet, byte-wise, starting from 0 is dataLengthgth of packet, 3 beacon's x & y co-ordinates, NULL character, checksum and null again
// TO DO : Add Z co-ordinate
// IMPORTANT : LCD + PUSH BUTTONS
// TO DO : Re-structure packet format
// TO DO : Add ability to trigger transmission (reference signals) on user request only
// TO DO : Add bluetooth for seamless cellphone integration


// FUTURE GOALS:
// Use cheap 8051 for all beacons and off-load computation to reference source to save cost and increase efficiency

#include <stdint.h>
#include <stdbool.h>
#include <strings.h>
#include "tm4c123gh6pm.h"
#include "initHw.h"
#include "driverlib/gpio.h"
//#include "lcdDriver.h"


#define RED_BL_LED   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))
#define GREEN_BL_LED (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define BLUE_BL_LED  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))

#define CS_NOT       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))

// Set pixel arguments
#define CLEAR  0
#define SET    1
#define INVERT 2

// Bit banding peripherals
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

#define PORTF_BASE_ADDR 0x400253FC
#define PORTA_BASE_ADDR 0x400043FC

// Push button initialization
#define RIGHT_PB      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // PA2
#define LEFT_PB    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // PA3
#define ENTER_PB   (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4))) // PF4

#define X1_COOR 0
#define Y1_COOR 0
#define X2_COOR 496
#define Y2_COOR 0
#define X3_COOR 496
#define Y3_COOR 240

#define LEFT_PB_PRESS 0x11
#define RIGHT_PB_PRESS 0x22
#define ENTER_PB_PRESS 0x33


static uint8_t leftPbCount = 1;
static uint8_t rightPbCount = 0;
static uint8_t enterPbCount = 0;

bool leftPbPressed = false;
bool rightPbPressed = false;
bool enterPbPressed = false;
uint8_t pbPressedValue = 0x00;

uint32_t sum;
uint8_t data[12];
uint8_t packet[20];
uint16_t chksum;
uint8_t dataLength = 11;
float axis[3]={25.1,2.2,3.3};
float *locate,value;

uint16_t dec_point[2];

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

uint8_t  pixelMap[1024];
uint16_t txtIndex = 0;

// 96 character 5x7 bitmaps based on ISO-646 (BCT IRV extensions)
const uint8_t charGen[100][5] = {
    // Codes 32-127
    // Space ! " % $ % & ' ( ) * + , - . /
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x4F, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00},
    {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x40},
    {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00},
    {0x00, 0x41, 0x22, 0x1C, 0x00},
    {0x14, 0x08, 0x3E, 0x08, 0x14},
    {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00},
    {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02},
    // 0-9
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},
    // : ; < = > ? @
    {0x00, 0x36, 0x36, 0x00, 0x00},
    {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x08, 0x14, 0x22, 0x41, 0x00},
    {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08},
    {0x02, 0x01, 0x51, 0x09, 0x3E},
    {0x32, 0x49, 0x79, 0x41, 0x3E},
    // A-Z
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x3A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x70, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43},
    // [ \ ] ^ _ `
    {0x00, 0x7F, 0x41, 0x41, 0x00},
    {0x02, 0x04, 0x08, 0x10, 0x20},
    {0x00, 0x41, 0x41, 0x7F, 0x00},
    {0x04, 0x02, 0x01, 0x02, 0x04},
    {0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x01, 0x02, 0x04, 0x00},
    // a-z
    {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x7F, 0x44, 0x44, 0x44, 0x38},
    {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7F},
    {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x08, 0x7E, 0x09, 0x01, 0x02},
    {0x0C, 0x52, 0x52, 0x52, 0x3E},
    {0x7F, 0x08, 0x04, 0x04, 0x78},
    {0x00, 0x44, 0x7D, 0x40, 0x00},
    {0x20, 0x40, 0x44, 0x3D, 0x00},
    {0x7F, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00},
    {0x7C, 0x04, 0x18, 0x04, 0x78},
    {0x7C, 0x08, 0x04, 0x04, 0x78},
    {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7C, 0x14, 0x14, 0x14, 0x08},
    {0x08, 0x14, 0x14, 0x18, 0x7C},
    {0x7C, 0x08, 0x04, 0x04, 0x08},
    {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3F, 0x44, 0x40, 0x20},
    {0x3C, 0x40, 0x40, 0x20, 0x7C},
    {0x1C, 0x20, 0x40, 0x20, 0x1C},
    {0x3C, 0x40, 0x20, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44},
    {0x0C, 0x50, 0x50, 0x50, 0x3C},
    {0x44, 0x64, 0x54, 0x4C, 0x44},
    // { | } ~ cc
    {0x00, 0x08, 0x36, 0x41, 0x00},
    {0x00, 0x00, 0x7F, 0x00, 0x00},
    {0x00, 0x41, 0x36, 0x08, 0x00},
    {0x0C, 0x04, 0x1C, 0x10, 0x18},
    {0x00, 0x00, 0x00, 0x00, 0x00},
    // Custom assignments beyond ISO646
    // Codes 128+: right arrow, left arrow, degree sign
    {0x08, 0x08, 0x2A, 0x1C, 0x08},
    {0x08, 0x1C, 0x2A, 0x08, 0x08},
    {0x07, 0x05, 0x07, 0x00, 0x00},
};

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Blocking function that writes data to the SPI bus and waits for the data to complete transmission
void sendGraphicsLcdCommand(uint8_t command)
{
    CS_NOT = 0;                        // assert chip select
    __asm (" NOP");                    // allow line to settle
    __asm (" NOP");
    __asm (" NOP");
    __asm (" NOP");
    A0 = 0;                            // clear A0 for commands
    SSI2_DR_R = command;               // write command
    while (SSI2_SR_R & SSI_SR_BSY);
    CS_NOT = 1;                        // de-assert chip select
}

// Blocking function that writes data to the SPI bus and waits for the data to complete transmission
void sendGraphicsLcdData(uint8_t data)
{
    CS_NOT = 0;                        // assert chip select
    __asm (" NOP");                    // allow line to settle
    __asm (" NOP");
    __asm (" NOP");
    __asm (" NOP");
    A0 = 1;                            // set A0 for data
    SSI2_DR_R = data;                  // write data
    while (SSI2_SR_R & SSI_SR_BSY);    // wait for transmission to stop
    CS_NOT = 1;                        // de-assert chip select
}

void setGraphicsLcdPage(uint8_t page)
{
  sendGraphicsLcdCommand(0xB0 | page);
}

void setGraphicsLcdColumn(uint8_t x)
{
  sendGraphicsLcdCommand(0x10 | (x >> 4));
  sendGraphicsLcdCommand(0x00 | (x & 0x0F));
}

void refreshGraphicsLcd()
{
    uint8_t x, page;
    uint16_t i = 0;
    for (page = 0; page < 8; page ++)
    {
        setGraphicsLcdPage(page);
        setGraphicsLcdColumn(0);
        for (x = 0; x < 128; x++)
            sendGraphicsLcdData(pixelMap[i++]);
    }
}

void clearGraphicsLcd()
{
    uint16_t i;
    // clear data memory pixel map
    for (i = 0; i < 1024; i++)
        pixelMap[i] = 0;
    // copy to display
    refreshGraphicsLcd();
}

void initGraphicsLcd()
{
    sendGraphicsLcdCommand(0x40); // set start line to 0
    sendGraphicsLcdCommand(0xA1); // reverse horizontal order
    sendGraphicsLcdCommand(0xC0); // normal vertical order
    sendGraphicsLcdCommand(0xA6); // normal pixel polarity
    sendGraphicsLcdCommand(0xA3); // set led bias to 1/9 (should be A2)
    sendGraphicsLcdCommand(0x2F); // turn on voltage booster and regulator
    sendGraphicsLcdCommand(0xF8); // set internal volt booster to 4x Vdd
    sendGraphicsLcdCommand(0x00);
    sendGraphicsLcdCommand(0x27); // set contrast
    sendGraphicsLcdCommand(0x81); // set LCD drive voltage
    sendGraphicsLcdCommand(0x04);
    sendGraphicsLcdCommand(0xAC); // no flashing indicator
    sendGraphicsLcdCommand(0x00);
    clearGraphicsLcd();           // clear display
    sendGraphicsLcdCommand(0xAF); // display on
}

void drawGraphicsLcdPixel(uint8_t x, uint8_t y, uint8_t op)
{
    uint8_t data, mask, page;
    uint16_t index;

    // determine pixel map entry
    page = y >> 3;

    // determine pixel map index
    index = page << 7 | x;

    // generate mask
    mask = 1 << (y & 7);

    // read pixel map
    data = pixelMap[index];

    // apply operator (0 = clear, 1 = set, 2 = xor)
    switch(op)
    {
        case 0: data &= ~mask; break;
        case 1: data |= mask; break;
        case 2: data ^= mask; break;
    }

    // write to pixel map
    pixelMap[index] = data;

    // write to display
    setGraphicsLcdPage(page);
    setGraphicsLcdColumn(x);
    sendGraphicsLcdData(data);
}

void drawGraphicsLcdRectangle(uint8_t xul, uint8_t yul, uint8_t dx, uint8_t dy, uint8_t op)
{
    uint8_t page, page_start, page_stop;
    uint8_t bit_index, bit_start, bit_stop;
    uint8_t mask, data;
    uint16_t index;
    uint8_t x;

    // determine pages for rectangle
    page_start = yul >> 3;
    page_stop = (yul + dy - 1) >> 3;

    // draw in pages from top to bottom within extent
    for (page = page_start; page <= page_stop; page++)
    {
        // calculate mask for this page
        if (page > page_start)
            bit_start = 0;
        else
            bit_start = yul & 7;
        if (page < page_stop)
            bit_stop = 7;
        else
            bit_stop = (yul + dy - 1) & 7;
        mask = 0;
        for (bit_index = bit_start; bit_index <= bit_stop; bit_index++)
            mask |= 1 << bit_index;

        // write page
        setGraphicsLcdPage(page);
        setGraphicsLcdColumn(xul);
        index = (page << 7) | xul;
        for (x = 0; x < dx; x++)
        {
            // read pixel map
            data = pixelMap[index];
            // apply operator (0 = clear, 1 = set, 2 = xor)
            switch(op)
            {
                case 0: data &= ~mask; break;
                case 1: data |= mask; break;
                case 2: data ^= mask; break;
            }
            // write to pixel map
            pixelMap[index++] = data;
            // write to display
            sendGraphicsLcdData(data);
        }
    }
}

void setGraphicsLcdTextPosition(uint8_t x, uint8_t page)
{
    txtIndex = (page << 7) + x;
    setGraphicsLcdPage(page);
    setGraphicsLcdColumn(x);
}

void putcGraphicsLcd(char c)
{
    uint8_t i, val;
    uint8_t uc;
    // convert to unsigned to access characters > 127
    uc = (uint8_t) c;
    for (i = 0; i < 5; i++)
    {
        val = charGen[uc-' '][i];
        pixelMap[txtIndex++] = val;
        sendGraphicsLcdData(val);
    }
    pixelMap[txtIndex++] = 0;
    sendGraphicsLcdData(0);
}

void putsGraphicsLcd(char str[])
{
    uint8_t i = 0;
    while (str[i] != 0)
        putcGraphicsLcd(str[i++]);
}


void waitMicrosecond(uint32_t us)
{
                                                // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}


void putcUart1(char c)
{
    while (UART1_FR_R & UART_FR_TXFF);
    UART1_DR_R = c;
}

void puthUart1(uint8_t hex)
{
    while (UART1_FR_R & UART_FR_TXFF);
    UART1_DR_R = hex;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart1(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
    putcUart1(str[i]);
}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
    putcUart0(str[i]);
}

void initialize_data()
{
    data[0] = ((X1_COOR  & 0xFF00) >> 8) + 1;
    data[1] = (X1_COOR & 0x00FF) + 1;

    data[2] = ((Y1_COOR & 0xFF00) >> 8) + 1;
    data[3] = (Y1_COOR & 0x00FF) + 1;

    data[4] = ((X2_COOR & 0xFF00) >> 8) + 1;
    data[5] = (X2_COOR & 0x00FF) + 1;

    data[6] = ((Y2_COOR & 0xFF00) >> 8) + 1;
    data[7] = (Y2_COOR & 0x00FF) + 1;

    data[8] = ((X3_COOR & 0xFF00) >> 8) + 1;
    data[9] = (X3_COOR & 0x00FF) + 1;

    data[10] = ((Y3_COOR & 0xFF00) >> 8) + 1;
    data[11] = (Y3_COOR & 0x00FF) + 1;

}




void SumWords(void* data, uint16_t size_in_bytes)
{
    uint8_t* pData = (uint8_t*)data;
    uint16_t i;
    uint8_t phase = 1;
    uint16_t data_temp;
    for (i = 0; i < size_in_bytes; i++)
    {
        if (phase)
        {
            data_temp = *pData;
            sum += data_temp << 8;
        }
        else
          sum += *pData;
        phase = 1 - phase;
        pData++;
    }
}
// Completes 1's compliment addition by folding carries back uint8_to field
uint16_t getChecksum()
{
    uint16_t result;
    // this is based on rfc1071
    while ((sum >> 16) > 0)
      sum = (sum & 0xFFFF) + (sum >> 16);
    result = sum & 0xFFFF;
    return ~result;
}

void createPacket(void* Pdata, uint16_t size_in_bytes)
{
    uint8_t i = 0;
    uint8_t j = 0;
    sum = 0;
    initialize_data();
    uint8_t* pData = (uint8_t*)Pdata;

    SumWords(&data, dataLength);
    chksum = getChecksum();

    packet[i++] = dataLength + 2;           // first byte represents size of whole packet


    while (j <= dataLength)
    {
        packet[i++] = pData[j++];

    }
    packet[i++] = ((chksum & 0xFF00) >> 8);
    packet[i++] = (chksum & 0x00FF);
    packet[i] = '\0';
}



void trainingPackets(uint16_t numberOfTrainSequence)
{
    uint8_t l;

    for(l = 0; l < numberOfTrainSequence; l++)
    {
        puthUart1(0x00);
        puthUart1(0xFF);
    }

}

void debounceISR()
{


}

void putPacket()
{
    uint8_t lenInd = 14;
            uint8_t z;
    for(z = 0; z <= lenInd; z++)
    {
        puthUart1(packet[z]);
    }
}

void transmitPacketISR()
{

            //BLUE_LED ^= 1;
            trainingPackets(4);
            puthUart1(0x41);
            puthUart1(0x42);
            puthUart1(0x43);
            puthUart1(0x44);
            puthUart1(0xC0);
            putPacket();
            TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

void blinkLED()
{
    RED_LED = 1;
    GREEN_LED = 1;
    BLUE_LED = 1;
    waitMicrosecond(500000);
    BLUE_LED = 0;
    GREEN_LED = 0;
    RED_LED = 0;
}

bool buttonPress()
{
   // return (UP_PB | RIGHT_PB | LEFT_PB);
}

void digitset()
{
    uint16_t dec_point[3], temp, *curser_ptr,curser_ptr_value=0;
    float *locate_float, fvalue, value2;

    curser_ptr = &dec_point[0];

    locate_float = &axis[0];


        fvalue = *locate_float;
        temp = fvalue;
          if (temp / 10 >= 1)
        {

          value2 = (fvalue - temp) * 10;
          dec_point[0] = temp / 10;  // in 32.1 contain 3
          dec_point[1] = temp % 10;  // in 32.1 contain 2
          dec_point[2] = value2;     // in 32.1 contain 1
        }
          else
          {
              value2 = (fvalue - temp) * 10;
              dec_point[0] = 0 ;
              dec_point[1] = temp;
              dec_point[2] = value2;
              curser_ptr++;
          }


        while(!(RIGHT_PB == 0 && LEFT_PB ==0 ))
        {

         if (RIGHT_PB == 0 && curser_ptr_value<=2)
          {
            curser_ptr_value++;
            curser_ptr++;
          }
         if (LEFT_PB == 0 && curser_ptr_value>0)
         {
            curser_ptr_value--;
            curser_ptr--;
         }
         //if (UP_PB == 0)
         {
           *curser_ptr = *curser_ptr+1;
           if(*curser_ptr >= 10)
                  *curser_ptr=0;
         }


         BLUE_LED=1;

        }
          axis[0]=dec_point[0]*10+dec_point[1]+(0.1*dec_point[2]);



}

void portAisr()
{

    switch(GPIO_PORTA_RIS_R)
    {
    case 0x00000004 :
    {
        leftPbPressed = true;
        pbPressedValue = LEFT_PB_PRESS;          // Left
        RED_LED ^= 1;
        break;
    }

    case 0x00000008 :
    {

        rightPbPressed = true;
        pbPressedValue = RIGHT_PB_PRESS;          // Right
        GREEN_LED ^= 1;
        break;
    }

    default :
    {
        leftPbPressed = false;
        rightPbPressed = false;
        enterPbPressed = false;
        pbPressedValue = 0x00;
        break;
    }

    }

    GPIO_PORTA_ICR_R = 0x0C; // 0x0C
   // GPIOIntDisable(PORTA_BASE_ADDR, GPIO_INT_PIN_2 | GPIO_INT_PIN_3);
}

void portFisr()
{

    if(!ENTER_PB)
    {
        enterPbPressed = true;
        pbPressedValue = RIGHT_PB_PRESS;
        BLUE_LED ^= 1;
        GPIOIntDisable(PORTF_BASE_ADDR, GPIO_INT_PIN_4);
    }

        GPIO_PORTF_ICR_R = 0x10; // 0x10

}

bool pbHit()
{
    return (leftPbPressed | rightPbPressed | enterPbPressed);
}




int main(void)
{


    initHw();
    initGraphicsLcd();
    blinkLED();
    createPacket(&data, dataLength);

    // Draw text on screen
       setGraphicsLcdTextPosition(3, 3);           // (col, page)
       putsGraphicsLcd("Text");


    while (1)
    {

        if(pbHit())
        {
            leftPbPressed = false;
            rightPbPressed = false;
            enterPbPressed = false;

            switch(pbPressedValue)
            {
                case LEFT_PB_PRESS :
                {

                    RED_LED ^= 1;
                    break;
                }
                case RIGHT_PB_PRESS :
                {
                    GREEN_LED ^= 1;
                    break;
                }
                case ENTER_PB_PRESS :
                {
                    BLUE_LED ^= 1;
                    break;
                }
                default :
                {
                    break;
                }

            }
        }


    }
}
