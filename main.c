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
#include "tm4c123gh6pm.h"
#include "lcdDriver.h"
#include "initHw.h"
#include <strings.h>

// Bit banding peripherals
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED

// Push button initialization
#define UP_PB      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) // port C, pin6
#define LEFT_PB    (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4))) // port C, pin7
#define RIGHT_PB   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4))) // port D, pin6


#define X1_COOR 0
#define Y1_COOR 0

#define X2_COOR 496
#define Y2_COOR 0

#define X3_COOR 496
#define Y3_COOR 240


uint32_t sum;
uint8_t data[12];
uint8_t packet[20];
uint16_t chksum;
uint8_t dataLength = 11;
float axis[3]={25.1,2.2,3.3};
float *locate,value;

uint16_t dec_point[2];


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

           // BLUE_LED ^= 1;
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
    BLUE_LED = 1;
    waitMicrosecond(5000);
    BLUE_LED = 0;
}

bool buttonPress()
{
    return (UP_PB | RIGHT_PB | LEFT_PB);
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
         if (UP_PB == 0)
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
    BLUE_LED = 1;

    GPIO_PORTA_ICR_R = 0x0C;

}

void portFisr()
{

        BLUE_LED = 0;

        GPIO_PORTF_ICR_R = 0x10;
}



int main(void)
{

    initHw();
    blinkLED();


    initGraphicsLcd();
    createPacket(&data, dataLength);




    // Draw X in left half of screen
   // uint8_t i;
    //for (i = 0; i < 64; i++)
    //    drawGraphicsLcdPixel(i, i, SET);
   // for (i = 0; i < 64; i++)
   //     drawGraphicsLcdPixel(63 - i, i, INVERT);

  // Draw text on screen
  //  setGraphicsLcdTextPosition(84, 5);
  //  putsGraphicsLcd("Text");

  //  TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts

    while (1)
    {
        //drawGraphicsLcdRectangle(83, 39, 25, 9, INVERT);


           // UP_PB = 1;
            //LEFT_PB = 1;
            //RIGHT_PB = 1;
           // digitset();





    }
}
