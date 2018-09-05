// Reference source is responsible to transmit 'synchronization signal' to beacons and mobile receiver using TXM 418 LR RF modules.
// TXM 418 LR modules are connected to the controller on UART serial interface.
// Packet is transmitted every 1sec in while(1) loop
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
#include "inc/lcdDriver.h"
#include <strings.h>

// Bit banding peripherals
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED


#define X1_COOR 0
#define X2_COOR 496
#define X3_COOR 496
#define Y1_COOR 0
#define Y2_COOR 0
#define Y3_COOR 240



uint32_t sum;
uint8_t data[12];
uint8_t packet[20];
uint16_t chksum;
uint8_t dataLength = 11;





void initHw()
{
    // SYSTEM SECTION
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Note UART on port C must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A to F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOD
            | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOB
            | SYSCTL_RCGC2_GPIOC;

    // Configure port F
    GPIO_PORTF_DIR_R = 0x04;  // bits 2 is outputs for blue led
    GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x04;  // enable bit 2

    // LCD SECTION
    // Configure three backlight LEDs for LCD
    GPIO_PORTB_DIR_R |= 0x20;  // make bit5 an output
    GPIO_PORTB_DR2R_R |= 0x20; // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= 0x20;  // enable bit5 for digital
    GPIO_PORTE_DIR_R |= 0x30;  // make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R |= 0x30; // set drive strength to 2mA
    GPIO_PORTE_DEN_R |= 0x30;  // enable bits 4 and 5 for digital

    // Configure A0 and ~CS for graphics LCD
    GPIO_PORTB_DIR_R |= 0x42;  // make bits 1 and 6 outputs
    GPIO_PORTB_DR2R_R |= 0x42; // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= 0x42;  // enable bits 1 and 6 for digital

    // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0x90;                       // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0x90;                      // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= 0x90; // select alternative functions for MOSI, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0x90;        // enable digital operation on TX, CLK pins
    GPIO_PORTB_PUR_R |= 0x10;                      // must be enabled when SPO=1

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;       // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                    // select system clock as the clock source
    SSI2_CPSR_R = 40;                  // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2

    // RF SECTION
    // Configure UART1 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1; // turn-on UART0, leave other uarts in same status
    GPIO_PORTC_DEN_R |= 0x30;                      // default, added for clarity
    GPIO_PORTC_AFSEL_R |= 0x30;                    // default, added for clarity
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;
    /* GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
     SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
     GPIO_PORTA_DEN_R |= 0x03;                         // default, added for clarity
     GPIO_PORTA_AFSEL_R |= 0x03;*/

    // UART1 initialization to transmit RF data
    UART1_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART1_IBRD_R = 2083; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16    //1200 BUADRATE
    UART1_FBRD_R = 21;                               // round(fract(r)*64)=45
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // TIMER SECTION
    // Configure Timer 1 as the time base to transmit packets in specified time period
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x04C4B400;
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);     // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer


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

void putPacket()
{
    uint8_t lenInd = 14;
            uint8_t z;
    for(z = 0; z <= lenInd; z++)
    {
        puthUart1(packet[z]);
    }
}

void timer1ISR()
{

            BLUE_LED ^= 1;
            trainingPackets(4);
            puthUart1(0x41);
            puthUart1(0x42);
            puthUart1(0x43);
            puthUart1(0x44);
            puthUart1(0xC0);
            putPacket();
            TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}


int main(void)
{

    initHw();
    BLUE_LED = 1;
    waitMicrosecond(5000);
    BLUE_LED = 0;
    createPacket(&data, dataLength);
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts

    while(1)
    {


    }
}
