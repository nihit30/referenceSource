

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "initHw.h"

// PF4, PA2, PA3

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

    // 1 Push button on PF4, 0x10
    // Configure port C for external push buttons
    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;
    GPIO_PORTF_DEN_R |= 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= 0x10;  // enable internal pull-up for push button
    GPIOIntTypeSet(0x40025000, GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntEnable (0x40025000, GPIO_PIN_4);
    IntEnable(INT_GPIOF);

    // 2 push buttons on PA2 and PA3, 0x0C
    // Configure port D for ext push button
    GPIO_PORTA_DEN_R = 0x0C;  // enable data on pc6 and pc7
    GPIO_PORTA_PUR_R = 0x0C;  // Enable internal pull-ups
    GPIOIntTypeSet(0x40004000 , GPIO_PIN_2 | GPIO_PIN_3, GPIO_FALLING_EDGE);
    GPIOIntEnable(0x40004000, GPIO_PIN_2 | GPIO_PIN_3);
    IntEnable(INT_GPIOA);

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

    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    GPIO_PORTB_DEN_R |= 0xC0;
    GPIO_PORTB_AFSEL_R |= 0xC0;
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB6_M0PWM0;

    PWM0_0_CTL_R = 0;
    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_M;
    PWM0_0_LOAD_R = 0x674;
    PWM0_0_CMPA_R = 0x33A;
    PWM0_0_CTL_R |= 1;
    PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;


}
