#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "ST7735.h"
#include "tm4c123gh6pm.h"
#include "Pins.h"



int But1_history = 0xFF , But2_history = 0xFF, But3_history= 0xFF;

bool But1_pressed= false, But2_pressed= false, But3_pressed = false;

int But3_count = 0;


int8_t initHw()
{
    // CLOCK CONFIGURATION
    // Following procedure from p231 datasheet to transition to 16MHz clock
    // Extra commands added to ensure value in every field
    SYSCTL_RCC_R &= ~SYSCTL_RCC_MOSCDIS;                                // Enable main oscillator
    SYSCTL_RCC_R |= SYSCTL_RCC_BYPASS;                                  // Bypass PLL to change frequency
    SYSCTL_RCC_R &= ~SYSCTL_RCC_USESYSDIV;                              // Disable SYSDIV
    SYSCTL_RCC_R &= ~(SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M);         // Clear XTAL and OSCSRC fields
    SYSCTL_RCC_R |= SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN;     // Set XTAL to 16MHz and OSCSRC to main
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWRDN;                                  // Clear PWRDN
    SYSCTL_RCC_R &= ~SYSCTL_RCC_SYSDIV_M;                               // Clear SYSDIV
    SYSCTL_RCC_R |=  SYSCTL_RCC_USESYSDIV | (1 << SYSCTL_RCC_SYSDIV_S); // Use divisor of 5 (SYSDIV=4) p223  ==> 40MHz
    while (!(SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS));                       // Wait for PLL to lock
    SYSCTL_RCC_R &= ~SYSCTL_RCC_BYPASS;                                 // Use PLL clock
    SYSCTL_RCC_R &= ~SYSCTL_RCC_ACG;                                    // Disable ACG (irrelevant to this project)
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;                               // Clear PWMDIV
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;         // Set up PWM clock


    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;


    // CLOCK GATING
    // Enable GPIO ports ABCDEF peripherals
    // Enable Timers
    SYSCTL_RCGCGPIO_R = BT_0 | BT_1 | BT_2 | BT_3 | BT_4 | BT_5;

    // CONFIGURE BUTTONS
        // Pin Mapping:  PA6 PA7 PD2 PD3 PC4 PC5

        // Define inputs (clear bits)
        GPIO_PORTC_DIR_R &= ~(  BT_7 | BT_6                                               )  ;
        GPIO_PORTF_DIR_R &= ~(                  BT_2                                      )  ;

        // Enable pins for digital (set bits)
        GPIO_PORTC_DEN_R |=  (  BT_7 | BT_6                                               )  ;
        GPIO_PORTF_DEN_R |=  (                  BT_2                                      )  ;

        // Enable pull up resistors (set bits)
        GPIO_PORTC_PUR_R |=  (  BT_7 | BT_6                                               )  ;
        GPIO_PORTF_PUR_R |=  (                  BT_2                                      )  ;


    // CONFIGURE LED
        // SMALL    RED:PB1  ORANGE:PE3  GREEN:PE2  YELLOW:PE1
        // LARGE    RED:PC6  YELLOW:PC7  GREEN:PD6    BLUE:PB3
        // Configure pins as outputs with 2mA strength
        GPIO_PORTA_DIR_R  |=      BT_7                                  ;
        GPIO_PORTA_DR2R_R |=      BT_7                                  ;
        GPIO_PORTA_DEN_R  |=      BT_7                                  ;

        GPIO_PORTC_DIR_R  |=                     BT_5 | BT_4                                  ;
        GPIO_PORTC_DR2R_R |=                     BT_5 | BT_4                                  ;
        GPIO_PORTC_DEN_R  |=                     BT_5 | BT_4                                  ;

        GPIO_PORTD_DIR_R  |=       BT_7 | BT_6 |                      BT_2                    ;
        GPIO_PORTD_DR8R_R |=       BT_7 | BT_6 |                      BT_2                    ;
        GPIO_PORTD_DEN_R  |=       BT_7 | BT_6 |                      BT_2                    ;

        GPIO_PORTE_DIR_R  |=                     BT_5 | BT_4                                  ;
        GPIO_PORTE_DR2R_R |=                     BT_5 | BT_4                                  ;
        GPIO_PORTE_DEN_R  |=                     BT_5 | BT_4                                  ;


    // CONFIGURE Ethernet
        // based on ENC28J60 chip
        // CS: PB0  Clk: PB4  MISO PB6  MOSI PB7
        // Enable pins for digital (set bits)
        GPIO_PORTB_DEN_R |=  (    BT_7 | BT_6 |       BT_4   )  ;
        // Define inputs (clear bits)
        GPIO_PORTB_DIR_R &= ~(           BT_6                 )  ;
        // Set pull-up resistors
        //    GPIO_PORTB_PUR_R |=  (           BT_6                 )  ;
        GPIO_PORTB_PDR_R |=  (                         BT_4   )  ;
        // Define outputs (set bits)
        GPIO_PORTB_DIR_R |=  (    BT_7 |        BT_5 | BT_4   )  ;
        // Set output level
        GPIO_PORTB_DR2R_R |=  (   BT_7 |        BT_5 | BT_4   )  ;

        //    *BITBAND(GPIO_PORTB_DATA_R,5) = 0;

        // Set alternative functions
        GPIO_PORTB_AFSEL_R |= (   BT_7 | BT_6 |        BT_4  )  ;
        GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB6_SSI2RX | GPIO_PCTL_PB7_SSI2TX;
        // Enable clock gating and wait a little
        SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;
        __asm("     NOP");
        __asm("     NOP");
        __asm("     NOP");
        __asm("     NOP");
        __asm("     NOP");
        // Disable SSI2 and set other fields while here EOT=0   MS=0   SSE=0   LBM=0
        SSI2_CR1_R &= ~( SSI_CR1_SSE);
        SSI2_CR1_R &= ~(SSI_CR1_EOT | SSI_CR1_MS | SSI_CR1_LBM);
        // Select PLL clock
        SSI2_CC_R = SSI_CC_CS_SYSPLL;

        // Set the prescale to 2 (minimum value).  Leave SCR field of CR0 at zero.  20MHz result.  p969 Datasheet
        SSI2_CPSR_R = 2;
        // SCR=0   SPH=0   SPO=0   FRF=0   DSS=7 (8bit data) Top 16 bits are reserved and untouched
        SSI2_CR0_R &= 0xffff0000;
        SSI2_CR0_R |= 0x0007;
        SSI2_CR0_R |= 0x2 << 3;

        SSI2_CR1_R |= SSI_CR1_SSE;

    // CONFIGURE Screen
        // based on ST7735 chip
        // CS: GND  Clk: PA2  MOSI: PA5 RST: PA3 DC: PA4 BL: PA6

        // Enable pins for digital (set bits)
        GPIO_PORTA_DEN_R |=  (    BT_6 | BT_5 | BT_4 | BT_3 | BT_2  )  ;

        // Define inputs (clear bits)
        // GPIO_PORTA_DIR_R &= ~(                            )  ;

        // Set pull-up resistors
        // GPIO_PORTB_PUR_R |=  (           BT_6                 )  ;
        GPIO_PORTA_PDR_R |=  (                      BT_2      )  ;

        // Define outputs (set bits)
        GPIO_PORTA_DIR_R |=  (    BT_6 | BT_5 | BT_4 | BT_3 | BT_2  )  ;

        // Set output level
        GPIO_PORTA_DR2R_R |=  (   BT_6 | BT_5 | BT_4 | BT_3 | BT_2   )  ;

        //    *BITBAND(GPIO_PORTB_DATA_R,5) = 0;

        // Set alternative functions
        // default alternative functions
        GPIO_PORTA_AFSEL_R |= (          BT_5 |        BT_2  )  ;
        GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA2_SSI0CLK | GPIO_PCTL_PA5_SSI0TX;

        // Enable clock gating and wait a little
        SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;
        __asm("     NOP");
        __asm("     NOP");
        __asm("     NOP");
        __asm("     NOP");
        __asm("     NOP");

        *SCR_CSX= 0;
        *SCR_RESET= 1;
        Delay1ms(500);
        *SCR_RESET= 0;
        Delay1ms(500);
        *SCR_RESET= 1;
        Delay1ms(500);
        *SCR_CSX= 1;

        // Disable SSI2 and set other fields while here EOT=0   MS=0   SSE=0   LBM=0
        SSI0_CR1_R &= ~( SSI_CR1_SSE);
        // SSI0_CR1_R &= ~(SSI_CR1_EOT | SSI_CR1_MS | SSI_CR1_LBM);
        SSI0_CR1_R &= ~( SSI_CR1_MS );

        // Select PLL clock
        SSI0_CC_R &= ~SSI_CC_CS_M;
        SSI0_CC_R |= SSI_CC_CS_SYSPLL;

        // Set the prescale to 2 (minimum value).  Leave SCR field of CR0 at zero.  20MHz result.  p969 Datasheet
        SSI0_CPSR_R = 2;

        // SCR=0   SPH=0   SPO=0   FRF=0   DSS=7 (8bit data) Top 16 bits are reserved and untouched
        SSI0_CR0_R &= 0xffff0000;
        SSI0_CR0_R |= 0x0007;
        // SSI0_CR0_R |= 0x2 << 3;

        SSI0_CR1_R |= SSI_CR1_SSE;

    // CONFIGURE TIMER0 (Flashes lights at opening)
        SYSCTL_PPTIMER_R |=               BT_0             ;
        SYSCTL_RCGCTIMER_R |=             BT_0             ;
        TIMER0_CTL_R &= ~(TIMER_CTL_TAEN);
        TIMER0_CFG_R = 0;
        TIMER0_TAMR_R = 2;
        TIMER0_TAILR_R = 80000;
        TIMER0_IMR_R |=                   BT_0             ;
        TIMER0_CTL_R |= TIMER_CTL_TAEN;
        NVIC_EN0_R |= (1 << 19);
  /*
    // CONFIGURE TIMER1 (Time limit on escape sequence)
        SYSCTL_PPTIMER_R |=         BT_1                   ;
        SYSCTL_RCGCTIMER_R |=       BT_1                   ;
        TIMER1_CTL_R &= ~(TIMER_CTL_TAEN);
        TIMER1_CFG_R = 0;
        TIMER1_TAMR_R = 1;
        TIMER1_TAILR_R = 95000;                                 // set timer interval to about 1.43ms
        TIMER1_IMR_R |=  TIMER_IMR_TATOIM;
        NVIC_EN0_R |= (1 << 21);                                // Enable interrupts for A


*/

        // CONFIGUE RGB LED


        * ( BITBAND(GPIO_PORTE_DATA_R,4) ) =0;
        * ( BITBAND(GPIO_PORTC_DATA_R,4) ) =0;
        * ( BITBAND(GPIO_PORTC_DATA_R,5) ) =0;


        GPIO_PORTC_AFSEL_R |= (          BT_4 |        BT_5  )  ;
        GPIO_PORTE_AFSEL_R |= (          BT_4                )  ;
        GPIO_PORTC_PCTL_R |= (GPIO_PCTL_PC4_M0PWM6 | GPIO_PCTL_PC5_M0PWM7);
        GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE4_M0PWM4;
        SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
        __asm(" NOP");                                   // wait 3 clocks
        __asm(" NOP");
        __asm(" NOP");
        SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R0;                // reset PWM0 module
        SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R0;               // leave reset state
        while (!(SYSCTL_PRPWM_R & SYSCTL_PRPWM_R0));     // wait until ready
        PWM0_3_CTL_R = 0;// &= ~PWM_0_CTL_ENABLE;               // turn-off PWM0 generator 3
        PWM0_3_GENA_R = PWM_3_GENA_ACTCMPBD_ONE | PWM_3_GENA_ACTLOAD_ZERO;
                                                       // Turn on when number hit going down; turn off at load
        PWM0_3_LOAD_R = 4096;                            // set period to 66.67 MHz sys clock / 2 / 4096 = 8.138 kHz
                                                       // also using 4096 because load overrides cmpb
        PWM0_3_CMPA_R = 1000;                               // light off (0 is off, 4096 is always on)
        PWM0_3_CTL_R |= PWM_3_CTL_ENABLE;                // turn-on PWM0 generator 3
        PWM0_ENABLE_R = PWM_ENABLE_PWM6EN;               // enable outputs

        SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R0;                // reset PWM0 module
        SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R0;               // leave reset state
        while (!(SYSCTL_PRPWM_R & SYSCTL_PRPWM_R0));     // wait until ready
        PWM0_3_CTL_R = 0;// &= ~PWM_0_CTL_ENABLE;               // turn-off PWM0 generator 0
        PWM0_3_GENB_R = PWM_3_GENB_ACTCMPBD_ONE | PWM_3_GENB_ACTLOAD_ZERO;
                                                       // Turn on when number hit going down; turn off at load
        PWM0_3_LOAD_R = 4096;                            // set period to 66.67 MHz sys clock / 2 / 4096 = 8.138 kHz
                                                       // also using 4096 because load overrides cmpb
        PWM0_3_CMPB_R = 0;                               // light off (0 is off, 4096 is always on)
        PWM0_3_CTL_R |= PWM_3_CTL_ENABLE;                // turn-on PWM0 generator 0
        PWM0_ENABLE_R = PWM_ENABLE_PWM7EN;               // enable outputs

        SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R0;                // reset PWM0 module
        SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R0;               // leave reset state
        while (!(SYSCTL_PRPWM_R & SYSCTL_PRPWM_R0));     // wait until ready
        PWM0_2_CTL_R = 0;// &= ~PWM_0_CTL_ENABLE;               // turn-off PWM0 generator 0
        PWM0_2_GENA_R = PWM_2_GENA_ACTCMPBD_ONE | PWM_2_GENA_ACTLOAD_ZERO;
                                                         // Turn on when number hit going down; turn off at load
        PWM0_2_LOAD_R = 4096;                            // set period to 66.67 MHz sys clock / 2 / 4096 = 8.138 kHz
                                                         // also using 4096 because load overrides cmpb
        PWM0_2_CMPA_R = 0;                               // light off (0 is off, 4096 is always on)
        PWM0_2_CTL_R |= PWM_2_CTL_ENABLE;                // turn-on PWM0 generator 0
        PWM0_ENABLE_R = PWM_ENABLE_PWM4EN;               // enable outputs





        *LED_BL =1;
        ST7735_InitR(INITR_GREENTAB);


return 0;
}

int timeCount = 0 ;
uint16_t status= 0;
void timingIncrement(){
    timeCount++;

    if(timeCount%1000 == 0){
      //LED_RED ^= 1;
      //LED_BLUE ^= 1;


    }

    But1_history = (But1_history*2 + BUTTON_0) & 0xFF;
    But2_history = (But2_history*2 + BUTTON_1) & 0xFF;
    But3_history = (But3_history*2 + BUTTON_2) & 0xFF;

    if(But1_history == 0) But1_pressed=true;
    else But1_pressed= false;

    if(But2_history == 0) But2_pressed=true;
        else But2_pressed= false;

    if(But3_history == 0) But3_pressed=true;
        else But3_pressed= false;

    if (But3_pressed == true) But3_count++;
    else But3_count = 0;

    if (But3_count == 3000){
        status ^= 1;
        LED_RED ^= 1;
    }


    //clears interrupt
    TIMER0_ICR_R = BT_0;
}

/**
 * main.c
 */
int main(void)

{

    initHw();
    ST7735_SetRotation(3);
    uint16_t color_white;
    uint16_t color_black;

    //LED_RED = 0;
    //LED_BLUE = 1;

    RGB_RED_REG = 2000;

    RGB_BLU_REG = 10;

    RGB_GRN_REG = 10;



    while(1)
    {
        ST7735_SetRotation(3);
        color_white = ST7735_Color565(255,255,255);
        color_black = ST7735_Color565(0,0,0);

       // ST7735_SetCursor(10,10);
        //ST7735_OutChar('G');

       // ST7735_DrawString(1, 1, "HELLO WORLD ggg!",color_white, 1);

        ST7735_PlotLine(50);

        if (status==0){
            ST7735_DrawChar( 0,1,' ',color_black, color_white, 1);
            ST7735_DrawChar( 7,1,'C',color_black, color_white, 1);
            ST7735_DrawChar(13,1,'L',color_black, color_white, 1);
            ST7735_DrawChar(19,1,'O',color_black, color_white, 1);
            ST7735_DrawChar(25,1,'C',color_black, color_white, 1);
            ST7735_DrawChar(31,1,'K',color_black, color_white, 1);

            ST7735_DrawChar(43,1,'A',color_white, color_black, 1);
            ST7735_DrawChar(49,1,'L',color_white, color_black, 1);
            ST7735_DrawChar(55,1,'A',color_white, color_black, 1);
            ST7735_DrawChar(61,1,'R',color_white, color_black, 1);
            ST7735_DrawChar(67,1,'M',color_white, color_black, 1);




        }else {
            ST7735_DrawChar( 0,1,' ',color_white, color_black, 1);
            ST7735_DrawChar( 7,1,'C',color_white, color_black, 1);
            ST7735_DrawChar(13,1,'L',color_white, color_black, 1);
            ST7735_DrawChar(19,1,'O',color_white, color_black, 1);
            ST7735_DrawChar(25,1,'C',color_white, color_black, 1);
            ST7735_DrawChar(31,1,'K',color_white, color_black, 1);

            ST7735_DrawChar(43,1,'A',color_black, color_white, 1);
            ST7735_DrawChar(49,1,'L',color_black, color_white, 1);
            ST7735_DrawChar(55,1,'A',color_black, color_white, 1);
            ST7735_DrawChar(61,1,'R',color_black, color_white, 1);
            ST7735_DrawChar(67,1,'M',color_black, color_white, 1);
        }

        if(But1_pressed == true) LED_BLUE = 1;
        else LED_BLUE = 0;

  //      if(But2_pressed == true) LED_RED = 1;
 //       else LED_RED = 0;




    }
}
