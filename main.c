
///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Bitband and Pin Definitions
//
///////////////////////////////////////////////////////////////////////////////////////////////////


#define BITBAND(BYTE_ADDR,BIT) (((volatile uint32_t *)(   (((uint32_t)&(BYTE_ADDR))&(0xF0000000)) + 0x02000000 + (((uint32_t)&(BYTE_ADDR)) << 5) + ((BIT) << 2)    )))

#define  LED_RED      BITBAND(GPIO_PORTD_DATA_R,2)
#define  LED_YELLOW   BITBAND(GPIO_PORTE_DATD_R,7)
#define  LED_BLUE     BITBAND(GPIO_PORTD_DATA_R,6)

#define RED_LED      *LED_RED
#define YELLOW_LED   *LED_YELLOW
#define BLUE_LED     *LED_BLUE

#define  BUTTON_0        BITBAND(GPIO_PORTC_DATA_R,6)
#define  BUTTON_1        BITBAND(GPIO_PORTC_DATA_R,7)
#define  BUTTON_2        BITBAND(GPIO_PORTD_DATA_R,2)

#define BT_0 (0x01)
#define BT_1 (0x02)
#define BT_2 (0x04)
#define BT_3 (0x08)
#define BT_4 (0x10)
#define BT_5 (0x20)
#define BT_6 (0x40)
#define BT_7 (0x80)
#define BT_8 (0x0100)
#define BT_9 (0x0200)
#define BT_10 (0x0400)
#define BT_11 (0x0800)
#define BT_12 (0x1000)
#define BT_13 (0x2000)
#define BT_14 (0x4000)
#define BT_15 (0x8000)



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
    SYSCTL_RCC_R |=  SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S); // Use divisor of 5 (SYSDIV=4) p223  ==> 40MHz
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

}


/**
 * main.c
 */
int main(void)
{
	return 0;
}
