/*
 * Pins.h
 *
 *  Created on: Apr 30, 2018
 *      Author: muell498227
 */

#ifndef PINS_H_
#define PINS_H_

///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Bitband and Pin Definitions
//
///////////////////////////////////////////////////////////////////////////////////////////////////




#define BITBAND(BYTE_ADDR,BIT) (((volatile uint32_t *)(   (((uint32_t)&(BYTE_ADDR))&(0xF0000000)) + 0x02000000 + (((uint32_t)&(BYTE_ADDR)) << 5) + ((BIT) << 2)    )))

#define  LED_RED      * ( BITBAND(GPIO_PORTD_DATA_R,2) )
#define  LED_YELLOW   * ( BITBAND(GPIO_PORTD_DATA_R,7) )
#define  LED_BLUE     * ( BITBAND(GPIO_PORTD_DATA_R,6) )

#define  LED_BL       BITBAND(GPIO_PORTA_DATA_R,6)
#define  SCR_DATA     BITBAND(GPIO_PORTA_DATA_R,4)
#define  SCR_RESET    BITBAND(GPIO_PORTA_DATA_R,3)
#define  SCR_CSX      BITBAND(GPIO_PORTA_DATA_R,7)

// The registers for setting the brightness of the RGB LED
// 0 is off, 4096 is full brightness
#define RGB_BLU_REG   PWM0_2_CMPA_R
#define RGB_RED_REG   PWM0_3_CMPA_R
#define RGB_GRN_REG   PWM0_3_CMPB_R



#define  BUTTON_0        * (BITBAND(GPIO_PORTC_DATA_R,6))
#define  BUTTON_1        * (BITBAND(GPIO_PORTC_DATA_R,7))
#define  BUTTON_2        * (BITBAND(GPIO_PORTF_DATA_R,2))

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


typedef int bool;
enum { false, true };

#endif /* PINS_H_ */
