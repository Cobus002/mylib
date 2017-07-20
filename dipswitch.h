/**   
 ******************************************************************************   
 * @file    mylib/dipswitch.h  
 * @author  Jacobus Hoffmann   
 * @date    12072017   
 * @brief   NRF24l01 peripheral driver 
 *	      
 *
 *			
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************

 ******************************************************************************   
 */

 #ifndef DIPSWITCH_H
 #define DIPSWITCH_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
#define DIPSWITCH_PORT_PTR          GPIOB
#define DPISWITCH_ENABLE_CLK()		__HAL_RCC_GPIOB_CLK_ENABLE();
#define DIPSWITCH_PIN_0             GPIO_PIN_5
#define DIPSWITCH_PIN_1             GPIO_PIN_6
#define DIPSWITCH_PIN_2             GPIO_PIN_7

#define DIPSWITCH_NUM_SWITCHES		3

#define DIPSWITCH_INTR_PRIORITY		9
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
extern uint8_t switchUpdate;


/* External function prototypes -----------------------------------------------*/



void dipswitch_init_pins();

uint8_t dipswitch_get_pin_states(void);

#endif /*****DIPSWITCH_H*****/
