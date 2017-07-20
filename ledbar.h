/**   
 ******************************************************************************   
 * @file    mylib/ledbar.h
 * @author  Jacobus Hoffmann   
 * @date    20072017
 * @brief   ledbar peripheral driver
 *	      
 *
 *			
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************

 ******************************************************************************   
 */

 #ifndef LEDBAR_H
 #define LEDBAR_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
#define LEDBAR_PORT_PTR          	GPIOB
#define LEDBAR_ENABLE_CLK()			__HAL_RCC_GPIOB_CLK_ENABLE();
#define LEDBAR_PIN_0             	GPIO_PIN_3
#define LEDBAR_PIN_1             	GPIO_PIN_4
#define LEDBAR_PIN_2             	GPIO_PIN_5

#define LEDBAR_LED0_OFFSET			3

#define LEDBAR_NUM_LEDS				3

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* External function prototypes -----------------------------------------------*/



void ledbar_init_pins();

void ledbar_dsiplay(uint8_t num);

#endif /*****LEDBAR_H*****/
