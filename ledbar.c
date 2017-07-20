/**   
 ******************************************************************************   
 * @file    mylib/ledbar.c
 * @author  Jacobus Hoffmann  
 * @date    20072017
 * @brief   ledbar peripheral driver
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "ledbar.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t ledbarArray[]={LEDBAR_PIN_0,LEDBAR_PIN_1,LEDBAR_PIN_2};
/* Private prototypes ---------------------------------------------------------*/


/**
 * Initialise all the pins used by the dipswitch driver and set the interrupts
 */
void ledbar_init_pins(){
	//Enable the GPIO CLK
	LEDBAR_ENABLE_CLK();
	GPIO_InitTypeDef GPIO_InitStruct;
	for(int i=0; i<LEDBAR_NUM_LEDS; i++){
		GPIO_InitStruct.Pin=ledbarArray[i];
		GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull=GPIO_NOPULL;
		GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_MEDIUM;

		HAL_GPIO_Init(LEDBAR_PORT_PTR, &GPIO_InitStruct);
	}
}

void ledbar_dsiplay(uint8_t num){

	for(int i=0; i<LEDBAR_NUM_LEDS; i++){
		if(0x01&(num>>i)){
			//set the bit
			HAL_GPIO_WritePin(LEDBAR_PORT_PTR, ledbarArray[i], GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LEDBAR_PORT_PTR, ledbarArray[i], GPIO_PIN_RESET);
		}
	}

}
