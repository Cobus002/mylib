/**   
 ******************************************************************************   
 * @file    mylib/dipswitch.c    
 * @author  Jacobus Hoffmann  
 * @date    12072017 
 * @brief   dipswitch peripheral driver  
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "dipswitch.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t dipswitchArray[]={DIPSWITCH_PIN_0,DIPSWITCH_PIN_1,DIPSWITCH_PIN_2};
/* Private prototypes ---------------------------------------------------------*/


/**
 * Initialise all the pins used by the dipswitch driver and set the interrupts
 */
void dipswitch_init_pins(){
	//Enable the GPIO CLK
	DPISWITCH_ENABLE_CLK();
	//Initialise the interrupt vector ptiority
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, DIPSWITCH_INTR_PRIORITY, 0);
	GPIO_InitTypeDef GPIO_InitStruct;
	for(int i=0; i<DIPSWITCH_NUM_SWITCHES; i++){
		GPIO_InitStruct.Pin=dipswitchArray[i];
		GPIO_InitStruct.Mode=GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull=GPIO_PULLDOWN;
		GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_MEDIUM;

		HAL_GPIO_Init(DIPSWITCH_PORT_PTR, &GPIO_InitStruct);
	}
	//Enable the interrupt vector
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
 * This is function loops through the set pins and then gets the states
 */
uint8_t dipswitch_get_pin_states(void){
	uint8_t switchStatus=0;
	for(uint8_t i=0; i<DIPSWITCH_NUM_SWITCHES; i++){
		switchStatus|=(HAL_GPIO_ReadPin(DIPSWITCH_PORT_PTR, dipswitchArray[i])<<i);
	}
	return switchStatus;
}


/**
 * This is the interrupt handler for the pins of the dipswitch driver
 */
void EXTI9_5_IRQHandler(){
	//Check which itterupt line was triggered
	for(uint8_t i=0; i<DIPSWITCH_NUM_SWITCHES; i++){
		if(__HAL_GPIO_EXTI_GET_IT(dipswitchArray[i]) != RESET){
			//If triggered then reset the interrupt
			switchUpdate=1;
			__HAL_GPIO_EXTI_CLEAR_IT(dipswitchArray[i]);
		}

	}

}
