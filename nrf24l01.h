/**   
 ******************************************************************************   
 * @file    mylib/nrf24l01.h  
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

#ifndef NRF24L01_H
#define NRF24L01_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "spi.h"

/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
#define NRF24L01_CE_PIN                     			GPIO_PIN_1
#define NRF24L01_CE_PORT                    			GPIOA
#define NRF24L01_CE_CLK_ENABLE()            			__HAL_RCC_GPIOA_CLK_ENABLE()
#define NRF24L01_CE_LOW()								HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET)
#define NRF24L01_CE_HIGH()								HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_SET)

#define NRF24L01_CSN_PIN                    			GPIO_PIN_3
#define NRF24L01_CSN_PORT                   			GPIOA
#define NRF24L01_CSN_CLK_ENABLE()           			__HAL_RCC_GPIOA_CLK_ENABLE()
#define NRF24L01_CSN_LOW()								HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_RESET)
#define NRF24L01_CSN_HIGH()								HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_SET)

#define NRF24L01_INTR_PIN                   			GPIO_PIN_0
#define NRF24L01_INTR_PORT                  			GPIOB
#define NRF24L01_INTR_CLK_ENABLE()          			__HAL_RCC_GPIOB_CLK_ENABLE()

#define NRF24L01_INTR_IRQn                  			EXTI0_IRQn
#define NRF24L01_INTR_IRQn_PRIORITY         			10

#define NRF24L01_LED_PIN                      			GPIO_PIN_13
#define NRF24L01_LED_PORT                     			GPIOC
#define NRF24L01_LED_CLK_ENABLE()           			__HAL_RCC_GPIOC_CLK_ENABLE()
#define NRF_TOGGLE_LED()                   				 HAL_GPIO_TogglePin(NRF24L01_LED_PORT, NRF24L01_LED_PIN)

/* Public macro -------------------------------------------------------------*/
#define nrf24l01_write_spi(txBuff, rxBuff, numBytes)	send_receive_spi1(txBuff, rxBuff, numBytes)
#define nrf24l01_spi_handle								(SPI_HandleTypeDef)hspi1

#define NRF24L01_PAYLOAD_SIZE							(uint8_t)1	//Number of bytes
/* Public variables ---------------------------------------------------------*/
extern uint8_t nrf24l01IRQUpdate;
/* External function prototypes -----------------------------------------------*/

HAL_StatusTypeDef nrf24l01_init(void);

HAL_StatusTypeDef nrf24l01_setup_tx(void);

HAL_StatusTypeDef nrf24l01_setup_rx(void);

HAL_StatusTypeDef nrf24l01_data_available(void);

HAL_StatusTypeDef nrf24l01_send_data(uint8_t *pTX, uint8_t numBytes);

HAL_StatusTypeDef nrf24l01_data_sent(void);

void nrf24l01_reset_tx(void);

void nrf24l01_reset_rx(void);

uint8_t nrf24l01_read_reg(uint8_t reg);

void nrf24l01_read_reg_multi(uint8_t reg, uint8_t *regData, uint8_t numBytes);

HAL_StatusTypeDef nrf24l01_get_data(uint8_t *dataBuff);

void nrf24l01_IRQ_handler(void);


#endif
