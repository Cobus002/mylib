/**   
 ******************************************************************************   
 * @file    mylib/spi.c    
 * @author  Jacobus Hoffmann  
 * @date    12072017 
 * @brief   spi peripheral driver 
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 
 ******************************************************************************   
 */

 /* Includes ------------------------------------------------------------------*/
#include "spi.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private prototypes ---------------------------------------------------------*/


void spi1_init(void){


    //initialise spi1
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI1_MODE;
    hspi1.Init.Direction = SPI1_DIRECTION;
    hspi1.Init.DataSize = SPI1_DATASIZE;
    hspi1.Init.CLKPolarity = SPI1_CLKPOLARITY;
    hspi1.Init.CLKPhase = SPI1_CLKPHASE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI1_BAUD_PRESCALER;
    hspi1.Init.FirstBit = SPI1_FIRSTBIT;
    hspi1.Init.TIMode = SPI1_TIMODE;
    hspi1.Init.CRCCalculation = SPI1_CRCCALC;
    hspi1.Init.CRCPolynomial = SPI1_CRCPOLY;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }


}

void spi2_init(){

}

void send_receive_spi(SPI_HandleTypeDef *spi, uint8_t *txBuff, uint8_t *rxBuff, uint16_t numBytes){

	HAL_SPI_TransmitReceive(spi, txBuff, rxBuff, numBytes, SPI1_MAX_TIMEOUT);
}


