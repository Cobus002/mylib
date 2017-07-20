/**   
 ******************************************************************************   
 * @file    mylib/spi.h  
 * @author  Jacobus Hoffmann   
 * @date    12072017   
 * @brief   SPI peripheral driver
 *	      
 *
 *			
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************

 ******************************************************************************   
 */

 #ifndef SPI_H
 #define SPI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
#define SPI1_MAX_TIMEOUT            0x100
#define SPI1_MODE					SPI_MODE_MASTER
#define SPI1_DIRECTION				SPI_DIRECTION_2LINES
#define SPI1_DATASIZE				SPI_DATASIZE_8BIT
#define SPI1_CLKPOLARITY			SPI_POLARITY_LOW
#define SPI1_CLKPHASE				SPI_PHASE_1EDGE
#define SPI1_BAUD_PRESCALER			SPI_BAUDRATEPRESCALER_32
#define SPI1_FIRSTBIT				SPI_FIRSTBIT_MSB
#define SPI1_TIMODE					SPI_TIMODE_DISABLE
#define SPI1_CRCCALC				SPI_CRCCALCULATION_DISABLE
#define SPI1_CRCPOLY				10


#define SPI2_MAX_TIMEOUT            0x100



/* Public macro -------------------------------------------------------------*/
#define send_receive_spi1(txBuff, rxBuff, numBytes)	 	send_receive_spi(&hspi1, txBuff, rxBuff, numBytes)
#define send_receive_spi2(txBuff, rxBuff, numBytes)		send_receive_spi(&hspi2, txBuff, rxBuff, numBytes)
/* Public variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
/* External function prototypes -----------------------------------------------*/

void spi1_init();
void spi2_init();
void send_receive_spi(SPI_HandleTypeDef *spi, uint8_t *txBuff, uint8_t *rxBuff, uint16_t numBytes);



#endif
