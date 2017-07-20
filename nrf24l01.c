/**   
 ******************************************************************************   
 * @file    mylib/nrf24l01.c    
 * @author  Jacobus Hoffmann  
 * @date    12072017 
 * @brief   nrf24l01 peripheral driver  
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "nrf24l01.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/************Commands********************/
#define R_MASK				0b00000000
#define W_MASK				0b00100000
#define R_RX_PAYLOAD		0b01100001
#define W_TX_PAYLOAD		0b10100000
#define FLUSH_TX			0b11100001
#define NOP 				0xff

/***********Registers********************/
#define CONFIG_REG				0x00
/*
 * 7: 	Reserved
 * 6:	MASK_RX_DR
 * 5:	MASK_TX_DS
 * 4:	MASK_MAX_RT
 * 3:	EN_CRC
 * 2:	CRCO
 * 1:	PWR_UP				[1: PWR UP, 0: PWR DOWN]
 * 0:	PRIM_RX				RX/TX Control [1: PRX, 0: PTX]
 */
#define	EN_CRC					(uint8_t)(3)
#define	PWR_UP					(uint8_t)(1)
#define PRIM_RX					(uint8_t)(0)

#define STATUS_REG				(uint8_t)0x07

/*
 * 7: 	Reserved
 * 6:	RX_DR
 * 5:	TX_DS					Set when data successfully sent
 * 4:	MAX_RT					Maximum retransmit tries, write 1 to clear
 * 3:	RX_P_NO[3]
 * 2:	RX_P_NO[2]
 * 1:	RX_P_NO[1]				[1: PWR UP, 0: PWR DOWN]
 * 0:	TX_FULL					TX_FIFO full
 */
#define RX_DR					(uint8_t)(6)
#define TX_DS					(uint8_t)(5)
#define MAX_RT					(uint8_t)(4)
#define TX_FULL					(uint8_t)(0)

#define EN_AA_REG 				0x01
#define EN_RXADDR_REG			0x02
#define RX_PW_P0				(uint8_t)0x11

/* Private macro -------------------------------------------------------------*/
#define clear(x)				memset(x, '\0', sizeof(x))
/* Private variables ---------------------------------------------------------*/
uint8_t statusReg = 0;
uint8_t statusUpdate = 0;
/* Private prototypes ---------------------------------------------------------*/

//This function needs to toggle all the pins in order to send spi data
void nrf24l01_write(uint8_t *txBuff, uint8_t *rxBuff, uint8_t numBytes);

//Implementation of functions
HAL_StatusTypeDef nrf24l01_init(void) {
	//Initialise the gpios pins
	GPIO_InitTypeDef GPIO_InitStruct;

	//Enable the gpio clocks
	NRF24L01_CE_CLK_ENABLE();
	NRF24L01_CSN_CLK_ENABLE();
	NRF24L01_INTR_CLK_ENABLE();
	NRF24L01_LED_CLK_ENABLE();

	/*Configure GPIO pins : NRF_CE_Pin*/
	GPIO_InitStruct.Pin = NRF24L01_CE_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStruct);

	/*Configure GPIO pins : NRF_CSN_Pin*/
	GPIO_InitStruct.Pin = NRF24L01_CSN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(NRF24L01_CSN_PORT, &GPIO_InitStruct);

	/*Configure GPIO pins : NRF_LED_Pin*/
	GPIO_InitStruct.Pin = NRF24L01_LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(NRF24L01_LED_PORT, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(NRF24L01_INTR_IRQn, NRF24L01_INTR_IRQn_PRIORITY, 0);
    //Set the vector handler
    HAL_NVIC_EnableIRQ(NRF24L01_INTR_IRQn);

    /*Configure GPIO pins : NRF_CE_Pin*/
    GPIO_InitStruct.Pin = NRF24L01_INTR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(NRF24L01_INTR_PORT, &GPIO_InitStruct);
	return HAL_OK;

}

/**
 * @brief  This function is used to set the nrf module into tx mode
 * @param  None
 * @retval None
 */
HAL_StatusTypeDef nrf24l01_setup_tx(void) {
	uint8_t buff[] = { 0x00, 0x00 };
	uint8_t rxBuff[] = { 0x00, 0x00 };

	//Set the number of bytes to be received
	buff[0] = (uint8_t) (W_MASK | RX_PW_P0 );
	buff[1] = NRF24L01_PAYLOAD_SIZE;
	/*>>>>>>>>>>Send the data to the nrf24l01 over spi<<<<<<<<<<*/
	nrf24l01_write(buff, rxBuff, NRF24L01_PAYLOAD_SIZE);
	//Power up the module
	HAL_Delay(1);
	return HAL_OK;
}

/**
 * @brief  This function is used to set the nrf module into rx mode
 * @param  None
 * @retval None
 */
HAL_StatusTypeDef nrf24l01_setup_rx(void) {

	uint8_t buff[] = { 0x00, 0x00 };
	uint8_t rxBuff[] = { 0x00, 0x00 };
	uint8_t check = 0x00;
	uint8_t errStr[20];
	memset(errStr, '\0', sizeof(errStr));

	//Set in stanby mode, so that registers can be configured
	NRF24L01_CE_LOW();
	HAL_Delay(1);
	//Send NOP just to make sure all pins are in correct configuration
	buff[0] = (uint8_t) (NOP);
	nrf24l01_write(buff, rxBuff, 1);
	HAL_Delay(1);

	buff[0] = (uint8_t) (W_MASK | CONFIG_REG);
	buff[1] = (uint8_t)((1 << EN_CRC )|(1 << PWR_UP )|(1 << PRIM_RX ));
	/*>>>>>>>>>>Send the data to the nrf24l01 over spi<<<<<<<<<<*/
	nrf24l01_write(buff, rxBuff, 2);
	HAL_Delay(1);
	clear(buff);
	clear(rxBuff);

	buff[0] = (uint8_t) (R_MASK | CONFIG_REG);
	buff[1] = (uint8_t) NOP;
	/*>>>>>>>>>>Send the data to the nrf24l01 over spi<<<<<<<<<<*/
	nrf24l01_write(buff, rxBuff, 2);
	//check if the data was set successfully
	check = (uint8_t) (uint8_t) ((1 << EN_CRC ) | (1 << PWR_UP )
			| (1 << PRIM_RX ));
	if (check != rxBuff[1]) {
		sprintf(errStr, "515 STAT: %02x CONF: %02x\n\r", rxBuff[0], rxBuff[1]);
		CDC_Transmit_FS(errStr, strlen(errStr));
		return HAL_ERROR;
	}else{
		CDC_Transmit_FS("NRF24L01 RX Setup\n", strlen("NRF24L01 RX Setup\n"));
	}
	HAL_Delay(1);

	clear(buff);
	clear(rxBuff);
	//Set the number of bytes to be received
	buff[0] = (uint8_t) (W_MASK | RX_PW_P0 );
	buff[1] = NRF24L01_PAYLOAD_SIZE;
	/*>>>>>>>>>>Send the data to the nrf24l01 over spi<<<<<<<<<<*/
	nrf24l01_write(buff, rxBuff, 2);
	//Power up the module
	HAL_Delay(1);
	NRF24L01_CE_HIGH();

	//All is well and unicorns live
	return HAL_OK;

}
/**
 * @brief  This function is used check if there is data available
 * @param  None
 * @retval HAL_StatusTypedef
 * 		HAL_OK: 	data available
 * 		HAL_ERROR: 	data not available
 */
HAL_StatusTypeDef nrf24l01_data_available(void) {
	//Check the data_available bit
	uint8_t txBuff = NOP;
	uint8_t rxBuff = 0;

	//Check the status reg
	nrf24l01_write(&txBuff, &rxBuff, 1);

	if ((rxBuff & (1 << RX_DR )) > 0) {
		//there is data
		CDC_Transmit_FS("Data available\n", strlen("Data available\n"));
		return HAL_OK;
	} else {
		//there is no data
		CDC_Transmit_FS("no data\n", strlen("no data\n"));
		return HAL_ERROR;
	}

}
/**
 * @brief  This function is used to send data
 * @param  uint8_t pTX: the data buffer containing the information
 * 						to be sent.
 * @retval None
 */
HAL_StatusTypeDef nrf24l01_send_data(uint8_t *pTX, uint8_t numBytes) {
	uint8_t txBuff[numBytes+1];
	uint8_t rxBuff[numBytes+1];
	memset(txBuff, 0, sizeof(txBuff));
	memset(rxBuff, 0, sizeof(rxBuff));

	//Set the power up bit high
	txBuff[0] = (uint8_t) (W_MASK | CONFIG_REG);
	txBuff[1] = (uint8_t) ((1 << EN_CRC ) | (1 << PWR_UP ));
	nrf24l01_write(txBuff, rxBuff, 2);

	//Now send the data
	txBuff[0]=(uint8_t)(W_TX_PAYLOAD);
	memcpy(txBuff+sizeof(uint8_t), pTX, numBytes);
	nrf24l01_write(txBuff, rxBuff, numBytes+1);
	HAL_Delay(1);


	//Data loaded into the device now pulse the CE pin for more than 10us
	NRF24L01_CE_HIGH();
	HAL_Delay(1);
	NRF24L01_CE_LOW();

	return HAL_OK;

}



HAL_StatusTypeDef nrf24l01_data_sent(void){
	//Check the status register to see if the data was sent successfully
	uint8_t txBuff=NOP;
	uint8_t rxBuff=0;

	nrf24l01_write(&txBuff, &rxBuff, 1);

	if(rxBuff&(1<<TX_DS)){
		//Send was successful
		CDC_Transmit_FS("Data sent\n", strlen("Data sent\n"));
		return HAL_OK;

	}else if(rxBuff&(1<<MAX_RT)){
		//the send was unsuccessful
		CDC_Transmit_FS("max re-try\n", strlen("max re-try\n"));
		return HAL_ERROR;
	}else{
		//something else is wrong
		return HAL_BUSY;
	}

}

//This function resets the tx state, can be called if unsuccessful/successful send occured
void nrf24l01_reset_tx(void){
	uint8_t txBuffer[2];
	uint8_t rxBuffer[2];
	memset(rxBuffer, 0, sizeof(rxBuffer));
	//Flush the tx buffer
	txBuffer[0]=FLUSH_TX;
	nrf24l01_write(txBuffer, rxBuffer, 1);
	HAL_Delay(1);

	//Now clear the max_rt flag && the TX_DS flag
	txBuffer[0]=(W_MASK|STATUS_REG);
	txBuffer[1]=(1<<MAX_RT)|(1<<TX_DS);
	rxBuffer[0]=0;
	rxBuffer[1]=0;
	nrf24l01_write(txBuffer, rxBuffer, 2);
	HAL_Delay(1);
}

//This function resets the rx state, can be called after data has been received
void nrf24l01_reset_rx(void){
	uint8_t txBuff[]={(W_MASK|STATUS_REG), (1<<RX_DR)};
	uint8_t rxBuff[]={0,0};
	nrf24l01_write(txBuff, rxBuff, 2);
	HAL_Delay(1);


}

/**
 * @brief  	This function is used get the data, up to the user to check if there
 * 			is any data available before calling this function.
 * @param  uint8_t *dataBuff: 	buffer used to store the data in the RX fifo buffer
 * 								up to user to make sure it is initialised at the correct
 * 								size (NRF24L01_PAYLOAD_SIZE+1)
 * @retval None
 */
HAL_StatusTypeDef nrf24l01_get_data(uint8_t *dataBuff) {
	//get the data in the register
	uint8_t txBuff[NRF24L01_PAYLOAD_SIZE + 1];

	//Make sure the buffers are clean
	memset(txBuff, NOP, sizeof(txBuff + 1));	//Set all the other bytes to NOP
	memset(dataBuff, 0, sizeof(dataBuff));

	txBuff[0] = R_RX_PAYLOAD;
	//Send the command and retrieve the data
	nrf24l01_write(txBuff, dataBuff, (NRF24L01_PAYLOAD_SIZE + 1));
	//Now clear the flag in the status reg
	txBuff[0] = (uint8_t) (W_MASK | STATUS_REG );
	txBuff[1] = (uint8_t) (1 << RX_DR );
	uint8_t tempBuff[2];
	//write the register
	nrf24l01_write(txBuff, tempBuff, 2);

	return HAL_OK;

}

//This function is used to write data to the nrf24l01 module
void nrf24l01_write(uint8_t *txBuff, uint8_t *rxBuff, uint8_t numBytes) {
	//Pull the CE pin low
	NRF24L01_CSN_LOW();
	nrf24l01_write_spi(txBuff, rxBuff, numBytes);
	NRF24L01_CSN_HIGH();
}

//Read the register
uint8_t nrf24l01_read_reg(uint8_t reg){
	uint8_t regData[]={0,0};

	uint8_t data[] = {(R_MASK|reg), NOP};
	nrf24l01_write(&data, &regData, 2);
	HAL_Delay(1);
	return regData[1];
}

void nrf24l01_read_reg_multi(uint8_t reg, uint8_t *regData, uint8_t numBytes){
	uint8_t txData;
	//make sure the buffer is clean
	memset(regData, 0, sizeof(regData));


}

void EXTI0_IRQHandler(){
	//The interrupt line was triggered
	  if(__HAL_GPIO_EXTI_GET_IT(NRF24L01_INTR_PIN) != RESET)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(NRF24L01_INTR_PIN);
	    //Now handle the interrupt
	    nrf24l01IRQUpdate=1;
	  }

}


