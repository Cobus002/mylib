/*
 * mcp2515.c
 *
 *  Created on: Jan 24, 2019
 *      Author: Cobus Hoffmann
 */
#include "mcp2515.h"

char mcp2515DataBuff[20];

char mcp2515ReadReg(char addr){
	char rxBuff[3];
	mcp2515DataBuff[0] = MCP2515_READ_CMD;
	mcp2515DataBuff[1] = addr;
	mcp2515DataBuff[2] = 0xff;

	mcp2515CEReset();
	spiTransmitReceive(mcp2515DataBuff, rxBuff, 3);
	mcp2515CESet();

	return rxBuff[2];

}


void mcp2515WriteReg(char addr, char value){

	//Load the data into the buffer
	mcp2515DataBuff[0]=MCP2515_WRITE_CMD;
	mcp2515DataBuff[1]=addr;
	mcp2515DataBuff[2]=value;

	//Transmit to the mcp2515 device
	mcp2515CEReset();
	spiTransmit(mcp2515DataBuff, 3);
	mcp2515CESet();

}



//Initialise the mcp2515
char mcp2515Init(char speed){

	//Reset the device
	mcp2515DataBuff[0]=MCP2515_RESET_CMD;
	//Pull chip enable low
	mcp2515CEReset();
	spiTransmit(mcp2515DataBuff, 1);
	mcp2515CESet();

	//Delay 1ms

	//Set up the speed
	mcp2515DataBuff[0]=MCP2515_WRITE_CMD;
	mcp2515DataBuff[1]=MCP2515_CNF3_REG;
	mcp2515DataBuff[2]=0x05;
	mcp2515DataBuff[3]=0xb1;
	mcp2515DataBuff[4]=0x00;
	mcp2515DataBuff[5]=(1<<MCP2515_RX0IE)|(1<<MCP2515_RX1IE);

	//Send the configuration
	mcp2515CEReset();
	spiTransmit(mcp2515DataBuff, 6);
	mcp2515CESet();



	//Turn of the filters and receive any message
	mcp2515WriteReg(MCP2515_RXB0CTRL_REG, (1<<MCP2515_RXM1)|(1<<MCP2515_RXM0));
	mcp2515WriteReg(MCP2515_RXB1CTRL_REG, (1<<MCP2515_RXM1)|(1<<MCP2515_RXM0));

	//Set device into normal mode
	mcp2515WriteReg(MCP2515_CANCTRL_REG, 0);
	return 1;
}


//Send message
void mcp2515SendMessage(CanMessage_t *message){
	//Check the status of the mcp2515 device
	char status = mcp2515ReadReg(MCP2515_READ_STATUS_CMD);

	//Set the address
	char addr = 0x00;
	mcp2515DataBuff[0]=MCP2515_LOAD_TX_BUFF_CMD|addr;
	mcp2515DataBuff[1]=(1<<4)|(1<<2)|(1<<0);
	mcp2515DataBuff[2]=(1<<6);
	mcp2515DataBuff[3]=0;
	mcp2515DataBuff[4]=0;
	char length = message->header.len & 0x0f;
	mcp2515DataBuff[5]=length;

	//Load the data
	for(char i=0; i<length; i++){
		mcp2515DataBuff[6+i] = message->data[i];
	}

	//Load the TX buffer to the mcp2515 device
	mcp2515CEReset();
	spiTransmit(mcp2515DataBuff, 6+length);
	mcp2515CESet();

	//Delay 1 ms

	//Request to send
	mcp2515DataBuff[0] = (MCP2515_RTS_CMD)|(1<<0);
	mcp2515CEReset();
	spiTransmit(mcp2515DataBuff, 1);
	mcp2515CESet();

}
