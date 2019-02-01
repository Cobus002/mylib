/*
 * mcp2515.h
 *
 *  Created on: Jan 24, 2019
 *      Author: Cobus Hoffmann
 */

#ifndef INC_MCP2515_H_
#define INC_MCP2515_H_

#define MCP2515			1

//Need to have a spiTransmit function available
extern void spiTransmit(char *data, char len);

extern void spiTransmitReceive(char *txData, char *rxData, char len);
//Pull the Chip enable pin low
extern void mcp2515CESet();
//Set the Chip enable pin high
extern void mcp2515CEReset();

/*******************MCP2515 COMMANDS*****************************/
#define MCP2515_RESET_CMD				0b11000000
#define MCP2515_READ_CMD				0b00000011
#define MCP2515_READ_RX_BUFF_CMD		0b10010000
#define MCP2515_WRITE_CMD				0b00000010
#define MCP2515_LOAD_TX_BUFF_CMD		0b01000000
#define MCP2515_RTS_CMD					0b10000000 //See datasheet for some more info
#define MCP2515_READ_STATUS_CMD			0b10100000
#define MCP2515_RX_STATUS_CMD			0b10110000
#define MCP2515_BIT_MODIFY_CMD			0b00000101

/*******************MCP2515 REGISTERS****************************/
#define MCP2515_BFPCTRL_REG				0x0C
#define MCP2515_B1BFS					5
#define MCP2515_B0BFS					4
#define MCP2515_B1BFE					3
#define MCP2515_B0BFE					2
#define MCP2515_B1BFM					1
#define MCP2515_B0BFM					0

#define MCP2515_TXRTSCTRL_REG			0x0D
#define MCP2515_B2RTS					5
#define MCP2515_B1RTS					4
#define MCP2515_B0RTS					3
#define MCP2515_B2RTSM					2
#define MCP2515_B1RTSM					1
#define MCP2515_B0RTSM					0

#define MCP2515_CANSTAT_REG				0x0E
#define MCP2515_OPMOD2					7
#define MCP2515_OPMOD1					6
#define MCP2515_OPMOD0					5
//—
#define MCP2515_ICOD2					3
#define MCP2515_ICOD1					2
#define MCP2515_ICOD0					1
//—

#define MCP2515_CANCTRL_REG				0x0F
#define MCP2515_REQOP2					7
#define MCP2515_REQOP1					6
#define MCP2515_REQOP0					5
#define MCP2515_ABAT					4
#define MCP2515_OSM						3
#define MCP2515_CLKEN					2
#define MCP2515_CLKPRE1 				1
#define MCP2515_CLKPRE0					0

//Transmit Error Counter Register
#define MCP2515_TEC_REG					0x1C

//Receive Error Counter Register
#define MCP2515_REC_REG					0x1D

//
#define MCP2515_CNF3_REG				0x28
#define MCP2515_SOF						7
#define MCP2515_WAKFIL					6
//—
//—
//—
#define MCP2515_PHSEG22					2
#define MCP2515_PHSEG21					1
#define MCP2515_PHSEG20					0

#define MCP2515_CNF2_REG				0x29
#define MCP2515_BTLMODE 				7
#define MCP2515_SAM 					6
#define MCP2515_PHSEG12 				5
#define MCP2515_PHSEG11 				4
#define MCP2515_PHSEG10 				3
#define MCP2515_PRSEG2 					2
#define MCP2515_PRSEG1 					1
#define MCP2515_PRSEG0					0

#define MCP2515_CNF1_REG				0x2A
#define MCP2515_SJW1 					7
#define MCP2515_SJW0 					6
#define MCP2515_BRP5 					5
#define MCP2515_BRP4 					4
#define MCP2515_BRP3 					3
#define MCP2515_BRP2 					2
#define MCP2515_BRP1 					1
#define MCP2515_BRP0					0

#define MCP2515_CANINTE_REG				0x2B
#define MCP2515_MERRE 					7
#define MCP2515_WAKIE 					6
#define MCP2515_ERRIE 					5
#define MCP2515_TX2IE 					4
#define MCP2515_TX1IE 					3
#define MCP2515_TX0IE 					2
#define MCP2515_RX1IE 					1
#define MCP2515_RX0IE					0

#define MCP2515_CANINTF_REG				0x2C
#define MCP2515_MERRF
#define MCP2515_WAKIF
#define MCP2515_ERRIF
#define MCP2515_TX2IF
#define MCP2515_TX1IF
#define MCP2515_TX0IF
#define MCP2515_RX1IF
#define MCP2515_RX0IF

#define MCP2515_EFLG_REG				0x2D
#define MCP2515_RX1OVR 					7
#define MCP2515_RX0OVR 					6
#define MCP2515_TXBO 					5
#define MCP2515_TXEP 					4
#define MCP2515_RXEP 					3
#define MCP2515_TXWAR 					2
#define MCP2515_RXWAR 					1
#define MCP2515_EWARN					0

#define MCP2515_TXB0CTRL_REG			0x30
#define MCP2515_TXB0DLC_REG				0x35
#define MCP2515_TXB1CTRL_REG			0x40
#define MCP2515_TXB1DLC_REG				0x45
#define MCP2515_TXB2CTRL_REG			0x50
//—
#define MCP2515_ABTF					6
#define MCP2515_MLOA					5
#define MCP2515_TXERR					4
#define MCP2515_TXREQ					3
//—
#define MCP2515_TXP1					1
#define MCP2515_TXP0					0


#define MCP2515_RXB0CTRL_REG			0x60
#define MCP2515_RXB1CTRL_REG			0x70
//—
#define MCP2515_RXM1 					6
#define MCP2515_RXM0 					5
//—
#define MCP2515_RXRTR					3
#define MCP2515_BUKT_1 					2
#define MCP2515_BUKT_0 					1
#define MCP2515_FILHIT0					0

/*******************MCP2515 REGISTERS END*************************/

//CanMessage type declaration
typedef struct {
	unsigned short id;
	struct {
		char rtr:1;
		char len:4;
	}header;
	char data[8];

}CanMessage_t;

//Read a register at address of the mcp2515
char mcp2515ReadReg(char addr);

//Write a regster at address of mcp2515 with value
void mcp2515WriteReg(char addr, char value);

//Initialise the mcp2515
char mcp2515Init(char speed);

//Send message
void mcp2515SendMessage(CanMessage_t *message);







#endif /* INC_MCP2515_H_ */
