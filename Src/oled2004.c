/*
 * oled2004.c
 *
 *  Created on: Apr 12, 2017
 *      Author: jamesliu
 */

#include "oled2004.h"

static SPI_HandleTypeDef* hspi;
static osMutexId oledTxMtx;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* cbhspi){
	if(cbhspi==hspi){
		xSemaphoreGiveFromISR(oledTxMtx, NULL);
	}
}

static uint8_t Oled_init(){
	static const uint8_t cmds[] = {
			CLEAR_DISPLAY,
			RETURN_HOME,
			DISP_ON_OFF(DISP_ON, CURS_ON, BLINK_ON),
			ENTRY_MODE(CURS_INC, TEXT_STAY),
			FUNCTION_SET(DL_8, ENG_RUS)
	};
	return Oled_sendCatCmdSync(sizeof(cmds), cmds);
}

uint8_t Oled_sendOneCmd(uint8_t cmd){
	//get write buffer mutex:
	xSemaphoreTake(oledTxMtx, portMAX_DELAY);
	static uint8_t temp[2];
	temp[0] = cmd>>2;
	temp[1] = cmd<<6;
	return HAL_SPI_Transmit_IT(hspi, temp, 2);
}

static uint8_t* catCmd(uint8_t len, uint8_t* cmds){
	if(len == 0) return 0xff;
	static uint8_t temp[320];

	//place in all the 4s of commands:
	int8_t groups = len/4;
	for(int i=0; i<groups; i++){
		temp[5*i+0] = cmds[4*i+0] >> 2;
		temp[5*i+1] = cmds[4*i+0] << 6 | cmds[4*i+1] >> 4;
		temp[5*i+2] = cmds[4*i+1] << 4 | cmds[4*i+2] >> 6;
		temp[5*i+3] = cmds[4*i+2] << 2;
		temp[5*i+4] = cmds[4*i+3];
	}

	//place in last group of commands:
	temp[5*groups+2]=temp[5*groups+1]=temp[5*groups+0]=0;
	uint8_t lastLen = 0;
	switch(len%4){
	case 3:
		temp[5*groups+2] |= cmds[4&groups+2] >> 6;
		temp[5*groups+3] |= cmds[4*groups+2] << 2;
		lastLen = 4;
	case 2:
		temp[5*groups+1] |= cmds[4*groups+1] >> 4;
		temp[5*groups+2] |= cmds[4*groups+1] << 4;
		lastLen = 3;
	case 1:
		temp[5*groups+0] |= cmds[4*groups+0] >> 2;
		temp[5*groups+1] |= cmds[4*groups+0] << 6;
		lastLen = 2;
	default: break;
	}

	return temp;
}

uint8_t Oled_sendCatCmd(uint8_t len, uint8_t* cmds){
	if(len == 0) return 0xff;

	//get write buffer mutex:
	xSemaphoreTake(oledTxMtx, portMAX_DELAY);

	//transmit
	return HAL_SPI_Transmit_DMA(hspi, catCmd(len,cmds), (len/4)*5 + ((len%4) ? len%4+1 : 0));
}

uint8_t Oled_sendCatCmdSync(uint8_t len, uint8_t* cmds){
	if(len == 0) return 0xff;
	if(HAL_SPI_GetState(hspi) != HAL_OK) return 0xfe;

	//transmit
	return HAL_SPI_Transmit(hspi, catCmd(len,cmds), (len/4)*5 + ((len%4) ? len%4+1 : 0));
}

uint8_t Oled_writeOneData(uint8_t x){
	//get write buffer mutex:
	xSemaphoreTake(oledTxMtx, portMAX_DELAY);
	static uint8_t temp[2];
	temp[0] = 0b10<<6 | x>>2;
	temp[1] = x<<6;
	return HAL_SPI_Transmit_IT(hspi, temp, 2);
}

static uint8_t* catData(uint8_t len, uint8_t* data){
	if(len == 0) return 0xff;
	static uint8_t temp[256];
	temp[0] = 0b10000000;
	for(int i=0; i<len; i++){
		temp[i+0] |= data[i]>>2;
		temp[i+1] = data[i]<<6;
	}
	return temp;
}

uint8_t Oled_writeCatData(uint8_t len, uint8_t* data){
	if(len == 0) return 0xff;

	//get write buffer mutex:
	xSemaphoreTake(oledTxMtx, portMAX_DELAY);

	//transmit
	return HAL_SPI_Transmit_DMA(hspi, catData(len,data), len+1);
}

uint8_t Oled_writeCatDataSync(uint8_t len, uint8_t* data){
	if(len == 0) return 0xff;
	if(HAL_SPI_GetState(hspi) != HAL_OK) return 0xfe;
	//transmit
	return HAL_SPI_Transmit(hspi, catData(len,data), len+1);
}

int Oled_begin(SPI_HandleTypeDef *newHspi){
	hspi = newHspi;
	osMutexDef(oledTxMtx);
	oledTxMtx = osMutexCreate(osMutex(oledTxMtx));
	return Oled_init();
}
