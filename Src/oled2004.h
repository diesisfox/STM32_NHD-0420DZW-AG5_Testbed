/*
 * oled2004.h
 *
 *  Created on: Apr 12, 2017
 *      Author: jamesliu
 */

#ifndef OLED2004_H_
#define OLED2004_H_


#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define CLEAR_DISPLAY	0b00000001

#define RETURN_HOME		0b00000010

/*c = cursor shift mode, s = text shift enable*/
#define ENTRY_MODE(c,s)	(0b00000100|c|s)
#define CURS_INC	0b00000010
#define CURS_DEC	0b00000000
#define TEXT_SHIFT	0b00000001
#define TEXT_STAY	0b00000000

/*d = disp enable, c = curs enable, b = blink enable*/
#define DISP_ON_OFF(d,c,b)	(0b00001000|d|c|b)
#define DISP_ON		0b00000100
#define CURS_ON		0b00000010
#define BLINK_ON	0b00000001
#define DISP_OFF	0b00000000
#define CURS_OFF	0b00000000
#define BLINK_OFF	0b00000000

/*just use one of the four under the first*/
#define DISP_CURS_SHIFT		0b00010000
#define SHIFT_CURS_LEFT		0b00010000
#define SHIFT_CURS_RIGHT	0b00010100
#define SHIFT_DISP_LEFT		0b00011000
#define SHIFT_DISP_RIGHT	0b00011100

/*l = data length (use 8), f = font bank*/
#define FUNCTION_SET(l,f)	(0b00101000|l|f)
#define DL_8	0b00010000
#define DL_4	0b00000000
#define ENG_JAP	0b00000000
#define EURO_1	0b00000001
#define ENG_RUS	0b00000010
#define EURO_2	0b00000011

/*c = char num, r = row num*/
#define SET_CGRAM_ADDR(x)	(0b01000000|((x)&0x3f))
#define CGRAM_ADDRESSIFY(c,r) ((((c)&7)<<3)|((r)&7))

/*r = row, c = column*/
#define SET_DDRAM_ADDR(a)	(0b10000000|((a)&0x7f))
#define DDRAM_ADDRESSIFY(r,c) ((((r)&1)?0x40:0x00)+(((r)&2)?0x14:0x00)+((c)%20))


int Oled_begin(SPI_HandleTypeDef *newHspi);
uint8_t Oled_sendOneCmd(uint8_t cmd);
uint8_t Oled_sendOneCmdSync(uint8_t cmd);
uint8_t Oled_sendCatCmd(uint8_t len, uint8_t* cmds);
uint8_t Oled_sendCatCmdSync(uint8_t len, uint8_t* cmds);
uint8_t Oled_writeOneData(uint8_t x);
uint8_t Oled_writeOneDataSync(uint8_t x);
uint8_t Oled_writeCatData(uint8_t len, uint8_t* data);
uint8_t Oled_writeCatDataSync(uint8_t len, uint8_t* data);

#endif /* OLED2004_H_ */
