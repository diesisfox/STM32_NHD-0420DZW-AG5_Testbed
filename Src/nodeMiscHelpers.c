/*
 * nodeMiscHelpers.c
 *
 *  Created on: Dec 31, 2016
 *      Author: frank
 */
#include "nodeMiscHelpers.h"
#include "nodeConf.h"

extern uint32_t 	selfStatusWord;
extern osMutexId 	swMtxHandle;
extern osMessageQId mainCanTxQHandle;
extern osMessageQId mainCanRxQHandle;
extern osTimerId 	HBTmrHandle;

/*
 * Command executer for implementing node command responses
 */

void dumpFrame(Can_frame_t *newFrame){
//	static uint8_t radrxmsg[13];
//	for(int i=0; i<4; i++){
//		radrxmsg[3-i] = (newFrame->id >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
//	}
//	radrxmsg[4] = newFrame->dlc;
//	for(int i=0; i<newFrame->dlc; i++){
//		radrxmsg[5+i] = newFrame->Data[i];
//	}
//	// TODO: Change the following 4 lines for alternative radio options
//	Serial2_writeBytes(radrxmsg, newFrame->dlc+5);
//	Serial2_write('\n');
	frameToBase64(newFrame);
}

static void flushCanQueue(){
	static Can_frame_t newFrame;
	while(xQueueReceive(mainCanRxQHandle, &newFrame, 0) == pdTRUE){
		dumpFrame(&newFrame);
	}
}

void executeCommand(uint8_t cmd){
	switch(cmd){
	// Hard reset
	case NODE_HRESET: //WORKS
		NVIC_SystemReset();					// CMSIS System reset function
		break;

	// Soft Reset
	case NODE_RESET: //WORKS
//		node_shutdown();					// Soft shutdown
		soft_shutdown(flushCanQueue);
		NVIC_SystemReset();					// CMSIS System reset function
		break;

	// Clean shutdown
	// NOTE: CAN command processor is still active!
	case NODE_SHUTDOWN: //WORKS
		if((selfState == ACTIVE) || (selfState == INIT)){
			node_shutdown();						// Soft shutdown if node is active
			xTimerStop(HBTmrHandle, /*portMAX_DELAY*/0);	// Stop the heartbeat timer
			//DISCUSS the max_delay
			// XXX 4: User must suspend any additional application tasks
			// xTaskSuspend(ApplicationHandle);		// Suspend any active non-CAN tasks
		}
		break;

	// Node start command from shutdown state
	case NODE_START: //WORKS
		if(selfState == SHUTDOWN){
			setState(INIT);
			// Flush the Rx queue for fresh state on start-up
			xQueueReset(mainCanRxQHandle);
			// XXX 2: Flush the application queues!
			// xQueueReset();

			xTimerReset(HBTmrHandle,portMAX_DELAY);	// Start the heartbeat timer

			// XXX 3: User must resume additional application tasks
			// xTaskResume(ApplicationHandle);			// Resume any application tasks
		}
		break;

	// CC Acknowledgement of node addition attempt
	case CC_ACK: //WORKS
		if(selfState == INIT){
			setState(ACTIVE);
			static Can_frame_t newFrame;
			newFrame.id = selfNodeID + swOffset;
			newFrame.dlc = CAN_HB_DLC;
			for(int i=0; i<4; i++){
				newFrame.Data[3-i] = (selfStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
			}
			bxCan_sendFrame(&newFrame);
		}
		break;

	// CC Negation of node addition attempt
	case CC_NACK: //WORKS
		if(selfState == INIT){
			setState(SHUTDOWN);
		}
		break;

	default:
		// Do nothing if the command is invalid
		break;
	}
}

/* CHECKED
 * Thread-safe node state accessor
 */
nodeState inline getSelfState(){
	xSemaphoreTake(swMtxHandle, portMAX_DELAY);
	nodeState ret = (nodeState)(selfStatusWord & 0x07);
	xSemaphoreGive(swMtxHandle);
	return ret;
}

/* CHECKED
 * Thread-safe node state mutator
 */
inline void setSelfState(nodeState newState){
	xSemaphoreTake(swMtxHandle, portMAX_DELAY);
	selfStatusWord &= 0xfffffff8;	// Clear the current status word
	selfStatusWord |= newState;
	xSemaphoreGive(swMtxHandle);
}

/*
 * Soft shutdown routine that will complete any critical cleanups via callback
 * Assembles node SHUTDOWN statusword CAN frame
 * Flushes CanTx queue via broadcast on bxCAN
 */
inline void soft_shutdown(void(*usr_clbk)()){
	// Don't care about locking the statusWord here since we are in Critical Area
	setState(SHUTDOWN);

	// Broadcast node shutdown state to main CAN
	Can_frame_t newFrame;
	newFrame.id = radio_SW;
	newFrame.dlc = CAN_HB_DLC;
	newFrame.isExt = 0;
	for(int i=0; i<4; i++){
		newFrame.Data[3-i] = (selfStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	}
	bxCan_sendFrame(&newFrame);

	// User defined shutdown routine
	if(usr_clbk != NULL) usr_clbk();
	// TODO: Test if bxCan_sendFrame can successfully send the new frame and flush the queue
}

inline void waitTilAvail(uint length){ //blocks current taks (runs others) until true
	while(Serial2_available() < length){
		osDelay(1);
	}
}

/*
 * BASE64 ENCODING STUFF BELOW
 */

uint8_t startingBytes[2][9] = {
		{0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28},
		{0x29, 0x2a, 0x2c, 0x2d, 0x2e, 0x3a, 0x3b, 0x3c, 0x3e}
};
uint8_t altA = 0x3F, altPlus = 0x40;

static uint8_t encodedString[17];
static uint8_t frameString[12];

static uint8_t bitsTo64(uint8_t bits){
	if(bits < 26) return 'A'+bits;
	if(bits < 52) return 'a'+bits-26;
	if(bits < 62) return '0'+bits-52;
	if(bits < 63) return '+';
	if(bits < 64) return '/';
	return '^'; //input invalid
}

static void replaceBadChars(uint length){
	for(int j=0; j<length; j++){
		if(encodedString[j] == 'A'){
			encodedString[j] = altA;
		}else if(encodedString[j] == '+'){
			encodedString[j] = altPlus;
		}
	}
}

static uint base64encode(uint8_t bytes){
	int i=0;
	for(;i<bytes/3; i++){
		encodedString[4*i+1] = bitsTo64((frameString[3*i+0] >> 2) & 0x3f);
		encodedString[4*i+2] = bitsTo64((frameString[3*i+0] << 4 | frameString[3*i+1] >> 4) & 0x3f);
		encodedString[4*i+3] = bitsTo64((frameString[3*i+1] << 2 | frameString[3*i+2] >> 6) & 0x3f);
		encodedString[4*i+4] = bitsTo64((frameString[3*i+2]) & 0x3f);
	}
	uint8_t pad = bytes%3; //the number of bytes in the last triplet
	if(pad == 1){
		encodedString[4*i+1] = bitsTo64((frameString[3*i+0] >> 2) & 0x3f);
		encodedString[4*i+2] = bitsTo64((frameString[3*i+0] << 4) & 0x3f);
		encodedString[4*i+3] = '='; encodedString[4*i+4] = '=';
	}else if(pad == 2){
		encodedString[4*i+1] = bitsTo64((frameString[3*i+0] >> 2) & 0x3f);
		encodedString[4*i+2] = bitsTo64((frameString[3*i+0] << 4 | frameString[3*i+1] >> 4) & 0x3f);
		encodedString[4*i+3] = bitsTo64((frameString[3*i+1] << 2) & 0x3f);
		encodedString[4*i+4] = '=';
	}
	replaceBadChars(4*i+(pad?5:1));
	return 4*i+(pad?5:1);
}

void frameToBase64(Can_frame_t *newFrame){
	encodedString[0] = startingBytes[newFrame->isExt?1:0][newFrame->dlc];
	if(newFrame->isExt){
		frameString[0] = (newFrame->id >> 21) & 0xff;
		frameString[1] = (newFrame->id >> 13) & 0xff;
		frameString[2] = (newFrame->id >> 5) & 0xff;
		frameString[3] = (newFrame->id << 3) & 0xff;
		for(int i=0; i<newFrame->dlc; i++){
			frameString[3+i] |= newFrame->Data[i] >> 5;
			frameString[4+i] = newFrame->Data[i] << 3;
		}
		Serial1_writeBytes(encodedString, base64encode(4+newFrame->dlc));
		Serial2_writeBytes(encodedString, base64encode(4+newFrame->dlc));
//		Serial2_writeBuf("printed frame\n");
	}else{
		frameString[0] = (newFrame->id >> 3) & 0xff;
		frameString[1] = (newFrame->id << 5) & 0xff;
		for(int i=0; i<newFrame->dlc; i++){
			frameString[1+i] |= newFrame->Data[i] >> 3;
			frameString[2+i] = newFrame->Data[i] << 5;
		}
		Serial1_writeBytes(encodedString, base64encode(2+newFrame->dlc));
		Serial2_writeBytes(encodedString, base64encode(2+newFrame->dlc));
//		Serial2_writeBuf("printed frame\n");
	}
}

/*
 * BASE64 DECODING STUFF BELOW
 */

static uint8_t decodedString[12];

static uint8_t b64ToBits(uint8_t b64){
	if(b64 == '=') return 64;
	if(b64 >= 'A' && b64 <= 'Z') return b64-'A';
	if(b64 >= 'a' && b64 <= 'z') return b64-'a'+26;
	if(b64 >= '0' && b64 <= '9') return b64-'0'+52;
	if(b64 == '+') return 62;
	if(b64 == '/') return 63;
	return 0xff; //input invalid
}

static uint8_t unreplace(uint8_t x){
	if(x == altA) return 'A';
	if(x == altPlus) return '+';
	return x;
}

static uint8_t isStartingByte(uint8_t x){
	for(uint8_t i=0; i<2; i++){
		for(uint8_t j=0; i<9; j++){
			if(x == startingBytes[i][j]){
				return 1;
			}
		}
	}
	return 0;
}

static uint8_t decodeString(uint8_t clusters){
	uint8_t padding = 0;
	for(uint8_t i=0; i<clusters; i++){ //processed in fours so incomplete frames don't block IO
		waitTilAvail(4);
		uint8_t tempchar;
		tempchar = b64ToBits(unreplace(Serial2_peek()));
		if(tempchar < 64){
			decodedString[3*i+0] = tempchar << 2;
			Serial2_read();
		}else{return 0xff;}
		tempchar = b64ToBits(unreplace(Serial2_peek()));
		if(tempchar < 64){
			decodedString[3*i+0] |= tempchar >> 4;
			decodedString[3*i+1] = tempchar << 4;
			Serial2_read();
		}else{return 0xff;}
		tempchar = b64ToBits(unreplace(Serial2_peek()));
		if(tempchar < ((i==clusters-1)?65:64)){
			if(tempchar == 64){
				padding++; tempchar = 0;
			}
			decodedString[3*i+1] |= tempchar >> 2;
			decodedString[3*i+2] = tempchar << 6;
			Serial2_read();
		}else{return 0xff;}
		tempchar = b64ToBits(unreplace(Serial2_peek()));
		if(tempchar < ((i==clusters-1)?65:64)){
			if(tempchar == 64){
				padding++; tempchar = 0;
			}
			decodedString[3*i+2] |= tempchar;
			Serial2_read();
		}else{return 0xff;}
	}
	return padding;
}

//this function packs decodedString into newFrame, returns 1 if successful
uint8_t packFrame(uint8_t ide, uint8_t dlc, Can_frame_t *newFrame){
	if(ide){
		newFrame->id = decodedString[0]<<21 | decodedString[1] << 13 |
				decodedString[2] << 5 | decodedString[3] >> 3;
		if(newFrame->id > 0x1FFFFFFF) return 0; //in retrospect, this step is unneccesary.
		for(int i=0; i<dlc; i++){
			newFrame->Data[i] = decodedString[3+i] << 5;
			newFrame->Data[i] |= decodedString[4+i] >> 3;
		}
	}else{
		newFrame->id = decodedString[0] << 3 | decodedString[1] >> 5;
		if(newFrame->id > 0x7FF) return 0; //in retrospect, this step is unneccesary.
		for(int i=0; i<dlc; i++){
			newFrame->Data[i] = decodedString[1+i] << 3;
			newFrame->Data[i] |= decodedString[2+i] >> 5;
		}
	}
	return 1;
}

//this function is the entry and does all the error checking, returns 1 if successful
uint8_t parseFrame(uint8_t ide, uint8_t dlc, Can_frame_t *newFrame){
	//calculate stuffs
	uint8_t expectedLength = (ide?29:11)+dlc*8;
	uint8_t clusters = expectedLength/24+((expectedLength%24)?1:0);
	//decode string (checks for erroneous characters and stuff)
	uint8_t padding = decodeString(clusters);
	if(padding == 0xff) return 0;
	//check padding length error
	if(padding != 2-((expectedLength-1)%24)/8) return 0; //wolfram alpha: 2-floor(((x-1)mod24)/8)
	//check for extra bits in last byte
	if(decodedString[(expectedLength-1)/8] & (0x7f >> (expectedLength-1)%8)) return 0;
	//actually parse frame (checks if id out of range and stuff)
	if(!packFrame(ide, dlc, newFrame)) return 0;
	//check more characters on stream
	if(Serial2_available() && !isStartingByte(Serial2_peek())) return 0;
	//finally, send: handled in calling function
	return 1;
}
