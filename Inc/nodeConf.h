/*
 * nodeConf.h
 *
 *  Created on: Dec 31, 2016
 *      Author: frank
 *  Edit this file for all node configurations!
 */

/*
 * Node configuration instructions:
 * 1. Make sure all parameters in this files is set properly according to node specifications
 * 2. Implement additional application-layer parsing in do
 * 		- main.c (loc "XXX 1")
 * 3. Implement flush queues in executeCommand()
 * 		- nodeMiscHelpers.c (loc "XXX 2" and loc "XXX 3")
 * 4. Suspend any application layer tasks in shutdown command
 * 		- nodeMiscHelpers.c (loc "XXX 4")
 *
 *
 * If you have any additional tasks/queues/mutexes/semaphores, you should try to add them through CubeMX if possible.
 * As always, ensure that there is sufficient FLASH and HEAP!
 *
 * The task priorities should ALWAYS be set as the following:
 * PRIORITY |	TASK
 * 	HIGH	  watchdogRefresh (if applicable)
 * 	 ^	 	  FreeRTOS_Timer
 *   |		  Can_Processor
 *   |---------------------------------------
 *   |		  Application Layer	Tasks		|
 *   |---------------------------------------
 *  LOW		  IdleTask
 *
 *	Deviation from this priority list may result in unstable node behavior and/or timing failure!
 */

#ifndef NODECONF_H_
#define NODECONF_H_

#define HB_Interval		1000		// Node heartbeat send interval	(soft ms)
#define WD_Interval		16			// Watdog timer refresh interval (soft ms) | MUST BE LESS THAN 26!!!

#ifdef FRANK
static const uint32_t firmwareString = 0x00000100;	// v00.00.01.0
static const uint8_t selfNodeID = 6;	// Radio
uint32_t selfStatusWord = 0x0;
#define NODE_CONFIGURED
#elif defined (__JAMES__)
const uint32_t firmwareString = 0x00000100;
const uint8_t selfNodeID = 6;
uint32_t selfStatusWord = 0x0;
#define NODE_CONFIGURED
#else
// SW_Sentinel will fail the CC firmware check and result in node addition failure!
static const uint32_t firmwareString = 0x00000001;			// Firmware Version string
static const uint8_t selfNodeID = radio_nodeID;					// The nodeID of this node
uint32_t selfStatusWord;	// Initialize
#define NODE_CONFIGURED
#endif

#ifndef NODE_CONFIGURED
#error "NODE NOT CONFIGURED. GO CONFIGURE IT IN NODECONF.H!"
#endif

#endif /* NODECONF_H_ */
