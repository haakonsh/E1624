#ifndef __MAIN_H__
#define __MAIN_H__
#endif

#include "stdio.h"
#include "nrf_drv_rtc.h"

/********************** Debugging tools, UART and ERRORS **********************/
//#define DEBUG_WITH_UART       // Debug ADXL362 with UART (Segger RTT)
//#define DEBUG_WITH_ERRORS     // Debug ADXL362 with ERRORS (app_error_handler)
/******************************************************************************/

/* Event adress get function does not work */
#define NRF_LPCOMP_EVENT_UP_address 0x40013108UL

#define ADXL_ERROR_BASE_NUM      (0x4000)       ///< ADXL362 error base

#define ADXL362_REGISTER_WRITE_FAILED	(ADXL_ERROR_BASE_NUM + 0)  ///< Failed to program register

/********************************** Pin map **********************************/
#define SCK_PIN				  	11
#define MOSI_PIN			  	12
#define MISO_PIN			  	5
#define SS_PIN				  	15
#define VDD_PIN					14
#define ADXL362_INT_PIN 		17
#define LPCOMP_PIN				NRF_LPCOMP_INPUT_3
/******************************************************************************/

/********************************** ADXL362 ***********************************/
#define ADXL_ORC         	    0xFF   	// OverReadCharacter
#define CHIP_RESET_TIME  		1000	// timeout in multiples of ~30uS, 1000
#define STARTUP_TIME  			15		// timeout in multiples of ~30uS
#define ADXL362_FIFO_SIZE     	260    	// # samples in ADXL fifo, 0-511
#define ADXL362_FIFO_TEMP     	0      	// 0: skip temp read 1: store temp in fifo
#define ADXL362_ACT_RefAbs    	1      	// 0: absolute 1: reference
#define ADXL362_ACT_THRESH    	100   	// 11 bit unsigned int
#define ADXL362_ACT_TIME      	1		// (# of seconds * Hz)
#define ADXL362_INACT_RefAbs  	1      	// 0: absolute 1: reference
#define ADXL362_INACT_THRESH  	100   	// 11 bit unsigned int
#define ADXL362_INACT_TIME    	1     	// (# of seconds * Hz)
#define ADXL362_ODR           	ADXL362_ODR_12_5_HZ  // Output data rate
/******************************************************************************/

extern uint8_t ADXL362_TX_BUFFER[8];
extern uint8_t ADXL362_RX_BUFFER[8];
extern uint32_t ADXL362_BUFFER_LENGTH;
extern const nrf_drv_rtc_t rtc;
extern bool volatile timer_evt_called;
/*
*	Low power blocking delay fucnction.
*	parameter time_delay, is multiples of 30uS.
*/
void rtc_delay(uint32_t time_delay);
