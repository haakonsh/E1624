#ifndef __MAIN_H__
#define __MAIN_H__
#endif

#include "stdio.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"

/********************** Debugging tools, UART and ERRORS **********************/
//#define DEBUG_WITH_UART       // Debug ADXL362 with UART (Segger RTT)
//#define DEBUG_WITH_ERRORS     // Debug ADXL362 with ERRORS (app_error_handler)
#define ADXL_ERROR_BASE_NUM      (0x4000)       ///< ADXL362 error base
#define ADXL362_REGISTER_WRITE_FAILED	(ADXL_ERROR_BASE_NUM + 0)
/******************************************************************************/
/* Event adress get function does not work */
#define NRF_LPCOMP_EVENT_UP_address 0x40013108UL
/********************************** Pin map **********************************/
#define SCK_PIN				  	11
#define MOSI_PIN			  	12
#define MISO_PIN			  	6
#define SS_PIN				  	15
#define VDD_PIN					14
#define ADXL362_INT_PIN 		17
#define LPCOMP_PIN				NRF_LPCOMP_INPUT_3
/******************************************************************************/

/********************************** ADXL362 ***********************************/
#define ADXL_ORC         	    0xFF   	// OverReadCharacter
#define CHIP_RESET_TIME  		2000	// timeout in multiples of ~30uS, (1000)
#define STARTUP_TIME  			500		// timeout in multiples of ~30uS  (15)
#define ADXL362_FIFO_SIZE     	260    	// # samples in ADXL fifo, 0-511
#define ADXL362_FIFO_TEMP     	0      	// 0: skip temp read 1: store temp in fifo
#define ADXL362_ACT_RefAbs    	1      	// 0: absolute 1: reference
#define ADXL362_ACT_THRESH    	100   	// 11 bit unsigned int
#define ADXL362_ACT_TIME      	1		// (# of seconds * Hz)
#define ADXL362_INACT_RefAbs  	1      	// 0: absolute 1: reference
#define ADXL362_INACT_THRESH  	50   	// 11 bit unsigned int
#define ADXL362_INACT_TIME    	3     	// (# of seconds * Hz)
#define ADXL362_ODR           	ADXL362_ODR_12_5_HZ  // Output data rate
/******************************************************************************/

/*********************************** Radio ************************************/
#define BD_ADDR_OFFS			3
#define TEMP_OFFS				16
#define STEPS_OFFS				18
#define INITIAL_TIMEOUT         500
#define HFXO_STARTUP_TIME_US    400
#define INTERVAL_US				1000000
/*******************************************************************************/

/******************************** Misc ************************************/
#define CHARGING_INTERVAL		3000000
/*******************************************************************************/

/******************************** Variables ************************************/
extern uint8_t ADXL362_TX_BUFFER[8];
extern uint8_t ADXL362_RX_BUFFER[8];
extern uint32_t ADXL362_BUFFER_LENGTH;
extern const nrf_drv_rtc_t rtc;
extern bool volatile timer_evt_called;
void rtc_delay(uint32_t time_delay);
/*******************************************************************************/

/************************** Function declarations ******************************/
static void rtc_handler(nrf_drv_rtc_int_type_t int_type);
void gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void timer_event_handler1(nrf_timer_event_t event_type, void * p_context);
void timer_event_handler2(nrf_timer_event_t event_type, void * p_context);
void rtc_delay_us(uint32_t timeout_us);
/*******************************************************************************/
