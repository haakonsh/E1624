/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
*
* @defgroup ppi_example_main main.c
* @{
* @ingroup ppi_example
* @brief PPI Example Application main file.
*
* This file contains the source code for a sample application using PPI to communicate between timers.
*
*/
#include "SEGGER_RTT.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_delay.h"
#include "nrf_drv_lpcomp.h"
#include "app_error.h"
//#include "boards.h" 
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nordic_common.h"
#include "nrf_drv_config.h"     
#include "app_util_platform.h"

#define NRF_LPCOMP_EVENT_UP_address 0x40013108UL

volatile uint32_t Counter = 10;

const nrf_drv_timer_t timer0 = NRF_DRV_TIMER_INSTANCE(0);
nrf_ppi_channel_t ppi_channel1;


const nrf_drv_timer_config_t timer0_config = {
	.frequency					= NRF_TIMER_FREQ_31250Hz,
	.mode								= NRF_TIMER_MODE_LOW_POWER_COUNTER,
	.bit_width					= NRF_TIMER_BIT_WIDTH_16,
	.interrupt_priority = TIMER0_CONFIG_IRQ_PRIORITY,
	.p_context					= NULL
};

void timer_event_handler(nrf_timer_event_t event_type, void * p_context){}

//nrf_lpcomp_config_t lpcomp_hal = {
//	.reference =  NRF_LPCOMP_REF_EXT_REF0,
//	.detection =  NRF_LPCOMP_DETECT_UP 
// };

nrf_drv_lpcomp_config_t lpcomp_config = {
   .hal                = {.reference = NRF_LPCOMP_REF_SUPPLY_3_8, .detection = NRF_LPCOMP_DETECT_UP}, 						 
   .input              = NRF_LPCOMP_INPUT_3,	
	 .interrupt_priority = LPCOMP_CONFIG_IRQ_PRIORITY
 };
	// Timer even handler. 
	//Not used since timer is used only for PPI.


void lpcomp_event_handler(nrf_lpcomp_event_t event){
		
				nrf_timer_task_trigger(
																NRF_TIMER0,
																NRF_TIMER_TASK_CAPTURE0);
			
				Counter = nrf_drv_timer_capture_get(
																				&timer0,
																				NRF_TIMER_CC_CHANNEL0);
				SEGGER_RTT_printf(0,"Counter = %u\n",Counter);
	
}
lpcomp_events_handler_t p_lpcomp_event_handler = lpcomp_event_handler;

	
/** @brief Function for initializing the PPI peripheral.
*/
static void ppi_init(void)
{
    uint32_t err_code = NRF_SUCCESS;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    // Configure 1st available PPI channel to stop TIMER0 counter on TIMER1 COMPARE[0] match, which is every even number of seconds.
    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel1);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(ppi_channel1,
                                         NRF_LPCOMP_EVENT_UP_address,
                                         nrf_drv_timer_task_address_get(&timer0, NRF_TIMER_TASK_COUNT));
	
    APP_ERROR_CHECK(err_code);

    // Enable both configured PPI channels
    err_code = nrf_drv_ppi_channel_enable(ppi_channel1);
    APP_ERROR_CHECK(err_code);

}
/** @brief Function for Timer 0 initialization, which will be started and stopped by timer1 and timer2 using PPI.
*/
static void timer0_init(void)
{
    ret_code_t err_code = nrf_drv_timer_init(&timer0, &timer0_config, timer_event_handler);
    APP_ERROR_CHECK(err_code);
		NRF_LPCOMP->HYST = COMP_HYST_HYST_Hyst50mV;
}

/**
 * @brief Function for application main entry.
 */

int main(void)
{
		timer0_init(); // Timer used to blink the LEDs.

		ret_code_t err_code;
		err_code = nrf_drv_lpcomp_init(&lpcomp_config,
														 lpcomp_event_handler);
		APP_ERROR_CHECK(err_code);
		nrf_drv_lpcomp_enable();
		
		
		SEGGER_RTT_WriteString(0,"Hello Lars!\n");


    ppi_init();    // PPI to redirect the event to timer start/stop tasks.


					

  
    while (true)
    {
				
    }
}


/** @} */