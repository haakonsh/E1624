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

/* ************************************************************************** *
 * This file is an application whose purpose is to count steps and advertise  *
 * it with the BLE radio. It uses timer0 to track the number of times lpcomp  *
 * sets an event via ppi. It also utilizes an ADXL362 accellerometer          *
 * to detect motion, and put the nRF52 to sleep in the absence of it. The     *
 * ADXL362 is programmed via a proprietary spi driver made by Hans Elfberg.   *
 * The ADXL362_drv was made by Analog Devices, ported to nRF52 and expanded   *
 * with debugging tools. BLE advertisement is done via a proprietary radio	  *
 * driver also made by Hans Elfberg.										  *
 * Made by Håkon S.Holdhus							                          *
 * ************************************************************************** */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf_drv_lpcomp.h"
#include "app_error.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nordic_common.h"
#include "nrf_drv_config.h"
#include "app_util_platform.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include "hal_spi.h"
#include "hal_serial.h"
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "nrf_drv_common.h"
#include "nrf_gpio.h"
#include "nrf_assert.h"
#include "ADXL362_drv.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "hal_radio.h"
#include "hal_timer.h"
#include "hal_clock.h"
#include "hal_power.h"

/************************************ Buffer declarations ***************************************/
uint8_t ADXL362_TX_BUFFER[8];
uint8_t ADXL362_RX_BUFFER[8];
static uint8_t adv_pdu[36 + 3] =
{
    0x42, 0x24, 0x00,
    0xE2, 0xA3, 0x01, 0xE7, 0x61, 0xF7, 0x02, 0x01, 0x04, 0x1A, 0xFF, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF
};
/************************************************************************************************/

/*********************************** variable declarations **************************************/
volatile uint32_t Counter = 0;
bool volatile timer_evt_called = false;
nrf_ppi_channel_t ppi_channel1;

bool volatile radio_isr_called;
uint32_t time_us;
volatile uint32_t scb_scr;
/************************************************************************************************/

/*************************************** Instantiations *****************************************/
const nrf_drv_timer_t timer0 	= NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_rtc_t rtc 		= NRF_DRV_RTC_INSTANCE(0);
/************************************************************************************************/

/****************************************** Configs *********************************************/
const nrf_drv_timer_config_t timer_config = {
	.frequency			= NRF_TIMER_FREQ_31250Hz,
	.mode				= NRF_TIMER_MODE_LOW_POWER_COUNTER,
	.bit_width			= NRF_TIMER_BIT_WIDTH_16,
	.interrupt_priority = TIMER0_CONFIG_IRQ_PRIORITY,
	.p_context			= NULL
};

nrf_drv_lpcomp_config_t lpcomp_config = {
	.hal                = {.reference = NRF_LPCOMP_CONFIG_REF_EXT_REF1, .detection = NRF_LPCOMP_DETECT_UP},
	.input              = NRF_LPCOMP_INPUT_3,
	.interrupt_priority = LPCOMP_CONFIG_IRQ_PRIORITY
};

const nrf_drv_rtc_config_t RTC_cfg = {
    .prescaler          = 0,
    .interrupt_priority = RTC0_CONFIG_IRQ_PRIORITY,
    .tick_latency       = RTC_US_TO_TICKS(NRF_MAXIMUM_LATENCY_US,RTC0_CONFIG_FREQUENCY),
    .reliable           = RTC0_CONFIG_RELIABLE,
};

static const hal_serial_cfg_t serial_cfg =
{
    .spi0.psel.sck  	= SCK_PIN,
    .spi0.psel.mosi 	= MOSI_PIN,
    .spi0.psel.miso 	= MISO_PIN,
};

static const hal_spi_cfg_t hal_spi_cfg =
{
    .config =
    (
        (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos) |
        (SPI_CONFIG_CPHA_Leading    << SPI_CONFIG_CPHA_Pos) |
        (SPI_CONFIG_ORDER_MsbFirst  << SPI_CONFIG_ORDER_Pos)
    ),
    .frequency = (SPI_FREQUENCY_FREQUENCY_M8 << SPI_FREQUENCY_FREQUENCY_Pos)
};

/* ADXL362_INT_PIN configuration */
nrf_drv_gpiote_in_config_t ADXL362_int_pin_cfg = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
/************************************************************************************************/

/*************************************** Intitializations ***************************************/
void gpio_init(void){
    nrf_gpio_cfg	(	VDD_PIN,
                        NRF_GPIO_PIN_DIR_OUTPUT,
                        NRF_GPIO_PIN_INPUT_DISCONNECT,
                        NRF_GPIO_PIN_NOPULL,
                        NRF_GPIO_PIN_H0H1,
                        NRF_GPIO_PIN_NOSENSE
                    );
    nrf_gpio_cfg	(	SS_PIN,
                        NRF_GPIO_PIN_DIR_OUTPUT,
                        NRF_GPIO_PIN_INPUT_DISCONNECT,
                        NRF_GPIO_PIN_NOPULL,
                        NRF_GPIO_PIN_H0H1,
                        NRF_GPIO_PIN_NOSENSE
                    );
    nrf_gpio_cfg	(	MISO_PIN,
                        NRF_GPIO_PIN_DIR_INPUT,
                        NRF_GPIO_PIN_INPUT_CONNECT,
                        NRF_GPIO_PIN_NOPULL,
                        NRF_GPIO_PIN_S0S1,
                        NRF_GPIO_PIN_NOSENSE
                    );
    nrf_gpio_cfg	(	MOSI_PIN,
                        NRF_GPIO_PIN_DIR_OUTPUT,
                        NRF_GPIO_PIN_INPUT_DISCONNECT,
                        NRF_GPIO_PIN_NOPULL,
                        NRF_GPIO_PIN_H0H1,
                        NRF_GPIO_PIN_NOSENSE
                    );
    nrf_gpio_cfg	(	SCK_PIN,
                        NRF_GPIO_PIN_DIR_OUTPUT,
                        NRF_GPIO_PIN_INPUT_DISCONNECT,
                        NRF_GPIO_PIN_NOPULL,
                        NRF_GPIO_PIN_H0H1,
                        NRF_GPIO_PIN_NOSENSE
                    );
    // nrf_gpio_cfg	(	DEBUG_PIN,
    //                     NRF_GPIO_PIN_DIR_OUTPUT,
    //                     NRF_GPIO_PIN_INPUT_DISCONNECT,
    //                     NRF_GPIO_PIN_NOPULL,
    //                     NRF_GPIO_PIN_H0H1,
    //                     NRF_GPIO_PIN_NOSENSE
    //                 );
}

void gpio_reset_programming_pins(void){
    nrf_gpio_cfg_default(SS_PIN);
    nrf_gpio_cfg_default(MISO_PIN);
    nrf_gpio_cfg_default(MOSI_PIN);
    nrf_gpio_cfg_default(SCK_PIN);
}

static void ppi_init(void)
{
	/* Initialize ppi */
    APP_ERROR_CHECK(nrf_drv_ppi_init());

    /* Configure 1st available PPI channel to increment TIMER0 counter on NRF_LPCOMP_EVENT_UP event */
    APP_ERROR_CHECK(nrf_drv_ppi_channel_alloc(&ppi_channel1));
    APP_ERROR_CHECK(nrf_drv_ppi_channel_assign(ppi_channel1,
                                         NRF_LPCOMP_EVENT_UP_address,
                                         nrf_drv_timer_task_address_get(&timer0, NRF_TIMER_TASK_COUNT)));

    /* Enable the configured PPI channel */
    APP_ERROR_CHECK(nrf_drv_ppi_channel_enable(ppi_channel1));
}

/* Function starting the internal LFCLK XTAL oscillator */
static void lfclk_config(void)
{
        /* Allow interrupts to fire in equal interrupt priority ISR's */
    scb_scr = SCB->SCR;
    SCB->SCR = scb_scr | SCB_SCR_SEVONPEND_Msk;

    /* Start LFCLK (32kHz) crystal oscillator. If you don't have crystal on your board, choose RCOSC instead */
    NRF_CLOCK->LFCLKSRC = ((CLOCK_CONFIG_LF_SRC << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;

    NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);

    nrf_clock_int_enable(NRF_CLOCK_INT_LF_STARTED_MASK);
    NRF_CLOCK->INTENSET = 1;

    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    while (!NRF_CLOCK->EVENTS_LFCLKSTARTED)
    {
        __WFE();
        __SEV();
        __WFE();
    }

    NRF_CLOCK->INTENCLR = 1;

    /* Prevent interrupts to fire in equal interrupt priority ISR's */
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
}

void rtc_init(void)
{
	/* Start the internal LFCLK XTAL oscillator */
    lfclk_config();
    /* Initialize RTC driver */
    APP_ERROR_CHECK(nrf_drv_rtc_init(&rtc, &RTC_cfg, rtc_handler));
}

void ADXL362_init(void)
{
    /* Drain decoupling capacitors */
    NRF_GPIO->DIRSET = (1 << VDD_PIN);
    NRF_GPIO->OUTCLR = (1 << VDD_PIN);
	rtc_delay_us(CHIP_RESET_TIME);
	/* Power up ADXL362 */
	NRF_GPIO->OUTSET = (1 << VDD_PIN);
	rtc_delay_us(STARTUP_TIME);
}

void gpiote_init(void)
{
	/* Init GPIOTE module */
	if(!nrf_drv_gpiote_is_init())
	{
		APP_ERROR_CHECK(nrf_drv_gpiote_init());
	}
	/* Init GPIOTE pin */
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(ADXL362_INT_PIN,
	                             &ADXL362_int_pin_cfg,
	                             gpiote_handler));
}
/************************************************************************************************/

/************************************** Utility functions ***************************************/

void rtc_delay_us(uint32_t timeout_us){
    timer_evt_called = false;
	uint32_t rtc_units;
    uint32_t us_units;

    m_convert(timeout_us, &rtc_units, &us_units);
    (void)us_units;

    /* Set compare channel to Set interrupt after COMPARE_COUNTERTIME */
    APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc,0,rtc_units,true));
    /* Reset the COUNTER register */
    nrf_drv_rtc_counter_clear(&rtc);
    /* Power on RTC instance */
    nrf_drv_rtc_enable(&rtc);
    /* Check to block application program execution */
    do
    {
		__WFE();
		__SEV();
        __WFE();
    }
	while(!timer_evt_called);
    /* Stop the RTC */
	nrf_drv_rtc_disable(&rtc);
}

void get_number_of_steps(void)
{
    /* Read #steps and put it into tx buffer */
    uint16_t steps = 0;
    nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_CAPTURE0);
    steps = nrf_drv_timer_capture_get(&timer0, NRF_TIMER_CC_CHANNEL0);
    adv_pdu[STEPS_OFFS + 1] = steps;
    adv_pdu[STEPS_OFFS + 0] = (steps >> 8);
}

void get_temperature(void)
{
    uint8_t temp;
    uint32_t temp_32;
    /* Allow interrupts to fire in equal interrupt priority ISR's */
    scb_scr = SCB->SCR;
    SCB->SCR = scb_scr | SCB_SCR_SEVONPEND_Msk;

    NRF_TEMP->EVENTS_DATARDY = 0;
    NVIC_ClearPendingIRQ(TEMP_IRQn);
    NRF_TEMP->INTENSET = 1;
	NRF_TEMP->TASKS_START = 1;
	while(!NRF_TEMP->EVENTS_DATARDY)
	{
		__WFE();
		__SEV();
		__WFE();
	}
    NRF_TEMP->TASKS_STOP = 1;
    NRF_TEMP->INTENCLR = 1;

    temp_32 = (NRF_TEMP->TEMP / 4);
    temp = (temp_32 >> 24) | (temp_32 & 0x000000FF);
	adv_pdu[TEMP_OFFS] = temp;

    /* Prevent interrupts to fire in equal interrupt priority ISR's */
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
}

void ADXL362_motiondetect_cfg(void)
{
	/*! Find ADXL362 on SPI bus*/
	ADXL362_Init();

	/*! Reset all registers*/
	ADXL362_SoftwareReset();

	/*! Configure activity detection. */
	ADXL362_SetupActivityDetection(ADXL362_ACT_RefAbs,
	                                  ADXL362_ACT_THRESH,
	                                  ADXL362_ACT_TIME);

	/*! Configure inactivity detection. */
	ADXL362_SetupInactivityDetection(ADXL362_INACT_RefAbs,
	                                   ADXL362_INACT_THRESH,
	                                   ADXL362_INACT_TIME);

	/*! Configure activity/inactivity link mode*/
	ADXL362_SetupActivityInactivityLinkLoop(ADXL362_ACT_INACT_CTL_LINKLOOP(ADXL362_MODE_LOOP));

	/*! Configure the INTMAP1. */
	//ADXL362_SetupINTMAP1(ADXL362_INTMAP1_INT_LOW | ADXL362_INTMAP1_AWAKE);
	ADXL362_SetupINTMAP1(ADXL362_INTMAP1_AWAKE);

	/*! Configure the INTMAP2. */
	//ADXL362_SetupINTMAP2(ADXL362_INTMAP2_AWAKE);

	/*! Configure the measurement range. */
	ADXL362_SetRange(ADXL362_RANGE_2G);

	/*! Configure the Output Data Rate of the device. */
	ADXL362_SetOutputRate(ADXL362_ODR);

	/* Configure Power Control, starts measurements */
	ADXL362_SetPowerMode(ADXL362_POWER_CTL_WAKEUP);
	ADXL362_SetPowerMode(ADXL362_POWER_CTL_AUTOSLEEP);
	ADXL362_SetPowerMode(ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON));
}

static void send_one_packet(uint8_t channel_index)
{
    uint8_t i;

    radio_isr_called = false;
    hal_radio_channel_index_set(channel_index);
    hal_radio_send(adv_pdu);
    while ( !radio_isr_called )
    {
        __WFE();
        __SEV();
        __WFE();
    }

    for ( i = 0; i < 9; i++ )
    {
        __NOP();
    }
}

static void uicr_bd_addr_set(void)
{
    if ( ( NRF_FICR->DEVICEADDR[0]           != 0xFFFFFFFF)
    ||   ((NRF_FICR->DEVICEADDR[1] & 0xFFFF) != 0xFFFF) )
    {
        adv_pdu[BD_ADDR_OFFS    ] = (NRF_FICR->DEVICEADDR[0]      ) & 0xFF;
        adv_pdu[BD_ADDR_OFFS + 1] = (NRF_FICR->DEVICEADDR[0] >>  8) & 0xFF;
        adv_pdu[BD_ADDR_OFFS + 2] = (NRF_FICR->DEVICEADDR[0] >> 16) & 0xFF;
        adv_pdu[BD_ADDR_OFFS + 3] = (NRF_FICR->DEVICEADDR[0] >> 24)       ;
        adv_pdu[BD_ADDR_OFFS + 4] = (NRF_FICR->DEVICEADDR[1]      ) & 0xFF;
        adv_pdu[BD_ADDR_OFFS + 5] = (NRF_FICR->DEVICEADDR[1] >>  8) & 0xFF;
    }
}

/************************************************************************************************/

/************************************** Event/ISR handlers **************************************/
void RADIO_IRQHandler(void)
{
    NRF_RADIO->EVENTS_DISABLED = 0;
    radio_isr_called = true;
}

void gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // The driver toggles pin sense before the handler is called. A NRF_GPIO_PIN_SENSE_HIGH
    // means the gpiote triggered the port event on a sense low-to-high transition.

    nrf_gpio_pin_sense_t sense = nrf_gpio_pin_sense_get(ADXL362_INT_PIN);
	if(sense == NRF_GPIO_PIN_SENSE_HIGH)
	{
        NRF_RTC0->TASKS_START = 0;
		NRF_RTC0->TASKS_STOP = 1;
	}
	else if (sense == NRF_GPIO_PIN_SENSE_LOW)
	{
		NRF_RTC0->TASKS_STOP = 0;
		NRF_RTC0->TASKS_START = 1;
	}
}

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        timer_evt_called = true;
    }
}

void timer_event_handler(nrf_timer_event_t event_type, void * p_context){}

void lpcomp_event_handler(nrf_lpcomp_event_t event){}
lpcomp_events_handler_t p_lpcomp_event_handler = lpcomp_event_handler;
/**********************************************************************************************/

/************************************ Application handler**************************************/
static void application_handler(void)
{
    hal_radio_reset();

    uicr_bd_addr_set();

    time_us = INITIAL_TIMEOUT;

    do
    {
        /* Enable gpiote from executing gpiote_handler(); */
        nrf_drv_gpiote_in_event_enable(ADXL362_INT_PIN, false);

        rtc_delay_us(time_us);
        /* Disable gpiote from executing gpiote_handler(); */
        nrf_drv_gpiote_in_event_disable(ADXL362_INT_PIN);

        get_number_of_steps();

        hal_clock_hfclk_enable();

        time_us = HFXO_STARTUP_TIME_US;
        rtc_delay_us(time_us);

		/* Read temperature and put it into tx buffer */
		get_temperature();

        send_one_packet(37);
        send_one_packet(38);
        send_one_packet(39);

        hal_clock_hfclk_disable();

        time_us = INTERVAL_US - HFXO_STARTUP_TIME_US;
    } while ( 1 );
}
/*********************************************************************************************/
int main(void)
{
    /* Use DC-DC regulator */
    NRF_POWER->DCDCEN = 1;

    /* Initialize RTC */
    rtc_init();

    /* configure programming pins as outputs with high drive strengths */
    gpio_init();

    /* ADXL362 startup sequence */
    ADXL362_init();

    /* Initialize SS pin */
    NRF_GPIO->OUTSET = (1 << SS_PIN);
    NRF_GPIO->DIRSET = (1 << SS_PIN);

    hal_serial_init(&serial_cfg);
    hal_spi_init();

    (void)hal_spi_open(HAL_SPI_ID_SPI0, &hal_spi_cfg);
    ADXL362_motiondetect_cfg();
    (void)hal_spi_close(HAL_SPI_ID_SPI0);

    gpio_reset_programming_pins();

	ppi_init();

	APP_ERROR_CHECK(nrf_drv_timer_init(&timer0, &timer_config, timer_event_handler));

	APP_ERROR_CHECK(nrf_drv_lpcomp_init(&lpcomp_config, lpcomp_event_handler));
	NRF_LPCOMP->HYST = COMP_HYST_HYST_Hyst50mV;
	nrf_drv_lpcomp_enable();

	gpiote_init();

    while (true)
    {
		application_handler();
    }
}
