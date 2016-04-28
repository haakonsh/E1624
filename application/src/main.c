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
 * This file is an application which purpose is to count steps and advertise  *
 * it with the BLE radio. It uses timer0 to track the number of times lpcomp  *
 * triggers an event, using ppi. It also utilizes an ADXL362 accellerometer   *
 * to detect motion, and put the nRF52 to sleep in the abcense of it. The     *
 * ADXL362 is programmed via a proprietary spi driver made by Hans Elfberg.   *
 * The ADXL362_drv was made by Analog Devices, ported to nRF52 and expanded   *
 * with debugging tools. BLE advertisement is done via a proprietary radio	  *
 * driver also made by Hans Elfberg.										  *
 * Made by Håkon S.Holdhus and Lars J. Hammervold							  *
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
    0xE2, 0xA3, 0x01, 0xE7, 0x61, 0xF7, 0x02, 0x01, 0x04, 0x1A, 0xFF, 0x59, 0x00, 0x02, 0x15, 0x01, 0x12, 0x23,
    0x34, 0xFF, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF, 0xF0, 0x01, 0xDE, 0xAD, 0xBE, 0xEF
};
/************************************************************************************************/

/*********************************** variable declarations **************************************/
volatile uint32_t Counter = 0;
bool volatile timer_evt_called = false;
nrf_ppi_channel_t ppi_channel1;

bool volatile radio_isr_called;
uint32_t time_us;
uint32_t interval_us = 1000000;
/************************************************************************************************/

/*************************************** Instantiations *****************************************/
const nrf_drv_timer_t timer0 	= NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_rtc_t rtc 		= NRF_DRV_RTC_INSTANCE(0);
/************************************************************************************************/

/****************************************** Configs *********************************************/
const nrf_drv_timer_config_t timer0_config = {
	.frequency			= NRF_TIMER_FREQ_31250Hz,
	.mode				= NRF_TIMER_MODE_LOW_POWER_COUNTER,
	.bit_width			= NRF_TIMER_BIT_WIDTH_16,
	.interrupt_priority = TIMER0_CONFIG_IRQ_PRIORITY,
	.p_context			= NULL
};

nrf_drv_lpcomp_config_t lpcomp_config = {
	.hal                = {.reference = NRF_LPCOMP_REF_SUPPLY_3_8, .detection = NRF_LPCOMP_DETECT_UP},
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

/**************************************** Event/ISR handlers ****************************************/
void RADIO_IRQHandler(void)
{
    NRF_RADIO->EVENTS_DISABLED = 0;
    radio_isr_called = true;
}

void gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // If PIN is high
    if(nrf_gpio_pin_read(ADXL362_INT_PIN))
    {
		NRF_RTC0->TASKS_STOP;
        __WFE();
        __SEV();
        __WFE();
		NRF_RTC0->TASKS_START;
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

void lpcomp_event_handler(nrf_lpcomp_event_t event){};
lpcomp_events_handler_t p_lpcomp_event_handler = lpcomp_event_handler;
/************************************************************************************************/

/*************************************** Intitializations ***************************************/
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
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

void rtc_init(void)
{
	/* Start the internal LFCLK XTAL oscillator */
    lfclk_config();
    /* Initialize RTC driver */
    APP_ERROR_CHECK(nrf_drv_rtc_init(&rtc, NULL, rtc_handler));
    /* Enable tick event & interrupt */
    nrf_drv_rtc_tick_enable(&rtc,true);
}

void ADXL362_init(void)
{
	/* Initialize RTC */
    rtc_init();

    /* Drain decoupling capacitors */
    NRF_GPIO->OUTCLR = (1 << VDD_PIN);
    NRF_GPIO->DIRSET = (1 << VDD_PIN);
	rtc_delay(CHIP_RESET_TIME);
	/* Power up ADXL362 */
	NRF_GPIO->OUTSET = (1 << VDD_PIN);
	rtc_delay(STARTUP_TIME);
	/*  */
	nrf_drv_rtc_uninit(&rtc);
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
	/* Eneable toggle event and interrupt on pin */
	nrf_drv_gpiote_in_event_enable(ADXL362_INT_PIN, true);
}
/************************************************************************************************/

/************************************** Utility functions ***************************************/
void rtc_delay(uint32_t time_delay){
    timer_evt_called = false;
    /* Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds */
    APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc,0,time_delay,true));
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
void rtc_delay_ms(uint32_t timeout_us){
    timer_evt_called = false;
	uint32_t rtc_units;
    uint32_t us_units;

    m_convert(timeout_us, &rtc_units, &us_units);
    (void)us_units;

    /* Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds */
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

void temperature_measurement(void)
{
	uint32_t temp;
	NRF_TEMP->TASKS_START = 1;
	while(!NRF_TEMP->EVENTS_DATARDY)
	{
		__WFE();
		__SEV();
		__WFE();
	}
	NRF_TEMP->EVENTS_DATARDY = 0;
	NRF_TEMP->TASKS_STOP = 1;

	temp = (NRF_TEMP->TEMP / 4);
	adv_pdu[TEMP_OFFS + 3] = temp;
	adv_pdu[TEMP_OFFS + 2] = (temp >> 8);
	adv_pdu[TEMP_OFFS + 1] = (temp >> 16);
	adv_pdu[TEMP_OFFS + 0] = (temp >> 24);
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
	ADXL362_SetupINTMAP2(ADXL362_INTMAP2_AWAKE);

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

static void beacon_handler(void)
{
    hal_radio_reset();
    hal_timer_start();

    uicr_bd_addr_set();

    time_us = INITIAL_TIMEOUT - LFCLK_STARTUP_TIME_US;

    do
    {
        timer_evt_called = false;
        rtc_delay_ms(time_us);
        /* Disable gpiote from executing ADXL362_int_pin_event_handler(); */
        nrf_drv_gpiote_in_event_disable(ADXL362_INT_PIN);

        get_number_of_steps();

        hal_clock_hfclk_enable();

        timer_evt_called = false;
        time_us = LFCLK_STARTUP_TIME_US;
        rtc_delay_ms(time_us);

		/* Read temperature and put it into tx buffer */
		//temperature_measurement();

        send_one_packet(37);
        send_one_packet(38);
        send_one_packet(39);

        hal_clock_hfclk_disable();

        time_us = interval_us - LFCLK_STARTUP_TIME_US;
        /* Enable gpiote from executing ADXL362_int_pin_event_handler(); */
        nrf_drv_gpiote_in_event_enable(ADXL362_INT_PIN, true);
    } while ( 1 );
}
/************************************************************************************************/

int main(void)
{
	uint32_t scb_scr;

	ppi_init();

	APP_ERROR_CHECK(nrf_drv_timer_init(&timer0, &timer0_config, timer_event_handler));

	APP_ERROR_CHECK(nrf_drv_lpcomp_init(&lpcomp_config, lpcomp_event_handler));
	NRF_LPCOMP->HYST = COMP_HYST_HYST_Hyst50mV;
	nrf_drv_lpcomp_enable();

	/* Set interrupt pin pull */
	//ADXL362_int_pin_cfg.pull = NRF_GPIO_PIN_PULLDOWN;   //Interrupt is active high

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

	/* Allow interrupts to fire in equal interrupt priority ISR's */
	scb_scr = SCB->SCR;
	SCB->SCR = scb_scr | SCB_SCR_SEVONPEND_Msk;

	/* Prevent interrupts to fire in equal interrupt priority ISR's */
	//SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;

	gpiote_init();

    while (true)
    {
		beacon_handler();
    }
}
