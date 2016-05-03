
#include <stdbool.h>
#include <stdint.h>

#include "hal_spi.h"
#include "hal_serial.h"
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "nordic_common.h"
#include "nrf_drv_common.h"
#include "nrf_gpio.h"
#include "nrf_assert.h"
#include "app_util_platform.h"
#include "ADXL362_drv.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"

uint8_t ADXL362_TX_BUFFER[8];
uint8_t ADXL362_RX_BUFFER[8];

/* Check value to block application program execution */
bool volatile timer_evt_called = false;

/* Create a rtv instance */
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0);

/* Create a timer config */
const nrf_drv_rtc_config_t RTC_cfg = {
    .prescaler          = 0,
    .interrupt_priority = RTC0_CONFIG_IRQ_PRIORITY,
    .tick_latency       = RTC_US_TO_TICKS(NRF_MAXIMUM_LATENCY_US,RTC0_CONFIG_FREQUENCY),
    .reliable           = RTC0_CONFIG_RELIABLE,
};

/* Timer event handler */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        timer_evt_called = true;
    }
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

void rtc_delay(uint32_t time_delay){
    ret_code_t err_code;
    timer_evt_called = false;
    /* Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds */
    err_code = nrf_drv_rtc_cc_set(&rtc,0,time_delay,true);
    APP_ERROR_CHECK(err_code);
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

static const hal_serial_cfg_t serial_cfg =
{
    .spi0.psel.sck  = SCK_PIN,
    .spi0.psel.mosi = MOSI_PIN,
    .spi0.psel.miso = MISO_PIN,
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

/* ADXL362_INT_PIN vent handler */
void ADXL362_int_pin_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // If PIN is high
    if(nrf_gpio_pin_read(ADXL362_INT_PIN))
    {
        __WFE();
        __SEV();
        __WFE();
    }
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

int main(void)
{
     uint32_t scb_scr;

	 ADXL362_init();

	 /* Initialize SS pin */
     NRF_GPIO->OUTSET = (1 << SS_PIN);
     NRF_GPIO->DIRSET = (1 << SS_PIN);

    hal_serial_init(&serial_cfg);
    hal_spi_init();

    (void)hal_spi_open(HAL_SPI_ID_SPI0, &hal_spi_cfg);

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

    (void)hal_spi_close(HAL_SPI_ID_SPI0);

	/* Set pin pull */
    ADXL362_int_pin_cfg.pull = NRF_GPIO_PIN_PULLDOWN;   //Interrupt is active high

    /* Allow interrupts to fire in equal interrupt priority ISR's */
    scb_scr = SCB->SCR;
    SCB->SCR = scb_scr | SCB_SCR_SEVONPEND_Msk;

    /* Prevent interrupts to fire in equal interrupt priority ISR's */
    //SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;

    /* Init GPIOTE module */
    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }
    /* Init GPIOTE pin */
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(ADXL362_INT_PIN,
                                      &ADXL362_int_pin_cfg,
                                      ADXL362_int_pin_event_handler));
    /* Eneable toggle event and interrupt on pin */
    nrf_drv_gpiote_in_event_enable(ADXL362_INT_PIN, true);

    while (true)
    {
        // Do Nothing
    }
}
