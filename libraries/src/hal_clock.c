#include "hal_clock.h"
#include "nrf.h"


void hal_clock_lfclk_enable(void)
{
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
}


void hal_clock_hfclk_enable(void)
{
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
}


void hal_clock_hfclk_disable()
{
    NRF_CLOCK->TASKS_HFCLKSTOP = 1;
}
