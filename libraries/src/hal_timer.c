#include "hal_timer.h"
#include "hal_clock.h"
#include "nrf.h"

static void m_convert(uint32_t time_us, uint32_t *rtc_units, uint32_t *us_units)
{
   uint32_t u1, u2, t1, t2, t3;

   t1 = time_us / 15625;
   u1 = t1 * 512;
   t2 = time_us - t1 * 15625;
   u2 = (t2 << 9) / 15625;
   t3 = (((t2 << 9) - u2 * 15625) + 256) >> 9;

   *rtc_units = u1 + u2;
   *us_units = t3;
}

void hal_timer_start(void)
{
    hal_clock_lfclk_enable();

    NVIC_ClearPendingIRQ(RTC0_IRQn);
    NVIC_EnableIRQ(RTC0_IRQn);

    NRF_RTC0->TASKS_CLEAR = 1;
    NRF_RTC0->TASKS_START = 1;
}


void hal_timer_timeout_set(uint32_t timeout_us)
{
    uint32_t rtc_units;
    uint32_t us_units;
    
    m_convert(timeout_us, &rtc_units, &us_units);
    (void)us_units;

    NRF_RTC0->CC[0]    = rtc_units;
    NRF_RTC0->EVTENSET = (RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos);
    NRF_RTC0->INTENSET = (RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos);
}
