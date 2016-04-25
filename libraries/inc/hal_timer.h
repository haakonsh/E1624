#ifndef HAL_TIMER_H__
#define HAL_TIMER_H__

#include <stdint.h>


void hal_timer_start(void);

void hal_timer_timeout_set(uint32_t timeout_us);

#endif // HAL_TIMER_H__
