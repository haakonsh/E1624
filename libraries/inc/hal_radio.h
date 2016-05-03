#ifndef HAL_RADIO_H__
#define HAL_RADIO_H__

#include <stdint.h>

void hal_radio_channel_index_set(uint8_t channel_index);
void hal_radio_reset(void);

void hal_radio_send(uint8_t *data);

#endif // HAL_RADIO_H__
