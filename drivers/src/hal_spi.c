#include "hal_spi.h"
#include "hal_serial.h"

#include "nrf.h"
#include <stdlib.h>
#include <stdbool.h>


typedef struct
{
    hal_spi_sig_callback_t  current_sig_callback;
    struct
    {
        uint8_t * tx;
        uint8_t * rx;

        uint8_t   trx_index;
        uint8_t   trx_length;
    } current_buffer;
} hal_spi_t;

#ifdef SYS_CFG_USE_SPI0
static hal_spi_t hal_spi0;
#endif

#ifdef SYS_CFG_USE_SPI1
static hal_spi_t hal_spi1;
#endif


static uint32_t trx_start(NRF_SPI_Type * spi, hal_spi_t * context, uint32_t length, uint8_t * tx_buffer, uint8_t * rx_buffer);
static void hal_spi_isr_handler(NRF_SPI_Type * spi, hal_spi_t * context);


void hal_spi_init(void)
{
#ifdef SYS_CFG_USE_SPI0
    hal_spi0.current_sig_callback = NULL;
#endif
#ifdef SYS_CFG_USE_SPI1
    hal_spi1.current_sig_callback = NULL;
#endif
}

uint32_t hal_spi_open(hal_spi_id_t id, hal_spi_cfg_t const * const cfg)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_SPI0
        case HAL_SPI_ID_SPI0:
            if ( hal_serial_id_acquire(HAL_SERIAL_ID_SPI0) )
            {
                NRF_SPI0->INTENCLR = 0xFFFFFFFF;
                hal_spi0.current_sig_callback = NULL;
                
                NRF_SPI0->CONFIG    = cfg->config;
                NRF_SPI0->FREQUENCY = cfg->frequency;
                
                return ( HAL_SPI_STATUS_CODE_SUCCESS );
            }
            break;
#endif
#ifdef SYS_CFG_USE_SPI1
        case HAL_SPI_ID_SPI1:
            if ( hal_serial_id_acquire(HAL_SERIAL_ID_SPI1) )
            {
                NRF_SPI0->INTENCLR = 0xFFFFFFFF;
                hal_spi1.current_sig_callback = NULL;

                NRF_SPI1->CONFIG    = cfg->config;
                NRF_SPI1->FREQUENCY = cfg->frequency;
                
                return ( HAL_SPI_STATUS_CODE_SUCCESS );
            }
            break;
#endif
        default:
            break;
    }
    
    return ( HAL_SPI_STATUS_CODE_DISALLOWED );
}

uint32_t hal_spi_close(hal_spi_id_t id)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_SPI0
        case HAL_SPI_ID_SPI0:
            return ( (hal_serial_id_release(HAL_SERIAL_ID_SPI0)) ? HAL_SPI_STATUS_CODE_SUCCESS : HAL_SPI_STATUS_CODE_DISALLOWED );
#endif
#ifdef SYS_CFG_USE_SPI1
        case HAL_SPI_ID_SPI1:
            return ( (hal_serial_id_release(HAL_SERIAL_ID_SPI1)) ? HAL_SPI_STATUS_CODE_SUCCESS : HAL_SPI_STATUS_CODE_DISALLOWED );
#endif
        default:
            break;
    }
    
    return ( HAL_SPI_STATUS_CODE_DISALLOWED );
}

void hal_spi_callback_set(hal_spi_id_t id, hal_spi_sig_callback_t hal_spi_sig_callback)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_SPI0
        case HAL_SPI_ID_SPI0:
            hal_spi0.current_sig_callback = hal_spi_sig_callback;
            NRF_SPI0->INTENCLR = 0xFFFFFFFF;
            break;
#endif
#ifdef SYS_CFG_USE_SPI1
        case HAL_SPI_ID_SPI1:
            hal_spi1.current_sig_callback = hal_spi_sig_callback;
            NRF_SPI1->INTENCLR = 0xFFFFFFFF;
            break;
#endif
        default:
            break;
    }
}


uint32_t hal_spi_trx(hal_spi_id_t id, uint32_t length, uint8_t * tx_buffer, uint8_t * rx_buffer)
{
    // ASSERT( length > 0 );
    switch ( id )
    {
#ifdef SYS_CFG_USE_SPI0
        case HAL_SPI_ID_SPI0:
            return ( trx_start(NRF_SPI0, &hal_spi0, length, tx_buffer, rx_buffer) );
#endif
#ifdef SYS_CFG_USE_SPI1
        case HAL_SPI_ID_SPI1:
            return ( trx_start(NRF_SPI1, &hal_spi1, length, tx_buffer, rx_buffer) );
#endif
        default:
            return ( HAL_SPI_STATUS_CODE_DISALLOWED );
    }
}


static __inline uint8_t next_txd_get(hal_spi_t * context)
{
    return ( (context->current_buffer.tx != NULL) ? context->current_buffer.tx[context->current_buffer.trx_index] : 0x00 );
}

static uint32_t trx_start(NRF_SPI_Type * spi, hal_spi_t * context, uint32_t length, uint8_t * tx_buffer, uint8_t * rx_buffer)
{
        
    context->current_buffer.tx = tx_buffer;
    context->current_buffer.rx = rx_buffer;
    context->current_buffer.trx_index = 0;
    context->current_buffer.trx_length = length;
    
    spi->EVENTS_READY = 0;
    if ( context->current_sig_callback != NULL )
    {
        spi->INTENSET = (SPI_INTENSET_READY_Enabled << SPI_INTENSET_READY_Pos);
        spi->TXD = next_txd_get(context);
    }
    else
    {
        spi->INTENCLR = (SPI_INTENCLR_READY_Enabled << SPI_INTENCLR_READY_Pos);
        spi->TXD = next_txd_get(context);

        hal_spi_isr_handler(spi, context);
    }
    
    return ( HAL_SPI_STATUS_CODE_SUCCESS );
}


static void hal_spi_isr_handler(NRF_SPI_Type * spi, hal_spi_t * context)
{
    bool done = (context->current_sig_callback != NULL);

    do
    {
        if ( spi->EVENTS_READY != 0 )
        {
            uint8_t rxd = spi->RXD; // The HW requires the RXD register to always be read before a new transaction can start.

            spi->EVENTS_READY = 0;
            
            if ( context->current_buffer.rx != NULL )
            {
                context->current_buffer.rx[context->current_buffer.trx_index] = rxd;
            }
            ++context->current_buffer.trx_index;
            if ( context->current_buffer.trx_index < context->current_buffer.trx_length )
            {
                spi->TXD = next_txd_get(context);
            }
            else
            {
                if ( context->current_sig_callback != NULL )
                {
                    context->current_sig_callback(HAL_SPI_SIGNAL_TYPE_TRX_COMPLETE);
                }
                done = true;
            }
        }
    } while ( !done );
}


#ifdef SYS_CFG_USE_SPI0
void hal_serial_spi0_isr_handler(void)
{
    hal_spi_isr_handler(NRF_SPI0, &hal_spi0);
}
#endif


#ifdef SYS_CFG_USE_SPI1
void hal_serial_spi1_isr_handler(void)
{
    hal_spi_isr_handler(NRF_SPI1, &hal_spi1);
}
#endif
