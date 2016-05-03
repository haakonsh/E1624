#include "hal_serial.h"

#include "nrf.h"

#ifdef NRF51
#define SERIAL0_IRQn SPI0_TWI0_IRQn
#define SERIAL0_IRQHandler SPI0_TWI0_IRQHandler

#define SERIAL1_IRQn SPI1_TWI1_IRQn
#define SERIAL1_IRQHandler SPI1_TWI1_IRQHandler
#endif

#ifdef NRF52
#define SERIAL0_IRQn SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn
#define SERIAL0_IRQHandler SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler

#define SERIAL1_IRQn SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn
#define SERIAL1_IRQHandler SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler
#endif



typedef void (*hal_serial_irq_handler_t) (void);

struct
{
    hal_serial_cfg_t const * cfg;
    hal_serial_id_t          current_id[2];
    hal_serial_irq_handler_t current_handler[2];
} hal_serial;

void hal_serial_init(hal_serial_cfg_t const * const cfg)
{
    hal_serial.cfg = cfg;

#if defined(SYS_CFG_USE_SPI0) || defined(SYS_CFG_USE_TWI0)
    NVIC_DisableIRQ(SERIAL0_IRQn);
    NRF_SPI0->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
    NVIC_SetPriority(SERIAL0_IRQn, SYS_CFG_SERIAL_0_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(SERIAL0_IRQn);
    NVIC_EnableIRQ(SERIAL0_IRQn);
#endif
#if defined(SYS_CFG_USE_SPI1) || defined(SYS_CFG_USE_TWI1)
    NVIC_DisableIRQ(SERIAL1_IRQn);
    NRF_SPI1->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
    NVIC_SetPriority(SERIAL1_IRQn, SYS_CFG_SERIAL_1_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(SERIAL1_IRQn);
    NVIC_EnableIRQ(SERIAL1_IRQn);
#endif
}


bool hal_serial_id_acquire(hal_serial_id_t id)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_TWI0
        case HAL_SERIAL_ID_TWI0:
            if ( hal_serial.current_id[0] == HAL_SERIAL_ID_NONE )
            {
                NRF_TWI0->PSELSCL = hal_serial.cfg->twi0.psel.scl;
                NRF_TWI0->PSELSDA = hal_serial.cfg->twi0.psel.sda;
                NRF_TWI0->ENABLE = (TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos);
                hal_serial.current_id[0] = id;
                hal_serial.current_handler[0] = hal_serial_twi0_isr_handler;
            
                return ( true );
            }
            break;
#endif
#ifdef SYS_CFG_USE_SPI0
        case HAL_SERIAL_ID_SPI0:
            if ( hal_serial.current_id[0] == HAL_SERIAL_ID_NONE )
            {
                NRF_SPI0->PSELMOSI = hal_serial.cfg->spi0.psel.mosi;
                NRF_SPI0->PSELMISO = hal_serial.cfg->spi0.psel.miso;
                NRF_SPI0->PSELSCK  = hal_serial.cfg->spi0.psel.sck;
                NRF_SPI0->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
                hal_serial.current_id[0] = id;
                hal_serial.current_handler[0] = hal_serial_spi0_isr_handler;

                return ( true );
            }
            break;
#endif
#ifdef SYS_CFG_USE_TWI1
        case HAL_SERIAL_ID_TWI1:
            if ( hal_serial.current_id[1] == HAL_SERIAL_ID_NONE )
            {
                NRF_TWI1->PSELSCL = hal_serial.cfg->twi1.psel.scl;
                NRF_TWI1->PSELSDA = hal_serial.cfg->twi1.psel.sda;
                NRF_TWI1->ENABLE = (TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos);
                hal_serial.current_id[1] = id;
                hal_serial.current_handler[1] = hal_serial_twi1_isr_handler;
        
                return ( true );
            }
            break;
#endif
#ifdef SYS_CFG_USE_SPI1
        case HAL_SERIAL_ID_SPI1:
            if ( hal_serial.current_id[1] == HAL_SERIAL_ID_NONE )
            {
                NRF_SPI1->PSELMOSI = hal_serial.cfg->spi1.psel.mosi;
                NRF_SPI1->PSELMISO = hal_serial.cfg->spi1.psel.miso;
                NRF_SPI1->PSELSCK  = hal_serial.cfg->spi1.psel.sck;
                NRF_SPI1->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
                hal_serial.current_id[1] = id;
                hal_serial.current_handler[1] = hal_serial_spi1_isr_handler;

                return ( true );
            }
            break;
#endif
        default:
            break;
    }
    
    return ( false );
}

bool hal_serial_id_release(hal_serial_id_t id)
{
    switch ( id )
    {
#ifdef SYS_CFG_USE_TWI0
        case HAL_SERIAL_ID_TWI0:
            if ( hal_serial.current_id[0] == id )
            {
                NRF_TWI0->ENABLE = (TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos);
                hal_serial.current_id[0] = HAL_SERIAL_ID_NONE;

                return ( true );
            }
            break;
#endif
#ifdef SYS_CFG_USE_SPI0
        case HAL_SERIAL_ID_SPI0:
            if ( hal_serial.current_id[0] == id )
            {
                NRF_SPI0->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
                hal_serial.current_id[0] = HAL_SERIAL_ID_NONE;

                return ( true );
            }
            break;
#endif
#ifdef SYS_CFG_USE_TWI1
        case HAL_SERIAL_ID_TWI1:
            if ( hal_serial.current_id[1] == id )
            {
                NRF_TWI1->ENABLE = (TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos);
                hal_serial.current_id[1] = HAL_SERIAL_ID_NONE;

                return ( true );
            }
            break;
#endif
#ifdef SYS_CFG_USE_SPI1
        case HAL_SERIAL_ID_SPI1:
            if ( hal_serial.current_id[1] == id )
            {
                NRF_SPI1->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
                hal_serial.current_id[1] = HAL_SERIAL_ID_NONE;

                return ( true );
            }
            break;
#endif
        default:
            break;
    }
    
    return ( false );
}


#if defined(SYS_CFG_USE_SPI0) || defined(SYS_CFG_USE_TWI0)
void SERIAL0_IRQHandler(void)
{
    hal_serial.current_handler[0]();
}
#endif

#if defined(SYS_CFG_USE_SPI1) || defined(SYS_CFG_USE_TWI1)
void SERIAL1_IRQHandler(void)
{
    hal_serial.current_handler[1]();
}
#endif
