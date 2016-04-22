#ifndef HAL_SERIAL_H__
#define HAL_SERIAL_H__

#if !(defined(SYS_CFG_USE_SPI0) || defined(SYS_CFG_USE_TWI0) || defined(SYS_CFG_USE_SPI1) || defined(SYS_CFG_USE_TWI1))
#include "sys_cfg.h"
#endif

#include <stdint.h>
#include <stdbool.h>


#ifdef SYS_CFG_USE_SPI0
void hal_serial_spi0_isr_handler(void);
#endif


#ifdef SYS_CFG_USE_SPI1
void hal_serial_spi1_isr_handler(void);
#endif


#ifdef SYS_CFG_USE_TWI0
void hal_serial_twi0_isr_handler(void);
#endif


#ifdef SYS_CFG_USE_TWI1
void hal_serial_twi1_isr_handler(void);
#endif


/**@brief The spi pin configuration.
 */
typedef struct
{
    uint32_t    sck     : 6;
    uint32_t    mosi    : 6;
    uint32_t    miso    : 6;
} hal_serial_spi_psel_cfg_t;


/**@brief The twi pin configuration.
 */
typedef struct
{
    uint32_t    scl : 6;
    uint32_t    sda : 6;
} hal_serial_twi_psel_cfg_t;


/**@brief The serial configuration.
 */
typedef struct
{
#ifdef SYS_CFG_USE_SPI0
    struct
    {
        hal_serial_spi_psel_cfg_t   psel;
    } spi0;
#endif
#ifdef SYS_CFG_USE_SPI1
    struct
    {
        hal_serial_spi_psel_cfg_t   psel;
    } spi1;
#endif
#ifdef SYS_CFG_USE_TWI0
    struct
    {
        hal_serial_twi_psel_cfg_t   psel;
    } twi0;
#endif
#ifdef SYS_CFG_USE_TWI1
    struct
    {
        hal_serial_twi_psel_cfg_t   psel;
    } twi1;
#endif
} hal_serial_cfg_t;


/**@brief The IDs for the serial peripherals.
 */
typedef enum
{
    HAL_SERIAL_ID_NONE,
#ifdef SYS_CFG_USE_TWI0
    HAL_SERIAL_ID_TWI0,
#endif
#ifdef SYS_CFG_USE_SPI0
    HAL_SERIAL_ID_SPI0,
#endif
#ifdef SYS_CFG_USE_TWI1
    HAL_SERIAL_ID_TWI1,
#endif
#ifdef SYS_CFG_USE_SPI1
    HAL_SERIAL_ID_SPI1,
#endif
} hal_serial_id_t;


/**@brief Inits the serial peripheral core.
 */
void hal_serial_init(hal_serial_cfg_t const * const cfg);


/**@brief Acquires a specified peripheral.
 *
 * @param{in] id    The id of the HW peripheral to acquire.
 */
bool hal_serial_id_acquire(hal_serial_id_t id);


/**@brief Releases a previously acquired peripheral identified by the specified ID.
 *
 * @param{in] id    The id of the HW peripheral to release.
 */
bool hal_serial_id_release(hal_serial_id_t id);


#endif // HAL_SERIAL0_H__
