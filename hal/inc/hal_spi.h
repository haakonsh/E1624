#ifndef HAL_SPI_H__
#define HAL_SPI_H__

#include <stdbool.h>
#include <stdint.h>


/**@brief The callback signals of the driver.
 */
typedef enum
{
    HAL_SPI_SIGNAL_TYPE_TRX_COMPLETE,   ///< Sent when the transfer is complete.
} hal_spi_signal_type_t;


/**@brief The bit IDs.
 */
typedef enum
{
    HAL_SPI_ID_SPI0,
    HAL_SPI_ID_SPI1,
    HAL_SPI_ID_NONE,
} hal_spi_id_t;


/**@brief The SPI status codes.
 */
enum
{
    HAL_SPI_STATUS_CODE_SUCCESS,     ///< Successfull.
    HAL_SPI_STATUS_CODE_TRX_ERROR,   ///< Transfer failed.
    HAL_SPI_STATUS_CODE_DISALLOWED,  ///< Disallowed.
};


/**@brief The spi configuration.
 */
typedef struct
{
    uint32_t config;
    uint32_t frequency;
} hal_spi_cfg_t;


/**@brief Opens the driver to access the specified HW peripheral.
 *
 * @param{in] id    The id of the HW peripheral to open the driver for.
 * @param{in] cfg   The driver configuration.
 *
 * @retval ::HAL_SPI_STATUS_CODE_SUCCESS    if successful.
 * @retval ::HAL_SPI_STATUS_CODE_DISALLOWED if the driver could not be opened.
 */
uint32_t hal_spi_open(hal_spi_id_t id, hal_spi_cfg_t const * const cfg);


/**@brief The type of the signal callback conveying signals from the driver.
 */
typedef void (*hal_spi_sig_callback_t) (hal_spi_signal_type_t hal_spi_signal_type);


/**@brief Initializes the spi interface.
 */
void hal_spi_init(void);


/**@brief Sets the callback function.
 *
 * @nore Transfering data will be blocking calls if no callback is set.
 *
 * @param{in] id                    The id of the HW peripheral to set the callback for.
 * @param{in] hal_spi_sig_callback  The signal callback function, or NULL if not used.
 */
void hal_spi_callback_set(hal_spi_id_t id, hal_spi_sig_callback_t hal_spi_sig_callback);


/**@brief transfers bytes to a device.
 *
 * @note The buffers shall be available to the driver until the traansfer has completed.
 *
 * @param{in]   id          The id of the HW peripheral to use for the transfer.
 * @param[in]   length      The number of bytes to transmit/receive.
 * @param{ut]   tx_buffer   The transmit buffer to use.
 * @param{[out] rx_buffer   The receive buffer to use.
 *
 * @retval ::HAL_SPI_STATUS_CODE_SUCCESS    if successful.
 * @retval ::HAL_SPI_STATUS_CODE_TRC_FAILED if the buffers could not be transferred.
 */
uint32_t hal_spi_trx(hal_spi_id_t id, uint32_t length, uint8_t * tx_buffer, uint8_t * rx_buffer);


/**@brief Closes the specified driver.
 *
 * @param{in] id    The id of the HW peripheral to close the driver for.
 *
 * @retval ::HAL_SPI_STATUS_CODE_SUCCESS    if successful.
 * @retval ::HAL_SPI_STATUS_CODE_DISALLOWED if the driver could not be closed.
 */
uint32_t hal_spi_close(hal_spi_id_t id);


#endif // HAL_SPI_H__
