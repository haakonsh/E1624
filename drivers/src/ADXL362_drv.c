/***************************************************************************//**
 *   @file   ADXL362.c
 *   @brief  Implementation of ADXL362 Driver.
 *   @author DNechita(Dan.Nechita@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: $WCREV$
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "app_error.h"
#include "hal_spi.h"
#include "ADXL362_drv.h"
#include "main.h"
#include "SEGGER_RTT.h"
#include "nrf_gpio.h"
#include "nrf_drv_rtc.h"
#include "nrf.h"
#include "nordic_common.h"
#include "nrf_drv_common.h"
#include "nrf51_to_nrf52.h"

/******************************************************************************/
/************************* Variables Declarations *****************************/
/******************************************************************************/
char selectedRange = 0;

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Initializes communication with the device and checks if the part is
 *        present by reading the device id.
 *
 * @return  NRF_SUCCESS - the initialization was successful and the device is present;
 *         INVALID_PARID - an error occurred.
*******************************************************************************/
uint32_t ADXL362_BUFFER_LENGTH = 0;

void ADXL362_Init(void)
{
    /* Retry until successful */
    while (ADXL362_RX_BUFFER[2] != ADXL362_PART_ID)
    {
        ADXL362_GetRegisterValue(ADXL362_REG_PARTID, 1);
        #ifdef DEBUG_WITH_UART
            SEGGER_RTT_WriteString(0, "\n********************************************************\n");
            if((ADXL362_RX_BUFFER[2] == ADXL362_PART_ID))
            {
                SEGGER_RTT_WriteString(0, "Found ADXL362\n");        }
            else{SEGGER_RTT_WriteString(0, "#ERROR no device found!\n");}
        #endif
        #ifdef DEBUG_WITH_ERRORS
            ret_code_t err_code;
            /* Error generation*/
            if(ADXL362_RX_BUFFER[2] == ADXL362_PART_ID){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
            /* End of Error handling*/
        #endif
    };

    selectedRange = 2; // Measurement Range: +/- 2g (reset default).
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerValue   - Data value to write.
 * @param registerAddress - Address of the register.
 * @param bytesNumber     - Number of bytes. Accepted values: 0 - 1.
 *
 * @return None.
*******************************************************************************/
void ADXL362_SetRegisterValue(uint16_t registerValue,
                              uint8_t  registerAddress,
                              uint8_t  bytesNumber)
{
    //uint8_t low_byte = (registerValue & 0x00FF);
    //uint8_t hi_byte = (registerValue >> 8);

    ADXL362_TX_BUFFER[0] = ADXL362_WRITE_REG;
    ADXL362_TX_BUFFER[1] = registerAddress;
    ADXL362_TX_BUFFER[2] = (registerValue & 0x00FF);
    ADXL362_TX_BUFFER[3] = (registerValue >> 8);

    ADXL362_BUFFER_LENGTH = 2 + bytesNumber;

    NRF_GPIO->OUTCLR = (1 << SS_PIN); //SS
    (void)hal_spi_trx(  HAL_SPI_ID_SPI0,
                        ADXL362_BUFFER_LENGTH,
                        &(ADXL362_TX_BUFFER[0]),
                        &(ADXL362_RX_BUFFER[0]));
    NRF_GPIO->OUTSET = (1 << SS_PIN); //SS
}

/***************************************************************************//**
 * @brief Performs a burst read of a specified number of registers.
 *
 * @param pReadData       - The read values are stored in this buffer.
 * @param registerAddress - The start address of the burst read.
 * @param bytesNumber     - Number of bytes to read.
 *
 * @return None.
*******************************************************************************/
void ADXL362_GetRegisterValue(uint8_t  registerAddress,
                              uint8_t  bytesNumber)
{

    ADXL362_TX_BUFFER[0] = ADXL362_READ_REG;
    ADXL362_TX_BUFFER[1] = registerAddress;
    /* ADXL362 needs to recieve 0xFF while sending data on MISO */
    for(uint8_t i = 0 ; i < bytesNumber ; i++)
    {
        ADXL362_TX_BUFFER[2 + i] = 0xFF;
    }

    ADXL362_BUFFER_LENGTH = 2 + bytesNumber;

    NRF_GPIO->OUTCLR = (1 << SS_PIN); //SS
    (void)hal_spi_trx(  HAL_SPI_ID_SPI0,
                        ADXL362_BUFFER_LENGTH,
                        &(ADXL362_TX_BUFFER[0]),
                        &(ADXL362_RX_BUFFER[0]));
    NRF_GPIO->OUTSET = (1 << SS_PIN); //SS

}

/***************************************************************************//**
 * @brief Reads multiple bytes from the device's FIFO buffer.
 *
 * @param pBuffer     - Stores the read bytes.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return None.
*******************************************************************************/
void ADXL362_GetFifoValue(uint16_t bytesNumber)
{

    ADXL362_TX_BUFFER[0] = ADXL362_WRITE_FIFO;
    ADXL362_BUFFER_LENGTH = 1 + bytesNumber;

    NRF_GPIO->OUTCLR = (1 << SS_PIN); //SS
    (void)hal_spi_trx(  HAL_SPI_ID_SPI0,
                        ADXL362_BUFFER_LENGTH,
                        &(ADXL362_TX_BUFFER[0]),
                        &(ADXL362_RX_BUFFER[0]));
    NRF_GPIO->OUTSET = (1 << SS_PIN); //SS
}

/***************************************************************************//**
 * @brief Resets the device via SPI communication bus.
 *
 * @return None.
*******************************************************************************/
void ADXL362_SoftwareReset(void)
{
    ADXL362_SetRegisterValue(ADXL362_RESET_KEY, ADXL362_REG_SOFT_RESET, 0);
}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwrMode - Power mode.
 *                  Example: 0 - standby mode.
 *                  1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL362_SetPowerMode(uint8_t pwrMode)
{
    #ifdef DEBUG_WITH_ERRORS
        ret_code_t err_code;
    #endif
    uint8_t oldPowerCtl = 0;
    uint8_t newPowerCtl = 0;
    uint8_t pwrMode_measure_msk   = (0x3 << 0);
    uint8_t pwrMode_low_noise_msk = (0x3 << 4);
    const uint8_t CaseConstant = ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_NORMAL) | LOW_NOISE_identifier;

    switch(pwrMode) {
        case ADXL362_POWER_CTL_RES:
            ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);

            oldPowerCtl = ADXL362_RX_BUFFER[2];
            newPowerCtl = oldPowerCtl & ~pwrMode;
            newPowerCtl |= pwrMode;
            /* Retry until successful */
            while(ADXL362_RX_BUFFER[2] != ((oldPowerCtl & ~pwrMode)| pwrMode))
            {
                ADXL362_SetRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
                ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
                #ifdef DEBUG_WITH_UART
                    /* Debugging with SEGGER_RTT*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode)
                    | pwrMode))
                    {
                      SEGGER_RTT_WriteString(0, "ADXL362_POWER_CTL_RES programmed successfully!\n");
                    }
                    else{ SEGGER_RTT_WriteString(0, "#ERROR failed to program ADXL362_POWER_CTL_RES!\n");}
                #endif
                #ifdef DEBUG_WITH_ERRORS
                    /* Error generation*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode)
                    | pwrMode)){
                        err_code = NRF_SUCCESS;
                    }
                    else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
                #endif
            }
            break;

        case ADXL362_POWER_CTL_EXT_CLK  :
            ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);

            oldPowerCtl = ADXL362_RX_BUFFER[2];
            newPowerCtl = oldPowerCtl & ~pwrMode;
            newPowerCtl |= pwrMode;

            /* Retry until successful */
            while(ADXL362_RX_BUFFER[2] != (( oldPowerCtl & ~pwrMode) | pwrMode))
            {

                ADXL362_SetRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
                ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
                #ifdef DEBUG_WITH_UART
                /* Debugging with SEGGER_RTT*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode)
                    | pwrMode))
                    {
                        SEGGER_RTT_WriteString(0, "ADXL362_POWER_CTL_EXT_CLK programmed successfully!\n");
                    }
                    else{ SEGGER_RTT_WriteString(0, "#ERROR failed to program ADXL362_POWER_CTL_EXT_CLK!\n");}
                #endif
                #ifdef DEBUG_WITH_ERRORS
                    /* Error generation*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode)
                    | pwrMode)){
                        err_code = NRF_SUCCESS;
                    }
                    else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
                #endif
            }
            break;

        case CaseConstant :
            ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);

            oldPowerCtl = ADXL362_RX_BUFFER[2];
            newPowerCtl = oldPowerCtl & ~pwrMode_low_noise_msk;
            newPowerCtl |= ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_NORMAL);
            /* Retry until successful */
            while(ADXL362_RX_BUFFER[2] != (( oldPowerCtl & ~pwrMode_low_noise_msk)))
            {
                ADXL362_SetRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
                ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
                #ifdef DEBUG_WITH_UART
                    /* Debugging with SEGGER_RTT*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode_low_noise_msk)
                    | ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_NORMAL) ))
                    {
                        SEGGER_RTT_WriteString(0, "ADXL362_NOISE_MODE_NORMAL programmed successfully!\n");
                    }
                    else{ SEGGER_RTT_WriteString(0, "#ERROR failed to program ADXL362_NOISE_MODE_NORMAL!\n");}
                #endif
                #ifdef DEBUG_WITH_ERRORS
                    /* Error generation*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode_low_noise_msk)
                    | ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_NORMAL) )){
                        err_code = NRF_SUCCESS;
                    }
                    else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
                #endif
            }
            break;

        case ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_LOW):
            ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);

            oldPowerCtl = ADXL362_RX_BUFFER[2];
            newPowerCtl = oldPowerCtl & ~pwrMode_low_noise_msk;
            newPowerCtl |= pwrMode;
            /* Retry until successful */
            while(ADXL362_RX_BUFFER[2] != ((oldPowerCtl & ~pwrMode_low_noise_msk) | pwrMode ))
            {
                ADXL362_SetRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
                ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
                #ifdef DEBUG_WITH_UART
                    /* Debugging with SEGGER_RTT*/
                    if((ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode_low_noise_msk)
                    | pwrMode )))
                    {
                        SEGGER_RTT_WriteString(0, "ADXL362_NOISE_MODE_LOW programmed successfully!\n");
                    }
                    else{SEGGER_RTT_WriteString(0, "#ERROR failed to program ADXL362_NOISE_MODE_LOW!\n");}
                #endif
                #ifdef DEBUG_WITH_ERRORS
                    /* Error generation*/
                    if((ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode_low_noise_msk)
                    | pwrMode ))){
                        err_code = NRF_SUCCESS;
                    }
                    else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
                #endif
            }
            break;

        case ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_ULTRALOW):
            ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);

            oldPowerCtl = ADXL362_RX_BUFFER[2];
            newPowerCtl = oldPowerCtl & ~pwrMode_low_noise_msk;
            newPowerCtl |= pwrMode;
            /* Retry until successful */
            while(ADXL362_RX_BUFFER[2] != (( oldPowerCtl & ~pwrMode_low_noise_msk) | pwrMode ))
            {
                ADXL362_SetRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
                ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
                #ifdef DEBUG_WITH_UART
                    /* Debugging with SEGGER_RTT*/
                    if((ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode_low_noise_msk)
                    | pwrMode )))
                    {
                        SEGGER_RTT_WriteString(0, "ADXL362_NOISE_MODE_ULTRALOW programmed successfully!\n");
                    }
                    else{ SEGGER_RTT_WriteString(0, "#ERROR failed to program ADXL362_NOISE_MODE_ULTRALOW!\n");}
                #endif
                #ifdef DEBUG_WITH_ERRORS
                    /* Error generation*/
                    if((ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode_low_noise_msk)
                    | pwrMode ))){
                        err_code = NRF_SUCCESS;
                    }
                    else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
                #endif
            }
            break;

        case ADXL362_POWER_CTL_WAKEUP:
            ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
            //TODO Debug!
            oldPowerCtl = ADXL362_RX_BUFFER[2];
            newPowerCtl = oldPowerCtl & ~pwrMode;
            newPowerCtl |= pwrMode;
            /* Retry until successful */
            while(ADXL362_RX_BUFFER[2] != (( oldPowerCtl & ~pwrMode) | pwrMode))
                {
                ADXL362_SetRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
                ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
                ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
                #ifdef DEBUG_WITH_UART
                    /* Debugging with SEGGER_RTT*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~ADXL362_POWER_CTL_WAKEUP) | ADXL362_POWER_CTL_WAKEUP))
                    {
                        SEGGER_RTT_WriteString(0, "ADXL362_POWER_CTL_WAKEUP programmed successfully!\n");
                    }
                    else{ SEGGER_RTT_WriteString(0, "#ERROR failed to program ADXL362_POWER_CTL_WAKEUP!\n");}
                #endif
                #ifdef DEBUG_WITH_ERRORS
                    /* Error generation*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode)
                    | pwrMode)){
                        err_code = NRF_SUCCESS;
                    }
                    else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
                #endif
            }
            break;

        case ADXL362_POWER_CTL_AUTOSLEEP:
            ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);

            oldPowerCtl = ADXL362_RX_BUFFER[2];
            newPowerCtl = oldPowerCtl & ~pwrMode;
            newPowerCtl |= pwrMode;
            /* Retry until successful */
            while(ADXL362_RX_BUFFER[2] != (( oldPowerCtl & ~pwrMode) | pwrMode))
            {
                ADXL362_SetRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
                ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
                #ifdef DEBUG_WITH_UART
                    /* Debugging with SEGGER_RTT*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode)
                    | pwrMode))
                    {
                    SEGGER_RTT_WriteString(0, "ADXL362_POWER_CTL_AUTOSLEEP programmed successfully!\n");
                    }
                    else{ SEGGER_RTT_WriteString(0, "#ERROR failed to program ADXL362_POWER_CTL_AUTOSLEEP!\n");}
                #endif
                #ifdef DEBUG_WITH_ERRORS
                    /* Error generation*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode)
                    | pwrMode)){
                        err_code = NRF_SUCCESS;
                    }
                    else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
                #endif
            }
            break;

        case ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_STANDBY):
            ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);

            oldPowerCtl = ADXL362_RX_BUFFER[2];
            newPowerCtl = oldPowerCtl & ~pwrMode_measure_msk;
            newPowerCtl |= pwrMode;
            /* Retry until successful */
            while(ADXL362_RX_BUFFER[2] != (( oldPowerCtl & ~pwrMode_measure_msk) | pwrMode ))
            {
                ADXL362_SetRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
                ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
                #ifdef DEBUG_WITH_UART
                    /* Debugging with SEGGER_RTT*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode_measure_msk)
                    | pwrMode ))
                    {
                        SEGGER_RTT_WriteString(0, "ADXL362_MEASURE_STANDBY programmed successfully!\n");
                    }
                    else{ SEGGER_RTT_WriteString(0, "#ERROR failed to program ADXL362_MEASURE_STANDBY!\n");}

                #endif
                #ifdef DEBUG_WITH_ERRORS
                    /* Error generation*/
                    if((ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode_measure_msk)
                    | pwrMode ))){
                       err_code = NRF_SUCCESS;
                    }
                    else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
                #endif
            }
            break;

        case ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON):
            ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);

            oldPowerCtl = ADXL362_RX_BUFFER[2];
            newPowerCtl = oldPowerCtl & ~pwrMode_measure_msk;
            newPowerCtl |= pwrMode;
            /* Retry until successful */
            while(ADXL362_RX_BUFFER[2] != (( oldPowerCtl & ~pwrMode_measure_msk) | pwrMode ))
            {
                ADXL362_SetRegisterValue(newPowerCtl, ADXL362_REG_POWER_CTL, 1);
                ADXL362_GetRegisterValue(ADXL362_REG_POWER_CTL, 1);
                #ifdef DEBUG_WITH_UART
                    /* Debugging with SEGGER_RTT*/
                    if(ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode_measure_msk)
                    | pwrMode ))
                    {
                        SEGGER_RTT_WriteString(0, "ADXL362_MEASURE_ON programmed successfully!\n");
                    }
                    else{ SEGGER_RTT_WriteString(0, "#ERROR failed to program ADXL362_MEASURE_ON!\n");}
                #endif
                #ifdef DEBUG_WITH_ERRORS
                    /* Error generation*/
                    if((ADXL362_RX_BUFFER[2] == (( oldPowerCtl & ~pwrMode_measure_msk)
                    | pwrMode ))){
                        err_code = NRF_SUCCESS;
                    }
                    else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
                #endif
            }
            break;
        default :
            #ifdef DEBUG_WITH_UART
                SEGGER_RTT_WriteString(0, "#ERROR invalid pwrMode!\n");
            #endif
            break;
    }
    #ifdef DEBUG_WITH_ERRORS
        APP_ERROR_CHECK(err_code);
    #endif
}

/***************************************************************************//**
 * @brief Selects the filter bandwidth.
 *
 * @param gRange -
 * @return None.
*******************************************************************************/
void ADXL362_SetFilterBandwidth(unsigned char bandwidth)
{
    unsigned char oldFilterCtl = 0;
    unsigned char newFilterCtl = 0;
    unsigned char bandwidth_msk = 0x08;

    ADXL362_GetRegisterValue(ADXL362_REG_FILTER_CTL, 1);

    oldFilterCtl = ADXL362_RX_BUFFER[2];
    newFilterCtl = oldFilterCtl & ~bandwidth_msk;
    newFilterCtl = newFilterCtl | bandwidth;
    /* Retry until successful */
    while(ADXL362_RX_BUFFER[2] != ((oldFilterCtl & ~bandwidth_msk ) | bandwidth))
    {
        ADXL362_SetRegisterValue(newFilterCtl, ADXL362_REG_FILTER_CTL, 1);
        ADXL362_GetRegisterValue(ADXL362_REG_FILTER_CTL, 1);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if(ADXL362_RX_BUFFER[2]  == ((oldFilterCtl & ~bandwidth_msk ) | bandwidth)){
                SEGGER_RTT_WriteString(0, "Filter Bandwidth programmed successfully!\n");
            }
            else{SEGGER_RTT_WriteString(0, "#ERROR failed to program Filter Bandwidth!\n");}
        #endif
        #ifdef DEBUG_WITH_ERRORS
            ret_code_t err_code;
            /* Error generation*/
            if(ADXL362_RX_BUFFER[2]  == ((oldFilterCtl & ~bandwidth_msk ) | bandwidth)){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }
}


/***************************************************************************//**
 * @brief Selects the measurement range.
 *
 * @param gRange - Range option.
 *                  Example: ADXL362_RANGE_2G  -  +-2 g
 *                           ADXL362_RANGE_4G  -  +-4 g
 *                           ADXL362_RANGE_8G  -  +-8 g
 *
 * @return None.
*******************************************************************************/
void ADXL362_SetRange(unsigned char gRange)
{
    unsigned char oldFilterCtl = 0;
    unsigned char newFilterCtl = 0;
    uint8_t gRange_msk = 0xC0;

    ADXL362_GetRegisterValue(ADXL362_REG_FILTER_CTL, 1);

    oldFilterCtl = ADXL362_RX_BUFFER[2];
    newFilterCtl = oldFilterCtl & ~gRange_msk;
    newFilterCtl = newFilterCtl | ADXL362_FILTER_CTL_RANGE(gRange);
    /* Retry until successful */
    while(ADXL362_RX_BUFFER[2] != ((oldFilterCtl & ~gRange_msk) | ADXL362_FILTER_CTL_RANGE(gRange)))
    {
        ADXL362_SetRegisterValue(newFilterCtl, ADXL362_REG_FILTER_CTL, 1);
        selectedRange = (1 << gRange) * 2;
        ADXL362_GetRegisterValue(ADXL362_REG_FILTER_CTL, 1);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if(ADXL362_RX_BUFFER[2] ==
              ((oldFilterCtl & ~gRange_msk) | ADXL362_FILTER_CTL_RANGE(gRange))){
                SEGGER_RTT_WriteString(0, "gRange programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program gRange!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            ret_code_t err_code;
            /* Error generation*/
            if(ADXL362_RX_BUFFER[2] ==
              ((oldFilterCtl & ~gRange_msk) | ADXL362_FILTER_CTL_RANGE(gRange))){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
            /* End of Error handling*/
        #endif
    }
}

/***************************************************************************//**
 * @brief Selects the Output Data Rate of the device.
 *
 * @param outRate - Output Data Rate option.
 *                  Example: ADXL362_ODR_12_5_HZ  -  12.5Hz
 *                           ADXL362_ODR_25_HZ    -  25Hz
 *                           ADXL362_ODR_50_HZ    -  50Hz
 *                           ADXL362_ODR_100_HZ   -  100Hz
 *                           ADXL362_ODR_200_HZ   -  200Hz
 *                           ADXL362_ODR_400_HZ   -  400Hz
 *
 * @return None.
*******************************************************************************/
void ADXL362_SetOutputRate(unsigned char outRate)
{
    unsigned char oldFilterCtl = 0;
    unsigned char newFilterCtl = 0;

    ADXL362_GetRegisterValue(ADXL362_REG_FILTER_CTL, 1);

    oldFilterCtl = ADXL362_RX_BUFFER[2];
    newFilterCtl = oldFilterCtl & ~ADXL362_FILTER_CTL_ODR(0x7);
    newFilterCtl = newFilterCtl | ADXL362_FILTER_CTL_ODR(outRate);
    /* Retry until successful */
    while(ADXL362_RX_BUFFER[2] != ((oldFilterCtl & ~ADXL362_FILTER_CTL_ODR(0x7)) |                            ADXL362_FILTER_CTL_ODR(outRate)))
    {
        ADXL362_SetRegisterValue(newFilterCtl, ADXL362_REG_FILTER_CTL, 1);
        ADXL362_GetRegisterValue(ADXL362_REG_FILTER_CTL, 1);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if(ADXL362_RX_BUFFER[2] ==
              ((oldFilterCtl & ~ADXL362_FILTER_CTL_ODR(0x7)) | ADXL362_FILTER_CTL_ODR(outRate))){
              SEGGER_RTT_WriteString(0, "Output Rate programmed successfully!\n");
            }
            else{
              SEGGER_RTT_WriteString(0, "#ERROR failed to program Output Rate!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            ret_code_t err_code;
            /* Error generation*/
            if(ADXL362_RX_BUFFER[2] ==
              ((oldFilterCtl & ~ADXL362_FILTER_CTL_ODR(0x7)) | ADXL362_FILTER_CTL_ODR(outRate))){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }
}

/***************************************************************************//**
 * @brief Reads the 3-axis raw data from the accelerometer.
 *
 * @param x - Stores the X-axis data(as two's complement).
 * @param y - Stores the Y-axis data(as two's complement).
 * @param z - Stores the Z-axis data(as two's complement).
 *
 * @return None.
*******************************************************************************/
void ADXL362_GetXyz(short* x, short* y, short* z)
{
    ADXL362_GetRegisterValue(ADXL362_REG_XDATA_L, 6);
    *x = ((short)ADXL362_RX_BUFFER[2 + 1] << 8) + ADXL362_RX_BUFFER[2 + 0];
    *y = ((short)ADXL362_RX_BUFFER[2 + 3] << 8) + ADXL362_RX_BUFFER[2 + 2];
    *z = ((short)ADXL362_RX_BUFFER[2 + 5] << 8) + ADXL362_RX_BUFFER[2 + 4];
}

/***************************************************************************//**
 * @brief Reads the 3-axis raw data from the accelerometer and converts it to g.
 *
 * @param x - Stores the X-axis data.
 * @param y - Stores the Y-axis data.
 * @param z - Stores the Z-axis data.
 *
 * @return None.
*******************************************************************************/
void ADXL362_GetGxyz(float* x, float* y, float* z)
{
        ADXL362_GetRegisterValue(ADXL362_REG_XDATA_L, 6);
    *x = ((short)ADXL362_RX_BUFFER[2 + 1] << 8) + ADXL362_RX_BUFFER[2 + 0];
    *x /= (1000 / (selectedRange / 2));
    *y = ((short)ADXL362_RX_BUFFER[2 + 3] << 8) + ADXL362_RX_BUFFER[2 + 2];
    *y /= (1000 / (selectedRange / 2));
    *z = ((short)ADXL362_RX_BUFFER[2 + 5] << 8) + ADXL362_RX_BUFFER[2 + 4];
    *z /= (1000 / (selectedRange / 2));
}

/***************************************************************************//**
 * @brief Reads the temperature of the device.
 *
 * @return tempCelsius - The value of the temperature(degrees Celsius).
*******************************************************************************/
float ADXL362_ReadTemperature(void)
{
    short signedTemp     = 0;
    float tempCelsius    = 0;

    ADXL362_GetRegisterValue(ADXL362_REG_TEMP_L, 2);
    signedTemp = (short)(ADXL362_RX_BUFFER[3] << 8) + ADXL362_RX_BUFFER[2];
    tempCelsius = (float)signedTemp * 0.065;

    return tempCelsius;
}

/***************************************************************************//**
 * @brief Configures the FIFO feature.
 *
 * @param mode         - Mode selection.
 *                       Example: ADXL362_FIFO_DISABLE      -  FIFO is disabled.
 *                                ADXL362_FIFO_OLDEST_SAVED -  Oldest saved mode.
 *                                ADXL362_FIFO_STREAM       -  Stream mode.
 *                                ADXL362_FIFO_TRIGGERED    -  Triggered mode.
 * @param waterMarkLvl - Specifies the number of samples to store in the FIFO.
 * @param enTempRead   - Store Temperature Data to FIFO.
 *                       Example: 1 - temperature data is stored in the FIFO
 *                                    together with x-, y- and x-axis data.
 *                                0 - temperature data is skipped.
 *
 * @return None.
*******************************************************************************/

void ADXL362_FifoSetup(unsigned char  mode,
                       unsigned short waterMarkLvl,
                       unsigned char  enTempRead)
{
    #ifdef DEBUG_WITH_ERRORS
        ret_code_t err_code;
    #endif
    unsigned char writeVal = 0;
    uint16_t AH_msk_16_bit = 0x0100;
    uint16_t FIFO_SAMPLES_msk = 0x00FF;
    uint8_t  FIFO_SAMPLES = 0;
    uint8_t Mode_msk = 0x03;
    uint8_t Temp_msk = 0x04;
    uint8_t AH_msk = 0x08;
    bool AH_bit = 0;

    AH_bit = ((waterMarkLvl & AH_msk_16_bit) >> 5 ) ;

    writeVal = ADXL362_FIFO_CTL_FIFO_MODE(mode) |
               (enTempRead * ADXL362_FIFO_CTL_FIFO_TEMP) |
               (AH_bit * ADXL362_FIFO_CTL_AH);
    FIFO_SAMPLES = (waterMarkLvl & FIFO_SAMPLES_msk) >> 8;

    ADXL362_GetRegisterValue(ADXL362_REG_FIFO_CTL, 1);
    /* Retry until successful */
    while( (( ADXL362_RX_BUFFER[2] & Mode_msk ) != ADXL362_FIFO_CTL_FIFO_MODE( mode ))
        || (( ADXL362_RX_BUFFER[2] & Temp_msk ) != ( enTempRead * ADXL362_FIFO_CTL_FIFO_TEMP ))
        || (( ADXL362_RX_BUFFER[2] & AH_msk )   != AH_bit))
    {
        ADXL362_SetRegisterValue(writeVal, ADXL362_REG_FIFO_CTL, 1);
        ADXL362_GetRegisterValue(ADXL362_REG_FIFO_CTL, 1);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if((ADXL362_RX_BUFFER[2] & Mode_msk) == ADXL362_FIFO_CTL_FIFO_MODE(mode)){
                SEGGER_RTT_WriteString(0, "FIFO_MODE programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program FIFO_MODE!\n");
            }

            if((ADXL362_RX_BUFFER[2] & Temp_msk) == (enTempRead * ADXL362_FIFO_CTL_FIFO_TEMP)){
                SEGGER_RTT_WriteString(0, "FIFO_TEMP programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program FIFO_TEMP!\n");
            }
            if((ADXL362_RX_BUFFER[2] & AH_msk) == AH_bit){
                SEGGER_RTT_WriteString(0, "FIFO_AH programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program FIFO_AH!\n");
            }
            if(((ADXL362_RX_BUFFER[2] & Mode_msk) == ADXL362_FIFO_CTL_FIFO_MODE(mode))
            && ((ADXL362_RX_BUFFER[2] & Temp_msk) == (enTempRead * ADXL362_FIFO_CTL_FIFO_TEMP))
            && ((ADXL362_RX_BUFFER[2] & AH_msk) == AH_bit)){
                SEGGER_RTT_WriteString(0, "FIFO_CONTROL programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program FIFO_CONTROL!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            /* Error generation*/
            if((ADXL362_RX_BUFFER[2] & Mode_msk) == ADXL362_FIFO_CTL_FIFO_MODE(mode)){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);

            if((ADXL362_RX_BUFFER[2] & Temp_msk) == (enTempRead * ADXL362_FIFO_CTL_FIFO_TEMP)){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);

            if((ADXL362_RX_BUFFER[2] & AH_msk) == AH_bit){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);

            if(((ADXL362_RX_BUFFER[2] & Mode_msk) == ADXL362_FIFO_CTL_FIFO_MODE(mode))
            && ((ADXL362_RX_BUFFER[2] & Temp_msk) == (enTempRead * ADXL362_FIFO_CTL_FIFO_TEMP))
            && ((ADXL362_RX_BUFFER[2] & AH_msk) == AH_bit)){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }

    ADXL362_GetRegisterValue(ADXL362_REG_FIFO_SAMPLES, 1);
    /* Retry until successful */
    while((ADXL362_RX_BUFFER[2]) != FIFO_SAMPLES)
    {
        ADXL362_SetRegisterValue(FIFO_SAMPLES, ADXL362_REG_FIFO_SAMPLES, 1);
        ADXL362_GetRegisterValue(ADXL362_REG_FIFO_SAMPLES, 1);

        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if((ADXL362_RX_BUFFER[2]) == FIFO_SAMPLES){
                SEGGER_RTT_WriteString(0, "FIFO_SAMPLES programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program FIFO_SAMPLES!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            /* Error generation*/
            if((ADXL362_RX_BUFFER[2]) == FIFO_SAMPLES){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }
}

/***************************************************************************//**
 * @brief Configures activity detection.
 *
 * @param refOrAbs  - Referenced/Absolute Activity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * @param threshold - 11-bit unsigned value that the adxl362 samples are
 *                    compared to.
 * @param time      - 8-bit value written to the activity timer register. The
 *                    amount of time (in seconds) is: time / ODR, where ODR - is
 *                    the output data rate.
 *
 * @return None.
*******************************************************************************/
void ADXL362_SetupActivityDetection(unsigned char  refOrAbs,
                                    unsigned short threshold,
                                    unsigned char  time)
{
    #ifdef DEBUG_WITH_ERRORS
        ret_code_t err_code ;
    #endif
    unsigned char oldActInactReg = 0;
    unsigned char newActInactReg = 0;

    /* Retry until successful */
    while((ADXL362_RX_BUFFER[2] != (threshold & 0x00FF)) ||
          (ADXL362_RX_BUFFER[3] != ((threshold & 0x0700) >> 8)))
          {
        /* Configure motion threshold. */
        ADXL362_SetRegisterValue((threshold & 0x7FF), ADXL362_REG_THRESH_ACT_L,2);
        ADXL362_GetRegisterValue(ADXL362_REG_THRESH_ACT_L, 2);

        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if((ADXL362_RX_BUFFER[2] == (threshold & 0x00FF)) &&
               (ADXL362_RX_BUFFER[3] == ((threshold & 0x0700) >> 8))){
                SEGGER_RTT_WriteString(0, "Activity Threshold programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program Activity Threshold!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            /* Error generation*/
            if((ADXL362_RX_BUFFER[2] == (threshold & 0x00FF)) &&
               (ADXL362_RX_BUFFER[3] == ((threshold & 0x0700) >> 8))){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }

    /* Retry until successful */
    while(ADXL362_RX_BUFFER[2] != time)
    {
        /* Configure activity timer. */
        ADXL362_SetRegisterValue(time, ADXL362_REG_TIME_ACT, 1);
        ADXL362_GetRegisterValue(ADXL362_REG_TIME_ACT, 1);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if(ADXL362_RX_BUFFER[2] == time){
                SEGGER_RTT_WriteString(0, "Activity Timer programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program Activity Timer!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            /* Error generation*/
            if(ADXL362_RX_BUFFER[2] == time){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }

    ADXL362_GetRegisterValue(ADXL362_REG_ACT_INACT_CTL, 1);
    /* Enable activity interrupt and select a referenced or absolute
       configuration. */
    oldActInactReg = ADXL362_RX_BUFFER[2];
    newActInactReg = oldActInactReg & ~ADXL362_ACT_INACT_CTL_ACT_REF;
    newActInactReg |= ADXL362_ACT_INACT_CTL_ACT_EN |
                     (refOrAbs * ADXL362_ACT_INACT_CTL_ACT_REF);

    /* Retry until successful */
    while(ADXL362_RX_BUFFER[2] != newActInactReg)
    {
        ADXL362_SetRegisterValue(newActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
        ADXL362_GetRegisterValue(ADXL362_REG_ACT_INACT_CTL, 1);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if(ADXL362_RX_BUFFER[2] == newActInactReg){
                SEGGER_RTT_WriteString(0, "Activity Ref/Abs programmed successfully!\n");
                SEGGER_RTT_WriteString(0, "Activity interrupt enabled successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program Activity Detection!\n");
                SEGGER_RTT_WriteString(0, "#ERROR failed to enable Inactivity interrupt!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            /* Error generation*/
            if(ADXL362_RX_BUFFER[2] == newActInactReg){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }
}

/***************************************************************************//**
 * @brief Configures inactivity detection.
 *
 * @param refOrAbs  - Referenced/Absolute Inactivity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * @param threshold - 11-bit unsigned value that the adxl362 samples are
 *                    compared to.
 * @param time      - 16-bit value written to the inactivity timer register. The
 *                    amount of time (in seconds) is: time / ODR, where ODR - is
 *                    the output data rate.
 *
 * @return None.
*******************************************************************************/
void ADXL362_SetupInactivityDetection(unsigned char  refOrAbs,
                                      unsigned short threshold,
                                      unsigned short time)
{
    #ifdef DEBUG_WITH_ERRORS
        ret_code_t err_code;
    #endif
    unsigned char oldActInactReg = 0;
    unsigned char newActInactReg = 0;

    /* Retry until successful */
    while((ADXL362_RX_BUFFER[2] != (threshold & 0x00FF)) ||
          (ADXL362_RX_BUFFER[3] != ((threshold & 0x0700) >> 8)))
    {
        /* Configure motion threshold. */
        ADXL362_SetRegisterValue((threshold & 0x7FF),
                                  ADXL362_REG_THRESH_INACT_L,
                                  2);

        ADXL362_GetRegisterValue(ADXL362_REG_THRESH_INACT_L, 2);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if((ADXL362_RX_BUFFER[2] == (threshold & 0x00FF)) &&
               (ADXL362_RX_BUFFER[3] == ((threshold & 0x0700) >> 8))){
                SEGGER_RTT_WriteString(0, "Inactivity Threshold programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program Inctivity Threshold!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            /* Error generation*/
            if((ADXL362_RX_BUFFER[2] == (threshold & 0x00FF)) &&
               (ADXL362_RX_BUFFER[3] == ((threshold & 0x0700) >> 8))){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    };

    /* Retry until successful */
    while(ADXL362_RX_BUFFER[2] != time)
    {
        /* Configure inactivity timer.*/
        ADXL362_SetRegisterValue(time, ADXL362_REG_TIME_INACT_L, 2);
        ADXL362_GetRegisterValue(ADXL362_REG_TIME_INACT_L, 2);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if(ADXL362_RX_BUFFER[2] == time){
                SEGGER_RTT_WriteString(0, "Inactivity Timer programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program Inctivity Timer!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            /* Error generation*/
            if(ADXL362_RX_BUFFER[2] == time){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }
    ADXL362_GetRegisterValue(ADXL362_REG_ACT_INACT_CTL, 1);
    /* Enable inactivity interrupt and select a referenced or absolute
       configuration. */
    oldActInactReg = ADXL362_RX_BUFFER[2];
    newActInactReg = oldActInactReg & ~ADXL362_ACT_INACT_CTL_INACT_REF;
    newActInactReg |= ADXL362_ACT_INACT_CTL_INACT_EN |
                     (refOrAbs * ADXL362_ACT_INACT_CTL_INACT_REF);
    /* Retry until successful */
    while(ADXL362_RX_BUFFER[2] != newActInactReg)
    {
        ADXL362_SetRegisterValue(newActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
        ADXL362_GetRegisterValue(ADXL362_REG_ACT_INACT_CTL, 1);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if(ADXL362_RX_BUFFER[2] == newActInactReg){
                SEGGER_RTT_WriteString(0, "Inactivity Ref/Abs programmed successfully!\n");
                SEGGER_RTT_WriteString(0, "Inactivity interrupt enabled successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program Inactivity Ref/Abs!\n");
                SEGGER_RTT_WriteString(0, "#ERROR failed to enable Inactivity interrupt!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            /* Error generation*/
            if(ADXL362_RX_BUFFER[2] == newActInactReg){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }
}

void ADXL362_SetupActivityInactivityLinkLoop(unsigned char  linkloop)
{
    unsigned char oldActInactReg = 0;
    unsigned char newActInactReg = 0;
    uint16_t linkloop_msk = 0x30;
    /* Get the current ADXL362_REG_ACT_INACT_CTL register value*/
    ADXL362_GetRegisterValue(ADXL362_REG_ACT_INACT_CTL, 1);
    oldActInactReg = ADXL362_RX_BUFFER[2];
    /* Configure the link mode. */
    newActInactReg = oldActInactReg & ~linkloop_msk;
    newActInactReg |= linkloop;
    /* Retry until successful */
    while(ADXL362_RX_BUFFER[2] != newActInactReg)
    {
        ADXL362_SetRegisterValue(newActInactReg, ADXL362_REG_ACT_INACT_CTL, 1);
        ADXL362_GetRegisterValue(ADXL362_REG_ACT_INACT_CTL, 1);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if(ADXL362_RX_BUFFER[2] == newActInactReg){
                SEGGER_RTT_WriteString(0, "Linkloop programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program Linkloop!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            /* Error generation*/
            ret_code_t err_code;
            if(ADXL362_RX_BUFFER[2] == newActInactReg){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }
}


void ADXL362_SetupINTMAP1(unsigned char  intmap)
{
    unsigned char oldIntmap1Reg = 0;
    unsigned char newIntmap1Reg = 0;

    /* Get the current INTMAP1 register value*/
    ADXL362_GetRegisterValue(ADXL362_REG_INTMAP1, 1);
    oldIntmap1Reg = ADXL362_RX_BUFFER[2];
    /* Configure the interrupt map. */
    newIntmap1Reg = oldIntmap1Reg & ~intmap;
    newIntmap1Reg |= intmap;
    /* Retry until successful */
    while(ADXL362_RX_BUFFER[2] != newIntmap1Reg)
    {
        ADXL362_SetRegisterValue(newIntmap1Reg, ADXL362_REG_INTMAP1, 1);
        ADXL362_GetRegisterValue(ADXL362_REG_INTMAP1, 1);
        #ifdef DEBUG_WITH_UART
                /* Debugging with SEGGER_RTT*/
                if(ADXL362_RX_BUFFER[2] == newIntmap1Reg){
                    SEGGER_RTT_WriteString(0, "INTMAP1 programmed successfully!\n");
                }
                else{
                    SEGGER_RTT_WriteString(0, "#ERROR failed to program INTMAP1!\n");
                }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            ret_code_t err_code;
            /* Error generation*/
            if(ADXL362_RX_BUFFER[2] == newIntmap1Reg){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }

}
void ADXL362_SetupINTMAP2(unsigned char  intmap)
{
    unsigned char oldIntmap2Reg = 0;
    unsigned char newIntmap2Reg = 0;

    /* Get the current INTMAP2 register value*/
    ADXL362_GetRegisterValue(ADXL362_REG_INTMAP2, 1);
    oldIntmap2Reg = ADXL362_RX_BUFFER[2];
    /* Configure the interrupt map. */
    newIntmap2Reg = oldIntmap2Reg & ~intmap;
    newIntmap2Reg |= intmap;
    /* Retry until successful */
    while(ADXL362_RX_BUFFER[2] != newIntmap2Reg)
    {
        ADXL362_SetRegisterValue(newIntmap2Reg, ADXL362_REG_INTMAP2, 1);
        ADXL362_GetRegisterValue(ADXL362_REG_INTMAP2, 1);
        #ifdef DEBUG_WITH_UART
            /* Debugging with SEGGER_RTT*/
            if(ADXL362_RX_BUFFER[2] == newIntmap2Reg){
                SEGGER_RTT_WriteString(0, "INTMAP2 programmed successfully!\n");
            }
            else{
                SEGGER_RTT_WriteString(0, "#ERROR failed to program INTMAP2!\n");
            }
        #endif
        #ifdef DEBUG_WITH_ERRORS
            ret_code_t err_code;
            /* Error generation*/
            if(ADXL362_RX_BUFFER[2] == newIntmap2Reg){
                err_code = NRF_SUCCESS;
            }
            else{err_code = ADXL362_REGISTER_WRITE_FAILED;}
            APP_ERROR_CHECK(err_code);
        #endif
    }
}
