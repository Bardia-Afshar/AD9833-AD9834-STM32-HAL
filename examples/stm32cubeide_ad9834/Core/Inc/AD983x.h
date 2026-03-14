/*************************************************************************************
 Title   :   Analog Devices AD983x DDS Wave Generator Library for STM32 Using HAL
 File    :   AD983x.h
 Author  :   Bardia Alikhan Afshar <bardia.a.afshar@gmail.com>
 Date    :   2026-03-15
*************************************************************************************/

#ifndef AD983X_H
#define AD983X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/*------------------------------------------------------------------------------------------------------
 | @brief SPI timeout used for HAL_SPI_Transmit (in milliseconds).
 -------------------------------------------------------------------------------------------------------*/
#define AD983X_SPI_TIMEOUT_MS (10u)

/*------------------------------------------------------------------------------------------------------
 | @brief Supported device types.
 -------------------------------------------------------------------------------------------------------*/
typedef enum
{
    AD983X_DEVICE_AD9833 = 0,
    AD983X_DEVICE_AD9834 = 1
} AD983x_Device_t;

/*------------------------------------------------------------------------------------------------------
 | @brief Supported waveform types.
 |
 | AD9833:
 |   SINE / SQUARE / TRIANGLE are all valid.
 |
 | AD9834:
 |   Only SINE / TRIANGLE are valid analog waveform selections.
 |   Square output must be configured using AD9834_SetSquareWaveMode().
 -------------------------------------------------------------------------------------------------------*/
typedef enum
{
    AD983X_WAVE_SINE     = 0,
    AD983X_WAVE_SQUARE   = 1,
    AD983X_WAVE_TRIANGLE = 2
} AD983x_Waveform_t;

/*------------------------------------------------------------------------------------------------------
 | @brief Register bank selector for FREQ and PHASE registers.
 -------------------------------------------------------------------------------------------------------*/
typedef enum
{
    AD983X_REG_0 = 0,
    AD983X_REG_1 = 1
} AD983x_RegBank_t;

/*------------------------------------------------------------------------------------------------------
 | @brief Sleep bit state — used for SLEEP1 and SLEEP12 control bits.
 -------------------------------------------------------------------------------------------------------*/
typedef enum
{
    AD983X_SLEEP_DISABLED = 0,
    AD983X_SLEEP_ENABLED  = 1
} AD983x_SleepState_t;

/*------------------------------------------------------------------------------------------------------
 | @brief AD9834 FSEL/PSEL control source selection.
 |        PIN mode: hardware pins drive register selection.
 |        SW  mode: control register bits drive register selection (same as AD9833).
 -------------------------------------------------------------------------------------------------------*/
typedef enum
{
    AD9834_REG_SELECT_SW  = 0,
    AD9834_REG_SELECT_PIN = 1
} AD9834_RegSelectMode_t;

/*------------------------------------------------------------------------------------------------------
 | @brief AD9834 square/digital output mode for SIGN BIT OUT.
 |
 | Use this enum only with AD9834_SetSquareWaveMode().
 -------------------------------------------------------------------------------------------------------*/
typedef enum
{
    AD9834_SQUARE_DISABLED   = 0,   /* SIGN BIT OUT disabled */
    AD9834_SQUARE_MSB        = 1,   /* DAC data MSB on SIGN BIT OUT */
    AD9834_SQUARE_COMPARATOR = 2    /* Comparator output on SIGN BIT OUT */
} AD9834_SquareMode_t;

/*------------------------------------------------------------------------------------------------------
 | @brief AD9834 comparator VIN connection state.
 |        Tells the driver whether VIN is connected in the external circuit.
 -------------------------------------------------------------------------------------------------------*/
typedef enum
{
    AD9834_COMPARATOR_VIN_NOT_AVAILABLE = 0,
    AD9834_COMPARATOR_VIN_AVAILABLE     = 1
} AD9834_ComparatorVinState_t;

/*------------------------------------------------------------------------------------------------------
 | @brief Status / error codes returned by the driver.
 -------------------------------------------------------------------------------------------------------*/
typedef enum
{
    AD983X_STATUS_OK          = 0,
    AD983X_STATUS_ERROR_NULL  = 1,
    AD983X_STATUS_ERROR_PARAM = 2,
    AD983X_STATUS_ERROR_SPI   = 3
} AD983x_Status_t;

/*------------------------------------------------------------------------------------------------------
 | @brief AD983x configuration and runtime handle.
 |
 | Fill this once with GPIO pins, clock frequency, and device type.
 | Then pass the same handle to the driver functions.
 |
 | For software SPI, set spiHandle to NULL.
 -------------------------------------------------------------------------------------------------------*/
typedef struct
{
    GPIO_TypeDef         * gpioPort;                       /* GPIO port used for all AD983x signals */
    uint16_t               serialDataGpioPin;              /* SDATA pin number */
    uint16_t               serialClockGpioPin;             /* SCLK pin number */
    uint16_t               frameSyncGpioPin;               /* FSYNC (chip select) pin number */

#if defined(HAL_SPI_MODULE_ENABLED)
    SPI_HandleTypeDef    * spiHandle;                       /* Set to NULL(Or do not SET) to force software SPI */
#endif

    uint32_t               masterClockFrequencyHz;          /* AD983x master clock frequency in Hz */
    AD983x_Device_t        deviceType;                      /* AD9833 or AD9834 */
    AD983x_Waveform_t      currentWaveformType;             /* Cached currently selected waveform */
    AD983x_RegBank_t       activeFrequencyRegister;         /* Currently selected frequency register */
    AD983x_RegBank_t       activePhaseRegister;             /* Currently selected phase register */
    AD983x_SleepState_t    sleep1BitState;                  /* SLEEP1 control bit state */
    AD983x_SleepState_t    sleep12BitState;                 /* SLEEP12 control bit state */
    AD9834_RegSelectMode_t ad9834RegSelectMode;             /* AD9834 only: PIN vs SW register select */
    AD9834_SquareMode_t    ad9834SquareMode;                /* AD9834 only: SIGN BIT OUT mode */
    uint32_t               frequencyHz[2];                  /* Cached FREQ0 and FREQ1 values */
    uint16_t               phaseDeg[2];                     /* Cached PHASE0 and PHASE1 values */
    uint8_t                ad9834ComparatorVinAvailable;    /* AD9834 only: 1 if VIN is connected */
} AD983x_Handle_t;


/*======================================================================================================
 |                                      FUNCTION PROTOTYPES
 ======================================================================================================*/

/*------------------------------------------------------------------------------------------------------
 | @brief Initialize the AD983x device.
 |        Resets the part, clears the registers, and programs waveform,
 |        frequency, and phase into register bank 0.
 |        The handle must already contain the GPIO pins, clock, and device type.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_Init(AD983x_Handle_t * ad983xHandle, AD983x_Waveform_t waveformType, uint32_t outputFrequencyHz, uint16_t outputPhaseDegrees);

/*------------------------------------------------------------------------------------------------------
 | @brief Set output waveform (SINE / SQUARE / TRIANGLE).
 |        AD9833: all three waveforms are valid.
 |        AD9834: only SINE / TRIANGLE; use AD9834_SetSquareWaveMode() for square.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SetWaveform(AD983x_Handle_t * ad983xHandle, AD983x_Waveform_t waveformType);

/*------------------------------------------------------------------------------------------------------
 | @brief (AD9834 only) Configure SIGN BIT OUT.
 |        TRIANGLE mode and SIGN BIT OUT cannot be used together.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD9834_SetSquareWaveMode(AD983x_Handle_t * ad983xHandle, AD9834_SquareMode_t mode);

/*------------------------------------------------------------------------------------------------------
 | @brief Set output frequency in Hz for the specified FREQ register (0 or 1).
 |        FTW = (outputFrequencyHz * 2^28) / masterClockFrequencyHz.
 |        Output must not exceed MCLK / 2 (Nyquist limit).
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SetFrequencyHz(AD983x_Handle_t * ad983xHandle, AD983x_RegBank_t registerNumber, uint32_t outputFrequencyHz);

/*------------------------------------------------------------------------------------------------------
 | @brief Set output phase in degrees for the specified PHASE register (0 or 1).
 |        Converts degrees (0–359) to a 12-bit phase word; values >= 360 wrap.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SetPhaseDeg(AD983x_Handle_t * ad983xHandle, AD983x_RegBank_t registerNumber, uint16_t outputPhaseDegrees);

/*------------------------------------------------------------------------------------------------------
 | @brief Set frequency and phase on the selected register bank.
 |        Uses one grouped write sequence instead of two separate API calls.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SetFrequencyPhase(AD983x_Handle_t * ad983xHandle, AD983x_RegBank_t registerNumber, uint32_t outputFrequencyHz, uint16_t outputPhaseDegrees);

/*------------------------------------------------------------------------------------------------------
 | @brief Select which FREQ register drives the output (FSELECT control bit).
 |        On AD9834 in PIN mode this bit is ignored — use the hardware pin.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SelectFreqReg(AD983x_Handle_t * ad983xHandle, AD983x_RegBank_t registerNumber);

/*------------------------------------------------------------------------------------------------------
 | @brief Select which PHASE register drives the output (PSELECT control bit).
 |        On AD9834 in PIN mode this bit is ignored — use the hardware pin.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SelectPhaseReg(AD983x_Handle_t * ad983xHandle, AD983x_RegBank_t registerNumber);

/*------------------------------------------------------------------------------------------------------
 | @brief Enter sleep mode — disable internal MCLK (SLEEP1=1, SLEEP12=0).
 |        NCO stops accumulating; DAC holds its last output level.
 |        Allow ~1 ms for MCLK to stabilise after waking.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_Sleep(AD983x_Handle_t * ad983xHandle);

/*------------------------------------------------------------------------------------------------------
 | @brief Enter deep sleep — disable both MCLK and DAC (SLEEP1=1, SLEEP12=1).
 |        This is the lowest power mode.
 |        Call AD983x_Wake() to resume; allow about 1 ms for MCLK to settle.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_DeepSleep(AD983x_Handle_t * ad983xHandle);

/*------------------------------------------------------------------------------------------------------
 | @brief Power down the DAC only (SLEEP1=0, SLEEP12=1).
 |        MCLK and the NCO keep running.
 |        Useful on AD9834 when only SIGN BIT OUT is needed.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_DacOff(AD983x_Handle_t * ad983xHandle);

/*------------------------------------------------------------------------------------------------------
 | @brief Wake from any sleep mode by clearing SLEEP1 and SLEEP12.
 |        If SLEEP1 was set, allow about 1 ms for MCLK to settle.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_Wake(AD983x_Handle_t * ad983xHandle);

/*------------------------------------------------------------------------------------------------------
 | @brief (AD9834 only) Select whether FSELECT/PSELECT come from the
 |        control register or the external pins.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD9834_SetRegSelectMode(AD983x_Handle_t * ad983xHandle, AD9834_RegSelectMode_t mode);

/*------------------------------------------------------------------------------------------------------
 | @brief (AD9834 only) Tell the driver whether VIN is connected.
 |        Comparator mode on SIGN BIT OUT is allowed only when this is enabled.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD9834_SetComparatorVinAvailable(AD983x_Handle_t * ad983xHandle, AD9834_ComparatorVinState_t vinState);


#ifdef __cplusplus
}
#endif

#endif /* AD983X_H */
