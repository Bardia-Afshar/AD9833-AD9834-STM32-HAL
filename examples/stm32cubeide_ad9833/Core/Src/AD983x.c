/*************************************************************************************
 Title   :   Analog Devices AD983x DDS Wave Generator Library for STM32 Using HAL
 File    :   AD983x.c
 Author  :   Bardia Alikhan Afshar <bardia.a.afshar@gmail.com>
 Date    :   2026-03-15
*************************************************************************************/

#include "AD983x.h"

/*------------------------------------------------------------------------------------------------------
 | @brief Control bits and register prefixes used in 16-bit serial words.
 |        AD9833 and AD9834 use the same bit positions unless noted.
 -------------------------------------------------------------------------------------------------------*/
#define AD9833_CTRL_B28                 (0x2000u)
#define AD9833_CTRL_HLB                 (0x1000u)
#define AD9833_CTRL_FSELECT             (0x0800u)
#define AD9833_CTRL_PSELECT             (0x0400u)
#define AD9833_CTRL_PIN_SW              (0x0200u)   /* AD9834 only: 0 = SW mode, 1 = PIN mode */
#define AD9833_CTRL_RESET               (0x0100u)
#define AD9833_CTRL_SLEEP1              (0x0080u)
#define AD9833_CTRL_SLEEP12             (0x0040u)
#define AD9833_CTRL_OPBITEN             (0x0020u)
#define AD9833_CTRL_SIGN_PIB            (0x0010u)   /* AD9834 only: SIGN BIT OUT selection */
#define AD9833_CTRL_DIV2                (0x0008u)   /* AD9833 square output / AD9834 SIGN BIT OUT selection */
#define AD9833_CTRL_MODE                (0x0002u)

#define AD9833_FREQ0_PREFIX             (0x4000u)
#define AD9833_FREQ1_PREFIX             (0x8000u)
#define AD9833_PHASE0_PREFIX            (0xC000u)
#define AD9833_PHASE1_PREFIX            (0xE000u)

/*------------------------------------------------------------------------------------------------------
 | @brief Numeric constants used by the driver.
 -------------------------------------------------------------------------------------------------------*/
#define AD9833_WORD_BITS                (16u)
#define AD9833_WORD_MSB_MASK            (0x8000u)
#define AD9833_FREQ_WORD_MASK_14        (0x3FFFu)
#define AD9833_PHASE_WORD_MASK_12       (0x0FFFu)
#define AD9833_FTW_MASK_28              (0x0FFFFFFFuL)

#define AD9833_TWO_POW_28               (268435456uL) /* 2^28 */
#define AD9833_PHASE_STEPS              (4096u)       /* 12-bit phase accumulator */
#define AD9833_PHASE_DEG_MAX            (360u)
#define AD9834_COMPARATOR_MIN_FREQ_HZ   (3000000uL)   /* Datasheet: comparator needs typically >= 3 MHz */

/*------------------------------------------------------------------------------------------------------
 | @brief Check whether AD9834 comparator mode can be enabled.
 |
 | Comparator mode is allowed only when:
 |   1. the selected device is AD9834
 |   2. the current waveform is not TRIANGLE
 |   3. VIN has been marked as connected
 |   4. the active frequency is at least AD9834_COMPARATOR_MIN_FREQ_HZ
 -------------------------------------------------------------------------------------------------------*/
static AD983x_Status_t AD9834_ValidateComparatorModeRequest(const AD983x_Handle_t * ad983xHandle)
{
    uint32_t activeFrequencyHz;

    if (ad983xHandle->deviceType != AD983X_DEVICE_AD9834)
    {
        return AD983X_STATUS_ERROR_PARAM;
    }

    if (ad983xHandle->currentWaveformType == AD983X_WAVE_TRIANGLE)
    {
        return AD983X_STATUS_ERROR_PARAM;
    }

    if (ad983xHandle->ad9834ComparatorVinAvailable == 0u)
    {
        return AD983X_STATUS_ERROR_PARAM;
    }

    activeFrequencyHz = ad983xHandle->frequencyHz[(uint32_t)ad983xHandle->activeFrequencyRegister];

    if (activeFrequencyHz < (uint32_t)AD9834_COMPARATOR_MIN_FREQ_HZ)
    {
        return AD983X_STATUS_ERROR_PARAM;
    }

    return AD983X_STATUS_OK;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Return the SPI handle when hardware SPI is being used.
 -------------------------------------------------------------------------------------------------------*/
#if defined(HAL_SPI_MODULE_ENABLED)
static inline SPI_HandleTypeDef * AD9833_GetSpiHandle(const AD983x_Handle_t * ad983xHandle)
{
    return ad983xHandle->spiHandle;
}
#else
static inline void * AD9833_GetSpiHandle(const AD983x_Handle_t * ad983xHandle)
{
    (void)ad983xHandle;
    return NULL;
}
#endif

/*------------------------------------------------------------------------------------------------------
 | @brief Write GPIO pin state using STM32 HAL.
 -------------------------------------------------------------------------------------------------------*/
static void AD9833_GpioWritePinState(const AD983x_Handle_t * ad983xHandle, uint16_t gpioPinNumber, GPIO_PinState gpioPinState)
{
    HAL_GPIO_WritePin(ad983xHandle->gpioPort, gpioPinNumber, gpioPinState);
}

/*------------------------------------------------------------------------------------------------------
 | @brief Very short delay used between GPIO changes.
 -------------------------------------------------------------------------------------------------------*/
static void AD983x_DelayShort(void)
{
    __NOP();
    __NOP();
    __NOP();
}

/*------------------------------------------------------------------------------------------------------
 | @brief Assert FSYNC low to begin a serial frame.
 -------------------------------------------------------------------------------------------------------*/
static void AD9833_AssertFrameSync(const AD983x_Handle_t * ad983xHandle)
{
    AD9833_GpioWritePinState(ad983xHandle, ad983xHandle->frameSyncGpioPin, GPIO_PIN_RESET);
    AD983x_DelayShort();
}

/*------------------------------------------------------------------------------------------------------
 | @brief Deassert FSYNC high to end a serial frame and latch data.
 -------------------------------------------------------------------------------------------------------*/
static void AD9833_DeassertFrameSync(const AD983x_Handle_t * ad983xHandle)
{
    AD983x_DelayShort();
    AD9833_GpioWritePinState(ad983xHandle, ad983xHandle->frameSyncGpioPin, GPIO_PIN_SET);
    AD983x_DelayShort();
}

/*------------------------------------------------------------------------------------------------------
 | @brief Transmit one 16-bit word to the device.
 -------------------------------------------------------------------------------------------------------*/
static AD983x_Status_t AD9833_WriteSerialWord16(const AD983x_Handle_t * ad983xHandle, uint16_t serialWord16)
{
    AD983x_Status_t driverStatus = AD983X_STATUS_OK;

    if (AD9833_GetSpiHandle(ad983xHandle) != NULL)
    {
#if defined(HAL_SPI_MODULE_ENABLED)
        uint8_t           transmitBytes[2];
        HAL_StatusTypeDef halTransferStatus;

        transmitBytes[0] = (uint8_t)(serialWord16 >> 8u);
        transmitBytes[1] = (uint8_t)(serialWord16 & 0xFFu);

        halTransferStatus = HAL_SPI_Transmit(ad983xHandle->spiHandle, transmitBytes, 2u, AD983X_SPI_TIMEOUT_MS);

        if (halTransferStatus != HAL_OK)
        {
            driverStatus = AD983X_STATUS_ERROR_SPI;
        }
#endif
    }
    else
    {
        /* Software bit-bang SPI */
        uint16_t serialWordShiftRegister = serialWord16;
        uint8_t  bitIndex;

        for (bitIndex = 0u; bitIndex < (uint8_t)AD9833_WORD_BITS; bitIndex++)
        {
            if ((serialWordShiftRegister & (uint16_t)AD9833_WORD_MSB_MASK) != 0u)
            {
                AD9833_GpioWritePinState(ad983xHandle, ad983xHandle->serialDataGpioPin, GPIO_PIN_SET);
            }
            else
            {
                AD9833_GpioWritePinState(ad983xHandle, ad983xHandle->serialDataGpioPin, GPIO_PIN_RESET);
            }

            AD983x_DelayShort();

            AD9833_GpioWritePinState(ad983xHandle, ad983xHandle->serialClockGpioPin, GPIO_PIN_RESET);
            AD983x_DelayShort();
            AD9833_GpioWritePinState(ad983xHandle, ad983xHandle->serialClockGpioPin, GPIO_PIN_SET);

            serialWordShiftRegister = (uint16_t)((uint32_t)serialWordShiftRegister << 1u);
        }

        AD9833_GpioWritePinState(ad983xHandle, ad983xHandle->serialDataGpioPin, GPIO_PIN_RESET);
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Check the driver handle and the basic configuration fields.
 -------------------------------------------------------------------------------------------------------*/
static AD983x_Status_t AD9833_ValidateDriverHandle(const AD983x_Handle_t * ad983xHandle)
{
    AD983x_Status_t driverStatus = AD983X_STATUS_OK;

    if (ad983xHandle == NULL)
    {
        driverStatus = AD983X_STATUS_ERROR_NULL;
    }
    else if (ad983xHandle->gpioPort == NULL)
    {
        driverStatus = AD983X_STATUS_ERROR_NULL;
    }
    else if (ad983xHandle->masterClockFrequencyHz == 0u)
    {
        driverStatus = AD983X_STATUS_ERROR_PARAM;
    }
    else if ((uint32_t)ad983xHandle->deviceType > (uint32_t)AD983X_DEVICE_AD9834)
    {
        driverStatus = AD983X_STATUS_ERROR_PARAM;
    }
    else
    {
        /* Valid handle */
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Calculate the 28-bit frequency tuning word.
 |        FTW = (f_out * 2^28) / MCLK
 |        Uses 64-bit math to avoid overflow.
 -------------------------------------------------------------------------------------------------------*/
static uint32_t AD9833_CalculateFrequencyTuningWord28(uint32_t outputFrequencyHz, uint32_t masterClockFrequencyHz)
{
    uint64_t frequencyNumerator;
    uint32_t frequencyTuningWord28;

    frequencyNumerator = ((uint64_t)outputFrequencyHz) * ((uint64_t)AD9833_TWO_POW_28);
    frequencyTuningWord28 = (uint32_t)(frequencyNumerator / (uint64_t)masterClockFrequencyHz);
    frequencyTuningWord28 = frequencyTuningWord28 & (uint32_t)AD9833_FTW_MASK_28;

    return frequencyTuningWord28;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Return the 14-bit FREQ register prefix for the given bank.
 -------------------------------------------------------------------------------------------------------*/
static uint16_t AD9833_GetFrequencyRegisterPrefix(AD983x_RegBank_t registerNumber)
{
    uint16_t frequencyRegisterPrefix;

    if (registerNumber == AD983X_REG_1)
    {
        frequencyRegisterPrefix = (uint16_t)AD9833_FREQ1_PREFIX;
    }
    else
    {
        frequencyRegisterPrefix = (uint16_t)AD9833_FREQ0_PREFIX;
    }

    return frequencyRegisterPrefix;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Return the PHASE register prefix for the given bank.
 -------------------------------------------------------------------------------------------------------*/
static uint16_t AD9833_GetPhaseRegisterPrefix(AD983x_RegBank_t registerNumber)
{
    uint16_t phaseRegisterPrefix;

    if (registerNumber == AD983X_REG_1)
    {
        phaseRegisterPrefix = (uint16_t)AD9833_PHASE1_PREFIX;
    }
    else
    {
        phaseRegisterPrefix = (uint16_t)AD9833_PHASE0_PREFIX;
    }

    return phaseRegisterPrefix;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Convert phase degrees to a PHASE register word.
 -------------------------------------------------------------------------------------------------------*/
static uint16_t AD9833_ConvertPhaseDegreesToPhaseWord(AD983x_RegBank_t registerNumber, uint16_t outputPhaseDegrees)
{
    uint16_t normalizedPhaseDegrees;
    uint32_t phaseAccumulatorSteps;

    normalizedPhaseDegrees = (uint16_t)(outputPhaseDegrees % (uint16_t)AD9833_PHASE_DEG_MAX);
    phaseAccumulatorSteps = ((uint32_t)normalizedPhaseDegrees * (uint32_t)AD9833_PHASE_STEPS) / (uint32_t)AD9833_PHASE_DEG_MAX;
    phaseAccumulatorSteps &= (uint32_t)AD9833_PHASE_WORD_MASK_12;

    return (uint16_t)((uint32_t)AD9833_GetPhaseRegisterPrefix(registerNumber) | (uint32_t)phaseAccumulatorSteps);
}

/*------------------------------------------------------------------------------------------------------
 | @brief Build the control register word from the current driver state.
 |
 | On AD9833, waveformType selects sine, triangle, or square.
 |
 | On AD9834, waveformType controls only the analog output path.
 | SIGN BIT OUT is controlled through ad9834SquareMode.
 |
 | Triangle mode and SIGN BIT OUT cannot be enabled together.
 -------------------------------------------------------------------------------------------------------*/
static uint16_t AD9833_BuildControlRegisterWord(const AD983x_Handle_t * ad983xHandle, AD983x_Waveform_t waveformType)
{
    uint16_t controlRegisterWord;

    controlRegisterWord = (uint16_t)AD9833_CTRL_B28;

    if (ad983xHandle->deviceType == AD983X_DEVICE_AD9834)
    {
           /* AD9834 PIN/SW bit:
           0 = control register selects FREQ/PHASE
           1 = external pins select FREQ/PHASE */

        if (ad983xHandle->ad9834RegSelectMode == AD9834_REG_SELECT_PIN)
        {
            controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_PIN_SW);
        }
    }

    if (ad983xHandle->activeFrequencyRegister == AD983X_REG_1)
    {
        controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_FSELECT);
    }

    if (ad983xHandle->activePhaseRegister == AD983X_REG_1)
    {
        controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_PSELECT);
    }

    if (ad983xHandle->sleep1BitState != AD983X_SLEEP_DISABLED)
    {
        controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_SLEEP1);
    }
    if (ad983xHandle->sleep12BitState != AD983X_SLEEP_DISABLED)
    {
        controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_SLEEP12);
    }

    switch (waveformType)
    {
        case AD983X_WAVE_SINE:
            break;

        case AD983X_WAVE_SQUARE:
            if (ad983xHandle->deviceType == AD983X_DEVICE_AD9834)
            {
                /* Rejected by public API. Keep analog path unchanged here. */
            }
            else
            {
                controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_OPBITEN);
                controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_DIV2);
            }
            break;

        case AD983X_WAVE_TRIANGLE:
            /* Triangle output uses MODE bit.
               On AD9834 this must remain analog-only, so squareMode is ignored below. */
            controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_MODE);
            break;

        default:
            break;
    }

    if (ad983xHandle->deviceType == AD983X_DEVICE_AD9834)
    {
        /* Strict AD9834 rule:
           only allow SIGN BIT OUT modes when waveform is not TRIANGLE. */
        if (waveformType != AD983X_WAVE_TRIANGLE)
        {
            switch (ad983xHandle->ad9834SquareMode)
            {
                case AD9834_SQUARE_MSB:
                    /* OPBITEN=1, MODE=0, SIGN_PIB=0, DIV2=1 => DAC data MSB */
                    controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_OPBITEN);
                    controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_DIV2);
                    break;

                case AD9834_SQUARE_COMPARATOR:
                    /* AD9834 comparator output on SIGN BIT OUT:
                       OPBITEN=1, MODE=0, SIGN_PIB=1, DIV2=1 */
                    controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_OPBITEN);
                    controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_SIGN_PIB);
                    controlRegisterWord = (uint16_t)((uint32_t)controlRegisterWord | (uint32_t)AD9833_CTRL_DIV2);
                    break;

                case AD9834_SQUARE_DISABLED:
                default:
                    break;
            }
        }
    }

    return controlRegisterWord;
}

/*======================================================================================================
 |                                      PUBLIC API FUNCTIONS
 ======================================================================================================*/

/*------------------------------------------------------------------------------------------------------
 | @brief Initialize the device and load the startup settings.
 |
 | This function:
 |   1. puts the serial pins into their idle state
 |   2. resets the cached driver state
 |   3. holds the part in reset
 |   4. clears both frequency registers and both phase registers
 |   5. writes the requested frequency and phase into register bank 0
 |   6. applies the requested waveform
 |
 | The handle must already contain the GPIO pins, clock frequency,
 | and device type. Set spiHandle to NULL to use software SPI.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_Init(AD983x_Handle_t * ad983xHandle, AD983x_Waveform_t waveformType, uint32_t outputFrequencyHz, uint16_t outputPhaseDegrees)
{
    AD983x_Status_t driverStatus;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        AD9833_GpioWritePinState(ad983xHandle, ad983xHandle->frameSyncGpioPin, GPIO_PIN_SET);

        if (AD9833_GetSpiHandle(ad983xHandle) == NULL)
        {
            AD9833_GpioWritePinState(ad983xHandle, ad983xHandle->serialDataGpioPin, GPIO_PIN_RESET);
            AD9833_GpioWritePinState(ad983xHandle, ad983xHandle->serialClockGpioPin, GPIO_PIN_SET);
        }

        ad983xHandle->currentWaveformType = AD983X_WAVE_SINE;
        ad983xHandle->activeFrequencyRegister = AD983X_REG_0;
        ad983xHandle->activePhaseRegister = AD983X_REG_0;
        ad983xHandle->sleep1BitState = AD983X_SLEEP_DISABLED;
        ad983xHandle->sleep12BitState = AD983X_SLEEP_DISABLED;

        ad983xHandle->frequencyHz[0] = 0u;
        ad983xHandle->frequencyHz[1] = 0u;
        ad983xHandle->phaseDeg[0] = 0u;
        ad983xHandle->phaseDeg[1] = 0u;
        ad983xHandle->ad9834ComparatorVinAvailable = 0u;

        if (ad983xHandle->deviceType == AD983X_DEVICE_AD9834)
        {
            ad983xHandle->ad9834RegSelectMode = AD9834_REG_SELECT_SW;
            ad983xHandle->ad9834SquareMode = AD9834_SQUARE_DISABLED;
        }

        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, (uint16_t)(AD9833_CTRL_B28 | AD9833_CTRL_RESET));
        AD9833_DeassertFrameSync(ad983xHandle);
    }

    if (driverStatus == AD983X_STATUS_OK)
    {
        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, (uint16_t)AD9833_FREQ0_PREFIX);
        AD9833_DeassertFrameSync(ad983xHandle);
    }
    if (driverStatus == AD983X_STATUS_OK)
    {
        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, (uint16_t)AD9833_FREQ0_PREFIX);
        AD9833_DeassertFrameSync(ad983xHandle);
    }
    if (driverStatus == AD983X_STATUS_OK)
    {
        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, (uint16_t)AD9833_FREQ1_PREFIX);
        AD9833_DeassertFrameSync(ad983xHandle);
    }
    if (driverStatus == AD983X_STATUS_OK)
    {
        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, (uint16_t)AD9833_FREQ1_PREFIX);
        AD9833_DeassertFrameSync(ad983xHandle);
    }
    if (driverStatus == AD983X_STATUS_OK)
    {
        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, (uint16_t)AD9833_PHASE0_PREFIX);
        AD9833_DeassertFrameSync(ad983xHandle);
    }
    if (driverStatus == AD983X_STATUS_OK)
    {
        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, (uint16_t)AD9833_PHASE1_PREFIX);
        AD9833_DeassertFrameSync(ad983xHandle);
    }

    if (driverStatus == AD983X_STATUS_OK)
    {
        driverStatus = AD983x_SetFrequencyPhase(ad983xHandle, AD983X_REG_0, outputFrequencyHz, outputPhaseDegrees);
    }

    if (driverStatus == AD983X_STATUS_OK)
    {
        driverStatus = AD983x_SetWaveform(ad983xHandle, waveformType);
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Set the output waveform.
 |
 | On AD9833 this selects sine, square, or triangle.
 |
 | On AD9834 this selects only the analog waveform.
 | SIGN BIT OUT is controlled separately through AD9834_SetSquareWaveMode().
 | Triangle mode forces SIGN BIT OUT off.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SetWaveform(AD983x_Handle_t * ad983xHandle, AD983x_Waveform_t waveformType)
{
    AD983x_Status_t driverStatus;
    uint16_t controlRegisterWord;
    AD9834_SquareMode_t previousSquareMode;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        if ((uint32_t)waveformType > (uint32_t)AD983X_WAVE_TRIANGLE)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else if ((ad983xHandle->deviceType == AD983X_DEVICE_AD9834) && (waveformType == AD983X_WAVE_SQUARE))
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else
        {
            previousSquareMode = ad983xHandle->ad9834SquareMode;

            if ((ad983xHandle->deviceType == AD983X_DEVICE_AD9834) && (waveformType == AD983X_WAVE_TRIANGLE))
            {
                ad983xHandle->ad9834SquareMode = AD9834_SQUARE_DISABLED;
            }

            controlRegisterWord = AD9833_BuildControlRegisterWord(ad983xHandle, waveformType);

            AD9833_AssertFrameSync(ad983xHandle);
            driverStatus = AD9833_WriteSerialWord16(ad983xHandle, controlRegisterWord);
            AD9833_DeassertFrameSync(ad983xHandle);

            if (driverStatus == AD983X_STATUS_OK)
            {
                ad983xHandle->currentWaveformType = waveformType;
            }
            else
            {
                ad983xHandle->ad9834SquareMode = previousSquareMode;
            }
        }
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Configure SIGN BIT OUT on AD9834.
 |
 | Triangle mode and SIGN BIT OUT cannot be used at the same time.
 |
 | Comparator mode is allowed only when VIN has been marked as connected
 | and the active output frequency is high enough.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD9834_SetSquareWaveMode(AD983x_Handle_t * ad983xHandle, AD9834_SquareMode_t mode)
{
    AD983x_Status_t driverStatus;
    uint16_t controlRegisterWord;
    AD9834_SquareMode_t previousMode;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        if (ad983xHandle->deviceType != AD983X_DEVICE_AD9834)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else if ((uint32_t)mode > (uint32_t)AD9834_SQUARE_COMPARATOR)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else if ((ad983xHandle->currentWaveformType == AD983X_WAVE_TRIANGLE) && (mode != AD9834_SQUARE_DISABLED))
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else if (mode == AD9834_SQUARE_COMPARATOR)
        {
            driverStatus = AD9834_ValidateComparatorModeRequest(ad983xHandle);
        }
        else
        {
        	/* Request is valid */
        }
    }

    if (driverStatus == AD983X_STATUS_OK)
    {
        previousMode = ad983xHandle->ad9834SquareMode;
        ad983xHandle->ad9834SquareMode = mode;

        controlRegisterWord = AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType);

        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, controlRegisterWord);
        AD9833_DeassertFrameSync(ad983xHandle);

        if (driverStatus != AD983X_STATUS_OK)
        {
            ad983xHandle->ad9834SquareMode = previousMode;
        }
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Write a frequency value into FREQ0 or FREQ1.
 |
 | The 28-bit tuning word is calculated from:
 |   FTW = (outputFrequencyHz * 2^28) / masterClockFrequencyHz
 |
 | The register is written as two 14-bit words, LSB first and then MSB.
 | The requested frequency must not be above MCLK / 2.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SetFrequencyHz(AD983x_Handle_t * ad983xHandle, AD983x_RegBank_t registerNumber, uint32_t outputFrequencyHz)
{
    AD983x_Status_t driverStatus;
    uint32_t maximumAllowedFrequencyHz;
    uint32_t frequencyTuningWord28;
    uint16_t frequencyWordLeastSignificant14;
    uint16_t frequencyWordMostSignificant14;
    uint16_t frequencyRegisterPrefix;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        if ((uint32_t)registerNumber > (uint32_t)AD983X_REG_1)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
    }

    if (driverStatus == AD983X_STATUS_OK)
    {
        maximumAllowedFrequencyHz = ad983xHandle->masterClockFrequencyHz / 2u;

        if (((maximumAllowedFrequencyHz == 0u) && (outputFrequencyHz > 0u)) || (outputFrequencyHz > maximumAllowedFrequencyHz))
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else
        {
            frequencyTuningWord28 = AD9833_CalculateFrequencyTuningWord28(outputFrequencyHz, ad983xHandle->masterClockFrequencyHz);
            frequencyRegisterPrefix = AD9833_GetFrequencyRegisterPrefix(registerNumber);

            frequencyWordLeastSignificant14 = (uint16_t)((uint32_t)frequencyRegisterPrefix | (frequencyTuningWord28 & (uint32_t)AD9833_FREQ_WORD_MASK_14));
            frequencyWordMostSignificant14 = (uint16_t)((uint32_t)frequencyRegisterPrefix | ((frequencyTuningWord28 >> 14u) & (uint32_t)AD9833_FREQ_WORD_MASK_14));

            AD9833_AssertFrameSync(ad983xHandle);

            driverStatus = AD9833_WriteSerialWord16(ad983xHandle, (uint16_t)((uint32_t)AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType) | (uint32_t)AD9833_CTRL_RESET));

            if (driverStatus == AD983X_STATUS_OK)
            {
                driverStatus = AD9833_WriteSerialWord16(ad983xHandle, frequencyWordLeastSignificant14);
            }

            if (driverStatus == AD983X_STATUS_OK)
            {
                driverStatus = AD9833_WriteSerialWord16(ad983xHandle, frequencyWordMostSignificant14);
            }

            if (driverStatus == AD983X_STATUS_OK)
            {
                driverStatus = AD9833_WriteSerialWord16(ad983xHandle, AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType));
            }

            AD9833_DeassertFrameSync(ad983xHandle);

            if (driverStatus == AD983X_STATUS_OK)
            {
                ad983xHandle->frequencyHz[(uint32_t)registerNumber] = outputFrequencyHz;
            }
        }
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Write a phase value in degrees into PHASE0 or PHASE1.
 |
 | The phase word is calculated as:
 |   PHASE_WORD = (degrees * 4096) / 360
 |
 | Values of 360 and above wrap around.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SetPhaseDeg(AD983x_Handle_t * ad983xHandle, AD983x_RegBank_t registerNumber, uint16_t outputPhaseDegrees)
{
    AD983x_Status_t driverStatus;
    uint16_t phaseRegisterWord;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        if ((uint32_t)registerNumber > (uint32_t)AD983X_REG_1)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
    }

    if (driverStatus == AD983X_STATUS_OK)
    {
        phaseRegisterWord = AD9833_ConvertPhaseDegreesToPhaseWord(registerNumber, outputPhaseDegrees);

        AD9833_AssertFrameSync(ad983xHandle);

        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, (uint16_t)((uint32_t)AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType) | (uint32_t)AD9833_CTRL_RESET));

        if (driverStatus == AD983X_STATUS_OK)
        {
            driverStatus = AD9833_WriteSerialWord16(ad983xHandle, phaseRegisterWord);
        }

        if (driverStatus == AD983X_STATUS_OK)
        {
            driverStatus = AD9833_WriteSerialWord16(ad983xHandle, AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType));
        }

        AD9833_DeassertFrameSync(ad983xHandle);

        if (driverStatus == AD983X_STATUS_OK)
        {
            ad983xHandle->phaseDeg[(uint32_t)registerNumber] = (uint16_t)(outputPhaseDegrees % (uint16_t)AD9833_PHASE_DEG_MAX);
        }
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Write frequency and phase to the same register bank in one sequence.
 |
 | This is the same idea as calling AD983x_SetFrequencyHz() and AD983x_SetPhaseDeg(),
 | but both values are sent in one grouped transfer.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SetFrequencyPhase(AD983x_Handle_t * ad983xHandle, AD983x_RegBank_t registerNumber, uint32_t outputFrequencyHz, uint16_t outputPhaseDegrees)
{
    AD983x_Status_t driverStatus;
    uint32_t maximumAllowedFrequencyHz;
    uint32_t frequencyTuningWord28;
    uint16_t frequencyWordLeastSignificant14;
    uint16_t frequencyWordMostSignificant14;
    uint16_t phaseRegisterWord;
    uint16_t frequencyRegisterPrefix;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        if ((uint32_t)registerNumber > (uint32_t)AD983X_REG_1)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
    }

    if (driverStatus == AD983X_STATUS_OK)
    {
        maximumAllowedFrequencyHz = ad983xHandle->masterClockFrequencyHz / 2u;

        if (((maximumAllowedFrequencyHz == 0u) && (outputFrequencyHz > 0u)) || (outputFrequencyHz > maximumAllowedFrequencyHz))
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else
        {
            frequencyTuningWord28 = AD9833_CalculateFrequencyTuningWord28(outputFrequencyHz, ad983xHandle->masterClockFrequencyHz);
            frequencyRegisterPrefix = AD9833_GetFrequencyRegisterPrefix(registerNumber);

            frequencyWordLeastSignificant14 = (uint16_t)((uint32_t)frequencyRegisterPrefix | (frequencyTuningWord28 & (uint32_t)AD9833_FREQ_WORD_MASK_14));
            frequencyWordMostSignificant14 = (uint16_t)((uint32_t)frequencyRegisterPrefix | ((frequencyTuningWord28 >> 14u) & (uint32_t)AD9833_FREQ_WORD_MASK_14));
            phaseRegisterWord = AD9833_ConvertPhaseDegreesToPhaseWord(registerNumber, outputPhaseDegrees);

            AD9833_AssertFrameSync(ad983xHandle);

            driverStatus = AD9833_WriteSerialWord16(ad983xHandle, (uint16_t)((uint32_t)AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType) | (uint32_t)AD9833_CTRL_RESET));

            if (driverStatus == AD983X_STATUS_OK)
            {
                driverStatus = AD9833_WriteSerialWord16(ad983xHandle, frequencyWordLeastSignificant14);
            }

            if (driverStatus == AD983X_STATUS_OK)
            {
                driverStatus = AD9833_WriteSerialWord16(ad983xHandle, frequencyWordMostSignificant14);
            }

            if (driverStatus == AD983X_STATUS_OK)
            {
                driverStatus = AD9833_WriteSerialWord16(ad983xHandle, phaseRegisterWord);
            }

            if (driverStatus == AD983X_STATUS_OK)
            {
                driverStatus = AD9833_WriteSerialWord16(ad983xHandle, AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType));
            }

            AD9833_DeassertFrameSync(ad983xHandle);

            if (driverStatus == AD983X_STATUS_OK)
            {
                ad983xHandle->frequencyHz[(uint32_t)registerNumber] = outputFrequencyHz;
                ad983xHandle->phaseDeg[(uint32_t)registerNumber] = (uint16_t)(outputPhaseDegrees % (uint16_t)AD9833_PHASE_DEG_MAX);
            }
        }
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Select which FREQ register drives the output (FSELECT control bit).
 |
 | Sets the FSELECT bit in the control register:
 |   FSELECT=0 : FREQ0 register drives the phase accumulator.
 |   FSELECT=1 : FREQ1 register drives the phase accumulator.
 |
 | On AD9834 in PIN mode (PIN_SW=1) this software bit is ignored by
 | the hardware — use the external FSELECT pin instead.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SelectFreqReg(AD983x_Handle_t * ad983xHandle, AD983x_RegBank_t registerNumber)
{
    AD983x_Status_t driverStatus;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        if ((uint32_t)registerNumber > (uint32_t)AD983X_REG_1)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else
        {
            ad983xHandle->activeFrequencyRegister = registerNumber;

            AD9833_AssertFrameSync(ad983xHandle);
            driverStatus = AD9833_WriteSerialWord16(ad983xHandle, AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType));
            AD9833_DeassertFrameSync(ad983xHandle);
        }
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Select which PHASE register drives the output (PSELECT control bit).
 |
 | Sets the PSELECT bit in the control register:
 |   PSELECT=0 : PHASE0 register is added to the phase accumulator output.
 |   PSELECT=1 : PHASE1 register is added to the phase accumulator output.
 |
 | On AD9834 in PIN mode (PIN_SW=1) this software bit is ignored by
 | the hardware — use the external PSELECT pin instead.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_SelectPhaseReg(AD983x_Handle_t * ad983xHandle, AD983x_RegBank_t registerNumber)
{
    AD983x_Status_t driverStatus;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        if ((uint32_t)registerNumber > (uint32_t)AD983X_REG_1)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else
        {
            ad983xHandle->activePhaseRegister = registerNumber;

            AD9833_AssertFrameSync(ad983xHandle);
            driverStatus = AD9833_WriteSerialWord16(ad983xHandle, AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType));
            AD9833_DeassertFrameSync(ad983xHandle);
        }
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Enter sleep mode by stopping the internal MCLK.
 |
 | The DAC stays powered, so the output holds its last level.
 | After waking, give the clock a little time to settle.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_Sleep(AD983x_Handle_t * ad983xHandle)
{
    AD983x_Status_t driverStatus;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        ad983xHandle->sleep1BitState = AD983X_SLEEP_ENABLED;
        ad983xHandle->sleep12BitState = AD983X_SLEEP_DISABLED;

        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType));
        AD9833_DeassertFrameSync(ad983xHandle);
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Enter deep sleep by turning off both MCLK and the DAC.
 |
 | This is the lowest power state.
 | After waking, give the clock a little time to settle.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_DeepSleep(AD983x_Handle_t * ad983xHandle)
{
    AD983x_Status_t driverStatus;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        ad983xHandle->sleep1BitState = AD983X_SLEEP_ENABLED;
        ad983xHandle->sleep12BitState = AD983X_SLEEP_ENABLED;

        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType));
        AD9833_DeassertFrameSync(ad983xHandle);
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Power down DAC only (SLEEP12 bit). MCLK continues running.
 |
 | Sets SLEEP1=0, SLEEP12=1 in the control register.
 | The DAC is powered down while the internal clock and NCO
 | continue running. This is useful when only a digital
 | (SIGN BIT OUT) output is needed on AD9834.
 |
 | Call AD983x_Wake() to re-enable the DAC.
 | No MCLK stabilisation delay is needed since the clock
 | was never disabled.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_DacOff(AD983x_Handle_t * ad983xHandle)
{
    AD983x_Status_t driverStatus;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        ad983xHandle->sleep1BitState = AD983X_SLEEP_DISABLED;
        ad983xHandle->sleep12BitState = AD983X_SLEEP_ENABLED;

        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType));
        AD9833_DeassertFrameSync(ad983xHandle);
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Leave sleep mode by clearing SLEEP1 and SLEEP12.
 |
 | If MCLK was stopped, give it a little time to settle after wake.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD983x_Wake(AD983x_Handle_t * ad983xHandle)
{
    AD983x_Status_t driverStatus;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        ad983xHandle->sleep1BitState = AD983X_SLEEP_DISABLED;
        ad983xHandle->sleep12BitState = AD983X_SLEEP_DISABLED;

        AD9833_AssertFrameSync(ad983xHandle);
        driverStatus = AD9833_WriteSerialWord16(ad983xHandle, AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType));
        AD9833_DeassertFrameSync(ad983xHandle);
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Select whether AD9834 uses control bits or external pins for
 |        FREQ and PHASE register selection.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD9834_SetRegSelectMode(AD983x_Handle_t * ad983xHandle, AD9834_RegSelectMode_t mode)
{
    AD983x_Status_t driverStatus;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        if (ad983xHandle->deviceType != AD983X_DEVICE_AD9834)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else if ((uint32_t)mode > (uint32_t)AD9834_REG_SELECT_PIN)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else
        {
            ad983xHandle->ad9834RegSelectMode = mode;

            AD9833_AssertFrameSync(ad983xHandle);
            driverStatus = AD9833_WriteSerialWord16(ad983xHandle, AD9833_BuildControlRegisterWord(ad983xHandle, ad983xHandle->currentWaveformType));
            AD9833_DeassertFrameSync(ad983xHandle);
        }
    }

    return driverStatus;
}

/*------------------------------------------------------------------------------------------------------
 | @brief Tell the driver whether AD9834 VIN is connected.
 |        Comparator mode is allowed only when this is enabled.
 -------------------------------------------------------------------------------------------------------*/
AD983x_Status_t AD9834_SetComparatorVinAvailable(AD983x_Handle_t * ad983xHandle, AD9834_ComparatorVinState_t vinState)
{
    AD983x_Status_t driverStatus;

    driverStatus = AD9833_ValidateDriverHandle(ad983xHandle);

    if (driverStatus == AD983X_STATUS_OK)
    {
        if (ad983xHandle->deviceType != AD983X_DEVICE_AD9834)
        {
            driverStatus = AD983X_STATUS_ERROR_PARAM;
        }
        else
        {
            ad983xHandle->ad9834ComparatorVinAvailable =
                (vinState == AD9834_COMPARATOR_VIN_AVAILABLE) ? 1u : 0u;
        }
    }

    return driverStatus;
}
