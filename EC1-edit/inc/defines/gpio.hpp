/**
 * @file
 *
 * GPIO preprocessor defines.
 */

#pragma once

#define FOR_ALL_BUTTONS(FUNC)                                                          \
    FUNC(USER_1)                                                                       \
    FUNC(USER_2)                                                                       \
    FUNC(USER_3)

#define FOR_ALL_LED_GPIO_OUTPUTS(FUNC)                                                 \
    FUNC(LED1)                                                                         \
    FUNC(LED2)                                                                         \
    FUNC(LED3)                                                                         \
    FUNC(LED4)

#define FOR_ALL_GPIO_INPUTS(FUNC)                                                      \
    FUNC(SWD_DETECT)                                                                   \
    FUNC(MSD_DETECT)                                                                   \
    FUNC(LEAK_DETECT)                                                                  \
    FUNC(STPR_HOME)                                                                    \
    FUNC(STPR_DCO)                                                                     \
    FUNC(STPR_DIAG0)                                                                   \
    FUNC(VLV_HOME)                                                                     \
    FOR_ALL_BUTTONS(FUNC)

#define FOR_ALL_GPIO_OUTPUTS(FUNC)                                                     \
    FUNC(EXP_RESET)                                                                    \
    FUNC(LCD_BL)                                                                       \
    FUNC(LCD_RESET)                                                                    \
    FUNC(SPI1_NSS1)                                                                    \
    FUNC(SPI2_NSS1)                                                                    \
    FUNC(SPI2_NSS2)                                                                    \
    FUNC(SPI2_NSS3)                                                                    \
    FUNC(SPI2_NSS4)                                                                    \
    FUNC(SPI2_NSS5)                                                                    \
    FUNC(HTR_EN)                                                                       \
    FUNC(STPR_EN)                                                                      \
    FUNC(STPR_DCEN)                                                                    \
    FUNC(STPR_DIR)                                                                     \
    FUNC(STPR_STEP)                                                                    \
    FUNC(VLV_MTR_1)                                                                    \
    FUNC(VLV_MTR_2)                                                                    \
    FUNC(SOL_VALVE)                                                                    \
    FOR_ALL_LED_GPIO_OUTPUTS(FUNC)

#define FOR_ALL_GPIOS(FUNC)                                                            \
    FOR_ALL_GPIO_INPUTS(FUNC)                                                          \
    FOR_ALL_GPIO_OUTPUTS(FUNC)
