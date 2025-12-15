/**
 * @file
 *
 * Driver functions for GPIO pins.
 */

#pragma once

#include "main.h"

#include "defines/gpio.hpp"
#include "defines/status.hpp"

namespace drivers::gpio {

namespace interface {

    class InputPin {
    public:
        using Status = defines::Status;
        virtual GPIO_PinState read(void) const = 0;
    };

    class OutputPin {
    public:
        using Status = defines::Status;
        virtual void write(GPIO_PinState state) const = 0;
        void set(void) const { write(GPIO_PIN_SET); }
        void reset(void) const { write(GPIO_PIN_RESET); }
    };

} // namespace interface

namespace dummy {

    class InputPin : public interface::InputPin {
        GPIO_PinState read(void) const override { return GPIO_PIN_RESET; }
    };

    class OutputPin : public interface::OutputPin {
    public:
        using Status = defines::Status;
        void write(GPIO_PinState state) const override { UNUSED(state); }
    };

} // namespace dummy

namespace regular {

    class InputPin : public interface::InputPin {
    public:
        using Status = defines::Status;
        InputPin(GPIO_TypeDef* port, uint16_t pin)
            : _port(port)
            , _pin(pin)
        {
        }

        GPIO_PinState read(void) const override
        {
            return HAL_GPIO_ReadPin(_port, _pin);
        }

    private:
        GPIO_TypeDef* const _port;
        const uint16_t _pin;
    };

    class OutputPin : public interface::OutputPin {
    public:
        using Status = defines::Status;
        OutputPin(GPIO_TypeDef* port, uint16_t pin)
            : _port(port)
            , _pin(pin)
        {
        }

        void write(GPIO_PinState state) const override
        {
            HAL_GPIO_WritePin(_port, _pin, state);
        }

    private:
        GPIO_TypeDef* const _port;
        const uint16_t _pin;
    };

} // namespace regular

namespace pins {

    namespace types {

#define DECLARE_REGULAR_INPUT_PIN_CLASS(NAME)                                          \
    class NAME : public regular::InputPin {                                            \
    public:                                                                            \
        NAME(void)                                                                     \
            : regular::InputPin(NAME##_GPIO_Port, PIN)                                 \
        {                                                                              \
        }                                                                              \
        GPIO_TypeDef* const PORT = NAME##_GPIO_Port;                                   \
        static constexpr uint16_t PIN = NAME##_Pin;                                    \
        static void on_rising(void);                                                   \
        static void on_falling(void);                                                  \
    };

#define DECLARE_DUMMY_INPUT_PIN_CLASS(NAME)                                            \
    class NAME : public dummy::InputPin {                                              \
    public:                                                                            \
        NAME(void) { }                                                                 \
        static constexpr GPIO_TypeDef* PORT = nullptr;                                 \
        static constexpr uint16_t PIN = 0;                                             \
        static void on_rising(void);                                                   \
        static void on_falling(void);                                                  \
    };

#if defined(SWD_DETECT_Pin) && defined(SWD_DETECT_GPIO_Port)
        DECLARE_REGULAR_INPUT_PIN_CLASS(SWD_DETECT)
#else
#warning "SWD_DETECT pin not defined."
        DECLARE_DUMMY_INPUT_PIN_CLASS(SWD_DETECT)
#endif

#if defined(MSD_DETECT_Pin) && defined(MSD_DETECT_GPIO_Port)
        DECLARE_REGULAR_INPUT_PIN_CLASS(MSD_DETECT)
#else
#warning "MSD_DETECT pin not defined."
        DECLARE_DUMMY_INPUT_PIN_CLASS(MSD_DETECT)
#endif

#if defined(LEAK_DETECT_Pin) && defined(LEAK_DETECT_GPIO_Port)
        DECLARE_REGULAR_INPUT_PIN_CLASS(LEAK_DETECT)
#else
#warning "LEAK_DETECT pin not defined."
        DECLARE_DUMMY_INPUT_PIN_CLASS(LEAK_DETECT)
#endif

#if defined(STPR_HOME_Pin) && defined(STPR_HOME_GPIO_Port)
        DECLARE_REGULAR_INPUT_PIN_CLASS(STPR_HOME)
#else
#warning "STPR_HOME pin not defined."
        DECLARE_DUMMY_INPUT_PIN_CLASS(STPR_HOME)
#endif

#if defined(STPR_DCO_Pin) && defined(STPR_DCO_GPIO_Port)
        DECLARE_REGULAR_INPUT_PIN_CLASS(STPR_DCO)
#else
#warning "STPR_DCO pin not defined."
        DECLARE_DUMMY_INPUT_PIN_CLASS(STPR_DCO)
#endif

#if defined(STPR_DIAG0_Pin) && defined(STPR_DIAG0_GPIO_Port)
        DECLARE_REGULAR_INPUT_PIN_CLASS(STPR_DIAG0)
#else
#warning "STPR_DIAG0 pin not defined."
        DECLARE_DUMMY_INPUT_PIN_CLASS(STPR_DIAG0)
#endif

#if defined(VLV_HOME_Pin) && defined(VLV_HOME_GPIO_Port)
        DECLARE_REGULAR_INPUT_PIN_CLASS(VLV_HOME)
#else
#warning "VLV_HOME pin not defined."
        DECLARE_DUMMY_INPUT_PIN_CLASS(VLV_HOME)
#endif

#if defined(USER_1_Pin) && defined(USER_1_GPIO_Port)
        DECLARE_REGULAR_INPUT_PIN_CLASS(USER_1)
#else
#warning "USER_1 pin not defined."
        DECLARE_DUMMY_INPUT_PIN_CLASS(USER_1)
#endif

#if defined(USER_2_Pin) && defined(USER_2_GPIO_Port)
        DECLARE_REGULAR_INPUT_PIN_CLASS(USER_2)
#else
#warning "USER_2 pin not defined."
        DECLARE_DUMMY_INPUT_PIN_CLASS(USER_2)
#endif

#if defined(USER_3_Pin) && defined(USER_3_GPIO_Port)
        DECLARE_REGULAR_INPUT_PIN_CLASS(USER_3)
#else
#warning "USER_3 pin not defined."
        DECLARE_DUMMY_INPUT_PIN_CLASS(USER_3)
#endif

#undef DECLARE_REGULAR_INPUT_PIN_CLASS
#undef DECLARE_DUMMY_INPUT_PIN_CLASS

#define DECLARE_REGULAR_OUTPUT_PIN_CLASS(NAME)                                         \
    class NAME : public regular::OutputPin {                                           \
    public:                                                                            \
        NAME(void)                                                                     \
            : regular::OutputPin(NAME##_GPIO_Port, PIN)                                \
        {                                                                              \
        }                                                                              \
        GPIO_TypeDef* const PORT = NAME##_GPIO_Port;                                   \
        static constexpr uint16_t PIN = NAME##_Pin;                                    \
    };

#define DECLARE_DUMMY_OUTPUT_PIN_CLASS(NAME)                                           \
    class NAME : public dummy::OutputPin {                                             \
    public:                                                                            \
        NAME(void) { }                                                                 \
        static constexpr GPIO_TypeDef* PORT = nullptr;                                 \
        static constexpr uint16_t PIN = 0;                                             \
    };

#if defined(EXP_RESET_Pin) && defined(EXP_RESET_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(EXP_RESET)
#else
#warning "EXP_RESET pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(EXP_RESET)
#endif

#if defined(LCD_BL_Pin) && defined(LCD_BL_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(LCD_BL)
#else
#warning "LCD_BL pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(LCD_BL)
#endif

#if defined(LCD_RESET_Pin) && defined(LCD_RESET_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(LCD_RESET)
#else
#warning "LCD_RESET pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(LCD_RESET)
#endif

#if defined(SPI1_NSS1_Pin) && defined(SPI1_NSS1_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(SPI1_NSS1)
#else
#warning "SPI1_NSS1 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(SPI1_NSS1)
#endif

#if defined(SPI2_NSS1_Pin) && defined(SPI2_NSS1_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(SPI2_NSS1)
#else
#warning "SPI2_NSS1 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(SPI2_NSS1)
#endif

#if defined(SPI2_NSS2_Pin) && defined(SPI2_NSS2_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(SPI2_NSS2)
#else
#warning "SPI2_NSS2 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(SPI2_NSS2)
#endif

#if defined(SPI2_NSS3_Pin) && defined(SPI2_NSS3_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(SPI2_NSS3)
#else
#warning "SPI2_NSS3 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(SPI2_NSS3)
#endif

#if defined(SPI2_NSS4_Pin) && defined(SPI2_NSS4_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(SPI2_NSS4)
#else
#warning "SPI2_NSS4 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(SPI2_NSS4)
#endif

#if defined(SPI2_NSS5_Pin) && defined(SPI2_NSS5_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(SPI2_NSS5)
#else
#warning "SPI2_NSS5 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(SPI2_NSS5)
#endif

#if defined(HTR_EN_Pin) && defined(HTR_EN_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(HTR_EN)
#else
#warning "HTR_EN pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(HTR_EN)
#endif

#if defined(STPR_EN_Pin) && defined(STPR_EN_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(STPR_EN)
#else
#warning "STPR_EN pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(STPR_EN)
#endif

#if defined(STPR_DCEN_Pin) && defined(STPR_DCEN_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(STPR_DCEN)
#else
#warning "STPR_DCEN pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(STPR_DCEN)
#endif

#if defined(STPR_DIR_Pin) && defined(STPR_DIR_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(STPR_DIR)
#else
#warning "STPR_DIR pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(STPR_DIR)
#endif

#if defined(STPR_STEP_Pin) && defined(STPR_STEP_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(STPR_STEP)
#else
#warning "STPR_STEP pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(STPR_DIR)
#endif

#if defined(VLV_MTR_1_Pin) && defined(VLV_MTR_1_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(VLV_MTR_1)
#else
#warning "VLV_MTR_1 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(VLV_MTR_1)
#endif

#if defined(VLV_MTR_2_Pin) && defined(VLV_MTR_2_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(VLV_MTR_2)
#else
#warning "VLV_MTR_2 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(VLV_MTR_2)
#endif

#if defined(SOL_VALVE_Pin) && defined(SOL_VALVE_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(SOL_VALVE)
#else
#warning "SOL_VALVE pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(SOL_VALVE)
#endif

#if defined(LED1_Pin) && defined(LED1_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(LED1)
#else
#warning "LED1 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(LED1)
#endif

#if defined(LED2_Pin) && defined(LED2_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(LED2)
#else
#warning "LED2 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(LED2)
#endif

#if defined(LED3_Pin) && defined(LED3_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(LED3)
#else
#warning "LED3 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(LED3)
#endif

#if defined(LED4_Pin) && defined(LED4_GPIO_Port)
        DECLARE_REGULAR_OUTPUT_PIN_CLASS(LED4)
#else
#warning "LED4 pin not defined."
        DECLARE_DUMMY_OUTPUT_PIN_CLASS(LED4)
#endif

#undef DECLARE_REGULAR_OUTPUT_PIN_CLASS
#undef DECLARE_DUMMY_OUTPUT_PIN_CLASS

    } // namespace types

#define DECLARE_PIN_OBJECT(NAME) extern types::NAME NAME;
    FOR_ALL_GPIOS(DECLARE_PIN_OBJECT)
#undef DECLARE_PIN_OBJECT

} // namspace pins

} // namespace drivers::gpio
