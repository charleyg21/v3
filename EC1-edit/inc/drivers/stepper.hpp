/**
 * @file
 *
 * Stepper motor controller driver functions.
 */

#pragma once

#include <cstdint>
#include <cstring>

#include "defines/status.hpp"

namespace drivers::stepper {
namespace registers {

    template <typename T> struct addresses;

    struct alignas(uint32_t) GCONF {
        bool recalibrate : 1;
        bool fastandstill : 1;
        bool en_pwm_mode : 1;
        bool multistep_filt : 1;
        bool shaft : 1;
        bool diag0_error : 1;
        bool diag0_otpw : 1;
        bool diag0_stall : 1;

        bool diag1_stall : 1;
        bool diag1_index : 1;
        bool diag1_onstate : 1;
        bool diag1_steps_skipped : 1;
        bool diag0_int_pushpull : 1;
        bool diag1_pushpull : 1;
        bool small_hysteresis : 1;
        bool stop_enable : 1;

        bool direct_mode : 1;
        bool test_mode : 1;
        unsigned int _unused0 : 6;

        unsigned int _unused1 : 8;
    } __attribute__((__packed__));

    static_assert(
        sizeof(GCONF) == sizeof(uint32_t), "Invalid size of GCONF register model.");

    template <> struct addresses<GCONF> {
        static constexpr uint8_t value = 0x00;
    };

    struct alignas(uint32_t) GSTAT {
        bool reset : 1;
        bool drv_err : 1;
        bool uv_cp : 1;
        unsigned int _unused0 : 5;
        unsigned int _unused1 : 24;
    } __attribute__((__packed__));

    static_assert(
        sizeof(GSTAT) == sizeof(uint32_t), "Invalid size of GSTATE register model.");

    template <> struct addresses<GSTAT> {
        static constexpr uint8_t value = 0x01;
    };

    struct alignas(uint32_t) IOIN {
        bool STEP : 1;
        bool DIR : 1;
        bool DCEN_CFG4 : 1;
        bool DCEN_CFG5 : 1;
        bool DRV_ENN : 1;
        bool DCO_CFG6 : 1;
        unsigned int _unused0 : 2;
        unsigned int _unused1 : 16;
        unsigned int VERSION : 8;
    } __attribute__((__packed__));

    static_assert(
        sizeof(IOIN) == sizeof(uint32_t), "Invalid size of IOIN register model.");

    template <> struct addresses<IOIN> {
        static constexpr uint8_t value = 0x04;
    };

    struct alignas(uint32_t) SHORT_CONF {
        unsigned int S2VS_LEVEL : 4;
        unsigned int _unused0 : 4;
        unsigned int S2G_LEVEL : 4;
        unsigned int _unused1 : 4;
        unsigned int SHORTFILTER : 2;
        unsigned int shortdelay : 1;
        unsigned int _unused2 : 13;
    } __attribute__((__packed__));

    static_assert(sizeof(SHORT_CONF) == sizeof(uint32_t),
        "Invalid size of SHORT_CONF register model.");

    template <> struct addresses<SHORT_CONF> {
        static constexpr uint8_t value = 0x09;
    };

    struct alignas(uint32_t) IHOLD_IRUN {
        unsigned int IHOLD : 5;
        unsigned int _unused0 : 3;
        unsigned int IRUN : 5;
        unsigned int _unused1 : 3;
        unsigned int IHOLDDELAY : 4;
        unsigned int _unused2 : 12;
    } __attribute__((__packed__));

    static_assert(sizeof(IHOLD_IRUN) == sizeof(uint32_t),
        "Invalid size of IHOLD_IRUN register model.");

    template <> struct addresses<IHOLD_IRUN> {
        static constexpr uint8_t value = 0x10;
    };

    struct alignas(uint32_t) TPOWERDOWN {
        unsigned int TPOWERDOWN : 8;
        unsigned int _unused : 24;
    } __attribute__((__packed__));

    static_assert(sizeof(TPOWERDOWN) == sizeof(uint32_t),
        "Invalid size of TPOWERDOWN register model.");

    template <> struct addresses<TPOWERDOWN> {
        static constexpr uint8_t value = 0x11;
    };

    struct alignas(uint32_t) TPWMTHRS {
        unsigned int TPWMTHRS : 20;
        unsigned int _unused : 12;
    } __attribute__((__packed__));

    static_assert(sizeof(TPWMTHRS) == sizeof(uint32_t),
        "Invalid size of TPWMTHRS register model.");

    template <> struct addresses<TPWMTHRS> {
        static constexpr uint8_t value = 0x13;
    };

    struct alignas(uint32_t) CHOPCONF {
        unsigned int toff : 4;
        unsigned int hstrt : 3;
        unsigned int hend : 4;
        unsigned int fd3 : 1;
        bool disfdcc : 1;
        unsigned int _reserved0 : 1;
        bool chm : 1;
        unsigned int tbl : 2;
        unsigned int _reserved1 : 1;
        bool vhighfs : 1;
        bool vhighchm : 1;
        unsigned int tpfd : 4;
        unsigned int mres : 4;
        bool intpol : 1;
        bool dedge : 1;
        bool diss2g : 1;
        bool diss2vs : 1;
    } __attribute__((__packed__));

    static_assert(sizeof(CHOPCONF) == sizeof(uint32_t),
        "Invalid size of CHOPCONF register model.");

    template <> struct addresses<CHOPCONF> {
        static constexpr uint8_t value = 0x6C;
    };

    struct alignas(uint32_t) DRV_STATUS {
        unsigned int SG_RESULT : 10;
        unsigned int _reserved0 : 1;
        unsigned int _reserved1 : 1;
        bool s2vsa : 1;
        bool s2vsb : 1;
        bool stealth : 1;
        bool fsactive : 1;
        unsigned int CS_ACTUAL : 5;
        unsigned int _reserved2 : 3;
        bool stallGuard : 1;
        bool ot : 1;
        bool otpw : 1;
        bool s2ga : 1;
        bool s2gb : 1;
        bool ola : 1;
        bool olb : 1;
        bool stst : 1;
    } __attribute__((__packed__));

    static_assert(sizeof(DRV_STATUS) == sizeof(uint32_t),
        "Invalid size of DRV_STATUS register model.");

    template <> struct addresses<DRV_STATUS> {
        static constexpr uint8_t value = 0x6F;
    };

    template <typename Register>
    constexpr uint8_t addresses_v = addresses<Register>::value;

} // namespace registers

struct alignas(uint8_t) Preamble {
    unsigned int address : 7;
    bool write_flag : 1;
} __attribute__((__packed__));

static_assert(sizeof(Preamble) == sizeof(uint8_t), "Invalid size of preamble model.");

template <typename Register, typename WriteBytes,
    uint8_t address = registers::addresses_v<Register>>
auto write_register(Register register_, WriteBytes& write_bytes)
{
    union {
        Preamble as_struct;
        uint8_t as_bytes[sizeof(as_struct)];
    } preamble_union = { .as_struct = { .address = address, .write_flag = true } };

    const auto preamble_status
        = write_bytes(preamble_union.as_bytes, sizeof(preamble_union.as_bytes));

    if (not defines::status::is_ok(preamble_status)) {
        return preamble_status;
    }

    union {
        Register as_struct;
        uint32_t as_word;
        uint8_t as_bytes[sizeof(as_struct)];
    } register_union = { .as_struct = register_ };

    register_union.as_word = __builtin_bswap32(register_union.as_word);
    return write_bytes(register_union.as_bytes, sizeof(register_union.as_bytes));
}

struct alignas(uint8_t) Status {
    bool reset_flag : 1;
    bool driver_error : 1;
    bool sg2 : 1;
    bool standstill : 1;
    unsigned int _unused : 4;
} __attribute__((__packed__));

static_assert(sizeof(Status) == sizeof(uint8_t), "Invalid size of status model.");

template <typename Register, typename WriteBytes,
    uint8_t address = registers::addresses_v<Register>>
auto request_read_register(WriteBytes& write_bytes)
{
    union PreambleUnion {
        Preamble as_struct;
        uint8_t as_bytes[sizeof(as_struct)];
    } preamble_union = { .as_struct = { .address = address, .write_flag = false } };

    const auto preamble_write_status
        = write_bytes(preamble_union.as_bytes, sizeof(preamble_union.as_bytes));

    if (not defines::status::is_ok(preamble_write_status)) {
        return preamble_write_status;
    }

    union RegisterUnion {
        Register as_struct;
        uint32_t as_word;
        uint8_t as_bytes[sizeof(as_struct)];
    } register_union = { .as_word = 0 };

    return write_bytes(register_union.as_bytes, sizeof(register_union.as_bytes));
}

template <typename Register, typename ReadBytes,
    uint8_t address = registers::addresses_v<Register>>
auto read_register(Status& status, Register& register_, ReadBytes& read_bytes)
{
    union StatusUnion {
        Status as_struct;
        uint8_t as_bytes[sizeof(as_struct)];
    } status_union = {};

    const auto status_read_status
        = read_bytes(status_union.as_bytes, sizeof(status_union.as_bytes));

    if (not defines::status::is_ok(status_read_status)) {
        return status_read_status;
    }

    union RegisterUnion {
        Register as_struct;
        uint32_t as_word;
        uint8_t as_bytes[sizeof(as_struct)];
    } register_union = { .as_word = 0 };

    const auto register_read_status
        = read_bytes(register_union.as_bytes, sizeof(register_union.as_bytes));

    if (not defines::status::is_ok(register_read_status)) {
        return register_read_status;
    }

    status = status_union.as_struct;

    register_union.as_word = __builtin_bswap32(register_union.as_word);
    register_ = register_union.as_struct;

    return register_read_status;
}

} // namespace drivers::stepper
