#pragma once

#include "ethercat_core/devices/motor_drives/drive_bases/ds402/data_types.hpp"
#include <cstdint>

namespace ethercat_core::ds402 {

// Decode CiA 402 logical state from the 16-bit statusword (0x6041).
Cia402State decodeCia402State(uint16_t status_word);

// Statusword bit fields decoded into named booleans.
struct StatuswordBits {
    bool ready_to_switch_on  = false;
    bool switched_on         = false;
    bool operation_enabled   = false;
    bool fault               = false;
    bool voltage_enabled     = false;
    bool quick_stop_active   = false;   // NOTE: active-low — bit 5 clear = active
    bool switch_on_disabled  = false;
    bool warning             = false;
    bool remote              = false;
    bool target_reached      = false;
};

StatuswordBits decodeStatuswordBits(uint16_t status_word);

// Generate the DS402 controlword (0x6040) that drives the state machine
// toward OPERATION_ENABLED given the current logical state and the caller's
// enable / clear_fault intent.
uint16_t controlwordFromCommand(bool enable_drive, bool clear_fault, Cia402State current_state);

// Integer clamping helpers.
inline int32_t clampI32(int64_t v) {
    if (v < -2147483648LL) return -2147483648;
    if (v >  2147483647LL) return  2147483647;
    return static_cast<int32_t>(v);
}

inline int16_t clampI16(int32_t v) {
    if (v < -32768) return -32768;
    if (v >  32767) return  32767;
    return static_cast<int16_t>(v);
}

} // namespace ethercat_core::ds402
