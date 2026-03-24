#include "ethercat_core/devices/motor_drives/drive_bases/ds402/pdo.hpp"

namespace ethercat_core::ds402 {

Cia402State decodeCia402State(uint16_t sw) {
    const uint16_t s004f = sw & 0x004Fu;
    const uint16_t s006f = sw & 0x006Fu;

    if (s004f == 0x0000u) return Cia402State::NOT_READY_TO_SWITCH_ON;
    if (s004f == 0x0040u) return Cia402State::SWITCH_ON_DISABLED;
    if (s006f == 0x0021u) return Cia402State::READY_TO_SWITCH_ON;
    if (s006f == 0x0023u) return Cia402State::SWITCHED_ON;
    if (s006f == 0x0027u) return Cia402State::OPERATION_ENABLED;
    if (s006f == 0x0007u) return Cia402State::QUICK_STOP_ACTIVE;
    if (s004f == 0x000Fu) return Cia402State::FAULT_REACTION_ACTIVE;
    if (s004f == 0x0008u) return Cia402State::FAULT;
    return Cia402State::NOT_READY_TO_SWITCH_ON;
}

StatuswordBits decodeStatuswordBits(uint16_t sw) {
    StatuswordBits b;
    b.ready_to_switch_on = (sw & (1u << 0)) != 0;
    b.switched_on        = (sw & (1u << 1)) != 0;
    b.operation_enabled  = (sw & (1u << 2)) != 0;
    b.fault              = (sw & (1u << 3)) != 0;
    b.voltage_enabled    = (sw & (1u << 4)) != 0;
    b.quick_stop_active  = (sw & (1u << 5)) == 0;  // active-low
    b.switch_on_disabled = (sw & (1u << 6)) != 0;
    b.warning            = (sw & (1u << 7)) != 0;
    b.remote             = (sw & (1u << 9)) != 0;
    b.target_reached     = (sw & (1u << 10)) != 0;
    return b;
}

uint16_t controlwordFromCommand(bool enable_drive, bool clear_fault, Cia402State state) {
    if (clear_fault)
        return 0x0080u;  // Fault reset — sent unconditionally; drive ignores it when not faulted

    if (!enable_drive) {
        switch (state) {
        case Cia402State::OPERATION_ENABLED:
        case Cia402State::SWITCHED_ON:
        case Cia402State::READY_TO_SWITCH_ON:
        case Cia402State::SWITCH_ON_DISABLED:
        case Cia402State::QUICK_STOP_ACTIVE:
            return 0x0006u;  // Shutdown
        default:
            return 0x0000u;  // Disable voltage
        }
    }

    switch (state) {
    case Cia402State::SWITCH_ON_DISABLED:   return 0x0006u;  // → Ready to switch on
    case Cia402State::READY_TO_SWITCH_ON:   return 0x0007u;  // Switch on
    case Cia402State::SWITCHED_ON:          return 0x000Fu;  // Enable operation
    case Cia402State::OPERATION_ENABLED:    return 0x000Fu;  // Hold enabled
    case Cia402State::QUICK_STOP_ACTIVE:    return 0x000Fu;  // Recover
    case Cia402State::FAULT:
        return clear_fault ? 0x0080u : 0x0000u;
    default:
        return 0x0000u;
    }
}

} // namespace ethercat_core::ds402
