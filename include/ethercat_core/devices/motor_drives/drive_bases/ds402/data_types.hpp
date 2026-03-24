#pragma once

#include <cstdint>

namespace ethercat_core::ds402 {

// CiA 402 mode of operation (object 0x6060).
enum class ModeOfOperation : int8_t {
    NO_MODE               = 0,
    PROFILE_POSITION      = 1,
    PROFILE_VELOCITY      = 2,
    PROFILE_TORQUE        = 4,
    CYCLIC_SYNC_POSITION  = 8,
    CYCLIC_SYNC_VELOCITY  = 9,
    CYCLIC_SYNC_TORQUE    = 10,
};

// Logical drive states decoded from the statusword (object 0x6041).
// These are symbolic states, not raw bitmasks.
enum class Cia402State : uint8_t {
    NOT_READY_TO_SWITCH_ON = 0,
    SWITCH_ON_DISABLED     = 1,
    READY_TO_SWITCH_ON     = 2,
    SWITCHED_ON            = 3,
    OPERATION_ENABLED      = 4,
    QUICK_STOP_ACTIVE      = 5,
    FAULT_REACTION_ACTIVE  = 6,
    FAULT                  = 7,
};

} // namespace ethercat_core::ds402
