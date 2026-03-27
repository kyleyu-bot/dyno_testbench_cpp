#pragma once

#include <any>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace ethercat_core {

// EtherCAT Application Layer states (same bit encoding as ec_state in SOEM).
enum class AlState : uint8_t {
    INIT             = 0x01,
    PRE_OPERATIONAL  = 0x02,
    BOOTSTRAP        = 0x03,
    SAFE_OPERATIONAL = 0x04,
    OPERATIONAL      = 0x08,
    ERROR_FLAG       = 0x10,  // OR'd with base state when an AL error is present
};

// Per-cycle multi-slave command container, keyed by configured slave name.
// The value type is std::any so each slave can carry its own typed command
// struct without requiring a common base class.
struct SystemCommand {
    std::unordered_map<std::string, std::any> by_slave;
    uint64_t seq      = 0;
    int64_t  stamp_ns = 0;
};

// Per-cycle multi-slave status container, keyed by configured slave name.
struct SystemStatus {
    std::unordered_map<std::string, std::any> by_slave;
    uint64_t seq      = 0;
    int64_t  stamp_ns = 0;
};

} // namespace ethercat_core
