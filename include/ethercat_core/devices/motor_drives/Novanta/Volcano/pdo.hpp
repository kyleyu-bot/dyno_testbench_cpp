#pragma once

// Novanta Volcano PDO — same wire format as Everest.
// Re-exports the Everest PDO constants and packed structs.
#include "ethercat_core/devices/motor_drives/Novanta/Everest/pdo.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/data_types.hpp"
#include <cstdint>
#include <vector>

namespace ethercat_core::novanta::volcano {

using PdoScaling = ethercat_core::novanta::everest::PdoScaling;
using RxPdo      = ethercat_core::novanta::everest::RxPdo;
using TxPdo      = ethercat_core::novanta::everest::TxPdo;

static constexpr int RX_PDO_SIZE       = ethercat_core::novanta::everest::RX_PDO_SIZE;
static constexpr int TX_PDO_SIZE       = ethercat_core::novanta::everest::TX_PDO_SIZE;
static constexpr int LEGACY_TX_PDO_SIZE = ethercat_core::novanta::everest::LEGACY_TX_PDO_SIZE;

std::vector<uint8_t> packCommand(
    const Command& cmd,
    uint16_t       current_status_word = 0,
    const PdoScaling* scaling          = nullptr
);

DriveStatus unpackStatus(
    const uint8_t* data,
    int            size,
    uint64_t       seq           = 0,
    int64_t        stamp_ns      = 0,
    int64_t        cycle_time_ns = 0,
    int64_t        dc_error_ns   = 0,
    const PdoScaling* scaling    = nullptr
);

} // namespace ethercat_core::novanta::volcano
