#pragma once

// Novanta Volcano data types — structurally identical to Everest but kept
// as a separate namespace to allow independent evolution.
#include "ethercat_core/devices/motor_drives/Novanta/Everest/data_types.hpp"

namespace ethercat_core::novanta::volcano {

// Re-use Everest's Command and DriveStatus directly.
using Command     = ethercat_core::novanta::everest::Command;
using DriveStatus = ethercat_core::novanta::everest::DriveStatus;

} // namespace ethercat_core::novanta::volcano
