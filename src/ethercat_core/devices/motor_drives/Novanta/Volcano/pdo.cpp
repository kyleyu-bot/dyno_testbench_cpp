#include "ethercat_core/devices/motor_drives/Novanta/Volcano/pdo.hpp"
// Volcano shares the exact same PDO wire format as Everest.
// Delegate to the Everest pack/unpack functions.
#include "ethercat_core/devices/motor_drives/Novanta/Everest/pdo.hpp"

namespace ethercat_core::novanta::volcano {

std::vector<uint8_t> packCommand(
    const Command& cmd,
    uint16_t       current_status_word,
    const PdoScaling* scaling)
{
    return ethercat_core::novanta::everest::packCommand(cmd, current_status_word, scaling);
}

DriveStatus unpackStatus(
    const uint8_t* data, int size,
    uint64_t seq, int64_t stamp_ns,
    int64_t cycle_time_ns, int64_t dc_error_ns,
    const PdoScaling* scaling)
{
    return ethercat_core::novanta::everest::unpackStatus(
        data, size, seq, stamp_ns, cycle_time_ns, dc_error_ns, scaling
    );
}

} // namespace ethercat_core::novanta::volcano
