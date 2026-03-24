#include "ethercat_core/devices/beckhoff/el2004/adapter.hpp"
#include <stdexcept>

namespace ethercat_core::beckhoff::el2004 {

std::vector<uint8_t> El2004Adapter::packRxPdo(const std::any& command) {
    if (!command.has_value()) {
        return std::vector<uint8_t>(1, 0);
    }
    const auto& cmd = std::any_cast<const Command&>(command);
    uint8_t val = 0;
    if (cmd.output_1) val |= 0x01u;
    if (cmd.output_2) val |= 0x02u;
    if (cmd.output_3) val |= 0x04u;
    if (cmd.output_4) val |= 0x08u;
    return {val};
}

std::any El2004Adapter::unpackTxPdo(
    const uint8_t* data, int size,
    uint64_t /*seq*/, int64_t /*stamp_ns*/,
    int64_t /*cycle_time_ns*/, int64_t /*dc_error_ns*/)
{
    Status s;
    if (data && size >= 1) {
        s.output_byte = data[0];
    }
    return s;
}

} // namespace ethercat_core::beckhoff::el2004
