#include "ethercat_core/devices/beckhoff/el5032/adapter.hpp"
#include <cstring>

namespace ethercat_core::beckhoff::el5032 {

std::vector<uint8_t> El5032Adapter::packRxPdo(const std::any& /*command*/) {
    return {};  // encoder terminal — no output
}

std::any El5032Adapter::unpackTxPdo(
    const uint8_t* data, int size,
    uint64_t /*seq*/, int64_t /*stamp_ns*/,
    int64_t /*cycle_time_ns*/, int64_t /*dc_error_ns*/)
{
    Data d;
    if (data && size > 0) {
        d.raw_pdo.assign(data, data + size);
    }

    if (size >= TX_PDO_SIZE) {
        // Read first 4 bytes as signed int32 for the raw value.
        int32_t raw_s = 0;
        std::memcpy(&raw_s, data, 4);
        d.encoder_value_raw = raw_s;

        // Encoder count: bits [40:16] of the 10-byte buffer, masked to 25 bits.
        // Build a 64-bit unsigned from the first 8 bytes, shift right 16.
        uint64_t wide = 0;
        std::memcpy(&wide, data, sizeof(wide));
        d.encoder_count_25bit = static_cast<uint32_t>((wide >> 16u) & ENCODER_MASK_25BIT);
    }

    return d;
}

} // namespace ethercat_core::beckhoff::el5032
