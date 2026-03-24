#pragma once

#include "ethercat_core/devices/base.hpp"
#include "ethercat_core/devices/beckhoff/el2004/data_types.hpp"

namespace ethercat_core::beckhoff::el2004 {

class El2004Adapter : public ISlaveAdapter {
public:
    explicit El2004Adapter(SlaveIdentity id) : ISlaveAdapter(std::move(id)) {}

    int rxPdoSize() const override { return 1; }
    int txPdoSize() const override { return 0; }

    std::vector<uint8_t> packRxPdo(const std::any& command) override;
    std::any unpackTxPdo(
        const uint8_t* data, int size,
        uint64_t seq = 0, int64_t stamp_ns = 0,
        int64_t cycle_time_ns = 0, int64_t dc_error_ns = 0
    ) override;
};

} // namespace ethercat_core::beckhoff::el2004
