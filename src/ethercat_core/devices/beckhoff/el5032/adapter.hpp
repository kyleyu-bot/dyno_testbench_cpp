#pragma once

#include "ethercat_core/devices/base.hpp"
#include "ethercat_core/devices/beckhoff/el5032/data_types.hpp"

namespace ethercat_core::beckhoff::el5032 {

class El5032Adapter : public ISlaveAdapter {
public:
    explicit El5032Adapter(SlaveIdentity id) : ISlaveAdapter(std::move(id)) {}

    int rxPdoSize() const override { return 0; }
    int txPdoSize() const override { return TX_PDO_SIZE; }

    std::vector<uint8_t> packRxPdo(const std::any& command) override;
    std::any unpackTxPdo(
        const uint8_t* data, int size,
        uint64_t seq = 0, int64_t stamp_ns = 0,
        int64_t cycle_time_ns = 0, int64_t dc_error_ns = 0
    ) override;
};

} // namespace ethercat_core::beckhoff::el5032
