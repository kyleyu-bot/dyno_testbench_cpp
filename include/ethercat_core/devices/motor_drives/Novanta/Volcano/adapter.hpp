#pragma once

#include "ethercat_core/devices/base.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/data_types.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/pdo.hpp"
#include <unordered_map>
#include <string>

namespace ethercat_core::novanta::volcano {

class NovantaVolcanoAdapter : public ISlaveAdapter {
public:
    explicit NovantaVolcanoAdapter(
        SlaveIdentity id,
        PdoScaling    scaling = {}
    ) : ISlaveAdapter(std::move(id)), scaling_(scaling) {}

    int rxPdoSize() const override { return RX_PDO_SIZE; }
    int txPdoSize() const override { return TX_PDO_SIZE; }

    std::vector<uint8_t> packRxPdo(const std::any& command) override;
    std::any unpackTxPdo(
        const uint8_t* data, int size,
        uint64_t seq = 0, int64_t stamp_ns = 0,
        int64_t cycle_time_ns = 0, int64_t dc_error_ns = 0
    ) override;

    static std::unordered_map<std::string, SdoReadSpec> startupReadSpecs();

    uint16_t lastStatusWord() const { return last_status_word_; }

private:
    PdoScaling scaling_;
    uint16_t   last_status_word_ = 0;
};

} // namespace ethercat_core::novanta::volcano
