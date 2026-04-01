#pragma once

#include "ethercat_core/devices/base.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Everest/data_types.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Everest/pdo.hpp"
#include <stdexcept>
#include <unordered_map>
#include <string>

namespace ethercat_core::novanta::everest {

class NovantaEverestAdapter : public ISlaveAdapter {
public:
    explicit NovantaEverestAdapter(
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

    // Returns the SDO read specs that should be read during master startup.
    // Keys match the field names in Command for convenient direct assignment.
    static std::unordered_map<std::string, SdoReadSpec> startupReadSpecs();

    uint16_t lastStatusWord() const { return last_status_word_; }

    float getGearRatio() const { return gear_ratio_; }

    // Computes and stores gear_ratio = 1 / sensor_ratio.
    // Throws std::runtime_error if sensor_ratio is too close to zero.
    void computeGearRatio(float sensor_ratio) {
        if (std::abs(sensor_ratio) < 1e-4f) {
            throw std::runtime_error(
                "Improper gear ratio for slave '" + identity_.name +
                "': sensor_ratio=" + std::to_string(sensor_ratio) +
                " is zero or below threshold (0x2364).");
        }
        gear_ratio_ = 1.0f / sensor_ratio;
    }

private:
    PdoScaling scaling_;
    uint16_t   last_status_word_ = 0;
    float      gear_ratio_       = 1.0f;
};

} // namespace ethercat_core::novanta::everest
