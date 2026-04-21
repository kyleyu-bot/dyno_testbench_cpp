#pragma once

#include "ethercat_core/devices/base.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Everest/data_types.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Everest/pdo.hpp"
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

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

    std::unordered_map<std::string, SdoReadSpec> startupReadSpecs() const override;

    uint16_t lastStatusWord() const { return last_status_word_; }

    float getGearRatio() const { return gear_ratio_; }

    // Computes and stores gear_ratio = 1 / sensor_ratio.
    // Throws std::runtime_error if sensor_ratio is too close to zero.
    void computeGearRatio(float sensor_ratio) override {
        if (std::abs(sensor_ratio) < 1e-4f) {
            throw std::runtime_error(
                "Improper gear ratio for slave '" + identity_.name +
                "': sensor_ratio=" + std::to_string(sensor_ratio) +
                " is zero or below threshold (0x2364).");
        }
        gear_ratio_ = 1.0f / sensor_ratio;
    }

    // Applies position and velocity limits read from startup SDOs.
    // max_vel_rev_s : 0x21E8 in rev/s  → stored as mrev/s (*1000)
    // min_pos_raw   : 0x21EA raw int32 (read as float)
    // max_pos_raw   : 0x21EB raw int32 (read as float)
    // If both position values are 0, limits are set to int32 min/max.
    void applyLimits(float max_vel_rev_s, float min_pos_raw, float max_pos_raw) override {
        max_velocity_abs_ = max_vel_rev_s * 1000.0f;
        const int32_t lo = static_cast<int32_t>(min_pos_raw);
        const int32_t hi = static_cast<int32_t>(max_pos_raw);
        if (lo == 0 && hi == 0) {
            min_position_ = std::numeric_limits<int32_t>::min();
            max_position_ = std::numeric_limits<int32_t>::max();
        } else {
            min_position_ = lo;
            max_position_ = hi;
        }
    }

private:
    PdoScaling scaling_;
    uint16_t   last_status_word_ = 0;
    float      gear_ratio_       = 1.0f;
    float      max_velocity_abs_ = 0.0f;
    int32_t    min_position_     = 0;
    int32_t    max_position_     = 0;
};

} // namespace ethercat_core::novanta::everest
