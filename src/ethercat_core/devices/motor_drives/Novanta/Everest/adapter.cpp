#include "ethercat_core/devices/motor_drives/Novanta/Everest/adapter.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Everest/pdo.hpp"

namespace ethercat_core::novanta::everest {

std::vector<uint8_t> NovantaEverestAdapter::packRxPdo(const std::any& command) {
    if (!command.has_value()) {
        return std::vector<uint8_t>(RX_PDO_SIZE, 0);
    }
    const auto& cmd = std::any_cast<const Command&>(command);
    return packCommand(cmd, last_status_word_, &scaling_);
}

std::any NovantaEverestAdapter::unpackTxPdo(
    const uint8_t* data, int size,
    uint64_t seq, int64_t stamp_ns,
    int64_t cycle_time_ns, int64_t dc_error_ns)
{
    DriveStatus status = unpackStatus(
        data, size, seq, stamp_ns, cycle_time_ns, dc_error_ns, &scaling_
    );
    last_status_word_ = status.status_word;
    return status;
}

std::unordered_map<std::string, SdoReadSpec> NovantaEverestAdapter::startupReadSpecs() {
    return {
        {"torque_loop_max_output", {.name="torque_loop_max_output", .index=0x2527, .subindex=0x00, .data_type="f32"}},
        {"torque_loop_min_output", {.name="torque_loop_min_output", .index=0x2528, .subindex=0x00, .data_type="f32"}},
        {"velocity_loop_kp",       {.name="velocity_loop_kp",       .index=0x250A, .subindex=0x00, .data_type="f32"}},
        {"velocity_loop_ki",       {.name="velocity_loop_ki",       .index=0x250B, .subindex=0x00, .data_type="f32"}},
        {"velocity_loop_kd",       {.name="velocity_loop_kd",       .index=0x250C, .subindex=0x00, .data_type="f32"}},
        {"position_loop_kp",       {.name="position_loop_kp",       .index=0x2511, .subindex=0x00, .data_type="f32"}},
        {"position_loop_ki",       {.name="position_loop_ki",       .index=0x2512, .subindex=0x00, .data_type="f32"}},
        {"position_loop_kd",       {.name="position_loop_kd",       .index=0x2513, .subindex=0x00, .data_type="f32"}},
        {"motor_kt",               {.name="motor_kt",               .index=0x243B, .subindex=0x00, .data_type="f32"}},
        {"sensor_ratio",           {.name="sensor_ratio",           .index=0x2364, .subindex=0x00, .data_type="f32"}},
    };
}

} // namespace ethercat_core::novanta::everest
