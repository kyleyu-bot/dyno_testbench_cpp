#pragma once

// ── Dyno Testbench IPC Types ──────────────────────────────────────────────────
//
// Shared data structures for communication between the C++ EtherCAT bridge
// and external interfaces (ROS2, Qt/PySide6, UDP clients).
//
// Wire format: newline-delimited JSON over UDP (localhost).
//
// Telemetry  (C++ bridge → clients):  port 7600
// Commands   (clients → C++ bridge):  port 7601
// ─────────────────────────────────────────────────────────────────────────────

#include <cstdint>
#include <string>

namespace dyno::ipc {

// ── Telemetry (published by C++ bridge each print cycle) ──────────────────────

struct DriveTelementry {
    std::string al_state;        // "OP", "SAFE-OP", etc.
    std::string cia402_state;    // "OP_ENABLED", "FAULT", etc.
    int32_t     cmd_velocity  = 0;
    int32_t     fb_velocity   = 0;
    int         mode_display  = 0;
    uint16_t    status_word   = 0;
    uint16_t    error_code    = 0;
};

struct Telemetry {
    uint64_t       cycle      = 0;
    int            wkc        = 0;
    double         cycle_us   = 0.0;

    DriveTelementry main_drive;
    DriveTelementry dut;

    uint32_t       encoder_count = 0;

    double         ch1_voltage   = 0.0;
    double         ch1_torque    = 0.0;
    double         ch2_voltage   = 0.0;
    double         ch2_torque    = 0.0;

    bool           out1          = false;
};

// ── Command (received by C++ bridge, applied to EtherCAT loop) ────────────────

struct Command {
    int32_t  main_speed    = 0;
    int32_t  dut_speed     = 0;
    bool     main_enable   = false;
    bool     dut_enable    = false;
    bool     fault_reset   = false;
    bool     hold_output1  = false;
};

// ── Ports ─────────────────────────────────────────────────────────────────────

static constexpr uint16_t TELEMETRY_PORT = 7600;
static constexpr uint16_t COMMAND_PORT   = 7601;
static constexpr uint16_t MAX_PACKET     = 4096;

} // namespace dyno::ipc
