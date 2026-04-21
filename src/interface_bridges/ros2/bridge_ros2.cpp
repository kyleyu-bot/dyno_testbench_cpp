// Dyno ROS2 Bridge
//
// Single C++ process: runs the EtherCAT loop and publishes telemetry directly
// to ROS2 topics using rclcpp.  Subscribes to a command topic to receive
// speed/enable commands from other ROS2 nodes or scripts.
//
// Build with colcon (after building the parent CMake project first):
//   cd <repo_root>
//   cmake -S . -B build && cmake --build build
//   source /opt/ros/humble/setup.bash
//   colcon build --packages-select dyno_ros2_bridge
//                --base-paths src/interface_bridges/ros2
//
// Run:
//   source install/setup.bash
//   sudo ros2 run dyno_ros2_bridge bridge_ros2 [--ros-args -p topology:=...]
//
// Topics published (std_msgs/String, JSON):
//   /dyno/main_drive/status
//   /dyno/dut/status
//   /dyno/loop/stats
//
// Topics published (primitive):
//   /dyno/encoder/count     (std_msgs/UInt32)
//   /dyno/torque/ch1        (std_msgs/Float64)
//   /dyno/torque/ch2        (std_msgs/Float64)
//
// Topics subscribed:
//   /dyno/command           (std_msgs/String, JSON)
//     Fields: main_speed, dut_speed, main_enable, dut_enable,
//             fault_reset, hold_output1

#include "ethercat_core/data_types.hpp"
#include "ethercat_core/loop.hpp"
#include "ethercat_core/master.hpp"
#include "ethercat_core/default_adapter_factory.hpp"
#include "ethercat_core/devices/beckhoff/el2004/adapter.hpp"
#include "ethercat_core/devices/beckhoff/el2004/data_types.hpp"
#include "ethercat_core/devices/beckhoff/el3002/adapter.hpp"
#include "ethercat_core/devices/beckhoff/el3002/data_types.hpp"
#include "ethercat_core/devices/beckhoff/el5032/adapter.hpp"
#include "ethercat_core/devices/beckhoff/el5032/data_types.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/adapter.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/data_types.hpp"
#include "ethercat_core/devices/motor_drives/drive_bases/ds402/data_types.hpp"

#include "pdo_log.hpp"
#include "testbench_utils/function_generator.hpp"

extern "C" {
#include "ethercat.h"
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <cmath>
#include <limits>
#include <any>
#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <signal.h>
#include <cstring>
#include <memory>
#include <mutex>
#include <set>
#include <pthread.h>
#include <sched.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

using namespace ethercat_core;
using namespace ethercat_core::novanta::volcano;
using Cia402State     = ethercat_core::ds402::Cia402State;
using ModeOfOperation = ethercat_core::ds402::ModeOfOperation;
using json            = nlohmann::json;

// ── Signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_shutdown{false};
static std::atomic<bool> g_rotate_log{false};
static void onSignal(int) { g_shutdown.store(true); }

// ── Defaults ──────────────────────────────────────────────────────────────────

static constexpr const char* DEFAULT_TOPOLOGY      = "config/ethercat_device_config/topology.dyno2.template6.json";
static constexpr const char* DEFAULT_DRIVE_SLAVE   = "main_drive";
static constexpr const char* DEFAULT_DUT_SLAVE     = "dut";
static constexpr const char* DEFAULT_ENCODER_SLAVE = "encoder_interface";
static constexpr const char* DEFAULT_TORQUE_SLAVE  = "analog_input_interface";
static constexpr const char* DEFAULT_IO_SLAVE      = "digital_IO";
static constexpr double      DEFAULT_PUB_HZ        = 200.0;
static constexpr double      DEFAULT_FAULT_RESET   = 0.5;

// ── Shared command state (written by ROS2 subscriber, read by main loop) ──────

struct CommandState {
    float    main_speed    = 0.0f;  // rad/s, converted to mrev/s before sending
    float    dut_speed     = 0.0f;
    float    main_position = 0.0f;  // rad, converted to enc_cnt before sending
    float    dut_position  = 0.0f;
    float    main_torque   = 0.0f;
    float    dut_torque    = 0.0f;
    float    main_current  = 0.0f;
    float    dut_current   = 0.0f;
    bool     main_enable   = false;
    bool     dut_enable    = false;
    bool     fault_reset   = false;
    bool     hold_output1  = false;
    int8_t   main_mode     = static_cast<int8_t>(ModeOfOperation::CYCLIC_SYNC_VELOCITY);
    int8_t   dut_mode      = static_cast<int8_t>(ModeOfOperation::CYCLIC_SYNC_VELOCITY);
    // Control gains (seeded from startup SDO; overridable via /dyno/command)
    float    main_torque_kp       = 0.0f;
    float    main_torque_max_out  = 0.0f;
    float    main_torque_min_out  = 0.0f;
    float    main_vel_kp          = 0.0f;
    float    main_vel_ki          = 0.0f;
    float    main_vel_kd          = 0.0f;
    float    main_pos_kp          = 0.0f;
    float    main_pos_ki          = 0.0f;
    float    main_pos_kd          = 0.0f;
    float    dut_torque_kp        = 0.0f;
    float    dut_torque_max_out   = 0.0f;
    float    dut_torque_min_out   = 0.0f;
    float    dut_vel_kp           = 0.0f;
    float    dut_vel_ki           = 0.0f;
    float    dut_vel_kd           = 0.0f;
    float    dut_pos_kp           = 0.0f;
    float    dut_pos_ki           = 0.0f;
    float    dut_pos_kd           = 0.0f;
    // Torque sensor ADC scale (Nm); use current value as default so omitted
    // command messages leave the scale unchanged.
    float    ch1_torque_scale     = 200.0f;  // matches El3002Adapter ch1 default
    float    ch2_torque_scale     = 20.0f;   // matches El3002Adapter ch2 default
    // One-shot zero flags — cleared by the bridge after applying.
    bool     zero_torque_ch1      = false;
    bool     zero_torque_ch2      = false;
    // One-shot log-rotation flag — triggers drain thread to close and reopen CSV.
    bool     save_log             = false;
};

static std::mutex      g_cmd_mutex;
static CommandState    g_cmd_state;
static bool            g_loop_running = false;

// ── SDO request / response (written by ROS2 subscriber, executed by main loop) ─

struct SdoRequest {
    bool     pending      = false;
    bool     is_write     = false;
    bool     is_pre_op    = false;  // pre_op_all / pre_op_off
    bool     is_store_all = false;  // atomic: pre-op → write 0x26DB → return to OP
    int      slave_idx    = 0;
    uint16_t index        = 0;
    uint8_t  subindex     = 0;
    int      size         = 4;   // bytes: 1, 2, or 4
    int64_t  value        = 0;   // for writes; for pre_op: 0=enter, 1=exit
};
struct SdoResponse {
    std::string op;
    uint16_t    index    = 0;
    uint8_t     subindex = 0;
    int         size     = 0;
    bool        success  = false;
    int64_t     value    = 0;
    std::string error;
};

static std::mutex   g_sdo_mutex;
static SdoRequest   g_sdo_req;

// ── DS402 helpers ─────────────────────────────────────────────────────────────

static const char* cia402Name(Cia402State s) {
    switch (s) {
    case Cia402State::NOT_READY_TO_SWITCH_ON: return "NOT_READY";
    case Cia402State::SWITCH_ON_DISABLED:     return "SW_ON_DISABLED";
    case Cia402State::READY_TO_SWITCH_ON:     return "READY";
    case Cia402State::SWITCHED_ON:            return "SWITCHED_ON";
    case Cia402State::OPERATION_ENABLED:      return "OP_ENABLED";
    case Cia402State::QUICK_STOP_ACTIVE:      return "QUICK_STOP";
    case Cia402State::FAULT_REACTION_ACTIVE:  return "FAULT_REACTION";
    case Cia402State::FAULT:                  return "FAULT";
    }
    return "UNKNOWN";
}

// ── ROS2 node ─────────────────────────────────────────────────────────────────

class DynoBridgeNode : public rclcpp::Node {
public:
    DynoBridgeNode() : Node("dyno_ros2_bridge") {
        // Publishers
        pub_main_  = create_publisher<std_msgs::msg::String>("/dyno/main_drive/status", 10);
        pub_dut_   = create_publisher<std_msgs::msg::String>("/dyno/dut/status",         10);
        pub_stats_ = create_publisher<std_msgs::msg::String>("/dyno/loop/stats",          10);
        pub_enc_   = create_publisher<std_msgs::msg::UInt32>("/dyno/encoder/count",       10);
        pub_ch1_t_ = create_publisher<std_msgs::msg::Float64>("/dyno/torque/ch1",         10);
        pub_ch2_t_ = create_publisher<std_msgs::msg::Float64>("/dyno/torque/ch2",         10);
        pub_sdo_   = create_publisher<std_msgs::msg::String>("/dyno/sdo_response",        10);
        pub_bus_   = create_publisher<std_msgs::msg::String>("/dyno/bus_status",          10);

        // Command subscriber
        sub_cmd_ = create_subscription<std_msgs::msg::String>(
            "/dyno/command", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    const json j = json::parse(msg->data);
                    std::lock_guard<std::mutex> lk(g_cmd_mutex);
                    g_cmd_state.main_speed    = j.value("main_velocity", j.value("main_speed", 0.0f));
                    g_cmd_state.dut_speed     = j.value("dut_velocity",  j.value("dut_speed",  0.0f));
                    g_cmd_state.main_position = j.value("main_position", 0.0f);
                    g_cmd_state.dut_position  = j.value("dut_position",  0.0f);
                    g_cmd_state.main_torque   = j.value("main_torque",   0.0f);
                    g_cmd_state.dut_torque    = j.value("dut_torque",    0.0f);
                    g_cmd_state.main_current  = j.value("main_current",  0.0f);
                    g_cmd_state.dut_current   = j.value("dut_current",   0.0f);
                    g_cmd_state.main_enable   = j.value("main_enable",   false);
                    g_cmd_state.dut_enable    = j.value("dut_enable",    false);
                    g_cmd_state.fault_reset   = j.value("fault_reset",   false);
                    g_cmd_state.hold_output1  = j.value("hold_output1",  false);
                    g_cmd_state.main_mode     = static_cast<int8_t>(j.value("main_mode", static_cast<int>(ModeOfOperation::CYCLIC_SYNC_VELOCITY)));
                    g_cmd_state.dut_mode      = static_cast<int8_t>(j.value("dut_mode",  static_cast<int>(ModeOfOperation::CYCLIC_SYNC_VELOCITY)));
                    // Gains: use current value as default so omitted fields persist
                    g_cmd_state.main_torque_kp      = j.value("main_torque_kp",      g_cmd_state.main_torque_kp);
                    g_cmd_state.main_torque_max_out = j.value("main_torque_max",      g_cmd_state.main_torque_max_out);
                    g_cmd_state.main_torque_min_out = j.value("main_torque_min",      g_cmd_state.main_torque_min_out);
                    g_cmd_state.main_vel_kp         = j.value("main_vel_kp",          g_cmd_state.main_vel_kp);
                    g_cmd_state.main_vel_ki         = j.value("main_vel_ki",          g_cmd_state.main_vel_ki);
                    g_cmd_state.main_vel_kd         = j.value("main_vel_kd",          g_cmd_state.main_vel_kd);
                    g_cmd_state.main_pos_kp         = j.value("main_pos_kp",          g_cmd_state.main_pos_kp);
                    g_cmd_state.main_pos_ki         = j.value("main_pos_ki",          g_cmd_state.main_pos_ki);
                    g_cmd_state.main_pos_kd         = j.value("main_pos_kd",          g_cmd_state.main_pos_kd);
                    g_cmd_state.dut_torque_kp       = j.value("dut_torque_kp",        g_cmd_state.dut_torque_kp);
                    g_cmd_state.dut_torque_max_out  = j.value("dut_torque_max",       g_cmd_state.dut_torque_max_out);
                    g_cmd_state.dut_torque_min_out  = j.value("dut_torque_min",       g_cmd_state.dut_torque_min_out);
                    g_cmd_state.dut_vel_kp          = j.value("dut_vel_kp",           g_cmd_state.dut_vel_kp);
                    g_cmd_state.dut_vel_ki          = j.value("dut_vel_ki",           g_cmd_state.dut_vel_ki);
                    g_cmd_state.dut_vel_kd          = j.value("dut_vel_kd",           g_cmd_state.dut_vel_kd);
                    g_cmd_state.dut_pos_kp          = j.value("dut_pos_kp",           g_cmd_state.dut_pos_kp);
                    g_cmd_state.dut_pos_ki          = j.value("dut_pos_ki",           g_cmd_state.dut_pos_ki);
                    g_cmd_state.dut_pos_kd          = j.value("dut_pos_kd",           g_cmd_state.dut_pos_kd);
                    g_cmd_state.ch1_torque_scale    = j.value("ch1_torque_scale",     g_cmd_state.ch1_torque_scale);
                    g_cmd_state.ch2_torque_scale    = j.value("ch2_torque_scale",     g_cmd_state.ch2_torque_scale);
                    // One-shot: OR with current so a true is never lost between snapshots.
                    g_cmd_state.zero_torque_ch1    |= j.value("zero_torque_ch1", false);
                    g_cmd_state.zero_torque_ch2    |= j.value("zero_torque_ch2", false);
                    g_cmd_state.save_log           |= j.value("save_log",        false);
                } catch (...) {
                    RCLCPP_WARN(get_logger(), "Failed to parse /dyno/command JSON");
                }
            }
        );

        // SDO request subscriber — stores request; main loop executes and publishes response.
        sub_sdo_ = create_subscription<std_msgs::msg::String>(
            "/dyno/sdo_request", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    const json j = json::parse(msg->data);
                    const std::string op_str = j.value("op", "read");
                    SdoRequest req;
                    req.pending      = true;
                    req.is_write     = (op_str == "write");
                    req.is_pre_op    = (op_str == "pre_op_all" || op_str == "pre_op_off");
                    req.is_store_all = (op_str == "store_all");
                    const std::string drv = j.value("drive", "main");
                    req.slave_idx = (drv == "dut") ? dut_soem_idx_ : drive_soem_idx_;
                    req.index     = static_cast<uint16_t>(
                                        std::stoul(j.value("index", "0"), nullptr, 16));
                    req.subindex  = static_cast<uint8_t>(
                                        std::stoul(j.value("subindex", "0"), nullptr, 16));
                    req.size      = j.value("size", 4);
                    req.value     = req.is_pre_op
                                    ? (op_str == "pre_op_off" ? int64_t{1} : int64_t{0})
                                    : j.value("value", int64_t{0});
                    std::lock_guard<std::mutex> lk(g_sdo_mutex);
                    g_sdo_req = req;
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "Bad /dyno/sdo_request: %s", e.what());
                }
            }
        );

        RCLCPP_INFO(get_logger(), "DynoBridgeNode ready.");
    }

    void setSlaveIndices(int drive_idx, int dut_idx) {
        drive_soem_idx_ = drive_idx;
        dut_soem_idx_   = dut_idx;
    }

    void publishSdoResponse(const SdoResponse& resp) {
        std::ostringstream idx_ss, val_ss;
        idx_ss << "0x" << std::hex << std::uppercase << std::setw(4)
               << std::setfill('0') << resp.index;
        val_ss << "0x" << std::hex << std::uppercase << resp.value;
        json jr;
        jr["op"]        = resp.op;
        jr["index"]     = idx_ss.str();
        jr["subindex"]  = resp.subindex;
        jr["size"]      = resp.size;
        jr["success"]   = resp.success;
        jr["value"]     = resp.value;
        jr["value_hex"] = val_ss.str();
        jr["error"]     = resp.error;
        std_msgs::msg::String out;
        out.data = jr.dump();
        pub_sdo_->publish(out);
    }

    void publishBusStatus() {
        json arr = json::array();
        for (int i = 1; i <= ec_slavecount; ++i) {
            json s;
            s["idx"]  = i;
            s["name"] = std::string(ec_slave[i].name);
            s["al"]   = alStateName(static_cast<int>(ec_slave[i].state));
            arr.push_back(s);
        }
        std_msgs::msg::String msg;
        msg.data = arr.dump();
        pub_bus_->publish(msg);
    }

    void publishTelemetry(
        uint64_t cycle, int wkc, double cycle_us,
        const std::string& main_json,
        const std::string& dut_json,
        uint32_t enc,
        double ch1_t, double ch2_t)
    {
        {
            std_msgs::msg::String msg;
            msg.data = main_json;
            pub_main_->publish(msg);
        }
        {
            std_msgs::msg::String msg;
            msg.data = dut_json;
            pub_dut_->publish(msg);
        }
        {
            std_msgs::msg::String msg;
            msg.data = json{
                {"cycle",   cycle},
                {"wkc",     wkc},
                {"cycle_us", cycle_us},
            }.dump();
            pub_stats_->publish(msg);
        }
        {
            std_msgs::msg::UInt32 msg;
            msg.data = enc;
            pub_enc_->publish(msg);
        }
        {
            std_msgs::msg::Float64 msg;
            msg.data = ch1_t;
            pub_ch1_t_->publish(msg);
        }
        {
            std_msgs::msg::Float64 msg;
            msg.data = ch2_t;
            pub_ch2_t_->publish(msg);
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_main_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_dut_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_stats_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr   pub_enc_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  pub_ch1_t_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  pub_ch2_t_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_sdo_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_bus_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_sdo_;
    int drive_soem_idx_ = 1;
    int dut_soem_idx_   = 2;
};

// ── PDO logging helpers ────────────────────────────────────────────────────────

/// Serialise one PdoLogRecord as a CSV row (no trailing newline).
/// Column order matches dyno::PDO_LOG_CSV_HEADER.
static std::string record_to_csv(const dyno::PdoLogRecord& r)
{
    std::ostringstream o;
    // metadata
    o << r.cycle_count   << ',' << r.stamp_ns      << ',' << r.wkc         << ','
      << r.cycle_time_ns << ',' << r.dc_error_ns   << ',' << r.period_ns   << ',';
    // main tx
    o << r.main_tx_statusword        << ',' << static_cast<int>(r.main_tx_mode_display)  << ','
      << r.main_tx_output_enc_pos    << ',' << r.main_tx_bus_voltage        << ','
      << r.main_tx_torque_nm         << ',' << r.main_tx_motor_temp         << ','
      << r.main_tx_error_code        << ',' << r.main_tx_motor_velocity     << ','
      << r.main_tx_input_enc_pos     << ',' << r.main_tx_position_setpoint  << ','
      << r.main_tx_velocity_setpoint << ',' << r.main_tx_iq_actual          << ','
      << r.main_tx_id_actual         << ',' << r.main_tx_idc_actual         << ','
      << r.main_tx_iq_command        << ',' << r.main_tx_id_command         << ',';
    // main rx
    o << static_cast<int>(r.main_rx_mode_of_operation) << ','
      << r.main_rx_target_position   << ',' << r.main_rx_target_velocity    << ','
      << r.main_rx_torque_command    << ',' << r.main_rx_torque_kp          << ','
      << r.main_rx_torque_max_out    << ',' << r.main_rx_torque_min_out     << ','
      << r.main_rx_vel_kp            << ',' << r.main_rx_vel_ki             << ','
      << r.main_rx_vel_kd            << ',' << r.main_rx_pos_kp             << ','
      << r.main_rx_pos_ki            << ',' << r.main_rx_pos_kd             << ','
      << static_cast<int>(r.main_rx_enable) << ',';
    // dut tx
    o << r.dut_tx_statusword         << ',' << static_cast<int>(r.dut_tx_mode_display)   << ','
      << r.dut_tx_output_enc_pos     << ',' << r.dut_tx_bus_voltage         << ','
      << r.dut_tx_torque_nm          << ',' << r.dut_tx_motor_temp          << ','
      << r.dut_tx_error_code         << ',' << r.dut_tx_motor_velocity      << ','
      << r.dut_tx_input_enc_pos      << ',' << r.dut_tx_position_setpoint   << ','
      << r.dut_tx_velocity_setpoint  << ',' << r.dut_tx_iq_actual           << ','
      << r.dut_tx_id_actual          << ',' << r.dut_tx_idc_actual          << ','
      << r.dut_tx_iq_command         << ',' << r.dut_tx_id_command          << ',';
    // dut rx
    o << static_cast<int>(r.dut_rx_mode_of_operation) << ','
      << r.dut_rx_target_position    << ',' << r.dut_rx_target_velocity     << ','
      << r.dut_rx_torque_command     << ',' << r.dut_rx_torque_kp           << ','
      << r.dut_rx_torque_max_out     << ',' << r.dut_rx_torque_min_out      << ','
      << r.dut_rx_vel_kp             << ',' << r.dut_rx_vel_ki              << ','
      << r.dut_rx_vel_kd             << ',' << r.dut_rx_pos_kp              << ','
      << r.dut_rx_pos_ki             << ',' << r.dut_rx_pos_kd              << ','
      << static_cast<int>(r.dut_rx_enable) << ',';
    // sensors
    o << r.encoder_count << ',' << r.torque_ch1_nm << ',' << r.torque_ch2_nm;
    return o.str();
}

// ── main ──────────────────────────────────────────────────────────────────────

/// Create test_data_log/YYYY-MM-DD/HHMMSS/dyno_pdo.csv, making all parent dirs.
static std::string make_run_csv_path(rclcpp::Logger logger)
{
    std::time_t t    = std::time(nullptr);
    std::tm*    tm_  = std::localtime(&t);
    char date_buf[16], time_buf[8];
    std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%d", tm_);
    std::strftime(time_buf, sizeof(time_buf), "%H%M%S",   tm_);
    std::string run_dir = std::string("test_data_log/") + date_buf + "/" + time_buf;
    std::error_code ec;
    std::filesystem::create_directories(run_dir, ec);
    if (ec) {
        RCLCPP_WARN(logger, "Could not create log dir '%s': %s",
                    run_dir.c_str(), ec.message().c_str());
    }
    return run_dir + "/dyno_pdo.csv";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // ROS2 node runs on its own thread via a SingleThreadedExecutor so it
    // doesn't block the main EtherCAT loop.
    auto node     = std::make_shared<DynoBridgeNode>();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    std::thread ros_thread([&executor]() { executor->spin(); });

    // Read ROS2 parameters (set via --ros-args -p key:=value).
    auto get_str = [&](const char* name, const char* def) -> std::string {
        node->declare_parameter(name, def);
        return node->get_parameter(name).as_string();
    };
    auto get_dbl = [&](const char* name, double def) -> double {
        node->declare_parameter(name, def);
        return node->get_parameter(name).as_double();
    };
    auto get_int = [&](const char* name, int def) -> int {
        node->declare_parameter(name, def);
        return node->get_parameter(name).as_int();
    };

    const std::string topology      = get_str("topology",      DEFAULT_TOPOLOGY);
    const std::string drive_slave   = get_str("drive_slave",   DEFAULT_DRIVE_SLAVE);
    const std::string dut_slave     = get_str("dut_slave",     DEFAULT_DUT_SLAVE);
    const std::string encoder_slave = get_str("encoder_slave", DEFAULT_ENCODER_SLAVE);
    const std::string torque_slave  = get_str("torque_slave",  DEFAULT_TORQUE_SLAVE);
    const std::string io_slave      = get_str("io_slave",      DEFAULT_IO_SLAVE);
    const double      pub_hz        = get_dbl("pub_hz",        DEFAULT_PUB_HZ);
    const double      fault_reset_s = get_dbl("fault_reset_s", DEFAULT_FAULT_RESET);
    const int         rt_priority   = get_int("rt_priority",   95);
    const std::string cpu_affinity_str = get_str("cpu_affinity", "2");
    const bool        debug_print   = get_int("debug",         0) != 0;

    // EtherCAT init.
    MasterConfig cfg;
    try {
        cfg = loadTopology(topology);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Topology load failed: %s", e.what());
        executor->cancel();
        ros_thread.join();
        return 1;
    }

    EthercatMaster master(cfg, ethercat_core::makeDefaultAdapterFactory());
    MasterRuntime* rt = nullptr;
    try {
        rt = &master.initialize();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Master init failed: %s", e.what());
        executor->cancel();
        ros_thread.join();
        return 1;
    }

    // Required slaves — fatal if missing.
    for (const auto& name : {drive_slave, encoder_slave, torque_slave, io_slave}) {
        if (rt->adapters.find(name) == rt->adapters.end()) {
            RCLCPP_FATAL(node->get_logger(), "Required slave '%s' not found in topology.", name.c_str());
            master.close();
            executor->cancel();
            ros_thread.join();
            return 1;
        }
    }

    // DUT is optional.
    const bool dut_present = rt->adapters.find(dut_slave) != rt->adapters.end();
    if (dut_present) {
        RCLCPP_INFO(node->get_logger(), "DUT slave '%s' found.", dut_slave.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "DUT slave '%s' not found — running without DUT.", dut_slave.c_str());
    }

    auto* el3002 = dynamic_cast<beckhoff::el3002::El3002Adapter*>(
        rt->adapters.at(torque_slave).get());
    if (!el3002) {
        RCLCPP_FATAL(node->get_logger(), "Slave '%s' is not an ELM3002.", torque_slave.c_str());
        master.close();
        executor->cancel();
        ros_thread.join();
        return 1;
    }

    const int drive_soem_idx = rt->slave_index.at(drive_slave);
    const int dut_soem_idx   = dut_present ? rt->slave_index.at(dut_slave) : -1;

    RCLCPP_INFO(node->get_logger(),
        "[init] SOEM indices — %s=%d  %s=%d",
        drive_slave.c_str(), drive_soem_idx,
        dut_slave.c_str(), dut_soem_idx);

    node->setSlaveIndices(drive_soem_idx, dut_soem_idx);

    // Extract startup gains for both drives from SDO reads done during init.
    struct DriveGains {
        float torque_kp              = 0.0f;
        float torque_loop_max_output = 0.0f;
        float torque_loop_min_output = 0.0f;
        float velocity_loop_kp       = 0.0f;
        float velocity_loop_ki       = 0.0f;
        float velocity_loop_kd       = 0.0f;
        float position_loop_kp       = 0.0f;
        float position_loop_ki       = 0.0f;
        float position_loop_kd       = 0.0f;
    };

    auto extractGains = [&](const std::string& slave_name) -> DriveGains {
        DriveGains g;
        auto it = rt->startup_params.find(slave_name);
        if (it == rt->startup_params.end()) return g;
        const auto& p = it->second;
        auto get = [&](const char* k) -> float {
            auto pit = p.find(k); return pit != p.end() ? pit->second : 0.0f;
        };
        const float kt = get("motor_kt");
        g.torque_kp              = (std::abs(kt) > 1e-9f) ? (1.0f / kt) : 0.0f;
        g.torque_loop_max_output = get("torque_loop_max_output");
        g.torque_loop_min_output = get("torque_loop_min_output");
        g.velocity_loop_kp       = get("velocity_loop_kp");
        g.velocity_loop_ki       = get("velocity_loop_ki");
        g.velocity_loop_kd       = get("velocity_loop_kd");
        g.position_loop_kp       = get("position_loop_kp");
        g.position_loop_ki       = get("position_loop_ki");
        g.position_loop_kd       = get("position_loop_kd");
        return g;
    };

    const DriveGains main_gains = extractGains(drive_slave);
    const DriveGains dut_gains  = extractGains(dut_slave);

    // Seed g_cmd_state gains from startup SDO values so they persist even when
    // the command topic doesn't explicitly send gain fields.
    {
        std::lock_guard<std::mutex> lk(g_cmd_mutex);
        g_cmd_state.main_torque_kp      = main_gains.torque_kp;
        g_cmd_state.main_torque_max_out = main_gains.torque_loop_max_output;
        g_cmd_state.main_torque_min_out = main_gains.torque_loop_min_output;
        g_cmd_state.main_vel_kp         = main_gains.velocity_loop_kp;
        g_cmd_state.main_vel_ki         = main_gains.velocity_loop_ki;
        g_cmd_state.main_vel_kd         = main_gains.velocity_loop_kd;
        g_cmd_state.main_pos_kp         = main_gains.position_loop_kp;
        g_cmd_state.main_pos_ki         = main_gains.position_loop_ki;
        g_cmd_state.main_pos_kd         = main_gains.position_loop_kd;
        g_cmd_state.dut_torque_kp       = dut_gains.torque_kp;
        g_cmd_state.dut_torque_max_out  = dut_gains.torque_loop_max_output;
        g_cmd_state.dut_torque_min_out  = dut_gains.torque_loop_min_output;
        g_cmd_state.dut_vel_kp          = dut_gains.velocity_loop_kp;
        g_cmd_state.dut_vel_ki          = dut_gains.velocity_loop_ki;
        g_cmd_state.dut_vel_kd          = dut_gains.velocity_loop_kd;
        g_cmd_state.dut_pos_kp          = dut_gains.position_loop_kp;
        g_cmd_state.dut_pos_ki          = dut_gains.position_loop_ki;
        g_cmd_state.dut_pos_kd          = dut_gains.position_loop_kd;
    }

    RCLCPP_INFO(node->get_logger(),
        "[main_drive] vel_kp=%.4f vel_ki=%.4f torque_kp=%.4f",
        static_cast<double>(main_gains.velocity_loop_kp),
        static_cast<double>(main_gains.velocity_loop_ki),
        static_cast<double>(main_gains.torque_kp));
    RCLCPP_INFO(node->get_logger(),
        "[dut]        vel_kp=%.4f vel_ki=%.4f torque_kp=%.4f",
        static_cast<double>(dut_gains.velocity_loop_kp),
        static_cast<double>(dut_gains.velocity_loop_ki),
        static_cast<double>(dut_gains.torque_kp));

    LoopRtConfig rt_cfg;
    rt_cfg.rt_priority = std::clamp(rt_priority, 0, 99);
    // Parse comma-separated CPU affinity list (e.g. "2" or "2,3").
    if (!cpu_affinity_str.empty()) {
        std::istringstream ss(cpu_affinity_str);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try { rt_cfg.cpu_affinity.insert(std::stoi(token)); }
            catch (...) {
                RCLCPP_WARN(node->get_logger(),
                    "Ignoring invalid cpu_affinity token: '%s'", token.c_str());
            }
        }
    }

    // Use sigaction instead of std::signal so we reliably override the handler
    // that rclcpp::init() installs via sigaction for SIGINT.
    struct sigaction sa{};
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    EthercatLoop loop(*rt, cfg.cycle_hz, rt_cfg);

    // ── CSV logging setup ─────────────────────────────────────────────────────
    std::string   log_path = make_run_csv_path(node->get_logger());
    std::ofstream csv_file(log_path);
    csv_file << dyno::PDO_LOG_CSV_HEADER << '\n';
    RCLCPP_INFO(node->get_logger(), "PDO log: %s", log_path.c_str());

    // Ring buffer shared between the RT cycle callback and the drain thread.
    // Depth 200 = ~200 ms of headroom at 1000 Hz before the drain thread catches up.
    dyno::PdoLogBuffer<200> log_buf;

    // Drain thread: pops records from the ring buffer and writes directly to CSV.
    // When g_rotate_log is set (Save Log button), it flushes the current file,
    // opens a new timestamped file, and continues logging there.
    std::thread log_drain([&]() {
        while (!g_shutdown.load() || !log_buf.empty()) {
            while (auto rec = log_buf.pop()) {
                csv_file << record_to_csv(*rec) << '\n';
            }
            if (g_rotate_log.exchange(false)) {
                csv_file.flush();
                csv_file.close();
                log_path = make_run_csv_path(node->get_logger());
                csv_file.open(log_path);
                csv_file << dyno::PDO_LOG_CSV_HEADER << '\n';
                RCLCPP_INFO(node->get_logger(), "PDO log rotated: %s", log_path.c_str());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        csv_file.close();
    });

    // ── Register cycle callback — fires from RT thread at EtherCAT cycle rate ─
    // Captures by reference; all referenced objects outlive loop.stop().
    loop.setCycleCallback([&](const SystemStatus& status, const LoopStats& stats) {
        dyno::PdoLogRecord rec;
        rec.cycle_count   = stats.cycle_count;
        rec.stamp_ns      = status.stamp_ns;
        rec.wkc           = stats.last_wkc;
        rec.cycle_time_ns = stats.last_cycle_time_ns;
        rec.dc_error_ns   = stats.last_dc_error_ns;
        rec.period_ns     = stats.last_period_ns;

        auto fill_tx = [&](const DriveStatus& ds, bool is_main) {
            if (is_main) {
                rec.main_tx_statusword        = ds.status_word;
                rec.main_tx_mode_display      = ds.mode_of_operation_display;
                rec.main_tx_output_enc_pos    = ds.measured_output_side_position_raw_cnt;
                rec.main_tx_bus_voltage       = ds.bus_voltage;
                rec.main_tx_torque_nm         = ds.measured_torque_nm;
                rec.main_tx_motor_temp        = ds.motor_temp;
                rec.main_tx_error_code        = ds.error_code;
                rec.main_tx_motor_velocity    = ds.measured_input_side_velocity_raw;
                rec.main_tx_input_enc_pos     = ds.input_encoder_pos;
                rec.main_tx_position_setpoint = ds.position_setpoint;
                rec.main_tx_velocity_setpoint = ds.velocity_command_received;
                rec.main_tx_iq_actual         = ds.iq_actual;
                rec.main_tx_id_actual         = ds.id_actual;
                rec.main_tx_idc_actual        = ds.idc_actual;
                rec.main_tx_iq_command        = ds.iq_command;
                rec.main_tx_id_command        = ds.id_command;
            } else {
                rec.dut_tx_statusword         = ds.status_word;
                rec.dut_tx_mode_display       = ds.mode_of_operation_display;
                rec.dut_tx_output_enc_pos     = ds.measured_output_side_position_raw_cnt;
                rec.dut_tx_bus_voltage        = ds.bus_voltage;
                rec.dut_tx_torque_nm          = ds.measured_torque_nm;
                rec.dut_tx_motor_temp         = ds.motor_temp;
                rec.dut_tx_error_code         = ds.error_code;
                rec.dut_tx_motor_velocity     = ds.measured_input_side_velocity_raw;
                rec.dut_tx_input_enc_pos      = ds.input_encoder_pos;
                rec.dut_tx_position_setpoint  = ds.position_setpoint;
                rec.dut_tx_velocity_setpoint  = ds.velocity_command_received;
                rec.dut_tx_iq_actual          = ds.iq_actual;
                rec.dut_tx_id_actual          = ds.id_actual;
                rec.dut_tx_idc_actual         = ds.idc_actual;
                rec.dut_tx_iq_command         = ds.iq_command;
                rec.dut_tx_id_command         = ds.id_command;
            }
        };

        auto main_it = status.by_slave.find(drive_slave);
        if (main_it != status.by_slave.end() && main_it->second.has_value())
            fill_tx(std::any_cast<const DriveStatus&>(main_it->second), true);

        auto dut_it = status.by_slave.find(dut_slave);
        if (dut_present && dut_it != status.by_slave.end() && dut_it->second.has_value())
            fill_tx(std::any_cast<const DriveStatus&>(dut_it->second), false);

        // RxPDO — snapshot current command under lock.
        {
            std::lock_guard<std::mutex> lk(g_cmd_mutex);
            rec.main_rx_mode_of_operation = g_cmd_state.main_mode;
            rec.main_rx_target_position   = g_cmd_state.main_position;
            rec.main_rx_target_velocity   = g_cmd_state.main_speed;
            rec.main_rx_torque_command    = g_cmd_state.main_current;
            rec.main_rx_enable            = g_cmd_state.main_enable;
            rec.dut_rx_mode_of_operation  = g_cmd_state.dut_mode;
            rec.dut_rx_target_position    = g_cmd_state.dut_position;
            rec.dut_rx_target_velocity    = g_cmd_state.dut_speed;
            rec.dut_rx_torque_command     = g_cmd_state.dut_current;
            rec.dut_rx_enable             = g_cmd_state.dut_enable;
        }

        // Sensors from this cycle's status.
        auto enc_it = status.by_slave.find(encoder_slave);
        if (enc_it != status.by_slave.end() && enc_it->second.has_value())
            rec.encoder_count = std::any_cast<const beckhoff::el5032::Data&>(
                enc_it->second).encoder_count_25bit;

        auto torque_it = status.by_slave.find(torque_slave);
        if (torque_it != status.by_slave.end() && torque_it->second.has_value()) {
            const auto& d = std::any_cast<const beckhoff::el3002::Data&>(torque_it->second);
            rec.torque_ch1_nm = el3002->scaledTorqueCh1(d);
            rec.torque_ch2_nm = el3002->scaledTorqueCh2(d);
        }

        log_buf.push(rec);
    });

    loop.start();
    g_loop_running = true;

    // Safe stop/start helpers — guard against double-stop or double-start.
    auto safe_stop  = [&]{ if ( g_loop_running) { loop.stop();  g_loop_running = false; } };
    auto safe_start = [&]{ if (!g_loop_running) { loop.start(); g_loop_running = true;  } };
    auto recover_bus_to_op = [&]() -> std::string {
        ec_slave[0].state = EC_STATE_SAFE_OP;
        ec_writestate(0);
        int safe_chk = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
        if ((safe_chk & 0x0F) != EC_STATE_SAFE_OP) {
            ec_readstate();
            return "Not all slaves reached SAFE-OP";
        }

        // Prime process data before requesting OP — some slaves need a few
        // exchanges before they will transition out of SAFE-OP.
        for (int i = 0; i < 5; ++i) {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
        }

        ec_slave[0].state = EC_STATE_OPERATIONAL;
        ec_writestate(0);

        for (int attempt = 0; attempt < 50; ++attempt) {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_readstate();

            bool all_in_op = true;
            for (int i = 1; i <= ec_slavecount; ++i) {
                if ((ec_slave[i].state & 0x0F) != EC_STATE_OPERATIONAL) {
                    all_in_op = false;
                    break;
                }
            }
            if (all_in_op) {
                return "";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::ostringstream oss;
        oss << "Slaves still not OP:";
        for (int i = 1; i <= ec_slavecount; ++i) {
            if ((ec_slave[i].state & 0x0F) != EC_STATE_OPERATIONAL) {
                oss << " [" << i << "] " << ec_slave[i].name
                    << " state=0x" << std::hex << static_cast<int>(ec_slave[i].state);
            }
        }
        return oss.str();
    };


    // Pin the main thread (publish/command loop) to all CPUs except the ones
    // reserved for the EtherCAT RT loop, so they never compete on the same core.
    {
        const int ncpus = static_cast<int>(std::thread::hardware_concurrency());
        cpu_set_t main_cpuset;
        CPU_ZERO(&main_cpuset);
        for (int i = 0; i < ncpus; ++i) {
            if (rt_cfg.cpu_affinity.find(i) == rt_cfg.cpu_affinity.end())
                CPU_SET(i, &main_cpuset);
        }
        if (pthread_setaffinity_np(pthread_self(), sizeof(main_cpuset), &main_cpuset) != 0) {
            RCLCPP_WARN(node->get_logger(),
                "Failed to set main thread CPU affinity (errno=%d). "
                "Main loop may share CPU with EtherCAT thread.", errno);
        } else {
            RCLCPP_INFO(node->get_logger(),
                "Main thread pinned to all CPUs except RT core(s).");
        }
    }

    RCLCPP_INFO(node->get_logger(),
        "bridge_ros2 running | pub_hz=%.1f fault_reset=%.1fs", pub_hz, fault_reset_s);

    // Extract encoder resolution from topology scaling config.
    int main_out_enc_bits = 20, dut_out_enc_bits = 20;
    for (const auto& sc : cfg.slaves) {
        if (sc.name == drive_slave) main_out_enc_bits = sc.scaling.output_encoder_res_bits;
        if (sc.name == dut_slave)   dut_out_enc_bits  = sc.scaling.output_encoder_res_bits;
    }

    const auto   t0           = std::chrono::steady_clock::now();
    const auto   reset_end    = t0 + std::chrono::duration<double>(fault_reset_s);
    const double pub_period   = 1.0 / std::max(pub_hz, 1.0);
    auto         next_pub     = t0;

    auto make_drive_json = [&](const std::string& slave_name,
                                int soem_idx,
                                float cmd_vel_rad_s,
                                const SystemStatus& status,
                                int out_enc_bits,
                                const DriveGains& gains) -> std::string {
        json j;
        const int al_raw = (soem_idx >= 1) ? static_cast<int>(ec_slave[soem_idx].state) : 0;
        j["al"]     = alStateName(al_raw);
        j["al_num"] = al_raw & 0x0F;   // numeric: INIT=1 PRE-OP=2 SAFE-OP=4 OP=8 (plottable)
        auto it = status.by_slave.find(slave_name);
        if (it != status.by_slave.end() && it->second.has_value()) {
            const auto& ds = std::any_cast<const DriveStatus&>(it->second);
            const double enc_to_rad = 2.0 * M_PI / static_cast<double>(1LL << out_enc_bits);
            j["state"]            = cia402Name(ds.cia402_state);
            j["cmd_vel_rev_per_s"]  = static_cast<double>(cmd_vel_rad_s) / (2.0 * M_PI);
            j["cmd_vel_rad_per_s"]  = static_cast<double>(cmd_vel_rad_s);
            j["fb_vel_raw"]       = ds.measured_input_side_velocity_raw;
            j["fb_vel_rad_per_s"] = static_cast<double>(ds.measured_input_side_velocity_raw)
                                    * (2.0 * M_PI / 1000.0);
            j["mode"]             = static_cast<int>(ds.mode_of_operation_display);
            j["sw"]               = ds.status_word;
            j["err"]              = ds.error_code;
            j["output_enc_pos_raw_cnt"] = ds.measured_output_side_position_raw_cnt;
            j["output_pos_rad"]   = static_cast<double>(ds.measured_output_side_position_raw_cnt)
                                    * enc_to_rad;
            j["in_enc_pos"]       = ds.input_encoder_pos;
            j["pos_setpoint_raw_enc_cnt"] = ds.position_setpoint;
            j["pos_setpoint_rad"] = static_cast<double>(ds.position_setpoint) * enc_to_rad;
            j["fb_torque"]        = ds.measured_torque_nm;
            j["bus_voltage"]      = ds.bus_voltage;
            j["motor_temp"]       = ds.motor_temp;
            j["iq_actual"]        = ds.iq_actual;
            j["id_actual"]        = ds.id_actual;
            j["idc_actual"]       = ds.idc_actual;
            j["iq_command"]       = ds.iq_command;
            j["id_command"]       = ds.id_command;
            // Limits in natural units for GUI slider ranging
            j["max_velocity_abs_rad_s"] = static_cast<double>(ds.max_velocity_abs)
                                          * (2.0 * M_PI / 1000.0);
            j["min_position_rad"] = (ds.min_position == std::numeric_limits<int32_t>::min())
                ? -1000.0 : static_cast<double>(ds.min_position) * enc_to_rad;
            j["max_position_rad"] = (ds.max_position == std::numeric_limits<int32_t>::max())
                ?  1000.0 : static_cast<double>(ds.max_position) * enc_to_rad;
            // Keep raw limits for backward compat
            j["max_velocity_abs"] = ds.max_velocity_abs;
            j["min_position"]     = ds.min_position;
            j["max_position"]     = ds.max_position;
            // Control gains (current values being sent to drive)
            j["torque_kp"]    = static_cast<double>(gains.torque_kp);
            j["torque_max"]   = static_cast<double>(gains.torque_loop_max_output);
            j["torque_min"]   = static_cast<double>(gains.torque_loop_min_output);
            j["vel_kp"]       = static_cast<double>(gains.velocity_loop_kp);
            j["vel_ki"]       = static_cast<double>(gains.velocity_loop_ki);
            j["vel_kd"]       = static_cast<double>(gains.velocity_loop_kd);
            j["pos_kp"]       = static_cast<double>(gains.position_loop_kp);
            j["pos_ki"]       = static_cast<double>(gains.position_loop_ki);
            j["pos_kd"]       = static_cast<double>(gains.position_loop_kd);
        } else {
            j["state"] = "unavailable";
        }
        return j.dump();
    };

    RCLCPP_INFO(node->get_logger(), "Entering main loop...");
    int  debug_iter    = 0;
    auto main_loop_prev = std::chrono::steady_clock::now();

    while (!g_shutdown.load()) {
        const auto   now        = std::chrono::steady_clock::now();
        const double main_dt_ms = std::chrono::duration<double, std::milli>(now - main_loop_prev).count();
        main_loop_prev          = now;
        const bool in_reset = now < reset_end;

        if (++debug_iter <= 3)
            RCLCPP_INFO(node->get_logger(), "Loop iter %d  in_reset=%d  now>=next_pub=%d",
                        debug_iter, (int)in_reset, (int)(now >= next_pub));

        // Snapshot command state.
        CommandState cmd;
        {
            std::lock_guard<std::mutex> lk(g_cmd_mutex);
            cmd = g_cmd_state;
        }

        // Handle one-shot save_log: signal the drain thread to rotate the CSV file.
        if (cmd.save_log) {
            g_rotate_log.store(true);
            std::lock_guard<std::mutex> lk(g_cmd_mutex);
            g_cmd_state.save_log = false;
        }

        // SDO / EtherCAT state operations — all serialised with PDO via safe_stop/safe_start.
        {
            SdoRequest req;
            {
                std::lock_guard<std::mutex> lk(g_sdo_mutex);
                if (g_sdo_req.pending) {
                    req = g_sdo_req;
                    g_sdo_req.pending = false;
                }
            }
            if (req.pending) {
                static constexpr int SDO_TIMEOUT_US = EC_TIMEOUTSAFE; // 20 ms

                if (req.is_pre_op) {
                    // ── Pre-OP toggle ────────────────────────────────────────
                    SdoResponse resp;
                    resp.op = (req.value == 0) ? "pre_op_all" : "pre_op_off";
                    if (req.value == 0) {
                        safe_stop();
                        ec_slave[0].state = EC_STATE_PRE_OP;
                        ec_writestate(0);
                        int chk = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
                        ec_readstate();  // refresh ec_slave[1..N].state for publishBusStatus
                        resp.success = ((chk & 0x0F) == EC_STATE_PRE_OP);
                        resp.error   = resp.success ? "" : "Not all slaves reached PRE-OP";
                        // loop stays stopped — PDO invalid in PRE-OP
                    } else {
                        const std::string op_err = recover_bus_to_op();
                        resp.success = op_err.empty();
                        resp.error   = op_err;
                        safe_start();
                    }
                    node->publishSdoResponse(resp);

                } else if (req.is_store_all) {
                    // ── Store All: pre-op → write 0x26DB → return to OP ─────
                    SdoResponse resp;
                    resp.op    = "store_all";
                    resp.index = 0x26DB;
                    safe_stop();
                    ec_slave[0].state = EC_STATE_PRE_OP;
                    ec_writestate(0);
                    int pre_chk = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
                    ec_readstate();  // refresh ec_slave[1..N].state
                    if ((pre_chk & 0x0F) != EC_STATE_PRE_OP) {
                        resp.success = false;
                        resp.error   = "Failed to reach PRE-OP";
                    } else if (req.slave_idx < 1) {
                        resp.success = false;
                        resp.error   = "Slave not present (index="
                                     + std::to_string(req.slave_idx) + ")";
                    } else {
                        uint32_t magic = 0x65766173u; // "evas" — DS301 save password
                        uint8_t  buf[4];
                        std::memcpy(buf, &magic, 4);
                        int rc = ec_SDOwrite(static_cast<uint16_t>(req.slave_idx),
                                             0x26DB, 0x00, FALSE, 4, buf, SDO_TIMEOUT_US);
                        resp.success = (rc > 0);
                        resp.value   = magic;
                        if (!resp.success)
                            resp.error = "ec_SDOwrite 0x26DB failed (rc="
                                         + std::to_string(rc) + ")";
                    }
                    // Return to OP regardless of SDO result, but report if recovery fails.
                    const std::string op_err = recover_bus_to_op();
                    if (!op_err.empty()) {
                        if (resp.error.empty()) {
                            resp.error = op_err;
                        } else {
                            resp.error += " | " + op_err;
                        }
                        resp.success = false;
                    }
                    safe_start();
                    node->publishSdoResponse(resp);

                } else {
                    // ── Regular SDO read / write ─────────────────────────────
                    SdoResponse resp;
                    resp.op       = req.is_write ? "write" : "read";
                    resp.index    = req.index;
                    resp.subindex = req.subindex;
                    resp.size     = req.size;
                    if (req.slave_idx < 1) {
                        resp.success = false;
                        resp.error   = "Slave not present (index="
                                       + std::to_string(req.slave_idx) + ")";
                    } else if ((ec_slave[req.slave_idx].state & 0x0F) < EC_STATE_PRE_OP) {
                        resp.success = false;
                        resp.error   = "Slave not mailbox-ready (state=0x"
                                       + std::to_string(ec_slave[req.slave_idx].state & 0x0F) + ")";
                    } else {
                        safe_stop();
                        uint8_t buf[8] = {};
                        if (req.is_write) {
                            std::memcpy(buf, &req.value, static_cast<size_t>(req.size));
                            int rc = ec_SDOwrite(static_cast<uint16_t>(req.slave_idx),
                                                 req.index, req.subindex,
                                                 FALSE, req.size, buf, SDO_TIMEOUT_US);
                            resp.success = (rc > 0);
                            resp.value   = req.value;
                            if (!resp.success)
                                resp.error = "ec_SDOwrite failed (rc=" + std::to_string(rc) + ")";
                        } else {
                            int sz = req.size;
                            int rc = ec_SDOread(static_cast<uint16_t>(req.slave_idx),
                                                req.index, req.subindex,
                                                FALSE, &sz, buf, SDO_TIMEOUT_US);
                            resp.success = (rc > 0);
                            resp.size    = sz;
                            if (resp.success) {
                                int64_t v = 0;
                                std::memcpy(&v, buf, static_cast<size_t>(sz));
                                resp.value = v;
                            } else {
                                resp.error = "ec_SDOread failed (rc=" + std::to_string(rc) + ")";
                            }
                        }
                        safe_start();
                    }
                    node->publishSdoResponse(resp);
                }
            }
        }

        // Apply torque sensor ADC scale to the adapter.
        // Note: setCh1/2TorqueScale() writes ch1_torque_scale_ which is also read
        // by the RT cycle callback's scaledTorqueCh1/2() — on x86_64, aligned float
        // read/write is naturally atomic, so the worst case is one stale cycle during
        // a user-initiated scale change. Acceptable for an infrequent measurement setting.
        try {
            el3002->setCh1TorqueScale(cmd.ch1_torque_scale);
            el3002->setCh2TorqueScale(cmd.ch2_torque_scale);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                "Ignoring invalid torque scale value: %s", e.what());
        }

        // Build EtherCAT commands.
        // velocity: rad/s → mrev/s;  position: rad → encoder counts
        static constexpr float TWO_PI = 2.0f * static_cast<float>(M_PI);
        Command main_cmd;
        main_cmd.mode_of_operation       = static_cast<ModeOfOperation>(cmd.main_mode);
        main_cmd.target_velocity_mrevs    = cmd.main_speed * 1000.0f / TWO_PI;
        main_cmd.target_position__enc_cnt = cmd.main_position
                                            * static_cast<float>(1LL << main_out_enc_bits) / TWO_PI;
        main_cmd.target_torque_nm        = cmd.main_torque;
        main_cmd.torque_command_2022     = cmd.main_current;
        main_cmd.enable_drive            = !in_reset && cmd.main_enable;
        main_cmd.clear_fault             = in_reset || cmd.fault_reset;
        main_cmd.torque_kp               = cmd.main_torque_kp;
        main_cmd.torque_loop_max_output  = cmd.main_torque_max_out;
        main_cmd.torque_loop_min_output  = cmd.main_torque_min_out;
        main_cmd.velocity_loop_kp        = cmd.main_vel_kp;
        main_cmd.velocity_loop_ki        = cmd.main_vel_ki;
        main_cmd.velocity_loop_kd        = cmd.main_vel_kd;
        main_cmd.position_loop_kp        = cmd.main_pos_kp;
        main_cmd.position_loop_ki        = cmd.main_pos_ki;
        main_cmd.position_loop_kd        = cmd.main_pos_kd;

        Command dut_cmd;
        dut_cmd.mode_of_operation       = static_cast<ModeOfOperation>(cmd.dut_mode);
        dut_cmd.target_velocity_mrevs    = cmd.dut_speed * 1000.0f / TWO_PI;
        dut_cmd.target_position__enc_cnt = cmd.dut_position
                                           * static_cast<float>(1LL << dut_out_enc_bits) / TWO_PI;
        dut_cmd.target_torque_nm        = cmd.dut_torque;
        dut_cmd.torque_command_2022     = cmd.dut_current;
        dut_cmd.enable_drive            = !in_reset && cmd.dut_enable;
        dut_cmd.clear_fault             = in_reset || cmd.fault_reset;
        dut_cmd.torque_kp               = cmd.dut_torque_kp;
        dut_cmd.torque_loop_max_output  = cmd.dut_torque_max_out;
        dut_cmd.torque_loop_min_output  = cmd.dut_torque_min_out;
        dut_cmd.velocity_loop_kp        = cmd.dut_vel_kp;
        dut_cmd.velocity_loop_ki        = cmd.dut_vel_ki;
        dut_cmd.velocity_loop_kd        = cmd.dut_vel_kd;
        dut_cmd.position_loop_kp        = cmd.dut_pos_kp;
        dut_cmd.position_loop_ki        = cmd.dut_pos_ki;
        dut_cmd.position_loop_kd        = cmd.dut_pos_kd;

        beckhoff::el2004::Command io_cmd;
        io_cmd.output_1 = cmd.hold_output1;

        SystemCommand sys_cmd;
        sys_cmd.by_slave[drive_slave] = main_cmd;
        if (dut_present) sys_cmd.by_slave[dut_slave] = dut_cmd;
        sys_cmd.by_slave[io_slave]    = io_cmd;
        loop.setCommand(sys_cmd);

        // Publish telemetry at pub_hz rate.
        if (now >= next_pub) { try {
            const SystemStatus status = loop.getStatus();
            const LoopStats    stats  = loop.stats();

            DriveGains cmd_main_gains;
            cmd_main_gains.torque_kp              = cmd.main_torque_kp;
            cmd_main_gains.torque_loop_max_output = cmd.main_torque_max_out;
            cmd_main_gains.torque_loop_min_output = cmd.main_torque_min_out;
            cmd_main_gains.velocity_loop_kp       = cmd.main_vel_kp;
            cmd_main_gains.velocity_loop_ki       = cmd.main_vel_ki;
            cmd_main_gains.velocity_loop_kd       = cmd.main_vel_kd;
            cmd_main_gains.position_loop_kp       = cmd.main_pos_kp;
            cmd_main_gains.position_loop_ki       = cmd.main_pos_ki;
            cmd_main_gains.position_loop_kd       = cmd.main_pos_kd;

            DriveGains cmd_dut_gains;
            cmd_dut_gains.torque_kp               = cmd.dut_torque_kp;
            cmd_dut_gains.torque_loop_max_output  = cmd.dut_torque_max_out;
            cmd_dut_gains.torque_loop_min_output  = cmd.dut_torque_min_out;
            cmd_dut_gains.velocity_loop_kp        = cmd.dut_vel_kp;
            cmd_dut_gains.velocity_loop_ki        = cmd.dut_vel_ki;
            cmd_dut_gains.velocity_loop_kd        = cmd.dut_vel_kd;
            cmd_dut_gains.position_loop_kp        = cmd.dut_pos_kp;
            cmd_dut_gains.position_loop_ki        = cmd.dut_pos_ki;
            cmd_dut_gains.position_loop_kd        = cmd.dut_pos_kd;

            const std::string main_json = make_drive_json(
                drive_slave, drive_soem_idx, cmd.main_speed, status, main_out_enc_bits, cmd_main_gains);
            const std::string dut_json = make_drive_json(
                dut_slave, dut_soem_idx, cmd.dut_speed, status, dut_out_enc_bits, cmd_dut_gains);

            uint32_t enc = 0;
            auto enc_it = status.by_slave.find(encoder_slave);
            if (enc_it != status.by_slave.end() && enc_it->second.has_value())
                enc = std::any_cast<const beckhoff::el5032::Data&>(
                    enc_it->second).encoder_count_25bit;

            double ch1_t = 0.0, ch2_t = 0.0;
            auto torque_it = status.by_slave.find(torque_slave);
            if (torque_it != status.by_slave.end() && torque_it->second.has_value()) {
                const auto& d = std::any_cast<const beckhoff::el3002::Data&>(torque_it->second);
                // Apply one-shot zero before reading, then clear flags in shared state.
                if (cmd.zero_torque_ch1) {
                    el3002->zeroTorqueCh1(d);
                    std::lock_guard<std::mutex> lk(g_cmd_mutex);
                    g_cmd_state.zero_torque_ch1 = false;
                }
                if (cmd.zero_torque_ch2) {
                    el3002->zeroTorqueCh2(d);
                    std::lock_guard<std::mutex> lk(g_cmd_mutex);
                    g_cmd_state.zero_torque_ch2 = false;
                }
                ch1_t = static_cast<double>(el3002->scaledTorqueCh1(d));
                ch2_t = static_cast<double>(el3002->scaledTorqueCh2(d));
            }

            node->publishTelemetry(
                stats.cycle_count,
                stats.last_wkc,
                static_cast<double>(stats.last_cycle_time_ns) / 1000.0,
                main_json, dut_json,
                enc, ch1_t, ch2_t
            );
            node->publishBusStatus();

            if (debug_print) {
                // main_drive
                auto main_it = status.by_slave.find(drive_slave);
                if (main_it != status.by_slave.end() && main_it->second.has_value()) {
                    const auto& ds = std::any_cast<const DriveStatus&>(main_it->second);
                    std::printf(
                        "[main] cycle=%lu wkc=%d "
                        "al=%s state=%s "
                        "cmd_60FF=%.3f speed_606C=%d "
                        "mode_6061=%d sw=0x%04X err=0x%04X "
                        "bus_v=%.2f "
                        "vel_kp=%.4f vel_ki=%.4f torque_kp=%.4f\n",
                        static_cast<unsigned long>(stats.cycle_count),
                        stats.last_wkc,
                        alStateName(static_cast<int>(ec_slave[drive_soem_idx].state)).c_str(),
                        cia402Name(ds.cia402_state),
                        static_cast<double>(cmd.main_speed),
                        ds.measured_input_side_velocity_raw,
                        static_cast<int>(ds.mode_of_operation_display),
                        static_cast<unsigned>(ds.status_word),
                        static_cast<unsigned>(ds.error_code),
                        static_cast<double>(ds.bus_voltage),
                        static_cast<double>(main_cmd.velocity_loop_kp),
                        static_cast<double>(main_cmd.velocity_loop_ki),
                        static_cast<double>(main_cmd.torque_kp)
                    );
                }
                // dut
                auto dut_it = status.by_slave.find(dut_slave);
                if (dut_present && dut_it != status.by_slave.end() && dut_it->second.has_value()) {
                    const auto& ds = std::any_cast<const DriveStatus&>(dut_it->second);
                    std::printf(
                        "[ dut] cycle=%lu wkc=%d "
                        "al=%s state=%s "
                        "cmd_60FF=%.3f speed_606C=%d "
                        "mode_6061=%d sw=0x%04X err=0x%04X "
                        "bus_v=%.2f "
                        "vel_kp=%.4f vel_ki=%.4f torque_kp=%.4f\n",
                        static_cast<unsigned long>(stats.cycle_count),
                        stats.last_wkc,
                        alStateName(dut_present ? static_cast<int>(ec_slave[dut_soem_idx].state) : 0).c_str(),
                        cia402Name(ds.cia402_state),
                        static_cast<double>(cmd.dut_speed),
                        ds.measured_input_side_velocity_raw,
                        static_cast<int>(ds.mode_of_operation_display),
                        static_cast<unsigned>(ds.status_word),
                        static_cast<unsigned>(ds.error_code),
                        static_cast<double>(ds.bus_voltage),
                        static_cast<double>(dut_cmd.velocity_loop_kp),
                        static_cast<double>(dut_cmd.velocity_loop_ki),
                        static_cast<double>(dut_cmd.torque_kp)
                    );
                }
                // encoder + torque
                std::printf(
                    "[sens] enc=%u ch1_t=%.4f ch2_t=%.4f\n",
                    enc,
                    ch1_t, ch2_t
                );
            }

            if (debug_print) {
                std::printf(
                    "[timing] main_dt=%.3f ms | rt_period=%.3f ms"
                    " | rt_cycle=%.3f ms | wakeup_lat=%.3f ms\n",
                    main_dt_ms,
                    static_cast<double>(stats.last_period_ns)          * 1e-6,
                    static_cast<double>(stats.last_cycle_time_ns)      * 1e-6,
                    static_cast<double>(stats.last_wakeup_latency_ns)  * 1e-6
                );
            }

            next_pub += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(pub_period));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Publish exception: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(node->get_logger(), "Publish unknown exception");
        } }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Shutdown: do not send any DS402 disable commands before stopping.
    //
    // Any path through READY_TO_SWITCH_ON causes the Capitan drive to reset its
    // velocity gain registers (0x250A/B) to 0.  The only safe approach is to
    // stop the EtherCAT loop directly — the drive's PDO watchdog fires and
    // the AL layer moves to SAFE-OP+ERR while the DS402 state is preserved.
    // On the next initialize(), the fault-reset phase clears any residual state.
    // This matches drive_simple_speed_test1 behaviour, which does not cause faults.
    {
        const SystemStatus s  = loop.getStatus();
        auto log_state = [&](const std::string& name) {
            auto it = s.by_slave.find(name);
            if (it == s.by_slave.end() || !it->second.has_value()) return;
            const auto& ds = std::any_cast<const DriveStatus&>(it->second);
            RCLCPP_INFO(node->get_logger(), "[shutdown] %s was in %s — stopping loop",
                name.c_str(), cia402Name(ds.cia402_state));
        };
        log_state(drive_slave);
        if (dut_present) log_state(dut_slave);
    }

    safe_stop();
    master.close();

    // Drain remaining log records and flush quill's async backend.
    log_drain.join();
    RCLCPP_INFO(node->get_logger(), "PDO log saved: %s", log_path.c_str());

    executor->cancel();
    ros_thread.join();
    rclcpp::shutdown();
    return 0;
}
