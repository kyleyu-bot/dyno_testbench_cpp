#pragma once

#include "ethercat_core/data_types.hpp"
#include "ethercat_core/devices/base.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace ethercat_core {

class MasterConfigError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

// PDO mapping write to apply at startup (before config_map).
struct PdoMappingEntry {
    uint16_t index;
    uint8_t  subindex;
    uint32_t value;
    int      size;  // bytes (1, 2, or 4)
};

// Per-slave topology configuration loaded from the JSON file.
struct SlaveConfig {
    std::string               name;
    int                       position      = -1;   // 0-based; -1 = auto-discover
    uint16_t                  alias_address = 0;    // EtherCAT alias (0 = not used for matching)
    std::string               kind;
    uint32_t                  vendor_id     = 0;
    uint32_t                  product_code  = 0;
    bool                      optional      = false; // if true, missing slave is not fatal
    std::vector<PdoMappingEntry> pdo_mapping;

    struct Scaling {
        float torque_lsb_per_nm      = 10.0f;
        float velocity_lsb_per_rad_s = 1000.0f;
        float position_lsb_per_rad   = 10000.0f;
    } scaling;
};

// Top-level master configuration.
struct MasterConfig {
    std::string              iface;
    int                      cycle_hz         = 1000;
    bool                     strict_pdo_size  = false;
    std::vector<SlaveConfig> slaves;
};

// Load a JSON topology file into MasterConfig.
// Throws MasterConfigError if the file is missing required fields.
MasterConfig loadTopology(const std::string& path);

// Runtime state returned by EthercatMaster::initialize().
struct MasterRuntime {
    // Adapters keyed by slave name (same order as MasterConfig::slaves).
    std::unordered_map<std::string, std::unique_ptr<ISlaveAdapter>> adapters;

    // Map from slave name to 1-based SOEM slave index.
    std::unordered_map<std::string, int> slave_index;

    // Values read from SDO at startup, keyed by slave name then param name.
    std::unordered_map<std::string, std::unordered_map<std::string, float>> startup_params;
};

// Manages the SOEM master lifecycle: open socket, scan slaves, configure PDOs,
// drive the state machine to OPERATIONAL, and tear down on close.
class EthercatMaster {
public:
    explicit EthercatMaster(MasterConfig config);
    ~EthercatMaster();

    // Non-copyable, non-movable (owns SOEM global state).
    EthercatMaster(const EthercatMaster&)            = delete;
    EthercatMaster& operator=(const EthercatMaster&) = delete;

    // Open the NIC, scan for slaves, configure PDOs, and transition to OP.
    // Returns a reference to the runtime state valid until close() is called.
    MasterRuntime& initialize();

    // Gracefully transition slaves back to INIT and close the socket.
    void close();

    bool isInitialized() const { return initialized_; }
    const MasterRuntime& runtime() const;

private:
    void transitionToPreOp();
    void transitionToOperational();
    void validateIdentity(const SlaveConfig& cfg, int soem_idx);
    int  resolvePosition(const SlaveConfig& cfg);
    void configurePdoMapping(const SlaveConfig& cfg, int soem_idx);
    void validatePdoSizes(const SlaveConfig& cfg, int soem_idx, ISlaveAdapter& adapter);
    void readStartupParams(const SlaveConfig& cfg, int soem_idx, ISlaveAdapter& adapter);
    bool allSlavesInOp() const;
    std::string formatStateError() const;

    MasterConfig config_;
    MasterRuntime runtime_;
    bool          initialized_ = false;
};

// Human-readable AL state label (e.g., "PRE-OP", "OP", "SAFE-OP+ERR").
std::string alStateName(int state_code);

} // namespace ethercat_core
