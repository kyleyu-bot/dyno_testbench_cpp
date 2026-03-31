#include "ethercat_core/master.hpp"

// SOEM C headers — must be wrapped in extern "C" for C++ compilation.
extern "C" {
#include "ethercat.h"
}

#include <nlohmann/json.hpp>

#include "ethercat_core/devices/beckhoff/el2004/adapter.hpp"
#include "ethercat_core/devices/beckhoff/el3002/adapter.hpp"
#include "ethercat_core/devices/beckhoff/el5032/adapter.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Everest/adapter.hpp"
#include "ethercat_core/devices/motor_drives/Novanta/Volcano/adapter.hpp"

#include <chrono>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>
#include <thread>
#include <unordered_map>

namespace ethercat_core {

// IOmap used by SOEM to exchange process data.  4096 bytes covers all
// practical configurations; increase if a very large bus is used.
static uint8_t g_IOmap[4096];

// ── Topology loading ──────────────────────────────────────────────────────────

MasterConfig loadTopology(const std::string& path) {
    std::ifstream fh(path);
    if (!fh.is_open()) {
        throw MasterConfigError("Cannot open topology file: " + path);
    }
    nlohmann::json raw;
    fh >> raw;

    std::string iface = raw.value("iface", "");
    if (iface.empty()) {
        throw MasterConfigError("Topology config must include non-empty 'iface'.");
    }

    auto raw_slaves = raw.value("slaves", nlohmann::json::array());
    if (raw_slaves.empty()) {
        throw MasterConfigError("Topology config 'slaves' must be a non-empty list.");
    }

    MasterConfig cfg;
    cfg.iface          = iface;
    cfg.cycle_hz       = raw.value("cycle_hz", 1000);
    cfg.strict_pdo_size = raw.value("strict_pdo_size", false);

    for (auto& entry : raw_slaves) {
        SlaveConfig sc;
        sc.name          = entry.at("name").get<std::string>();
        sc.position      = entry.value("position", -1);
        sc.alias_address = static_cast<uint16_t>(entry.value("alias_address", 0));
        sc.kind          = entry.at("kind").get<std::string>();
        sc.vendor_id     = entry.value("vendor_id",   0u);
        sc.product_code  = entry.value("product_code", 0u);
        sc.optional      = entry.value("optional", false);

        for (auto& pm : entry.value("pdo_mapping", nlohmann::json::array())) {
            PdoMappingEntry e;
            e.index    = static_cast<uint16_t>(pm.at("index").get<int>());
            e.subindex = static_cast<uint8_t>(pm.at("subindex").get<int>());
            e.value    = pm.at("value").get<uint32_t>();
            e.size     = pm.at("size").get<int>();
            sc.pdo_mapping.push_back(e);
        }

        if (entry.contains("scaling")) {
            auto& s = entry["scaling"];
            sc.scaling.torque_lsb_per_nm      = s.value("torque_lsb_per_nm",      10.0f);
            sc.scaling.velocity_lsb_per_rad_s = s.value("velocity_lsb_per_rad_s", 1000.0f);
            sc.scaling.position_lsb_per_rad   = s.value("position_lsb_per_rad",   10000.0f);
        }

        cfg.slaves.push_back(std::move(sc));
    }
    return cfg;
}

// ── Adapter factory ───────────────────────────────────────────────────────────

static std::unique_ptr<ISlaveAdapter> buildAdapter(const SlaveConfig& cfg) {
    SlaveIdentity id{cfg.name, cfg.position, cfg.vendor_id, cfg.product_code};

    if (cfg.kind == "everest") {
        novanta::everest::PdoScaling sc{
            cfg.scaling.torque_lsb_per_nm,
            cfg.scaling.velocity_lsb_per_rad_s,
            cfg.scaling.position_lsb_per_rad,
        };
        return std::make_unique<novanta::everest::NovantaEverestAdapter>(id, sc);
    }
    if (cfg.kind == "volcano") {
        novanta::volcano::PdoScaling sc{
            cfg.scaling.torque_lsb_per_nm,
            cfg.scaling.velocity_lsb_per_rad_s,
            cfg.scaling.position_lsb_per_rad,
        };
        return std::make_unique<novanta::volcano::NovantaVolcanoAdapter>(id, sc);
    }
    if (cfg.kind == "EL2004") {
        return std::make_unique<beckhoff::el2004::El2004Adapter>(id);
    }
    if (cfg.kind == "EL3002" || cfg.kind == "ELM3002") {
        return std::make_unique<beckhoff::el3002::El3002Adapter>(id);
    }
    if (cfg.kind == "EL5032") {
        return std::make_unique<beckhoff::el5032::El5032Adapter>(id);
    }

    throw MasterConfigError("Unsupported slave kind '" + cfg.kind + "' for '" + cfg.name + "'.");
}

// ── SDO helpers ───────────────────────────────────────────────────────────────

// Read a single SDO and decode it according to spec.  Returns the value as a
// float (all numeric types are widened; "bytes" type is not supported here).
static float readSdoFloat(int soem_idx, const SdoReadSpec& spec) {
    uint8_t buf[8] = {};
    int     sz     = static_cast<int>(sizeof(buf));

    int ret = ec_SDOread(
        static_cast<uint16>(soem_idx),
        spec.index, spec.subindex,
        FALSE, &sz, buf, EC_TIMEOUTRET
    );
    if (ret <= 0) {
        std::ostringstream oss;
        oss << "SDO read failed for '" << spec.name << "' at 0x"
            << std::hex << spec.index << ":" << static_cast<int>(spec.subindex);
        throw MasterConfigError(oss.str());
    }

    const std::string& dt = spec.data_type;
    if (dt == "u8"  && sz >= 1) return static_cast<float>(buf[0]);
    if (dt == "s8"  && sz >= 1) return static_cast<float>(static_cast<int8_t>(buf[0]));
    if (dt == "u16" && sz >= 2) { uint16_t v; std::memcpy(&v, buf, 2); return static_cast<float>(v); }
    if (dt == "s16" && sz >= 2) { int16_t  v; std::memcpy(&v, buf, 2); return static_cast<float>(v); }
    if (dt == "u32" && sz >= 4) { uint32_t v; std::memcpy(&v, buf, 4); return static_cast<float>(v); }
    if (dt == "s32" && sz >= 4) { int32_t  v; std::memcpy(&v, buf, 4); return static_cast<float>(v); }
    if (dt == "f32" && sz >= 4) { float    v; std::memcpy(&v, buf, 4); return v; }

    throw MasterConfigError("Unsupported SDO data_type '" + dt + "' for '" + spec.name + "'.");
}

// ── EthercatMaster ────────────────────────────────────────────────────────────

EthercatMaster::EthercatMaster(MasterConfig config)
    : config_(std::move(config))
{}

EthercatMaster::~EthercatMaster() {
    if (initialized_) {
        close();
    }
}

MasterRuntime& EthercatMaster::initialize() {
    if (ec_init(config_.iface.c_str()) <= 0) {
        throw MasterConfigError("ec_init failed on interface '" + config_.iface + "'. "
                                "Check the NIC name and that you have CAP_NET_RAW.");
    }

    const int slave_count = ec_config_init(FALSE);
    if (slave_count <= 0) {
        ec_close();
        throw MasterConfigError("No EtherCAT slaves detected on '" + config_.iface + "'.");
    }

    transitionToPreOp();

    // Dump all detected slaves so topology mismatches are easy to diagnose.
    std::fprintf(stderr, "[master] Detected %d slaves:\n", ec_slavecount);
    for (int i = 1; i <= ec_slavecount; ++i) {
        std::fprintf(stderr,
            "[master]   SOEM[%d] pos=%d vendor=0x%08X product=0x%08X alias=%u name='%s'\n",
            i, i - 1,
            static_cast<unsigned>(ec_slave[i].eep_man),
            static_cast<unsigned>(ec_slave[i].eep_id),
            static_cast<unsigned>(ec_slave[i].aliasadr),
            ec_slave[i].name
        );
    }

    // Resolve positions and build adapters.
    // Track claimed SOEM indices so two slaves never map to the same physical device.
    std::set<int> used_soem_indices;
    for (auto& sc : config_.slaves) {
        try {
            sc.position = resolvePosition(sc);
            const int soem_idx = sc.position + 1;  // 1-based
            if (used_soem_indices.count(soem_idx)) {
                std::ostringstream oss;
                oss << "Slave '" << sc.name << "' resolved to SOEM index " << soem_idx
                    << " which is already claimed by another slave.";
                throw MasterConfigError(oss.str());
            }
            used_soem_indices.insert(soem_idx);
        } catch (const MasterConfigError& e) {
            if (sc.optional) {
                std::fprintf(stderr, "[master] Optional slave '%s' skipped: %s\n",
                             sc.name.c_str(), e.what());
                continue;
            }
            throw;
        }
        runtime_.adapters[sc.name] = buildAdapter(sc);
        runtime_.slave_index[sc.name] = sc.position + 1;  // SOEM is 1-based
    }

    // Validate identity, read startup SDOs, apply PDO mapping writes.
    for (auto& sc : config_.slaves) {
        if (runtime_.slave_index.find(sc.name) == runtime_.slave_index.end()) continue;
        const int soem_idx = runtime_.slave_index.at(sc.name);
        validateIdentity(sc, soem_idx);
        readStartupParams(sc, soem_idx, *runtime_.adapters.at(sc.name));
        configurePdoMapping(sc, soem_idx);
    }

    // Build process data map after all PDO remap writes are done.
    ec_config_map(g_IOmap);
    ec_configdc();

    if (config_.strict_pdo_size) {
        for (auto& sc : config_.slaves) {
            if (runtime_.slave_index.find(sc.name) == runtime_.slave_index.end()) continue;
            const int soem_idx = runtime_.slave_index.at(sc.name);
            validatePdoSizes(sc, soem_idx, *runtime_.adapters.at(sc.name));
        }
    }

    transitionToOperational();
    initialized_ = true;
    return runtime_;
}

void EthercatMaster::close() {
    if (!initialized_) return;

    // Request INIT directly — do not go through SAFE-OP explicitly.
    // The Capitan/Volcano drive resets its volatile gain registers (0x250A/B)
    // during a master-driven OP→SAFE-OP transition, causing gains to read as 0
    // on the next initialize().  Going directly to INIT avoids this.
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE);
    ec_close();
    initialized_ = false;
}

const MasterRuntime& EthercatMaster::runtime() const {
    if (!initialized_) throw std::runtime_error("Master is not initialized.");
    return runtime_;
}

void EthercatMaster::transitionToPreOp() {
    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
}

void EthercatMaster::transitionToOperational() {
    // SAFE-OP first.
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    // Prime process data before requesting OP — some drives require it.
    for (int i = 0; i < 5; ++i) {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
    }

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

    // Exchange process data while waiting for all slaves to reach OP.
    for (int attempt = 0; attempt < 50; ++attempt) {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_readstate();
        if (allSlavesInOp()) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    throw MasterConfigError(formatStateError());
}

void EthercatMaster::validateIdentity(const SlaveConfig& cfg, int soem_idx) {
    const auto& sl = ec_slave[soem_idx];
    if (cfg.vendor_id && static_cast<uint32_t>(sl.eep_man) != cfg.vendor_id) {
        std::ostringstream oss;
        oss << std::hex << "Slave '" << cfg.name << "' vendor mismatch: "
            << "expected=0x" << cfg.vendor_id << " got=0x" << sl.eep_man;
        throw MasterConfigError(oss.str());
    }
    if (cfg.product_code && static_cast<uint32_t>(sl.eep_id) != cfg.product_code) {
        std::ostringstream oss;
        oss << std::hex << "Slave '" << cfg.name << "' product mismatch: "
            << "expected=0x" << cfg.product_code << " got=0x" << sl.eep_id;
        throw MasterConfigError(oss.str());
    }
}

int EthercatMaster::resolvePosition(const SlaveConfig& cfg) {
    // If an alias address is specified, scan for it first — alias uniquely
    // identifies a slave even when vendor_id + product_code are shared across
    // multiple slaves of the same model (e.g. two identical drives).
    if (cfg.alias_address != 0) {
        for (int i = 1; i <= ec_slavecount; ++i) {
            if (static_cast<uint16_t>(ec_slave[i].aliasadr) == cfg.alias_address) {
                return i - 1;  // return 0-based
            }
        }
        // Alias not found — caller handles optional slaves.
        std::ostringstream oss;
        oss << "No slave found with alias 0x" << std::hex << cfg.alias_address
            << " for '" << cfg.name << "'.";
        throw MasterConfigError(oss.str());
    }

    // No alias: try the configured position first.
    if (cfg.position >= 0 && cfg.position < ec_slavecount) {
        const auto& sl = ec_slave[cfg.position + 1];  // 1-based
        bool vendor_ok  = !cfg.vendor_id    || static_cast<uint32_t>(sl.eep_man) == cfg.vendor_id;
        bool product_ok = !cfg.product_code || static_cast<uint32_t>(sl.eep_id)  == cfg.product_code;
        if (vendor_ok && product_ok) return cfg.position;
    }
    // Fall back: scan all slaves by vendor + product.
    for (int i = 1; i <= ec_slavecount; ++i) {
        const auto& sl = ec_slave[i];
        bool vendor_ok  = !cfg.vendor_id    || static_cast<uint32_t>(sl.eep_man) == cfg.vendor_id;
        bool product_ok = !cfg.product_code || static_cast<uint32_t>(sl.eep_id)  == cfg.product_code;
        if (vendor_ok && product_ok) return i - 1;
    }
    std::ostringstream oss;
    oss << std::hex << "No slave matched '" << cfg.name << "' "
        << "(vendor=0x" << cfg.vendor_id << ", product=0x" << cfg.product_code << ").";
    throw MasterConfigError(oss.str());
}

void EthercatMaster::configurePdoMapping(const SlaveConfig& cfg, int soem_idx) {
    for (const auto& pm : cfg.pdo_mapping) {
        uint8_t buf[4] = {};
        std::memcpy(buf, &pm.value, static_cast<std::size_t>(pm.size));

        bool ok = false;
        for (int attempt = 0; attempt < 5 && !ok; ++attempt) {
            int ret = ec_SDOwrite(
                static_cast<uint16>(soem_idx),
                pm.index, pm.subindex,
                FALSE, pm.size, buf, EC_TIMEOUTRET
            );
            if (ret > 0) {
                ok = true;
            } else if (attempt < 4) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
        if (!ok) {
            std::ostringstream oss;
            oss << std::hex << "PDO mapping SDO write failed for '" << cfg.name
                << "' at 0x" << pm.index << ":" << static_cast<int>(pm.subindex);
            throw MasterConfigError(oss.str());
        }
    }
}

void EthercatMaster::validatePdoSizes(
    const SlaveConfig& cfg, int soem_idx, ISlaveAdapter& adapter)
{
    const auto& sl        = ec_slave[soem_idx];
    const int   rx_actual = static_cast<int>(sl.Obytes);  // master→slave = outputs
    const int   tx_actual = static_cast<int>(sl.Ibytes);  // slave→master = inputs

    if (rx_actual != adapter.rxPdoSize()) {
        std::ostringstream oss;
        oss << "RX PDO size mismatch for '" << cfg.name << "': "
            << "expected=" << adapter.rxPdoSize() << " got=" << rx_actual;
        throw MasterConfigError(oss.str());
    }
    if (tx_actual != adapter.txPdoSize()) {
        std::ostringstream oss;
        oss << "TX PDO size mismatch for '" << cfg.name << "': "
            << "expected=" << adapter.txPdoSize() << " got=" << tx_actual;
        throw MasterConfigError(oss.str());
    }
}

void EthercatMaster::readStartupParams(
    const SlaveConfig& cfg, int soem_idx, ISlaveAdapter& adapter)
{
    // Only Novanta adapters expose startupReadSpecs(); other adapters skip this.
    std::unordered_map<std::string, SdoReadSpec> specs;

    // Use RTTI-free duck typing: attempt casts for known types.
    if (auto* ea = dynamic_cast<novanta::everest::NovantaEverestAdapter*>(&adapter)) {
        specs = ea->startupReadSpecs();
    } else if (auto* va = dynamic_cast<novanta::volcano::NovantaVolcanoAdapter*>(&adapter)) {
        specs = va->startupReadSpecs();
    }

    if (specs.empty()) return;

    auto& params = runtime_.startup_params[cfg.name];
    for (auto& [key, spec] : specs) {
        float val = 0.0f;
        bool  ok  = false;
        for (int attempt = 0; attempt < 5 && !ok; ++attempt) {
            try {
                val = readSdoFloat(soem_idx, spec);
                ok  = true;
            } catch (const MasterConfigError&) {
                if (attempt < 4) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }
        }
        if (!ok) {
            throw MasterConfigError(
                "Startup SDO read failed for '" + cfg.name + "' key='" + key + "'."
            );
        }
        params[key] = val;
    }

    // Compute gear ratio from sensor_ratio (0x2364).
    auto sr_it = params.find("sensor_ratio");
    if (sr_it != params.end()) {
        if (auto* ea = dynamic_cast<novanta::everest::NovantaEverestAdapter*>(&adapter)) {
            ea->computeGearRatio(sr_it->second);
        } else if (auto* va = dynamic_cast<novanta::volcano::NovantaVolcanoAdapter*>(&adapter)) {
            va->computeGearRatio(sr_it->second);
        }
    }
}

bool EthercatMaster::allSlavesInOp() const {
    for (const auto& sc : config_.slaves) {
        auto it = runtime_.slave_index.find(sc.name);
        if (it == runtime_.slave_index.end()) continue;  // optional slave not present
        if ((ec_slave[it->second].state & 0x0Fu) != EC_STATE_OPERATIONAL) {
            return false;
        }
    }
    return true;
}

std::string EthercatMaster::formatStateError() const {
    std::ostringstream oss;
    oss << "Failed to reach OPERATIONAL for all configured slaves:\n";
    for (const auto& sc : config_.slaves) {
        auto it = runtime_.slave_index.find(sc.name);
        if (it == runtime_.slave_index.end()) continue;
        const int   soem_idx  = it->second;
        const auto& sl        = ec_slave[soem_idx];
        oss << "  " << sc.name << " pos=" << sc.position
            << std::hex
            << " state=0x" << static_cast<int>(sl.state)
            << " al_status=0x" << static_cast<int>(sl.ALstatuscode)
            << "\n";
    }
    return oss.str();
}

// ── Utility ───────────────────────────────────────────────────────────────────

std::string alStateName(int state_code) {
    const int  base      = state_code & 0x0F;
    const bool has_error = (state_code & 0x10) != 0;

    static const std::unordered_map<int, std::string> labels = {
        {EC_STATE_INIT,        "INIT"},
        {EC_STATE_PRE_OP,      "PRE-OP"},
        {EC_STATE_BOOT,        "BOOT"},
        {EC_STATE_SAFE_OP,     "SAFE-OP"},
        {EC_STATE_OPERATIONAL, "OP"},
    };

    auto it = labels.find(base);
    std::string label = (it != labels.end()) ? it->second
                        : ("UNKNOWN(0x" + [&]{ std::ostringstream o; o << std::hex << state_code; return o.str(); }() + ")");
    return has_error ? label + "+ERR" : label;
}

} // namespace ethercat_core
