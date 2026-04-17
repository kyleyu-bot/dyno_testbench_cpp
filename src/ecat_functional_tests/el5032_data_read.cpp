// Read and print Beckhoff EL5032 TxPDO data in the cyclic loop.
//
// Usage:
//   sudo ./el5032_data_read [options]
//
// Options:
//   --topology <path>    Topology JSON file (default: config/topology.dyno2.template6.json)
//   --slave    <name>    Configured EL5032 slave name (default: encoder_interface)
//   --duration <s>       Monitor duration in seconds (default: 60)
//   --print-hz <hz>      Terminal update rate (default: 5)
//   --rt-priority <1-99> Loop thread SCHED_FIFO priority (0 = default scheduler)
//   --cpu-affinity <cpu> Comma-separated CPU indices (e.g. 2 or 2,3)

#include "ethercat_core/data_types.hpp"
#include "ethercat_core/loop.hpp"
#include "ethercat_core/master.hpp"
#include "ethercat_core/devices/beckhoff/el5032/adapter.hpp"
#include "ethercat_core/devices/beckhoff/el5032/data_types.hpp"

extern "C" {
#include "ethercat.h"
}

#include <getopt.h>

#include <algorithm>
#include <any>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

using namespace ethercat_core;
using namespace ethercat_core::beckhoff::el5032;

// ── Signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_shutdown{false};
static void onSignal(int) { g_shutdown.store(true); }

// ── Argument defaults ─────────────────────────────────────────────────────────

static constexpr const char* DEFAULT_TOPOLOGY = "config/ethercat_device_config/topology.dyno2.template6.json";
static constexpr const char* DEFAULT_SLAVE    = "encoder_interface";
static constexpr double      DEFAULT_DURATION = 60.0;
static constexpr double      DEFAULT_PRINT_HZ = 5.0;

struct Args {
    std::string   topology     = DEFAULT_TOPOLOGY;
    std::string   slave        = DEFAULT_SLAVE;
    double        duration_s   = DEFAULT_DURATION;
    double        print_hz     = DEFAULT_PRINT_HZ;
    int           rt_priority  = 0;
    std::set<int> cpu_affinity;
};

static void printUsage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "  --topology <path>      Topology JSON       (default: %s)\n"
        "  --slave    <name>      EL5032 slave name   (default: %s)\n"
        "  --duration <s>         Monitor duration    (default: %.1f s)\n"
        "  --print-hz <hz>        Print rate          (default: %.1f Hz)\n"
        "  --rt-priority <1-99>   SCHED_FIFO priority (0 = default)\n"
        "  --cpu-affinity <cpu>   CPU index(es), comma-separated (e.g. 2 or 2,3)\n",
        prog,
        DEFAULT_TOPOLOGY, DEFAULT_SLAVE, DEFAULT_DURATION, DEFAULT_PRINT_HZ
    );
}

static std::set<int> parseCpuAffinity(const char* str) {
    std::set<int> cpus;
    std::istringstream ss(str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        if (token.empty()) continue;
        int cpu = std::stoi(token);
        if (cpu < 0) throw std::invalid_argument("CPU index must be >= 0");
        cpus.insert(cpu);
    }
    if (cpus.empty()) throw std::invalid_argument("cpu-affinity must include at least one CPU");
    return cpus;
}

static Args parseArgs(int argc, char** argv) {
    Args a;
    static struct option long_opts[] = {
        {"topology",     required_argument, nullptr, 't'},
        {"slave",        required_argument, nullptr, 's'},
        {"duration",     required_argument, nullptr, 'd'},
        {"print-hz",     required_argument, nullptr, 'p'},
        {"rt-priority",  required_argument, nullptr, 'r'},
        {"cpu-affinity", required_argument, nullptr, 'c'},
        {"help",         no_argument,       nullptr, 'h'},
        {nullptr,        0,                 nullptr,  0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "t:s:d:p:r:c:h", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 't': a.topology     = optarg;                    break;
        case 's': a.slave        = optarg;                    break;
        case 'd': a.duration_s   = std::stod(optarg);         break;
        case 'p': a.print_hz     = std::stod(optarg);         break;
        case 'r': a.rt_priority  = std::stoi(optarg);         break;
        case 'c': a.cpu_affinity = parseCpuAffinity(optarg);  break;
        case 'h': printUsage(argv[0]); std::exit(0);
        default:  printUsage(argv[0]); std::exit(2);
        }
    }
    return a;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const Args args = parseArgs(argc, argv);

    MasterConfig cfg;
    try {
        cfg = loadTopology(args.topology);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load topology '%s': %s\n", args.topology.c_str(), e.what());
        return 1;
    }

    EthercatMaster master(cfg);
    MasterRuntime* rt = nullptr;

    try {
        rt = &master.initialize();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Master init failed: %s\n", e.what());
        return 1;
    }

    // Verify slave exists and is an EL5032.
    auto adapter_it = rt->adapters.find(args.slave);
    if (adapter_it == rt->adapters.end()) {
        std::fprintf(stderr, "Unknown slave '%s'. Available:", args.slave.c_str());
        for (auto& [k, _] : rt->adapters) std::fprintf(stderr, " %s", k.c_str());
        std::fprintf(stderr, "\n");
        master.close();
        return 1;
    }

    if (!dynamic_cast<El5032Adapter*>(adapter_it->second.get())) {
        std::fprintf(stderr, "Slave '%s' is not an EL5032 adapter.\n", args.slave.c_str());
        master.close();
        return 1;
    }

    const int soem_idx = rt->slave_index.at(args.slave);

    LoopRtConfig rt_cfg;
    rt_cfg.rt_priority  = std::clamp(args.rt_priority, 0, 99);
    rt_cfg.cpu_affinity = args.cpu_affinity;

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    EthercatLoop loop(*rt, cfg.cycle_hz, rt_cfg);
    loop.start();

    std::printf("Monitoring '%s' at position %d for %.1fs\n",
        args.slave.c_str(), soem_idx - 1, args.duration_s);

    const auto   t0           = std::chrono::steady_clock::now();
    const auto   deadline     = t0 + std::chrono::duration<double>(std::max(0.0, args.duration_s));
    const double print_period = 1.0 / std::max(args.print_hz, 0.1);
    auto         next_print   = t0;

    while (!g_shutdown.load() && std::chrono::steady_clock::now() < deadline) {
        const auto now = std::chrono::steady_clock::now();

        if (now >= next_print) {
            const SystemStatus status = loop.getStatus();

            auto slave_it = status.by_slave.find(args.slave);
            if (slave_it == status.by_slave.end() || !slave_it->second.has_value()) {
                std::printf(
                    "raw_pdo=unavailable "
                    "encoder_value_raw=unavailable "
                    "encoder_count_25bit=unavailable\n"
                );
            } else {
                const auto& d = std::any_cast<const Data&>(slave_it->second);
                std::printf("encoder_count_25bit=%u\n", d.encoder_count_25bit);
            }

            next_print += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(print_period));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    loop.stop();
    master.close();
    return 0;
}
