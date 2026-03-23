#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    cat <<EOF
Usage: sudo $0 [options]

Apply a small set of host-level real-time tuning actions that are kept separate
from Python environment bootstrap.

Options:
  --service NAME     irqbalance service name to stop/disable.
                     default: irqbalance
  --rtprio N         Suggested rtprio limit to print.
                     default: 95
  --memlock VALUE    Suggested memlock limit to print.
                     default: unlimited
  --apply-sysctl     Write kernel.sched_rt_runtime_us = -1 via sysctl.
  --dry-run          Print recommended actions only.
  -h, --help         Show this help text.

Notes:
  This script does not install packages or rewrite bootloader/kernel arguments.
  It is intended as a narrow helper for host tuning steps you may want to apply
  on a dedicated EtherCAT controller.
EOF
}

IRQBALANCE_SERVICE="irqbalance"
RTPRIO="95"
MEMLOCK="unlimited"
APPLY_SYSCTL=0
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --service)
            IRQBALANCE_SERVICE="$2"
            shift 2
            ;;
        --rtprio)
            RTPRIO="$2"
            shift 2
            ;;
        --memlock)
            MEMLOCK="$2"
            shift 2
            ;;
        --apply-sysctl)
            APPLY_SYSCTL=1
            shift
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [[ ${DRY_RUN} -eq 0 && ${EUID} -ne 0 ]]; then
    echo "error: run with sudo, or use --dry-run to print the steps only." >&2
    exit 1
fi

cat <<EOF
Recommended real-time tuning checklist
1. Confirm you are using an RT-capable kernel if deterministic latency matters.
2. Pin the EtherCAT loop and relevant IRQs to isolated CPUs.
3. Stop background balancing services on dedicated controllers when appropriate.
4. Raise rtprio and memlock limits for the user/service running the controller.
5. Grant CAP_SYS_NICE if you want SCHED_FIFO without full root.
EOF

if systemctl list-unit-files "${IRQBALANCE_SERVICE}.service" >/dev/null 2>&1; then
    if [[ ${DRY_RUN} -eq 1 ]]; then
        echo
        echo "Would stop and disable ${IRQBALANCE_SERVICE}.service"
    else
        echo
        echo "Stopping ${IRQBALANCE_SERVICE}.service"
        systemctl stop "${IRQBALANCE_SERVICE}.service"
        echo "Disabling ${IRQBALANCE_SERVICE}.service"
        systemctl disable "${IRQBALANCE_SERVICE}.service"
    fi
fi

if [[ ${APPLY_SYSCTL} -eq 1 ]]; then
    if [[ ${DRY_RUN} -eq 1 ]]; then
        echo
        echo "Would run: sysctl -w kernel.sched_rt_runtime_us=-1"
    else
        echo
        echo "Setting kernel.sched_rt_runtime_us=-1"
        sysctl -w kernel.sched_rt_runtime_us=-1
    fi
fi

cat <<EOF

Suggested limits configuration
Create a file such as /etc/security/limits.d/ethercat-rt.conf with:
  *               soft    rtprio    ${RTPRIO}
  *               hard    rtprio    ${RTPRIO}
  *               soft    memlock   ${MEMLOCK}
  *               hard    memlock   ${MEMLOCK}

Likely follow-up commands for this repo
  sudo "${REPO_ROOT}/env_setup_scripts/enable_ethercat_caps.sh" --caps cap_net_raw,cap_net_admin,cap_sys_nice
  taskset -c <cpu-list> "${REPO_ROOT}/.venv-ecat/bin/python" <script> [args]

This helper intentionally avoids mixing interpreter/bootstrap concerns with
system-level real-time policy changes.
EOF
