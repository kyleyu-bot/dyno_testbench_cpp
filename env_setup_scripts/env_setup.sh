#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<EOF
Usage: $0

Run the repo's environment setup scripts in sequence.

This wrapper runs:
  1. bootstrap_venv_ecat.sh
  2. enable_ethercat_caps.sh
  3. tune_realtime.sh
  4. rt_setup_part2.sh

It intentionally does not run run_ethercat_python.sh because that script is a
Python launcher for the repo-local venv, not an environment configuration step.
EOF
}

if [[ $# -gt 0 ]]; then
    case "$1" in
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
fi

run_step() {
    local label="$1"
    shift

    echo
    echo "==> ${label}"
    "$@"
}

run_step "Bootstrapping EtherCAT virtual environment" \
    "${SCRIPT_DIR}/bootstrap_venv_ecat.sh"

run_step "Applying interpreter capabilities" \
    sudo "${SCRIPT_DIR}/enable_ethercat_caps.sh"

run_step "Applying host-level realtime tuning" \
    sudo "${SCRIPT_DIR}/tune_realtime.sh"

run_step "Applying NIC/IRQ realtime setup" \
    "${SCRIPT_DIR}/rt_setup_part2.sh"

echo
echo "Environment setup complete."
