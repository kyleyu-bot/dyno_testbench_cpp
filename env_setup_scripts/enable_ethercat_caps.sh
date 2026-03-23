#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${REPO_ROOT}/.venv-ecat"
TARGET="${TARGET:-${VENV_DIR}/bin/python3}"
CAPS="${CAPS:-cap_net_raw,cap_net_admin,cap_sys_nice}"

usage() {
    cat <<EOF
Usage: sudo $0 [--target PATH] [--caps CAP_LIST]

Apply Linux file capabilities to the repo-local EtherCAT Python interpreter.

Options:
  --target PATH      Executable to grant capabilities to.
                     default: ${TARGET}
  --caps CAP_LIST    Comma-separated capabilities for setcap.
                     default: ${CAPS}
  -h, --help         Show this help text.

Examples:
  sudo $0
  sudo $0 --caps cap_net_raw,cap_net_admin
  sudo $0 --caps cap_net_raw,cap_net_admin,cap_sys_nice
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --target)
            TARGET="$2"
            shift 2
            ;;
        --caps)
            CAPS="$2"
            shift 2
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

if [[ ${EUID} -ne 0 ]]; then
    echo "error: run this script with sudo so it can call setcap." >&2
    exit 1
fi

if [[ ! -x "${TARGET}" ]]; then
    echo "error: target executable not found: ${TARGET}" >&2
    exit 1
fi

echo "Target: ${TARGET}"
echo "Caps:   ${CAPS}"
echo "Applying file capabilities"
setcap "${CAPS}=eip" "${TARGET}"

echo "Verifying file capabilities"
getcap "${TARGET}"

cat <<EOF

Complete.

This grants the listed capabilities directly to the interpreter binary, so any
script launched with that interpreter can use them without running the whole
program as root.
EOF
