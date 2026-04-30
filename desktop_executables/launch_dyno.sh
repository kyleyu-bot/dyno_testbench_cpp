#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Run from repo root so relative paths in dyno_gui.py resolve correctly.
cd "$REPO_ROOT"

source "$REPO_ROOT/env_setup_scripts/env_setup.sh"
exec bash "$REPO_ROOT/src/interface_bridges/ros2/run_gui.sh"
