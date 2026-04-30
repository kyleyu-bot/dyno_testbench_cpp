#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ASKPASS="$HOME/.local/bin/dyno-askpass"

# Create the zenity askpass helper once.
if [ ! -f "$ASKPASS" ]; then
    mkdir -p "$HOME/.local/bin"
    cat > "$ASKPASS" <<'EOF'
#!/bin/bash
zenity --password --title="Dyno Testbench" 2>/dev/null
EOF
    chmod +x "$ASKPASS"
fi

export SUDO_ASKPASS="$ASKPASS"

# One password prompt caches credentials for every sudo call below.
sudo -A -v

# Kill any leftover root processes from a previous run.
sudo pkill -f bridge_ros2 2>/dev/null || true
sudo pkill -f dyno_gui.py  2>/dev/null || true
sleep 0.5

cd "$REPO_ROOT"
source "$REPO_ROOT/env_setup_scripts/env_setup.sh"

# exec replaces this script with run_gui.sh (which exec's into sudo python3).
# Staying in one session means the cached sudo credentials carry through.
exec bash "$REPO_ROOT/src/interface_bridges/ros2/run_gui.sh"
