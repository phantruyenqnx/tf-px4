#!/usr/bin/env bash
# setup_dev_env.sh
#
# Install development dependencies for building libGZHILBridge.so.
# Checks each package before installing — only installs what is missing.
#
# Usage:
#   cd src/modules/simulation/gz_bridge/hil
#   bash setup_dev_env.sh

set -euo pipefail

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PACKAGES=(
    libgz-sim8-dev
    libgz-transport13-dev
    libgz-plugin2-dev
    libgz-msgs10-dev
    libgtest-dev
    ninja-build
    cmake
)

missing=()

echo "Checking installed packages..."

for pkg in "${PACKAGES[@]}"; do
    if dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
        echo -e "  ${GREEN}ok${NC}  $pkg"
    else
        echo -e "  ${YELLOW}missing${NC}  $pkg"
        missing+=("$pkg")
    fi
done

if [ ${#missing[@]} -eq 0 ]; then
    echo ""
    echo "All dependencies already installed."
    exit 0
fi

echo ""
echo "Installing missing packages: ${missing[*]}"
sudo apt-get update -qq
sudo apt-get install -y "${missing[@]}"

echo ""
echo "Done."
