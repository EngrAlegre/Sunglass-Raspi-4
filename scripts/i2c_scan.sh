#!/usr/bin/env bash
set -euo pipefail

if ! command -v i2cdetect >/dev/null 2>&1; then
  echo "i2cdetect not found. Install: sudo apt-get update && sudo apt-get install -y i2c-tools"
  exit 1
fi

echo "Scanning I2C bus 1..."
sudo i2cdetect -y 1

echo
echo "If all VL53L1X sensors are still at the default address, you may only see 0x29."
echo "After the app boots with XSHUT sequencing, expected addresses are 0x30, 0x31, 0x32."
