#!/bin/bash

# 1. Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo "Error: This script must be run as root."
   echo "Usage: sudo ./cpu_performance.sh"
   exit 1
fi

echo "==============================================="
echo "Configuring CPU for Real-Time Performance..."
echo "==============================================="

# 2. Method A: Using cpupower (Preferred if installed)
if command -v cpupower &> /dev/null; then
    echo "[*] Found 'cpupower' tool. Using it..."
    cpupower frequency-set -g performance
    echo "CPU frequency set to performance."
    
# 3. Method B: Direct System Write (Universal Fallback)
else
    echo "[*] 'cpupower' not found. Writing directly to /sys/..."
    
    # Check if scaling driver is enabled
    if [ -d "/sys/devices/system/cpu/cpu0/cpufreq" ]; then
        for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
            echo performance > "$governor"
        done
    else
        echo "[!] Error: CPU frequency scaling driver not loaded or unavailable."
        exit 1
    fi
fi

# 4. Verify the result
echo "-----------------------------------------------"
echo "Verification (Core 0):"
CURRENT_GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)
echo "Current Governor: $CURRENT_GOV"

if [ "$CURRENT_GOV" == "performance" ]; then
    echo "SUCCESS: CPU is in Performance Mode."
else
    echo "WARNING: CPU might still be in Powersave mode."
fi
echo "==============================================="