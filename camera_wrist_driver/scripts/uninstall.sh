#!/usr/bin/env bash
# uninstall.sh — remove wrist camera systemd services, env files and udev rule.
#
# Usage:
#   sudo ./uninstall.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: please run as root (sudo $0)" >&2
    exit 1
fi

# 1. Stop and disable services
for side in left right; do
    svc="realsense_wrist_${side}.service"
    if systemctl list-unit-files | grep -q "^${svc}"; then
        systemctl stop "${svc}" 2>/dev/null || true
        systemctl disable "${svc}" 2>/dev/null || true
        echo "==> Stopped and disabled ${svc}"
    fi
done

# 2. Remove service files
rm -f /etc/systemd/system/realsense_wrist_left.service
rm -f /etc/systemd/system/realsense_wrist_right.service
systemctl daemon-reload
systemctl reset-failed || true
echo "==> Removed service files"

# 3. Remove env directory
rm -rf /etc/realsense-wrist
echo "==> Removed /etc/realsense-wrist"

# 4. Remove udev rule
rm -f /etc/udev/rules.d/99-realsense-d405.rules
udevadm control --reload-rules || true
echo "==> Removed udev rule"

echo ""
echo "==> Uninstall complete."
