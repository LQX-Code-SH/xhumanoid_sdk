#!/usr/bin/env bash
# install.sh - install wrist camera systemd services for two Intel RealSense D405.
#
# Steps:
#   1. Check & auto-install runtime dependencies:
#        - ROS environments (/opt/ros/jazzy, /opt/humanoid, ~/xos) - checked
#        - pyrealsense2 + numpy for the service user - pip-installed if missing
#        - udev / systemd - checked (present on any Ubuntu 24.04)
#   2. Detect connected D405 cameras (serial numbers) via pyrealsense2.
#   3. Map left/right (interactive or --left-sn/--right-sn).
#   4. Write /etc/realsense-wrist/wrist_{left,right}.env (WRIST_SERIAL).
#   5. Install udev rule (0666) for D405 and reload.
#   6. Render service files (replace {REPO_DIR}) into /etc/systemd/system/.
#   7. daemon-reload + enable --now, then verify the services are active.
#
# Usage:
#   sudo ./install.sh
#   sudo ./install.sh --left-sn <SN> --right-sn <SN>
#   sudo ./install.sh --user <username>
#   sudo ./install.sh --skip-deps    # assume deps ready, only check
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts/.. is this project directory (camera_wrist_driver).
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

USER_NAME="${USER_NAME:-nvidia}"
LEFT_SN=""
RIGHT_SN=""
LEFT_ROTATE="${LEFT_ROTATE:-0}"
RIGHT_ROTATE="${RIGHT_ROTATE:-0}"
LEFT_FPS="${LEFT_FPS:-15}"
RIGHT_FPS="${RIGHT_FPS:-15}"
LEFT_WIDTH="${LEFT_WIDTH:-640}"
RIGHT_WIDTH="${RIGHT_WIDTH:-640}"
LEFT_HEIGHT="${LEFT_HEIGHT:-480}"
RIGHT_HEIGHT="${RIGHT_HEIGHT:-480}"
COMPRESSED="${COMPRESSED:-1}"
JPEG_QUALITY="${JPEG_QUALITY:-80}"
DEPTH_MAX="${DEPTH_MAX:-10.0}"
SKIP_DEPS=0
UDEV_RULE_FILE="/etc/udev/rules.d/99-realsense-d405.rules"
ENV_DIR="/etc/realsense-wrist"
SERVICES_DIR="/etc/systemd/system"
ROS_SETUP_FILES=(
    "/opt/ros/jazzy/setup.bash"
    "/opt/humanoid/install/setup.bash"
)

usage() {
    cat <<EOF
Usage: $0 [options]
Options:
  --left-sn <SN>    Serial number of the left wrist D405 camera
  --right-sn <SN>   Serial number of the right wrist D405 camera
  --left-rotate <D> Left camera rotation in degrees: 0/90/180/270 (default: 0)
  --right-rotate <D> Right camera rotation in degrees: 0/90/180/270 (default: 0)
  --left-fps <N>    Left camera frame rate: 5/15/30 (default: 15)
  --right-fps <N>   Right camera frame rate: 5/15/30 (default: 15)
  --left-res <WxH>  Left camera resolution, e.g. 640x480 or 848x480 (default: 640x480)
  --right-res <WxH> Right camera resolution, e.g. 640x480 or 848x480 (default: 640x480)
  --no-compressed   Disable the /compressed and /compressedDepth topics (default: on)
  --jpeg-quality <N> JPEG quality 1-100 for /compressed (default: 80)
  --depth-max <M>   compressedDepth clip distance in meters (default: 10.0)
  --user <name>     Systemd service User (default: nvidia)
  --skip-deps       Skip dependency installation (check only)
  -h, --help        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --left-sn)    LEFT_SN="$2"; shift 2 ;;
        --right-sn)   RIGHT_SN="$2"; shift 2 ;;
        --left-rotate)  LEFT_ROTATE="$2"; shift 2 ;;
        --right-rotate) RIGHT_ROTATE="$2"; shift 2 ;;
        --left-fps)     LEFT_FPS="$2"; shift 2 ;;
        --right-fps)    RIGHT_FPS="$2"; shift 2 ;;
        --left-res)
            if [[ ! "$2" =~ ^[0-9]+x[0-9]+$ ]]; then
                echo "ERROR: --left-res must be <W>x<H>, got '$2'" >&2
                exit 2
            fi
            LEFT_WIDTH="${2%x*}"; LEFT_HEIGHT="${2#*x}"; shift 2 ;;
        --right-res)
            if [[ ! "$2" =~ ^[0-9]+x[0-9]+$ ]]; then
                echo "ERROR: --right-res must be <W>x<H>, got '$2'" >&2
                exit 2
            fi
            RIGHT_WIDTH="${2%x*}"; RIGHT_HEIGHT="${2#*x}"; shift 2 ;;
        --no-compressed)  COMPRESSED=0; shift ;;
        --jpeg-quality) JPEG_QUALITY="$2"; shift 2 ;;
        --depth-max)    DEPTH_MAX="$2"; shift 2 ;;
        --user)       USER_NAME="$2"; shift 2 ;;
        --skip-deps)  SKIP_DEPS=1; shift ;;
        -h|--help)    usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
    esac
done

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: please run as root (sudo $0)" >&2
    exit 1
fi

if [[ ! -f "${SCRIPT_DIR}/start_wrist.sh" ]]; then
    echo "ERROR: start_wrist.sh not found next to install.sh" >&2
    exit 1
fi

# Resolve the service user's home (pip must install into THEIR environment).
USER_HOME="$(getent passwd "${USER_NAME}" | cut -d: -f6 || true)"
if [[ -z "${USER_HOME}" || ! -d "${USER_HOME}" ]]; then
    echo "ERROR: service user '${USER_NAME}' has no home directory." >&2
    echo "       Pass --user <name> with an existing account." >&2
    exit 1
fi
USER_XOS_SETUP="${USER_HOME}/xos/setup.bash"

err()  { echo "ERROR: $*" >&2; }
warn() { echo "WARNING: $*" >&2; }

# ---------------------------------------------------------------------------
# 1. Runtime dependencies: check & auto-install
# ---------------------------------------------------------------------------
echo "==> Checking runtime dependencies (service user: ${USER_NAME})..."

# 1a. Base system tools (udev / systemd are preinstalled on Ubuntu 24.04).
for cmd in python3 pip3 udevadm systemctl getent; do
    command -v "${cmd}" >/dev/null 2>&1 || { err "${cmd} not found in PATH"; exit 1; }
done
echo "   system tools: OK (python3, pip3, udevadm, systemctl)"

# 1b. ROS environments (factory-provisioned on the robot; cannot be pip-installed).
MISSING_ROS=()
for f in "${ROS_SETUP_FILES[@]}" "${USER_XOS_SETUP}"; do
    if [[ -f "$f" ]]; then
        echo "   ROS env: $f"
    else
        MISSING_ROS+=("$f")
    fi
done
if [[ ${#MISSING_ROS[@]} -gt 0 ]]; then
    warn "missing ROS environment files (cannot be auto-installed):"
    for f in "${MISSING_ROS[@]}"; do
        echo "     - $f" >&2
    done
    echo "     start_wrist.sh sources them when present; the service may fail" >&2
    echo "     to talk to other nodes if they stay missing." >&2
fi

# 1c. Python packages for the SERVICE USER (not root!): run pip as that user
#     via sudo -u so packages land in the environment the node runs in.
#     pip --user installs into ~/.local/lib/...site-packages, which plain
#     python3 picks up via user-site on sys.path (no ~/.local/bin/python3).
py_module_ok() {  # $1=user $2=module name
    sudo -u "$1" -- python3 -c "import $2" 2>/dev/null
}

PIP_TARGETS=(pyrealsense2 numpy)
for mod in "${PIP_TARGETS[@]}"; do
    if py_module_ok "${USER_NAME}" "${mod}"; then
        echo "   python: ${mod} already installed for ${USER_NAME}"
    elif [[ ${SKIP_DEPS} -eq 1 ]]; then
        err "python module '${mod}' missing for user ${USER_NAME} (--skip-deps given, not installing)"
        exit 1
    else
        # Ubuntu 24.04 marks the system python as externally managed (PEP 668);
        # --user installs to ~/.local cannot break the system interpreter.
        echo "   python: installing ${mod} for ${USER_NAME} (pip --user)..."
        if ! sudo -u "${USER_NAME}" -- pip3 install --user --quiet --break-system-packages "${mod}"; then
            # Offline robot: try the robot's apt mirror before giving up.
            echo "   pip install failed, trying apt (python3-${mod})..." >&2
            if ! apt-get install -y --no-install-recommends "python3-${mod}" >/dev/null 2>&1; then
                err "failed to install '${mod}' for user ${USER_NAME}"
                echo "  - check network/mirror, or install manually as ${USER_NAME}:" >&2
                echo "      sudo -u ${USER_NAME} pip3 install --user --break-system-packages ${mod}" >&2
                exit 1
            fi
            echo "   python: ${mod} installed via apt"
        else
            echo "   python: ${mod} installed via pip (--user)"
        fi
    fi
done

# ---------------------------------------------------------------------------
# 2. Detect D405 cameras
# ---------------------------------------------------------------------------
echo "==> Detecting RealSense cameras..."

# One serial per line, printed by a single python call (robust parsing).
# Run as the service user so USB permissions are validated the same way
# the node will see them at runtime.
DETECT_SNS_OUTPUT="$(sudo -u "${USER_NAME}" -- python3 - <<'PY' 2>/dev/null || true
import pyrealsense2 as rs

for dev in rs.context().query_devices():
    print(dev.get_info(rs.camera_info.serial_number))
PY
)"

if [[ -z "${DETECT_SNS_OUTPUT}" ]]; then
    # Retry once as root to distinguish "no camera" from "permission denied".
    ROOT_SNS="$(python3 - <<'PY' 2>/dev/null || true
import pyrealsense2 as rs

for dev in rs.context().query_devices():
    print(dev.get_info(rs.camera_info.serial_number))
PY
)"
    if [[ -n "${ROOT_SNS}" ]]; then
        err "cameras visible to root but not to ${USER_NAME}: udev rule not applied?"
        echo "  Replug the cameras or run: udevadm control --reload-rules && udevadm trigger" >&2
        exit 1
    fi
fi

readarray -t DETECTED_SNS <<<"${DETECT_SNS_OUTPUT}"

if [[ ${#DETECTED_SNS[@]} -eq 0 ]]; then
    err "no RealSense cameras detected. Check USB (lsusb | grep 8086) and connections."
    exit 1
fi

echo "   Detected cameras: ${DETECTED_SNS[*]}"
if [[ ${#DETECTED_SNS[@]} -lt 2 ]]; then
    warn "expected 2 wrist cameras, only ${#DETECTED_SNS[@]} found."
fi

# ---------------------------------------------------------------------------
# 3. Resolve left/right serials
# ---------------------------------------------------------------------------
if [[ -z "${LEFT_SN}" ]]; then
    echo ""
    echo "Select LEFT wrist camera serial (1..${#DETECTED_SNS[@]}):"
    for i in "${!DETECTED_SNS[@]}"; do
        echo "   $((i+1))) ${DETECTED_SNS[$i]}"
    done
    read -r -p "> " choice
    if [[ ! "${choice}" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#DETECTED_SNS[@]} )); then
        err "invalid choice '${choice}'"
        exit 1
    fi
    LEFT_SN="${DETECTED_SNS[$((choice-1))]}"
fi

if [[ -z "${RIGHT_SN}" ]]; then
    # Suggest the first serial that is not the left one
    CANDIDATES=()
    for sn in "${DETECTED_SNS[@]}"; do
        [[ "${sn}" != "${LEFT_SN}" ]] && CANDIDATES+=("${sn}")
    done
    if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
        err "need a second camera for the right wrist."
        exit 1
    fi
    echo ""
    echo "Select RIGHT wrist camera serial:"
    for i in "${!CANDIDATES[@]}"; do
        echo "   $((i+1))) ${CANDIDATES[$i]}"
    done
    read -r -p "> " choice
    if [[ ! "${choice}" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#CANDIDATES[@]} )); then
        err "invalid choice '${choice}'"
        exit 1
    fi
    RIGHT_SN="${CANDIDATES[$((choice-1))]}"
fi

for rot in "${LEFT_ROTATE}" "${RIGHT_ROTATE}"; do
    if [[ ! "${rot}" =~ ^(0|90|180|270)$ ]]; then
        err "rotation must be 0/90/180/270, got '${rot}'"
        exit 1
    fi
done
for fps in "${LEFT_FPS}" "${RIGHT_FPS}"; do
    if [[ ! "${fps}" =~ ^(5|15|30)$ ]]; then
        err "fps must be 5/15/30, got '${fps}'"
        exit 1
    fi
done
for res in "${LEFT_WIDTH}x${LEFT_HEIGHT}" "${RIGHT_WIDTH}x${RIGHT_HEIGHT}"; do
    if [[ ! "${res}" =~ ^[0-9]+x[0-9]+$ ]]; then
        err "resolution must be <W>x<H>, got '${res}'"
        exit 1
    fi
done
if [[ ! "${JPEG_QUALITY}" =~ ^[0-9]+$ ]] || (( JPEG_QUALITY < 1 || JPEG_QUALITY > 100 )); then
    err "jpeg quality must be 1-100, got '${JPEG_QUALITY}'"
    exit 1
fi
if [[ ! "${DEPTH_MAX}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    err "depth max must be a number (meters), got '${DEPTH_MAX}'"
    exit 1
fi

echo ""
echo "==> Mapping: left=${LEFT_SN} (rotate=${LEFT_ROTATE}, ${LEFT_WIDTH}x${LEFT_HEIGHT}@${LEFT_FPS})  right=${RIGHT_SN} (rotate=${RIGHT_ROTATE}, ${RIGHT_WIDTH}x${RIGHT_HEIGHT}@${RIGHT_FPS})"

# ---------------------------------------------------------------------------
# 4. Write environment files
# ---------------------------------------------------------------------------
mkdir -p "${ENV_DIR}"
cat > "${ENV_DIR}/wrist_left.env" <<EOF
# Generated by camera_wrist_driver install.sh on $(date)
WRIST_SERIAL=${LEFT_SN}
WRIST_SIDE=left
WRIST_ROTATE=${LEFT_ROTATE}
WRIST_WIDTH=${LEFT_WIDTH}
WRIST_HEIGHT=${LEFT_HEIGHT}
WRIST_FPS=${LEFT_FPS}
WRIST_COMPRESSED=${COMPRESSED}
WRIST_JPEG_QUALITY=${JPEG_QUALITY}
WRIST_DEPTH_MAX=${DEPTH_MAX}
EOF
cat > "${ENV_DIR}/wrist_right.env" <<EOF
# Generated by camera_wrist_driver install.sh on $(date)
WRIST_SERIAL=${RIGHT_SN}
WRIST_SIDE=right
WRIST_ROTATE=${RIGHT_ROTATE}
WRIST_WIDTH=${RIGHT_WIDTH}
WRIST_HEIGHT=${RIGHT_HEIGHT}
WRIST_FPS=${RIGHT_FPS}
WRIST_COMPRESSED=${COMPRESSED}
WRIST_JPEG_QUALITY=${JPEG_QUALITY}
WRIST_DEPTH_MAX=${DEPTH_MAX}
EOF
chmod 0644 "${ENV_DIR}"/wrist_*.env
echo "==> Wrote ${ENV_DIR}/wrist_{left,right}.env"

# ---------------------------------------------------------------------------
# 5. udev rule (D405: vendor 8086, product 0b5b)
# ---------------------------------------------------------------------------
cat > "${UDEV_RULE_FILE}" <<EOF
# Intel RealSense D405 wrist cameras - allow non-root access
SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5b", MODE="0666"
EOF
chmod 0644 "${UDEV_RULE_FILE}"
udevadm control --reload-rules || true
udevadm trigger || true
echo "==> Installed ${UDEV_RULE_FILE}"

# ---------------------------------------------------------------------------
# 6. Render and install systemd services
# ---------------------------------------------------------------------------
install_service() {
    local side="$1"
    local src="${SCRIPT_DIR}/../service/realsense_wrist_${side}.service"
    local dst="${SERVICES_DIR}/realsense_wrist_${side}.service"
    sed -e "s|{REPO_DIR}|${PROJECT_DIR}|g" \
        -e "s|^User=.*|User=${USER_NAME}|" \
        "${src}" > "${dst}"
    chmod 0644 "${dst}"
    echo "==> Installed ${dst}"
}

install_service left
install_service right

# ---------------------------------------------------------------------------
# 7. Enable, start and verify
# ---------------------------------------------------------------------------
systemctl daemon-reload
systemctl enable realsense_wrist_left.service
systemctl enable realsense_wrist_right.service
systemctl restart realsense_wrist_left.service
systemctl restart realsense_wrist_right.service

# Poll for the services to come up before reporting. Two D405 pipelines on
# one SoC take several seconds to open, so a fixed short sleep misreports
# healthy-but-still-activating services as failed. A crash-looping service
# sits in "activating" during restart backoff and still passes is-active,
# so also check the systemd restart counter.
FAILED=0
for side in left right; do
    svc="realsense_wrist_${side}.service"
    for ((t = 0; t < 30; t += 2)); do
        systemctl is-active --quiet "${svc}" && break
        systemctl is-failed --quiet "${svc}" && break
        sleep 2
    done
    restarts="$(systemctl show -p NRestarts --value "${svc}")"
    if systemctl is-active --quiet "${svc}" && [[ "${restarts}" -le 1 ]]; then
        echo "==> ${svc}: active (running)"
    else
        FAILED=1
        err "${svc} NOT healthy (state: $(systemctl is-active "${svc}"), restarts: ${restarts}). Journal hints:"
        journalctl -u "${svc}" -n 10 --no-pager | sed 's/^/     /' >&2 || true
    fi
done

echo ""
echo "==> Done. Verification:"
echo "   systemctl status realsense_wrist_left.service"
echo "   systemctl status realsense_wrist_right.service"
echo "   source /opt/ros/jazzy/setup.bash && source /opt/humanoid/install/setup.bash && source ~/xos/setup.bash"
echo "   ros2 topic hz /ob_camera_wrist_left/color/image_raw"
echo "   ros2 topic hz /ob_camera_wrist_right/depth/image_raw"

if [[ ${FAILED} -ne 0 ]]; then
    exit 1
fi
