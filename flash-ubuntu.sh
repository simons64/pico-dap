#!/bin/sh
# flash-ubuntu.sh - upload the newest UF2 to a Raspberry Pi Pico / Pico 2 W
#                  from Linux (Ubuntu). The macOS version is ./flash.sh.
#
#   ./flash-ubuntu.sh            flash the newest .uf2 in build/
#   ./flash-ubuntu.sh -b         build first, then flash
#   ./flash-ubuntu.sh foo.uf2    flash a specific image
#
# Linux differences from flash.sh:
#   * CDC ports are /dev/ttyACM* (macOS: /dev/cu.usbmodem*)
#   * the BOOTSEL volume is mounted by udisks under /media/$USER/<label>, and is
#     not always automounted, so it is located via lsblk and mounted on demand
#
# A running board is rebooted into BOOTSEL via the 1200-baud touch, so the
# BOOTSEL button is only needed if the board has no USB stdio firmware.
# The user must be able to open /dev/ttyACM* (group 'dialout') and mount
# removable media (udisks, i.e. an active desktop/session or polkit rule).
set -eu

BUILD_DIR=${BUILD_DIR:-build}

usage() {
    echo "usage: $0 [-b] [firmware.uf2]" >&2
    exit 2
}

build=0
while getopts b flag; do
    case $flag in
        b) build=1 ;;
        *) usage ;;
    esac
done
shift $((OPTIND - 1))

if [ "$build" = 1 ]; then
    cmake --build "$BUILD_DIR"
fi

fw=${1:-}
if [ -z "$fw" ]; then
    fw=$(ls -t "$BUILD_DIR"/*.uf2 2>/dev/null | head -n 1 || true)
    [ -n "$fw" ] || {
        echo "error: no .uf2 in $BUILD_DIR (run '$0 -b')" >&2
        exit 1
    }
fi
[ -f "$fw" ] || {
    echo "error: '$fw' not found" >&2
    exit 1
}
echo "firmware: $fw"

# udev normally makes the CDC port root:dialout 0660. Fail early with a hint
# instead of a bare permission error from the termios helper below.
check_port_access() {
    [ -r "$1" ] && [ -w "$1" ] && return 0
    echo "error: no permission for $1" >&2
    echo "       add yourself to 'dialout' (sudo usermod -aG dialout $USER)," >&2
    echo "       then log out and back in." >&2
    return 1
}

# The 1200-baud touch needs a working termios. Pick an interpreter that really
# runs: `python3` may be a pyenv shim whose configured version is not installed,
# which fails with "version X is not installed" instead of opening the port.
pick_python() {
    for p in ${PYTHON:-} /usr/bin/python3 python3; do
        [ -n "$p" ] || continue
        if "$p" -c 'import termios' >/dev/null 2>&1; then
            printf '%s\n' "$p"
            return 0
        fi
    done
    return 1
}

# Set 1200 baud on the CDC port without blocking (the bootloader resets into
# BOOTSEL when it sees this line coding). stty can hang for minutes on a port
# whose firmware has stopped servicing USB, so use termios directly.
touch_1200() {
    check_port_access "$1" || return 1
    py=$(pick_python) || {
        echo "error: no working python3 with termios (set PYTHON=/usr/bin/python3)" >&2
        return 1
    }
    "$py" - "$1" <<'PY'
import os, sys, termios
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
try:
    a = termios.tcgetattr(fd)
    a[4] = a[5] = termios.B1200
    a[2] = (a[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)) \
        | termios.CS8 | termios.CREAD | termios.CLOCAL
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
finally:
    os.close(fd)
PY
}

# Print the mountpoint of every mounted BOOTSEL volume.
bootsel_mounts() {
    lsblk -n -o LABEL,MOUNTPOINT 2>/dev/null | awk '
        ($1 == "RP2350" || $1 == "RPI-RP2") && $2 != "" { print $2 }'
}

# Print /dev/<name> for every BOOTSEL block device, mounted or not.
bootsel_devices() {
    lsblk -P -n -o NAME,LABEL 2>/dev/null | awk '
        /LABEL="(RP2350|RPI-RP2)"/ {
            name = $1
            sub(/^NAME="/, "", name)
            sub(/"$/, "", name)
            print "/dev/" name
        }'
}

find_bootsel_volume() {
    vols=$(bootsel_mounts)
    [ -n "$vols" ] || return 1
    # Prefer a volume udisks put under the user's own media dir; the first
    # entry is good enough otherwise.
    for vol in $vols; do
        case $vol in
            /media/"$USER"/*|/run/media/"$USER"/*) printf '%s\n' "$vol"; return 0 ;;
        esac
    done
    printf '%s\n' "$vols" | head -n 1
}

# Mount the (already attached) BOOTSEL device with udisks. The bootloader is
# in MSD mode but no desktop session always mounts it for us.
mount_bootsel() {
    for dev in $(bootsel_devices); do
        udisksctl mount -b "$dev" >/dev/null 2>&1 || true
    done
}

# Wait (0.1 s steps) until the BOOTSEL volume is mounted; sets $vol.
wait_for_volume() {
    i=0
    while [ "$i" -lt "${1:-150}" ]; do
        if vol=$(find_bootsel_volume); then
            return 0
        fi
        mount_bootsel
        sleep 0.1
        i=$((i + 1))
    done
    return 1
}

vol=
if ! find_bootsel_volume >/dev/null; then
    if [ -n "${PORT:-}" ]; then
        port=$PORT
    else
        set -- /dev/ttyACM*
        [ "$#" -ge 1 ] && [ -e "$1" ] || {
            echo "error: no Pico serial port and no BOOTSEL volume." >&2
            echo "       hold BOOTSEL while plugging the board in, then rerun." >&2
            exit 1
        }
        [ "$#" -eq 1 ] || {
            echo "error: several serial ports ($*); set PORT=/dev/ttyACMx" >&2
            exit 1
        }
        port=$1
    fi
    echo "rebooting $port into BOOTSEL (1200 baud touch)"
    touch_1200 "$port" || {
        echo "error: 1200-baud touch on $port failed (port busy? close screen/minicom)" >&2
        exit 1
    }
    wait_for_volume 150 || {
        echo "error: BOOTSEL volume did not appear." >&2
        echo "       hold BOOTSEL while plugging the board in, then rerun." >&2
        echo "       (if it is attached but unmounted: udisksctl mount -b /dev/sdX1)" >&2
        exit 1
    }
fi
vol=$(find_bootsel_volume)

echo "copying to $vol"
cp "$fw" "$vol/"
# Push the image out of the page cache: the bootloader reads the sectors and
# reboots by itself once it has the whole UF2.
sync

# The bootloader reboots as soon as it has consumed the UF2.
i=0
while [ "$i" -lt 100 ] && [ -n "$(bootsel_mounts)" ]; do
    sleep 0.1
    i=$((i + 1))
done
if [ -n "$(bootsel_mounts)" ]; then
    # Still mounted after the copy usually means the volume went read-only
    # (dirty vfat) and the writes were dropped.
    echo "warning: $vol is still mounted; flash may have failed" >&2
    mount | grep -F " $vol " >&2 || true
    exit 1
fi

# Wait for the new firmware to come back up as a CDC device.
i=0
while [ "$i" -lt 200 ]; do
    set -- /dev/ttyACM*
    if [ "$#" -ge 1 ] && [ -e "$1" ]; then
        echo "flashed; serial on $1"
        exit 0
    fi
    sleep 0.1
    i=$((i + 1))
done
echo "flashed; no CDC port yet (firmware may not use USB stdio)" >&2
