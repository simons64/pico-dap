#!/bin/sh
# flash.sh - upload the newest UF2 to a Raspberry Pi Pico / Pico 2 W.
#
#   ./flash.sh            flash the newest .uf2 in build/
#   ./flash.sh -b         build first, then flash
#   ./flash.sh foo.uf2    flash a specific image
#
# macOS only: on Linux use ./flash-ubuntu.sh (ports and mount points differ).
#
# A running board is rebooted into BOOTSEL via the 1200-baud touch, so the
# BOOTSEL button is only needed if the board has no USB stdio firmware.
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

# Set 1200 baud on the CDC port without blocking (the bootloader resets into
# BOOTSEL when it sees this line coding). stty can hang for minutes on a port
# whose firmware has stopped servicing USB, so use termios directly.
touch_1200() {
    python3 - "$1" <<'PY'
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

find_bootsel_volume() {
    for name in RP2350 RPI-RP2; do
        vol="/Volumes/$name"
        if mount | grep -q " on $vol "; then
            printf '%s\n' "$vol"
            return 0
        fi
    done
    return 1
}

# Wait (0.1 s steps) until the BOOTSEL volume is mounted; sets $vol.
wait_for_volume() {
    i=0
    while [ "$i" -lt "${1:-150}" ]; do
        if vol=$(find_bootsel_volume); then
            return 0
        fi
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
        set -- /dev/cu.usbmodem*
        [ "$#" -ge 1 ] && [ -e "$1" ] || {
            echo "error: no Pico serial port and no BOOTSEL volume." >&2
            echo "       hold BOOTSEL while plugging the board in, then rerun." >&2
            exit 1
        }
        [ "$#" -eq 1 ] || {
            echo "error: several serial ports ($*); set PORT=/dev/cu.usbmodemXXXX" >&2
            exit 1
        }
        port=$1
    fi
    echo "rebooting $port into BOOTSEL (1200 baud touch)"
    touch_1200 "$port" || {
        echo "error: cannot open $port (busy? close screen/miniterm)" >&2
        exit 1
    }
    wait_for_volume 150 || {
        echo "error: BOOTSEL volume did not appear." >&2
        echo "       hold BOOTSEL while plugging the board in, then rerun." >&2
        exit 1
    }
fi
vol=$(find_bootsel_volume)

echo "copying to $vol"
cp "$fw" "$vol/"
sync

# The bootloader reboots as soon as it has consumed the UF2.
i=0
while [ "$i" -lt 100 ] && mount | grep -q " on $vol "; do
    sleep 0.1
    i=$((i + 1))
done
if mount | grep -q " on $vol "; then
    echo "warning: $vol is still mounted; flash may have failed" >&2
    exit 1
fi

# Wait for the new firmware to come back up as a CDC device.
i=0
while [ "$i" -lt 200 ]; do
    set -- /dev/cu.usbmodem*
    if [ "$#" -ge 1 ] && [ -e "$1" ]; then
        echo "flashed; serial on $1"
        exit 0
    fi
    sleep 0.1
    i=$((i + 1))
done
echo "flashed; no CDC port yet (firmware may not use USB stdio)" >&2
