#!/usr/bin/env bash
# Sweep of CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP_MIN_SLP_MS against stock ESP-IDF.
# Plan and campaign status: tickets_and_plans/plan_min_slp_sweep.html (local, not in git).
#
#   ./sweep.sh build [values...]   build build-sweep-<v> for each value (default: all)
#   ./sweep.sh flash <v>           flash build-sweep-<v> over USB-Serial-JTAG; the board must already
#                                  be in download mode (hold BOOT, tap RESET); tap RESET after it
#   ./sweep.sh capture <log>       start the UART capture detached, so it survives the shell/session
#   ./sweep.sh status <log>        is the capture running, last lines, fit per segment
#   ./sweep.sh stop                stop the capture
#
# Values: "stock" (option off) or a MIN_SLP_MS value. Environment overrides: JTAG, UART, BUSY_MS.
# Each build has its own sdkconfig inside its build directory; the project's sdkconfig is not used.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ALL_VALUES="stock 0 10 50 100 200 500 1000"
JTAG="${JTAG:-/dev/ttyACM0}"
UART="${UART:-/dev/ttyUSB0}"

idf_env() {
    export IDF_PATH=/workspaces/esp-idf
    export IDF_TOOLS_PATH=/workspaces/.espressif-master
    # The container's default points at the IDF v6.0 Python env, which fails on master.
    export IDF_PYTHON_ENV_PATH=/workspaces/.espressif-master/python_env/idf6.2_py3.12_env
    # shellcheck disable=SC1091
    . "$IDF_PATH/export.sh" > /dev/null
}

build_dir() {
    echo "$HERE/build-sweep-$1${BUSY_MS:+-busy$BUSY_MS}"
}

cmd_build() {
    idf_env
    local values="${*:-$ALL_VALUES}"
    for v in $values; do
        local dir
        dir="$(build_dir "$v")"
        mkdir -p "$dir"
        if [ "$v" = stock ]; then
            echo "# CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP is not set" > "$dir/sweep.defaults"
        else
            printf 'CONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP=y\nCONFIG_PM_LIGHTSLEEP_RTC_OSC_CAL_AFTER_WAKEUP_MIN_SLP_MS=%s\n' "$v" > "$dir/sweep.defaults"
        fi
        if [ -n "${BUSY_MS:-}" ]; then
            echo "CONFIG_REPRO_BUSY_MS=$BUSY_MS" >> "$dir/sweep.defaults"
        fi
        # A stale sdkconfig would ignore changed defaults.
        rm -f "$dir/sdkconfig"
        echo "== build $v -> $dir"
        (cd "$HERE" && idf.py -B "$dir" -D SDKCONFIG="$dir/sdkconfig" \
            -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;$dir/sweep.defaults" build > "$dir/sweep_build.log" 2>&1) \
            || { tail -20 "$dir/sweep_build.log"; exit 1; }
        grep -E "CAL_AFTER_WAKEUP|FREERTOS_HZ |REPRO_BUSY_MS|REPRO_SEED" "$dir/config/sdkconfig.h" | sed 's/^/   /'
    done
}

cmd_flash() {
    local v="${1:?value}"
    local dir
    dir="$(build_dir "$v")"
    [ -f "$dir/flash_args" ] || { echo "no $dir/flash_args: run ./sweep.sh build $v"; exit 1; }
    idf_env
    (cd "$dir" && python -m esptool --chip esp32c6 -p "$JTAG" -b 460800 \
        --before no-reset --after hard-reset write-flash @flash_args)
    echo "flashed $v at $(date -u +%FT%TZ). Tap RESET: after this flash the chip stays in download mode."
}

capture_pids() {
    pgrep -f "clock_drift.py log $UART" || true
}

cmd_capture() {
    local log="${1:?log file}"
    if [ -n "$(capture_pids)" ]; then
        echo "a capture on $UART is already running: $(capture_pids)"; exit 1
    fi
    mkdir -p "$(dirname "$log")"
    # clock_drift.py appends, so restarting it on the same file continues the same capture.
    setsid nohup uv run --script "$HERE/clock_drift.py" log "$UART" -o "$log" > /dev/null 2>> "$log.err" < /dev/null &
    sleep 2
    echo "capture $UART -> $log, pid(s): $(capture_pids)"
}

cmd_status() {
    local log="${1:?log file}"
    echo "capture pid(s): $(capture_pids)"
    [ -s "$log.err" ] && { echo "--- $log.err:"; tail -5 "$log.err"; }
    echo "--- last lines:"; tail -3 "$log"
    echo "--- fit:"; uv run --script "$HERE/clock_drift.py" fit "$log" --skip 600
}

cmd_stop() {
    local pids
    pids="$(capture_pids)"
    [ -n "$pids" ] && kill $pids && echo "stopped $pids" || echo "no capture running"
}

case "${1:-}" in
    build) shift; cmd_build "$@" ;;
    flash) shift; cmd_flash "$@" ;;
    capture) shift; cmd_capture "$@" ;;
    status) shift; cmd_status "$@" ;;
    stop) cmd_stop ;;
    *) sed -n '2,15p' "$0"; exit 1 ;;
esac
