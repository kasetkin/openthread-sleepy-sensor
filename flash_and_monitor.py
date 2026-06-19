#!/usr/bin/env python3
"""Flash the Matter sensor firmware to the ESP32-C6 and capture serial output.

Tailored to this project: chip esp32c6, 4 MB flash, the dual-slot OTA layout, and the
artifacts produced by `idf.py build` in ./build (which writes a ready-made `flash_args`).

Device output goes to stdout; script diagnostics go to stderr, so a caller can do:

    python flash_and_monitor.py > serial.log 2> flash.log
    python flash_and_monitor.py --no-flash --until "Commissioning complete"

Exit codes:
    0  — flash ok (or skipped) and capture finished cleanly
    1  — flash failed or serial port error
    2  — device panicked / rebooted unexpectedly during capture
"""

import argparse
import os
import subprocess
import sys
import time

try:
    import esptool  # noqa: F401  (only needed for flashing; re-exec into IDF python if absent)
except ImportError:
    _idf_python = os.path.join(os.environ.get("IDF_PYTHON_ENV_PATH", ""), "bin", "python")
    if os.path.exists(_idf_python) and sys.executable != _idf_python:
        os.execv(_idf_python, [_idf_python] + sys.argv)
    print("[monitor] esptool not found and ESP-IDF Python not available — run inside the "
          "esp-matter devcontainer / `. $IDF_PATH/export.sh`", file=sys.stderr)
    sys.exit(1)

import serial

# Lines that mean the firmware crashed — abort the capture and exit 2.
PANIC_PATTERNS = (
    "Guru Meditation Error",
    "Stack canary watchpoint",
    "assert failed",
    "abort() was called",
    "rst:0x",            # bootrom reset banner = an unexpected reboot mid-run
    "Rebooting...",
)


def flash(build_dir: str, port: str, baud: int) -> bool:
    """Flash using the `flash_args` esptool argfile produced by `idf.py build`."""
    flash_args = os.path.join(build_dir, "flash_args")
    if not os.path.exists(flash_args):
        print(f"[monitor] {flash_args} not found — run `idf.py build` first", file=sys.stderr)
        return False
    # Mirror idf.py's esptool invocation; flash mode/size/freq + offsets come from @flash_args.
    # (This esptool build uses underscore action names: default_reset / hard_reset / write_flash.)
    cmd = [
        sys.executable, "-m", "esptool",
        "--chip", "esp32c6",
        "-b", str(baud),
        "-p", port,
        "--before", "default_reset",
        "--after", "hard_reset",
        "write_flash",
        "@flash_args",
    ]
    print(f"[monitor] flashing from {build_dir} ...", file=sys.stderr)
    return subprocess.run(cmd, cwd=build_dir).returncode == 0


def reset_into_app(ser: serial.Serial) -> None:
    """Pulse the auto-reset lines (RTS->EN, DTR->IO0) so capture starts from boot."""
    ser.setDTR(False)
    ser.setRTS(True)    # assert reset
    time.sleep(0.2)
    ser.setRTS(False)   # release -> chip boots the app
    ser.setDTR(True)
    time.sleep(0.4)
    ser.reset_input_buffer()


def _open(port: str, baud: int) -> "serial.Serial | None":
    """Open the port, tolerating the brief USB-JTAG re-enumeration after a reset."""
    try:
        return serial.Serial(port, baud, timeout=1)
    except (serial.SerialException, OSError):
        return None


def capture(port: str, baud: int, timeout: float, idle_timeout: float, until: str | None,
            do_reset: bool = True) -> int:
    print(f"[monitor] {port} @ {baud} baud — timeout={timeout}s idle={idle_timeout}s"
          + (f" until={until!r}" if until else "")
          + ("" if do_reset else " (no-reset)"), file=sys.stderr)
    # On the C6, a reset (button or esptool) re-enumerates the USB-Serial-JTAG CDC, so the
    # port handle dies and reappears. Reconnect across that so a from-boot capture survives a
    # mid-stream reset. (DTR/RTS auto-reset doesn't drive EN here — to reboot, press the board
    # reset or re-flash with --after hard-reset.)
    deadline = time.time() + timeout
    last_data = time.time()
    panic = False
    ser = _open(port, baud)
    if ser and do_reset:
        reset_into_app(ser)
    try:
        while time.time() < deadline:
            if ser is None:
                time.sleep(0.3)
                ser = _open(port, baud)
                if ser is None:
                    if time.time() - last_data > idle_timeout:
                        print(f"[monitor] no output/port for {idle_timeout}s — stopping", file=sys.stderr)
                        break
                    continue
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError):
                ser.close()
                ser = None          # re-enumeration: reconnect on next loop
                continue
            if raw:
                last_data = time.time()
                line = raw.decode("utf-8", errors="replace").rstrip()
                print(line, flush=True)
                if hit := next((p for p in PANIC_PATTERNS if p in line), None):
                    print(f"[monitor] panic/reboot detected ({hit!r}) — aborting", file=sys.stderr)
                    panic = True
                    break
                if until and until in line:
                    print(f"[monitor] reached end marker {until!r}", file=sys.stderr)
                    break
            elif time.time() - last_data > idle_timeout:
                print(f"[monitor] no output for {idle_timeout}s — stopping", file=sys.stderr)
                break
    finally:
        if ser is not None:
            ser.close()
    print("[monitor] done", file=sys.stderr)
    return 2 if panic else 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0", help="serial port (default: /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=115200, help="console baud (default: 115200)")
    p.add_argument("--flash-baud", type=int, default=460800, help="flashing baud (default: 460800)")
    p.add_argument("--build-dir", default="build", help="dir with flash_args (default: build)")
    p.add_argument("--timeout", type=float, default=60.0, help="global capture deadline s (default: 60)")
    p.add_argument("--idle-timeout", type=float, default=20.0, help="stop after N idle s (default: 20)")
    p.add_argument("--until", default=None, metavar="STR", help="stop when this string appears in a line")
    p.add_argument("--no-flash", action="store_true", help="skip flashing, only capture serial")
    p.add_argument("--no-reset", action="store_true",
                   help="don't pulse DTR/RTS; just read the running firmware (needed on USB-JTAG)")
    args = p.parse_args()

    if not args.no_flash and not flash(args.build_dir, args.port, args.flash_baud):
        return 1
    # A fresh flash already hard-reset the chip, so don't reset again afterwards.
    do_reset = not args.no_reset and args.no_flash
    return capture(args.port, args.baud, args.timeout, args.idle_timeout, args.until, do_reset)


if __name__ == "__main__":
    sys.exit(main())
