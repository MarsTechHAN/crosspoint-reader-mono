#!/usr/bin/env python3
"""Drive the Paper Mono panel lab (src/PanelLab.cpp) over serial.

The lab firmware prints "ready" after every command, so this just sends a line
and drains output until that marker (or a timeout). Development tool for
characterising the SSD1677 waveforms; not part of the firmware.

Usage:
  panel_lab.py boot                       # just capture the boot banner
  panel_lab.py 'card' 'g4'                # run commands in sequence
  panel_lab.py --timeout 90 'sweep 0 240 16'
"""

import argparse
import sys
import time

import serial

DEFAULT_PORT = "/dev/cu.usbmodem2101"
BAUD = 115200


def drain(ser, timeout, marker="ready"):
    """Read lines until `marker` appears alone on a line, or timeout expires."""
    deadline = time.time() + timeout
    lines = []
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(4096)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            text = raw.decode("utf-8", "replace").rstrip("\r")
            lines.append(text)
            print(text, flush=True)
            if text.strip() == marker:
                return lines
    if buf:
        text = buf.decode("utf-8", "replace")
        lines.append(text)
        print(text, flush=True)
    print(f"[timeout after {timeout}s]", file=sys.stderr, flush=True)
    return lines


def open_port(port, attempts=40):
    """The board's native USB CDC vanishes across a reset; wait for it to return."""
    last = None
    for _ in range(attempts):
        try:
            return serial.Serial(port, BAUD, timeout=0.2)
        except (serial.SerialException, OSError) as exc:
            last = exc
            time.sleep(1.0)
    raise SystemExit(f"could not open {port}: {last}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("commands", nargs="*")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--reset", action="store_true", help="toggle DTR/RTS to reboot the board first")
    args = ap.parse_args()

    ser = open_port(args.port)
    if args.reset:
        ser.setDTR(False)
        ser.setRTS(True)
        time.sleep(0.15)
        ser.setRTS(False)
        time.sleep(0.5)
    ser.reset_input_buffer()

    if not args.commands or args.commands == ["boot"]:
        # Nudge the shell so the banner replays even on an already-running board.
        ser.write(b"h\n")
        drain(ser, args.timeout)
        return

    for command in args.commands:
        print(f"\n### {command}", flush=True)
        ser.write((command + "\n").encode())
        ser.flush()
        drain(ser, args.timeout)


if __name__ == "__main__":
    main()
