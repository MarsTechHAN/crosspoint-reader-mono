#!/usr/bin/env python3
"""Render text pages on the host and upload them to the Paper Mono panel lab.

Solid staircases say nothing about how a waveform handles antialiased glyph
edges, which is the only reason a reader wants grays at all. Rasterising text on
the controller would mean dragging the font decompressor into src/PanelLab.cpp,
so the page is rendered here with PIL and shipped as the two bit planes the
SSD1677 actually wants.

The two pages are built to be hostile to ghosting: different body text (so every
glyph position changes), and a large solid block that swaps sides between A and
B, so any pigment the single activation failed to move shows up as a rectangle
on plain white.

  panel_page.py                 # render, upload both pages, deghost, enter A/B
  panel_page.py --preview out   # write out_a.png / out_b.png, upload nothing

Development tool for characterising the panel; not part of the firmware.
"""

import argparse
import sys
import time

import serial
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from panel_lab import BAUD, DEFAULT_PORT, drain, open_port  # noqa: E402

W, H = 800, 480
PLANE_BYTES = W * H // 8

# Ordered light -> dark. The firmware quantises levels 1 and 2 together for
# 'tri', so anything that is neither white nor black lands on the middle level.
LEVEL_WHITE, LEVEL_GRAY, LEVEL_BLACK = 0, 2, 3

FONT_CANDIDATES = [
    "lib/EpdFont/builtinFonts/source/NotoSerif/NotoSerif-Regular.ttf",
    "/System/Library/Fonts/Supplemental/Times New Roman.ttf",
    "/System/Library/Fonts/Supplemental/Georgia.ttf",
    "/Library/Fonts/Arial.ttf",
]
BOLD_CANDIDATES = [
    "lib/EpdFont/builtinFonts/source/NotoSerif/NotoSerif-Bold.ttf",
    "/System/Library/Fonts/Supplemental/Times New Roman Bold.ttf",
    "/System/Library/Fonts/Supplemental/Georgia Bold.ttf",
]

PAGE_A = """The controller gives two bits of RAM per pixel, which is four
codes per activation and not one more. Three grays need exactly four
classes: leave alone, go white, go gray, go black. Four grays need five,
so four grays cannot be done in a single pass no matter how the waveform
is arranged. The trick that makes three fit is that the reset to white is
not a class at all. It is the first phase, shared by every driven code,
and only afterwards do the codes separate onto their own schedules. A
pixel therefore lands on the same shade regardless of where it started,
while the pixels that did not change are never given a field and so never
flash."""

PAGE_B = """Gray is developed with the weak source rail rather than the
strong one, and the reason is measured rather than assumed. Driving
toward white takes the full optical swing in about ten frames, so one
frame is a tenth of the range, far too coarse to place a level on
purpose. Worse, the gate lines droop across eight hundred columns, and at
that speed the droop is large enough to read as a shade difference
between the left and right halves of the screen. The weak rail needs
roughly sixty frames for the same swing. That buys ten or more usable
steps and hides the droop entirely."""


def load_font(candidates, size):
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    raise SystemExit(f"no usable TTF among {candidates}")


def wrap(draw, text, font, width):
    """Greedy wrap on the rendered advance width."""
    lines = []
    for para in text.split("\n\n"):
        words = para.replace("\n", " ").split()
        cur = ""
        for word in words:
            trial = f"{cur} {word}".strip()
            if draw.textlength(trial, font=font) <= width:
                cur = trial
            else:
                lines.append(cur)
                cur = word
        if cur:
            lines.append(cur)
    return lines


def render(title, body, block_left):
    """One page: header, reference patches, body text, and a ghost-bait block."""
    img = Image.new("L", (W, H), 255)
    draw = ImageDraw.Draw(img)
    body_font = load_font(FONT_CANDIDATES, 20)
    head_font = load_font(BOLD_CANDIDATES, 30)
    tiny_font = load_font(FONT_CANDIDATES, 15)
    line_h = 27
    body_top = 88
    band_top = H - 96

    margin = 40
    draw.text((margin, 22), title, font=head_font, fill=0)

    # Reference patches, so the three levels are always visible next to the text
    # for comparison instead of having to be remembered between refreshes.
    px, py, pw, ph = W - margin - 210, 26, 70, 34
    for i, shade in enumerate((255, 128, 0)):
        draw.rectangle([px + i * pw, py, px + (i + 1) * pw - 1, py + ph], fill=shade)
    draw.rectangle([px, py, px + 3 * pw - 1, py + ph], outline=0)

    draw.line([(margin, 74), (W - margin, 74)], fill=0, width=2)

    y = body_top
    lines = wrap(draw, body, body_font, W - 2 * margin)
    fits = (band_top - 8 - body_top) // line_h
    if len(lines) > fits:
        print(f"warning: {title} body clipped, {len(lines)} lines into {fits}", file=sys.stderr)
    for line in lines[:fits]:
        draw.text((margin, y), line, font=body_font, fill=0)
        y += line_h

    # Ghost bait: a solid black slab that moves between the two pages. Whatever
    # the single activation failed to move shows up on plain white next time.
    bx = margin if block_left else W // 2 + 10
    bw = W // 2 - margin - 10
    draw.rectangle([bx, band_top, bx + bw, band_top + 56], fill=0)
    draw.text((bx + 14, band_top + 14), f"ghost bait {title[-1]}", font=body_font, fill=255)

    # And the inverse on the other side: gray text on white, the hardest thing
    # for a coarse waveform to hold steady.
    ox = (W // 2 + 10) if block_left else margin
    draw.text((ox + 14, band_top + 2), "gray on white, 128", font=body_font, fill=128)
    draw.text((ox + 14, band_top + 32), "and small gray text at 15px", font=tiny_font, fill=128)

    draw.text((margin, H - 34), "DOWN = turn the page, UP = deep clear + redraw (reference)",
              font=tiny_font, fill=128)
    return img


def quantise(img):
    """Grayscale -> level per pixel, thresholds at the midpoints of the 3 levels."""
    px = img.load()
    levels = bytearray(W * H)
    for y in range(H):
        row = y * W
        for x in range(W):
            v = px[x, y]
            if v >= 192:
                levels[row + x] = LEVEL_WHITE
            elif v >= 64:
                levels[row + x] = LEVEL_GRAY
            else:
                levels[row + x] = LEVEL_BLACK
    return levels


def pack(levels):
    """Levels -> the two SSD1677 planes. p24 = level & 1, p26 = level >> 1."""
    p24 = bytearray(PLANE_BYTES)
    p26 = bytearray(PLANE_BYTES)
    for y in range(H):
        row = y * W
        base = y * (W // 8)
        for xb in range(W // 8):
            b24 = b26 = 0
            for bit in range(8):
                level = levels[row + xb * 8 + bit]
                b24 = (b24 << 1) | (level & 1)
                b26 = (b26 << 1) | (level >> 1)
            p24[base + xb] = b24
            p26[base + xb] = b26
    return bytes(p24), bytes(p26)


CHUNK = 2048


def upload(ser, slot, payload, timeout):
    """Push one page, one acknowledged chunk at a time.

    The device has no flow control on its CDC RX ring and drops whatever
    overflows it, so every chunk waits for the firmware's 'K' before the next
    one goes out.
    """
    print(f"\n### img {slot}", flush=True)
    ser.write(f"img {slot}\n".encode())
    ser.flush()
    # Wait for the firmware's prompt so the payload cannot race the line parser.
    deadline = time.time() + 5
    buf = b""
    while time.time() < deadline and b"chunks:" not in buf:
        buf += ser.read(256)
    sys.stdout.write(buf.decode("utf-8", "replace"))

    started = time.time()
    for offset in range(0, len(payload), CHUNK):
        ser.write(payload[offset:offset + CHUNK])
        ser.flush()
        ack = b""
        ack_deadline = time.time() + 3
        while not ack and time.time() < ack_deadline:
            ack = ser.read(1)  # port timeout is short; retry until the deadline
        if ack != b"K":
            print(f"\nlost sync at offset {offset} (got {ack!r})", file=sys.stderr)
            break
    print(f"[{len(payload)} bytes in {time.time() - started:.1f}s]", flush=True)
    drain(ser, timeout)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--preview", help="write <name>_a.png/<name>_b.png and exit")
    ap.add_argument("--clear", type=int, default=4, help="deghost cycles before A/B")
    ap.add_argument("--no-ab", action="store_true", help="upload only, do not enter A/B mode")
    args = ap.parse_args()

    pages = [render("PAGE A", PAGE_A, True), render("PAGE B", PAGE_B, False)]
    if args.preview:
        for name, img in zip("ab", pages):
            img.save(f"{args.preview}_{name}.png")
            print(f"wrote {args.preview}_{name}.png")
        return

    planes = []
    for img in pages:
        p24, p26 = pack(quantise(img))
        planes.append(p24 + p26)

    ser = open_port(args.port)
    ser.reset_input_buffer()
    # 40 MHz is the safe ceiling on these non-IOMUX pins, and SPI resets to
    # 20 MHz on every reboot.
    print("\n### hz 40", flush=True)
    ser.write(b"hz 40\n")
    drain(ser, 10)

    for slot, payload in zip("ab", planes):
        upload(ser, slot, payload, args.timeout)

    if args.no_ab:
        return
    print(f"\n### ab {args.clear}", flush=True)
    ser.write(f"ab {args.clear}\n".encode())
    drain(ser, args.timeout)
    print("\nA/B mode is live -- DOWN(GPIO3) turns the page, UP(GPIO2) is a clean-redraw reference.")
    print("Ctrl-C to stop watching (the device stays in A/B mode).\n")

    # Stay attached so each press prints its own timing. It also keeps the CDC
    # host side reading, which matters: with no reader the device's TX path
    # stalls on every printf and pollutes the numbers we are trying to measure.
    buf = b""
    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                print(raw.decode("utf-8", "replace").rstrip("\r"), flush=True)
    except KeyboardInterrupt:
        print("\n[detached]")


if __name__ == "__main__":
    main()
