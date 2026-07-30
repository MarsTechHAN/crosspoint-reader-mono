#!/usr/bin/env python3
"""Build the Paper Mono's one-time LXGW WenKai GB2312 font image."""

import argparse
import pathlib
import struct
import subprocess
import sys
import tempfile
import zlib


MAGIC = b"CPFTTF1\0"
PARTITION_SIZE = 0x400000


def gb2312_characters() -> str:
    chars = {chr(cp) for cp in range(0x20, 0x7F)}
    for lead in range(0xA1, 0xF8):
        for trail in range(0xA1, 0xFF):
            try:
                chars.add(bytes((lead, trail)).decode("gb2312"))
            except UnicodeDecodeError:
                pass

    # Keep common Unicode punctuation and extended Latin usable when a mixed
    # Chinese title is routed as one string through the fallback face.
    chars.update(chr(cp) for cp in range(0x00A0, 0x0250))
    chars.update(chr(cp) for cp in range(0x3000, 0x3040))
    chars.update(chr(cp) for cp in range(0xFF00, 0xFFF0))
    chars.update("\uFFFD\u2026\u00B7")
    return "".join(sorted(chars))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=pathlib.Path, help="LXGWWenKai-Regular.ttf")
    parser.add_argument("output", type=pathlib.Path, help="output cjkfont.bin")
    args = parser.parse_args()

    if not args.source.is_file():
        parser.error(f"font does not exist: {args.source}")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="crosspoint-cjk-") as temporary_dir:
        temporary = pathlib.Path(temporary_dir)
        characters = temporary / "gb2312.txt"
        subset = temporary / "LXGWWenKai-GB2312.ttf"
        characters.write_text(gb2312_characters(), encoding="utf-8")
        subprocess.run(
            [
                sys.executable,
                "-m",
                "fontTools.subset",
                str(args.source),
                f"--text-file={characters}",
                f"--output-file={subset}",
                "--layout-features=*",
                "--notdef-glyph",
                "--notdef-outline",
                "--recommended-glyphs",
                "--name-IDs=*",
                "--name-languages=*",
                "--drop-tables+=DSIG",
            ],
            check=True,
        )
        font_data = subset.read_bytes()

    header = struct.pack("<8sII", MAGIC, len(font_data), zlib.crc32(font_data) & 0xFFFFFFFF)
    image = header + font_data
    if len(image) > PARTITION_SIZE:
        raise SystemExit(f"font image is {len(image)} bytes; partition is {PARTITION_SIZE} bytes")
    args.output.write_bytes(image)
    print(f"{args.output}: {len(font_data)} font bytes, {len(image)} image bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
