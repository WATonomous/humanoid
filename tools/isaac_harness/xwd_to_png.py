#!/usr/bin/env python3
"""Decode an X11 xwd dump to PNG. xwd isn't a format PIL reads natively.

Assumes a 32-bit-per-pixel, LSBFirst (little-endian) TrueColor visual with
masks R=0xFF0000 G=0xFF00 B=0xFF — the common case on modern Linux desktops.
If your capture comes out with swapped/wrong colors, your X server is using a
different visual and this parsing needs adjusting (check the XWDFileHeader
fields: byte_order, bits_per_pixel, {red,green,blue}_mask).
"""
import struct
import sys

from PIL import Image


def convert(xwd_path: str, png_path: str) -> None:
    data = open(xwd_path, "rb").read()
    header_size, = struct.unpack(">I", data[:4])
    ncolors, = struct.unpack(">I", data[76:80])
    width, height = struct.unpack(">II", data[80:88])
    offset = header_size + ncolors * 12
    pixel_data = data[offset:offset + width * height * 4]
    Image.frombytes("RGB", (width, height), pixel_data, "raw", "BGRX").save(png_path)


if __name__ == "__main__":
    convert(sys.argv[1], sys.argv[2])
