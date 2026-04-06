#!/usr/bin/env python3
"""
Reverse Bitstream for Analogue Pocket.

Converts a Quartus-generated RBF bitstream into the bit-reversed format used by
Analogue Pocket openFPGA cores.
"""

import os
import sys


def reverse_bits_in_byte(byte_val: int) -> int:
    result = 0
    for i in range(8):
        if byte_val & (1 << i):
            result |= 1 << (7 - i)
    return result


def reverse_bitstream(input_file: str, output_file: str) -> bool:
    try:
        with open(input_file, "rb") as infile:
            data = infile.read()

        if not data:
            print(f"Error: input file '{input_file}' is empty", file=sys.stderr)
            return False

        reversed_data = bytearray(reverse_bits_in_byte(byte_val) for byte_val in data)

        with open(output_file, "wb") as outfile:
            outfile.write(reversed_data)

        print(f"Successfully reversed bitstream: {input_file} -> {output_file}")
        print(f"  Input size:  {len(data)} bytes")
        print(f"  Output size: {len(reversed_data)} bytes")
        return True
    except IOError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return False
    except Exception as exc:
        print(f"Unexpected error: {exc}", file=sys.stderr)
        return False


def main() -> int:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input_rbf> <output_rbf_r>", file=sys.stderr)
        return 1

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if not os.path.exists(input_file):
        print(f"Error: input file '{input_file}' not found", file=sys.stderr)
        return 1

    return 0 if reverse_bitstream(input_file, output_file) else 1


if __name__ == "__main__":
    raise SystemExit(main())
