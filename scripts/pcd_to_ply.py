#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""Convert a binary PCD point cloud to binary little-endian PLY.

This helper is intentionally minimal and optimized for the binary PCD files in
this workspace, which may contain fields such as:

  FIELDS x y z rgba
  SIZE   4 4 4 4
  TYPE   F F F U
  COUNT  1 1 1 1

It processes the input in chunks so large maps can be converted without
materializing the full cloud in memory.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List

import numpy as np

try:
    import argcomplete
except Exception:
    argcomplete = None


@dataclass
class PCDHeader:
    fields: List[str]
    sizes: List[int]
    types: List[str]
    counts: List[int]
    points: int
    data: str
    data_offset: int


def _parse_header(path: Path) -> PCDHeader:
    fields: List[str] | None = None
    sizes: List[int] | None = None
    types: List[str] | None = None
    counts: List[int] | None = None
    points: int | None = None
    data: str | None = None

    with path.open("rb") as fh:
        while True:
            line = fh.readline()
            if not line:
                raise ValueError(f"{path} ended before DATA header")
            decoded = line.decode("ascii", errors="strict").strip()
            if not decoded or decoded.startswith("#"):
                continue

            parts = decoded.split()
            key = parts[0].upper()
            vals = parts[1:]
            if key == "FIELDS":
                fields = vals
            elif key == "SIZE":
                sizes = [int(v) for v in vals]
            elif key == "TYPE":
                types = vals
            elif key == "COUNT":
                counts = [int(v) for v in vals]
            elif key == "POINTS":
                points = int(vals[0])
            elif key == "DATA":
                data = vals[0].lower()
                offset = fh.tell()
                break

    if fields is None or sizes is None or types is None or points is None or data is None:
        raise ValueError(f"{path} is missing required PCD header fields")
    if counts is None:
        counts = [1] * len(fields)

    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f"{path} has inconsistent FIELDS/SIZE/TYPE/COUNT lengths")

    return PCDHeader(
        fields=fields,
        sizes=sizes,
        types=types,
        counts=counts,
        points=points,
        data=data,
        data_offset=offset,
    )


def _dtype_for_field(field: str, size: int, typ: str, count: int) -> tuple:
    if count != 1:
        raise ValueError(f"Unsupported COUNT {count} for field '{field}'")

    type_map = {
        ("F", 4): "<f4",
        ("F", 8): "<f8",
        ("U", 1): "u1",
        ("U", 2): "<u2",
        ("U", 4): "<u4",
        ("I", 1): "i1",
        ("I", 2): "<i2",
        ("I", 4): "<i4",
    }
    np_type = type_map.get((typ.upper(), size))
    if np_type is None:
        raise ValueError(f"Unsupported field type for '{field}': TYPE={typ} SIZE={size}")
    return (field, np_type)


def _build_input_dtype(header: PCDHeader) -> np.dtype:
    return np.dtype(
        [_dtype_for_field(f, s, t, c) for f, s, t, c in zip(header.fields, header.sizes, header.types, header.counts)]
    )


def _build_output_dtype(include_color: bool) -> np.dtype:
    fields = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
    if include_color:
        fields.extend([("red", "u1"), ("green", "u1"), ("blue", "u1"), ("alpha", "u1")])
    return np.dtype(fields)


def _write_ply_header(out_path: Path, points: int, include_color: bool) -> None:
    lines = [
        "ply",
        "format binary_little_endian 1.0",
        f"element vertex {points}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if include_color:
        lines.extend(
            [
                "property uchar red",
                "property uchar green",
                "property uchar blue",
                "property uchar alpha",
            ]
        )
    lines.append("end_header")
    out_path.write_bytes(("\n".join(lines) + "\n").encode("ascii"))


def convert_pcd_to_ply(src: Path, dst: Path, chunk_points: int = 1_000_000) -> None:
    header = _parse_header(src)
    if header.data != "binary":
        raise ValueError(f"{src} uses DATA {header.data!r}; only binary PCD is supported")

    input_dtype = _build_input_dtype(header)
    required_fields = {"x", "y", "z"}
    if not required_fields.issubset(header.fields):
        raise ValueError(f"{src} is missing one of the required fields: {sorted(required_fields)}")

    include_color = "rgba" in header.fields or "rgb" in header.fields
    color_field = "rgba" if "rgba" in header.fields else ("rgb" if "rgb" in header.fields else None)
    output_dtype = _build_output_dtype(include_color=include_color)

    _write_ply_header(dst, header.points, include_color=include_color)

    with src.open("rb") as fin, dst.open("ab") as fout:
        fin.seek(header.data_offset)
        remaining = header.points
        while remaining > 0:
            n = min(chunk_points, remaining)
            chunk = np.fromfile(fin, dtype=input_dtype, count=n)
            if chunk.size != n:
                raise ValueError(f"{src} ended unexpectedly while reading point data")

            out = np.empty(n, dtype=output_dtype)
            out["x"] = chunk["x"].astype(np.float32, copy=False)
            out["y"] = chunk["y"].astype(np.float32, copy=False)
            out["z"] = chunk["z"].astype(np.float32, copy=False)

            if include_color and color_field is not None:
                packed = chunk[color_field].astype(np.uint32, copy=False)
                out["red"] = (packed & 0xFF).astype(np.uint8, copy=False)
                out["green"] = ((packed >> 8) & 0xFF).astype(np.uint8, copy=False)
                out["blue"] = ((packed >> 16) & 0xFF).astype(np.uint8, copy=False)
                out["alpha"] = ((packed >> 24) & 0xFF).astype(np.uint8, copy=False)

            fout.write(out.tobytes())
            remaining -= n


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Convert a binary PCD file to binary little-endian PLY.")
    parser.add_argument("src", type=Path, help="Input .pcd path")
    parser.add_argument("dst", type=Path, help="Output .ply path")
    parser.add_argument(
        "--chunk-points",
        type=int,
        default=1_000_000,
        help="Number of points to process per chunk (default: 1000000)",
    )
    if argcomplete:
        argcomplete.autocomplete(parser)
    args = parser.parse_args(argv)

    convert_pcd_to_ply(args.src, args.dst, chunk_points=args.chunk_points)
    print(f"Converted {args.src} -> {args.dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
