#!/usr/bin/env python3
"""Create left_*.stl meshes by mirroring the current STL files."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from stl import mesh


AXES = {"x": 0, "y": 1, "z": 2}


def mirror_stl(input_path: Path, output_path: Path, axis: str) -> None:
    stl_mesh = mesh.Mesh.from_file(str(input_path))
    axis_index = AXES[axis]

    stl_mesh.vectors[:, :, axis_index] *= -1
    stl_mesh.vectors[:, [1, 2]] = stl_mesh.vectors[:, [2, 1]]
    stl_mesh.normals[:] = np.cross(
        stl_mesh.vectors[:, 1] - stl_mesh.vectors[:, 0],
        stl_mesh.vectors[:, 2] - stl_mesh.vectors[:, 0],
    )

    stl_mesh.save(str(output_path))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mirror STL files and save them with a left_ prefix.")
    parser.add_argument("--axis", choices=AXES, default="y", help="Axis to flip. Default: y")
    parser.add_argument("--prefix", default="left_", help="Output filename prefix. Default: left_")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing output files.")
    parser.add_argument("--dry-run", action="store_true", help="Show files that would be created.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mesh_dir = Path(__file__).resolve().parent
    stl_files = sorted(
        path
        for path in mesh_dir.glob("*.stl")
        if not path.name.startswith(args.prefix)
    )

    for input_path in stl_files:
        output_path = input_path.with_name(f"{args.prefix}{input_path.name}")
        if output_path.exists() and not args.overwrite:
            print(f"skip existing: {output_path.name}")
            continue

        print(f"{input_path.name} -> {output_path.name} (flip {args.axis})")
        if not args.dry_run:
            mirror_stl(input_path, output_path, args.axis)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
