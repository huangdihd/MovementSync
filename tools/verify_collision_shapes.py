#!/usr/bin/env python3
"""Verify the committed MovementSync collision-shape artifact and manifest."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "src/main/resources/collision_shapes.manifest.json"
OUTPUT = ROOT / "src/main/resources/collision_shapes.bin"
BLOCKS = ROOT / "src/main/resources/blocks.json"
GENERATOR = ROOT / "tools/generate_collision_shapes.py"
EXTRACTOR = ROOT / "tools/ExtractCollisionShapes.java"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_equal(label: str, actual: object, expected: object) -> None:
    if actual != expected:
        raise RuntimeError(f"{label} mismatch: actual={actual!r}, expected={expected!r}")


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    require_equal("blocks_json_sha256", sha256(BLOCKS), manifest["blocks_json_sha256"])
    require_equal("generator_sha256", sha256(GENERATOR), manifest["generator_sha256"])
    require_equal("extractor_sha256", sha256(EXTRACTOR), manifest["extractor_sha256"])
    require_equal("output_sha256", sha256(OUTPUT), manifest["output_sha256"])
    paper_jar = manifest["paper_jar"]
    if not isinstance(paper_jar, str) or not paper_jar or "/" in paper_jar or "\\" in paper_jar:
        raise RuntimeError(f"paper_jar portability mismatch: expected basename, got {paper_jar!r}")

    data = memoryview(OUTPUT.read_bytes())
    if len(data) < 12:
        raise RuntimeError("collision_shapes.bin is truncated")
    magic, version, state_count = struct.unpack_from(">4sII", data, 0)
    require_equal("magic", magic, b"MSCS")
    require_equal("format_version", version, manifest["format_version"])
    require_equal("state_count", state_count, manifest["state_count"])
    offset = 12
    box_count = 0
    for state_id in range(state_count):
        if offset >= len(data):
            raise RuntimeError(f"collision_shapes.bin truncated at state {state_id}")
        count = data[offset]
        offset += 1
        box_count += count
        offset += count * 24
        if offset > len(data):
            raise RuntimeError(f"collision_shapes.bin truncated in state {state_id} boxes")
    require_equal("collision_box_count", box_count, manifest["collision_box_count"])
    require_equal("binary length", offset, len(data))
    print(
        json.dumps(
            {
                "verified": True,
                "state_count": state_count,
                "collision_box_count": box_count,
                "output_sha256": manifest["output_sha256"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
