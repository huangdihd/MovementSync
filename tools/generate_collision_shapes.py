#!/usr/bin/env python3
"""Generate the pinned Minecraft collision-shape registry for MovementSync."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile

MAGIC = b"MSCS"
FORMAT_VERSION = 1


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def expected_states(blocks_json: Path) -> dict[int, tuple[str, dict[str, str]]]:
    result: dict[int, tuple[str, dict[str, str]]] = {}
    blocks = json.loads(blocks_json.read_text(encoding="utf-8"))
    for block in blocks:
        for state_id in range(block["minStateId"], block["maxStateId"] + 1):
            offset = state_id - block["minStateId"]
            properties: dict[str, str] = {}
            for prop in reversed(block.get("states") or []):
                value_index = offset % prop["num_values"]
                offset //= prop["num_values"]
                if prop.get("type") == "bool":
                    value = "true" if value_index == 0 else "false"
                elif prop.get("values") is not None and value_index < len(prop["values"]):
                    value = str(prop["values"][value_index])
                else:
                    value = str(value_index)
                properties[prop["name"]] = value
            result[state_id] = (block["name"], properties)
    return result


def parse_properties(text: str) -> dict[str, str]:
    return dict(item.split("=", 1) for item in text.split(",") if item)


def parse_boxes(text: str) -> list[tuple[float, float, float, float, float, float]]:
    if not text:
        return []
    boxes = []
    for encoded in text.split(";"):
        values = tuple(float(value) for value in encoded.split(","))
        if len(values) != 6:
            raise ValueError(f"invalid collision box: {encoded!r}")
        boxes.append(values)
    return boxes


def run_extractor(java: Path, paper_jar: Path, libraries: Path, extractor: Path, output: Path) -> None:
    jars = [paper_jar, *sorted(libraries.rglob("*.jar"))]
    classpath = os.pathsep.join(str(path) for path in jars)
    completed = subprocess.run(
        [str(java), "--class-path", classpath, str(extractor), str(output)],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"Paper collision extractor failed ({completed.returncode}):\n{completed.stdout}")


def main() -> int:
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parent.parent
    parser.add_argument("--paper-jar", type=Path, required=True)
    parser.add_argument("--libraries", type=Path, required=True)
    parser.add_argument("--java", type=Path, required=True, help="Java 21+ launcher")
    parser.add_argument("--blocks-json", type=Path, default=root / "src/main/resources/blocks.json")
    parser.add_argument("--extractor", type=Path, default=root / "tools/ExtractCollisionShapes.java")
    parser.add_argument("--output", type=Path, default=root / "src/main/resources/collision_shapes.bin")
    parser.add_argument("--manifest", type=Path, default=root / "src/main/resources/collision_shapes.manifest.json")
    args = parser.parse_args()

    for path in (args.paper_jar, args.libraries, args.java, args.blocks_json, args.extractor):
        if not path.exists():
            parser.error(f"not found: {path}")

    expected = expected_states(args.blocks_json)
    with tempfile.TemporaryDirectory(prefix="movementsync-collision-") as temp_dir:
        tsv = Path(temp_dir) / "collision-shapes.tsv"
        run_extractor(args.java, args.paper_jar, args.libraries, args.extractor, tsv)

        with tsv.open(encoding="utf-8") as stream:
            header = stream.readline().strip()
            if not header.startswith("#registry_size="):
                raise RuntimeError(f"invalid extractor header: {header!r}")
            state_count = int(header.split("=", 1)[1])
            if state_count != len(expected):
                raise RuntimeError(f"registry-size mismatch: Paper={state_count}, blocks.json={len(expected)}")

            shapes: list[list[tuple[float, float, float, float, float, float]] | None] = [None] * state_count
            for line_number, line in enumerate(stream, start=2):
                fields = line.rstrip("\n").split("\t")
                if len(fields) != 4:
                    raise RuntimeError(f"invalid TSV line {line_number}: expected 4 fields, got {len(fields)}")
                state_id = int(fields[0])
                name = fields[1].removeprefix("minecraft:")
                properties = parse_properties(fields[2])
                if expected.get(state_id) != (name, properties):
                    raise RuntimeError(
                        f"state mapping mismatch at {state_id}: Paper={(name, properties)}, "
                        f"blocks.json={expected.get(state_id)}"
                    )
                boxes = parse_boxes(fields[3])
                if len(boxes) > 255:
                    raise RuntimeError(f"state {state_id} has too many boxes: {len(boxes)}")
                shapes[state_id] = boxes

    missing = [index for index, boxes in enumerate(shapes) if boxes is None]
    if missing:
        raise RuntimeError(f"extractor omitted state IDs: {missing[:20]}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as stream:
        stream.write(struct.pack(">4sII", MAGIC, FORMAT_VERSION, state_count))
        for boxes in shapes:
            assert boxes is not None
            stream.write(struct.pack(">B", len(boxes)))
            for box in boxes:
                stream.write(struct.pack(">6f", *box))

    total_boxes = sum(len(boxes or ()) for boxes in shapes)
    manifest = {
        "format": "MovementSync collision shapes",
        "format_version": FORMAT_VERSION,
        "minecraft_version": "1.21.11",
        "collision_context": "CollisionContext.empty()",
        "scope": "all state-derived static collision shapes",
        "known_context_dependent_limitations": [
            "powder_snow: entity equipment and approach context",
            "scaffolding: entity descent/holding context",
            "moving_piston: block-entity movement state"
        ],
        "extraction_source": "Paper Block.stateById(id).getCollisionShape()",
        "paper_jar": args.paper_jar.name,
        "paper_jar_sha256": sha256(args.paper_jar),
        "blocks_json_sha256": sha256(args.blocks_json),
        "generator_sha256": sha256(Path(__file__).resolve()),
        "extractor_sha256": sha256(args.extractor),
        "state_count": state_count,
        "collision_box_count": total_boxes,
        "mapping_mismatches": 0,
        "shape_errors": 0,
        "output_file": args.output.name,
        "output_sha256": sha256(args.output),
    }
    args.manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
