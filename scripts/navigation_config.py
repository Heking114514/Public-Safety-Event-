#!/usr/bin/env python3
"""Validate navigation YAML and emit a non-executable tab-separated stream."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

try:
    import yaml
except ImportError as error:  # pragma: no cover - exercised by deployment setup
    raise SystemExit("PyYAML is required to read navigation configuration") from error


STARTUP_KEYS = {
    "serial_device": str,
    "serial_baud_rate": int,
    "use_serial": bool,
    "camera_serial": str,
    "use_imu": bool,
    "use_slam_imu": bool,
    "equalize": bool,
    "visualization": bool,
    "build_if_needed": bool,
    "check_camera": bool,
    "force_camera_reset": bool,
    "build_jobs": int,
    "default_autostart_route": str,
}


def load_mapping(path: Path, section: str) -> dict:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read {section} config {path}: {error}") from error
    if not isinstance(data, dict) or not isinstance(data.get(section), dict):
        raise ValueError(f"{path} must contain a '{section}' mapping")
    return data[section]


def validate_startup(path: Path) -> dict:
    values = load_mapping(path, "navigation_startup")
    unknown = sorted(set(values) - set(STARTUP_KEYS))
    missing = sorted(set(STARTUP_KEYS) - set(values))
    if unknown:
        raise ValueError(f"unknown startup keys: {', '.join(unknown)}")
    if missing:
        raise ValueError(f"missing startup keys: {', '.join(missing)}")
    for key, expected in STARTUP_KEYS.items():
        value = values[key]
        # bool is a subclass of int in Python; reject it for integer settings.
        if expected is int and (not isinstance(value, int) or isinstance(value, bool)):
            raise ValueError(f"startup key '{key}' must be an integer")
        if expected is str and not isinstance(value, str):
            raise ValueError(f"startup key '{key}' must be a string")
        if expected is bool and not isinstance(value, bool):
            raise ValueError(f"startup key '{key}' must be true or false")
    if values["serial_baud_rate"] <= 0 or values["build_jobs"] <= 0:
        raise ValueError("serial_baud_rate and build_jobs must be positive")
    for key, value in values.items():
        if isinstance(value, str) and any(character in value for character in "\t\r\n\0"):
            raise ValueError(f"startup key '{key}' contains a control character")
    return values


def validate_recording(path: Path) -> tuple[int, list[str]]:
    values = load_mapping(path, "navigation_recording")
    unknown = sorted(set(values) - {"retain_bags", "topics"})
    if unknown:
        raise ValueError(f"unknown recording keys: {', '.join(unknown)}")
    retain_bags = values.get("retain_bags")
    topics = values.get("topics")
    if not isinstance(retain_bags, int) or isinstance(retain_bags, bool) or retain_bags < 1:
        raise ValueError("retain_bags must be a positive integer")
    if not isinstance(topics, list) or not topics or any(
        not isinstance(topic, str)
        or not topic.startswith("/")
        or any(character in topic for character in "\t\r\n\0")
        for topic in topics
    ):
        raise ValueError("topics must be a non-empty list of absolute topic names")
    if len(set(topics)) != len(topics):
        raise ValueError("recording topics must be unique")
    return retain_bags, topics


def shell_value(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def emit_tsv(startup_path: Path, recording_path: Path) -> None:
    startup = validate_startup(startup_path)
    retain_bags, topics = validate_recording(recording_path)
    for key, value in startup.items():
        print(f"{key}\t{shell_value(value)}")
    print(f"retain_bags\t{retain_bags}")
    for topic in topics:
        print(f"topic\t{topic}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--startup", type=Path, required=True)
    parser.add_argument("--recording", type=Path, required=True)
    args = parser.parse_args()
    try:
        emit_tsv(args.startup, args.recording)
    except ValueError as error:
        print(f"navigation config error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
