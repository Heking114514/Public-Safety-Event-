#!/usr/bin/env python3

import datetime as dt
import json
import os
from pathlib import Path
import shutil
import tempfile
import uuid


SCHEMA_VERSION = 1
TERMINAL_STATUSES = {
    "completed",
    "interrupted",
    "failed",
    "legacy_imported",
    "abandoned",
}


class BagError(RuntimeError):
    pass


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="microseconds")


def atomic_write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(data, stream, indent=2, sort_keys=True, ensure_ascii=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def load_json(path):
    try:
        with open(path, "r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise BagError(f"cannot read manifest {path}: {error}") from error


def unique_path(parent, stem):
    candidate = Path(parent) / stem
    sequence = 0
    while candidate.exists() or candidate.is_symlink():
        sequence += 1
        candidate = Path(parent) / f"{stem}_{sequence}"
    return candidate


def atomic_symlink(target, link_path):
    target = Path(target)
    link_path = Path(link_path)
    if not target.exists():
        raise BagError(f"refusing to link to a missing target: {target}")
    if link_path.exists() and not link_path.is_symlink():
        raise BagError(f"refusing to replace non-symlink path: {link_path}")
    link_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = link_path.parent / (
        f".{link_path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp"
    )
    relative_target = os.path.relpath(target, link_path.parent)
    os.symlink(relative_target, temporary)
    try:
        os.replace(temporary, link_path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def bag_has_payload(bag_path):
    bag_path = Path(bag_path)
    if bag_path.is_symlink() or not bag_path.is_dir():
        return False
    try:
        for entry in bag_path.iterdir():
            if entry.is_symlink() or not entry.is_file():
                continue
            if entry.name == "metadata.yaml" or entry.suffix in {".db3", ".mcap"}:
                return True
    except OSError:
        return False
    return False


def bag_has_metadata(bag_path):
    metadata = Path(bag_path) / "metadata.yaml"
    return (
        bag_has_payload(bag_path)
        and not metadata.is_symlink()
        and metadata.is_file()
    )


def _run_manifests(runs_root):
    runs_root = Path(runs_root).resolve()
    if not runs_root.is_dir():
        return
    for run_dir in runs_root.iterdir():
        if run_dir.is_symlink() or not run_dir.is_dir():
            continue
        if run_dir.resolve().parent != runs_root:
            continue
        manifest_path = run_dir / "run_manifest.json"
        if manifest_path.is_symlink() or not manifest_path.is_file():
            continue
        try:
            manifest = load_json(manifest_path)
        except BagError:
            continue
        if (
            manifest.get("schema_version") != SCHEMA_VERSION
            or manifest.get("run_id") != run_dir.name
        ):
            continue
        yield run_dir, manifest_path, manifest


def recover_abandoned_runs(runs_root):
    for run_dir, manifest_path, manifest in _run_manifests(runs_root):
        if manifest.get("status") not in {"prepared", "running"}:
            continue
        bag_path = run_dir / "bag"
        manifest["status"] = "abandoned"
        manifest["end_time"] = utc_now()
        manifest["abandoned_reason"] = "superseded_by_new_run"
        manifest.setdefault("bag", {})["present"] = bag_has_payload(bag_path)
        manifest["bag"]["metadata_present"] = bag_has_metadata(bag_path)
        atomic_write_json(manifest_path, manifest)


def _run_time(manifest, run_dir):
    raw_time = (
        manifest.get("end_time")
        or manifest.get("imported_at")
        or manifest.get("start_time")
        or ""
    )
    try:
        parsed_time = dt.datetime.fromisoformat(raw_time.replace("Z", "+00:00"))
        timestamp = parsed_time.timestamp()
    except (AttributeError, TypeError, ValueError):
        timestamp = float("-inf")
    return timestamp, run_dir.name


def _candidates(runs_root):
    candidates = []
    for run_dir, manifest_path, manifest in _run_manifests(runs_root):
        if manifest.get("status") not in TERMINAL_STATUSES:
            continue
        bag_path = run_dir / "bag"
        if not bag_has_payload(bag_path):
            continue
        candidates.append(
            {
                "run_dir": run_dir,
                "bag_path": bag_path,
                "manifest_path": manifest_path,
                "manifest": manifest,
                "order": _run_time(manifest, run_dir),
            }
        )
    return candidates


def prune_bags(runs_root, latest_bag, current_run, retain_bags):
    if retain_bags < 1:
        raise BagError("bag retention must be at least 1")

    candidates = _candidates(runs_root)
    current_bag = Path(current_run).absolute() / "bag"
    latest_bag = Path(latest_bag)
    protected = {
        entry["run_dir"]
        for entry in candidates
        if entry["bag_path"].resolve() == current_bag.resolve()
    }

    keep = set(protected)
    newest = sorted(candidates, key=lambda entry: entry["order"], reverse=True)
    for entry in newest:
        if len(keep) >= retain_bags:
            break
        keep.add(entry["run_dir"])

    for entry in candidates:
        if entry["run_dir"] in keep:
            continue
        bag_path = entry["bag_path"]
        if bag_path.is_symlink() or not bag_path.is_dir():
            raise BagError(f"refusing to prune unsafe bag path: {bag_path}")
        if bag_path.parent.resolve() != entry["run_dir"].resolve():
            raise BagError(f"refusing to prune bag outside its run: {bag_path}")
        shutil.rmtree(bag_path)
        manifest = entry["manifest"]
        manifest.setdefault("bag", {})["present"] = False
        manifest["bag"]["pruned_at"] = utc_now()
        manifest["bag"]["pruned_reason"] = f"retention_limit_{retain_bags}"
        atomic_write_json(entry["manifest_path"], manifest)

    if latest_bag.is_symlink():
        try:
            latest_target = latest_bag.resolve(strict=True)
        except OSError:
            latest_bag.unlink()
        else:
            kept_bags = {
                entry["bag_path"].resolve()
                for entry in candidates
                if entry["run_dir"] in keep
            }
            if latest_target not in kept_bags:
                latest_bag.unlink()


def preserve_legacy_bag(latest_bag, runs_root):
    latest_bag = Path(latest_bag)
    if latest_bag.is_symlink() or not latest_bag.exists():
        return
    if not latest_bag.is_dir():
        raise BagError(
            f"refusing to replace non-directory compatibility path: {latest_bag}"
        )
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    legacy_run = unique_path(runs_root, f"legacy_{timestamp}")
    legacy_run.mkdir()
    destination = legacy_run / "bag"
    legacy_manifest = {
        "schema_version": SCHEMA_VERSION,
        "run_id": legacy_run.name,
        "status": "legacy_imported",
        "imported_at": utc_now(),
        "bag": {
            "path": str(destination),
            "metadata_present": bag_has_metadata(latest_bag),
            "present": bag_has_payload(latest_bag),
        },
        "legacy_source": str(latest_bag),
    }
    atomic_write_json(legacy_run / "run_manifest.json", legacy_manifest)
    temporary_link = latest_bag.parent / (
        f".{latest_bag.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp"
    )
    os.symlink(os.path.relpath(destination, latest_bag.parent), temporary_link)
    moved = False
    try:
        os.replace(latest_bag, destination)
        moved = True
        os.replace(temporary_link, latest_bag)
    except Exception:
        try:
            temporary_link.unlink()
        except FileNotFoundError:
            pass
        if moved and destination.exists() and not latest_bag.exists():
            os.replace(destination, latest_bag)
        if not destination.exists():
            try:
                (legacy_run / "run_manifest.json").unlink()
                legacy_run.rmdir()
            except OSError:
                pass
        raise
