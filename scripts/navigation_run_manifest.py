#!/usr/bin/env python3

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

from navigation_bags import (
    BagError,
    atomic_symlink,
    atomic_write_json,
    bag_has_metadata,
    bag_has_payload,
    load_json,
    preserve_legacy_bag,
    prune_bags,
    recover_abandoned_runs,
    unique_path,
    utc_now,
)


SCHEMA_VERSION = 1
SOURCE_ROOTS = {"config", "interfaces", "launch", "scripts", "src"}
SOURCE_SUFFIXES = {
    ".action", ".bash", ".c", ".cc", ".cmake", ".cpp", ".csv", ".cxx",
    ".h", ".hh", ".hpp", ".json", ".launch", ".msg", ".py", ".sh",
    ".srv", ".xml", ".yaml", ".yml",
}
EXCLUDED_COMPONENTS = {
    "__pycache__", "analysis_bags", "build", "install", "log",
    "navigation_runs",
}
EXCLUDED_PREFIXES = ("latest_navigation_",)
EXCLUDED_SUFFIXES = (".db3", ".mcap", ".pyc")


class ManifestError(RuntimeError):
    pass


def run_git(root, *args, check=True):
    result = subprocess.run(
        ["git", "-C", str(root), *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if check and result.returncode != 0:
        error = result.stderr.decode("utf-8", "replace").strip()
        raise ManifestError(f"git {' '.join(args)} failed: {error}")
    return result


def git_text(root, *args, check=True):
    return run_git(root, *args, check=check).stdout.decode("utf-8", "replace").strip()


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def feed_digest(digest, label, value):
    label_bytes = label.encode("utf-8", "surrogateescape")
    value_bytes = value if isinstance(value, bytes) else str(value).encode(
        "utf-8", "surrogateescape"
    )
    digest.update(len(label_bytes).to_bytes(8, "big"))
    digest.update(label_bytes)
    digest.update(len(value_bytes).to_bytes(8, "big"))
    digest.update(value_bytes)


def is_runtime_untracked(relative_path, submodule=False):
    path = Path(relative_path)
    if not path.parts:
        return False
    if any(part in EXCLUDED_COMPONENTS for part in path.parts):
        return False
    if any(part.startswith(EXCLUDED_PREFIXES) for part in path.parts):
        return False
    if str(path).endswith(EXCLUDED_SUFFIXES):
        return False
    if submodule:
        return path.name in {"CMakeLists.txt", "package.xml"} or (
            path.suffix.lower() in SOURCE_SUFFIXES
        )
    return path.parts[0] in SOURCE_ROOTS or path.name in {
        "CMakeLists.txt", "package.xml", "route_editor.py",
    }


def untracked_runtime_files(root, submodule=False):
    output = run_git(root, "ls-files", "--others", "--exclude-standard", "-z").stdout
    files = []
    for raw_path in output.split(b"\0"):
        if not raw_path:
            continue
        relative = raw_path.decode("utf-8", "surrogateescape")
        if is_runtime_untracked(relative, submodule=submodule):
            files.append(relative)
    return sorted(files)


def configured_submodules(root):
    gitmodules = Path(root) / ".gitmodules"
    if not gitmodules.is_file():
        return []
    result = run_git(
        root, "config", "-z", "-f", ".gitmodules", "--get-regexp",
        r"^submodule\..*\.path$", check=False,
    )
    if result.returncode not in (0, 1):
        raise ManifestError("could not read .gitmodules")
    entries = []
    fields = [entry for entry in result.stdout.split(b"\0") if entry]
    for field in fields:
        decoded = field.decode("utf-8", "surrogateescape")
        _, separator, path = decoded.partition("\n")
        if not separator:
            parts = decoded.split(None, 1)
            if len(parts) != 2:
                continue
            path = parts[1]
        entries.append(path)
    return sorted(entries)


def repository_payload(root, submodule=False):
    root = Path(root).resolve()
    commit = git_text(root, "rev-parse", "HEAD")
    staged = run_git(
        root, "diff", "--cached", "--binary", "--no-ext-diff",
        "--ignore-submodules=all", "--",
    ).stdout
    unstaged = run_git(
        root, "diff", "--binary", "--no-ext-diff", "--ignore-submodules=all", "--",
    ).stdout
    repository_status = run_git(
        root, "status", "--porcelain=v1", "--untracked-files=all",
    ).stdout
    untracked = []
    for relative in untracked_runtime_files(root, submodule=submodule):
        path = root / relative
        if path.is_symlink():
            content_hash = hashlib.sha256(os.readlink(path).encode()).hexdigest()
            kind = "symlink"
        elif path.is_file():
            content_hash = sha256_file(path)
            kind = "file"
        else:
            continue
        untracked.append({"path": relative, "kind": kind, "sha256": content_hash})

    submodules = []
    for relative in configured_submodules(root):
        path = root / relative
        if not (path / ".git").exists() and not path.is_dir():
            submodules.append({"path": relative, "initialized": False})
            continue
        try:
            child = repository_payload(path, submodule=True)
        except ManifestError:
            submodules.append({"path": relative, "initialized": False})
            continue
        child["path"] = relative
        child["initialized"] = True
        submodules.append(child)

    digest = hashlib.sha256()
    feed_digest(digest, "schema", "runtime-source-v1")
    feed_digest(digest, "commit", commit)
    feed_digest(digest, "staged", staged)
    feed_digest(digest, "unstaged", unstaged)
    for entry in untracked:
        feed_digest(digest, "untracked-path", entry["path"])
        feed_digest(digest, "untracked-kind", entry["kind"])
        feed_digest(digest, "untracked-sha256", entry["sha256"])
    for child in submodules:
        feed_digest(digest, "submodule-path", child["path"])
        feed_digest(digest, "submodule-initialized", child["initialized"])
        if child.get("initialized"):
            feed_digest(digest, "submodule-fingerprint", child["fingerprint"])

    return {
        "commit": commit,
        "fingerprint": digest.hexdigest(),
        "tracked_staged_diff_sha256": hashlib.sha256(staged).hexdigest(),
        "tracked_unstaged_diff_sha256": hashlib.sha256(unstaged).hexdigest(),
        "dirty": bool(repository_status),
        "repository_status_sha256": hashlib.sha256(repository_status).hexdigest(),
        "runtime_untracked_files": untracked,
        "submodules": submodules,
    }


def repository_info(workspace):
    root = Path(workspace).resolve()
    payload = repository_payload(root)
    branch_result = run_git(root, "symbolic-ref", "--short", "-q", "HEAD", check=False)
    branch = branch_result.stdout.decode("utf-8", "replace").strip() or None
    payload.update({
        "root": str(root),
        "branch": branch,
        "short_commit": payload["commit"][:12],
    })
    return payload


def parse_named_paths(values):
    parsed = []
    names = set()
    for value in values:
        name, separator, raw_path = value.partition("=")
        if not separator or not name or not raw_path:
            raise ManifestError(f"expected NAME=PATH, got: {value}")
        if name in names:
            raise ManifestError(f"duplicate name: {name}")
        names.add(name)
        parsed.append((name, Path(raw_path).absolute()))
    return parsed


def artifact_record(path):
    if not path.exists() or not path.is_file():
        raise ManifestError(f"required runtime artifact is missing: {path}")
    resolved = path.resolve(strict=True)
    return {
        "runtime_path": str(path),
        "resolved_path": str(resolved),
        "sha256": sha256_file(resolved),
        "executable": os.access(path, os.X_OK),
        "size": resolved.stat().st_size,
    }


def build_manifest_id(data):
    payload = dict(data)
    payload.pop("id", None)
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()


def command_fingerprint(args):
    print(repository_info(args.workspace)["fingerprint"])


def command_source_info(args):
    data = repository_info(args.workspace)
    if args.output:
        atomic_write_json(args.output, data)
    else:
        json.dump(data, sys.stdout, indent=2, sort_keys=True)
        sys.stdout.write("\n")


def command_record_build(args):
    source = repository_info(args.workspace)
    if args.expected_fingerprint and source["fingerprint"] != args.expected_fingerprint:
        raise ManifestError("runtime source changed during the build; rebuild from a stable tree")
    artifacts = {
        name: artifact_record(path) for name, path in parse_named_paths(args.artifact)
    }
    data = {
        "schema_version": SCHEMA_VERSION,
        "created_at": utc_now(),
        "source": source,
        "build_settings": dict(setting.split("=", 1) for setting in args.setting),
        "artifacts": artifacts,
    }
    data["id"] = build_manifest_id(data)
    atomic_write_json(args.output, data)
    print(data["id"])


def command_verify_build(args):
    manifest = load_json(args.manifest)
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ManifestError("build manifest schema is missing or unsupported; rebuild required")
    source = repository_info(args.workspace)
    recorded_source = manifest.get("source", {})
    if source["fingerprint"] != recorded_source.get("fingerprint"):
        raise ManifestError("runtime source fingerprint differs from the successful build; rebuild required")
    recorded_artifacts = manifest.get("artifacts", {})
    for name, path in parse_named_paths(args.artifact):
        recorded = recorded_artifacts.get(name)
        if not recorded:
            raise ManifestError(f"build manifest lacks required artifact '{name}'; rebuild required")
        current = artifact_record(path)
        if current["runtime_path"] != recorded.get("runtime_path"):
            raise ManifestError(f"runtime path for '{name}' changed; rebuild required")
        if current["resolved_path"] != recorded.get("resolved_path"):
            raise ManifestError(f"resolved path for '{name}' changed; rebuild required")
        if current["sha256"] != recorded.get("sha256"):
            raise ManifestError(f"runtime artifact '{name}' hash differs; rebuild required")
    identifier = manifest.get("id")
    if not identifier or identifier != build_manifest_id(manifest):
        raise ManifestError("build manifest identifier is invalid; rebuild required")
    print(identifier)


def parse_settings(values):
    settings = {}
    for setting in values:
        key, separator, value = setting.partition("=")
        if not separator or not key:
            raise ManifestError(f"expected KEY=VALUE, got: {setting}")
        if value == "true":
            parsed = True
        elif value == "false":
            parsed = False
        else:
            parsed = value
        settings[key] = parsed
    return settings


def command_create_run(args):
    workspace = Path(args.workspace).resolve()
    runs_root = Path(args.runs_root).absolute()
    latest_bag = Path(args.latest_bag).absolute()
    latest_run = Path(args.latest_run).absolute()
    source = repository_info(workspace)
    if source["fingerprint"] != args.source_fingerprint:
        raise ManifestError("runtime source changed after build verification; aborting launch")
    build = load_json(args.build_manifest)
    if build.get("source", {}).get("fingerprint") != source["fingerprint"]:
        raise ManifestError("build manifest does not match the runtime source; aborting launch")
    if build.get("id") != args.build_id or build.get("id") != build_manifest_id(build):
        raise ManifestError("build manifest identifier does not match the verified build")

    runs_root.mkdir(parents=True, exist_ok=True)
    preserve_legacy_bag(latest_bag, runs_root)
    recover_abandoned_runs(runs_root)
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    run_dir = unique_path(runs_root, f"{timestamp}_{source['short_commit']}")
    run_dir.mkdir()
    parameters_dir = run_dir / "parameters"
    parameters_dir.mkdir()
    bag_path = run_dir / "bag"
    if bag_path.exists() or bag_path.is_symlink():
        raise ManifestError(f"bag output path must not exist before ros2 bag record: {bag_path}")

    inputs = {}
    for name, path in parse_named_paths(args.input_file):
        if not path.is_file():
            raise ManifestError(f"runtime input is missing: {path}")
        inputs[name] = {
            "path": str(path),
            "resolved_path": str(path.resolve(strict=True)),
            "sha256": sha256_file(path.resolve(strict=True)),
        }

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_dir.name,
        "start_time": utc_now(),
        "end_time": None,
        "status": "prepared",
        "exit_code": None,
        "repository": source,
        "raw_argv": list(args.argv),
        "effective_settings": parse_settings(args.setting),
        "build_manifest": {
            "path": str(Path(args.build_manifest).absolute()),
            "id": args.build_id,
        },
        "runtime_artifacts": build.get("artifacts", {}),
        "runtime_inputs": inputs,
        "bag": {
            "path": str(bag_path),
            "topics": list(args.topic),
            "metadata_present": False,
            "latest_link_updated": False,
            "present": False,
        },
        "parameters": {},
        "parameter_snapshots_complete": None,
    }
    manifest_path = run_dir / "run_manifest.json"
    atomic_write_json(manifest_path, manifest)
    atomic_symlink(run_dir, latest_run)
    print(run_dir)


def command_record_parameter(args):
    manifest_path = Path(args.run_dir) / "run_manifest.json"
    manifest = load_json(manifest_path)
    entry = {
        "status": args.status,
        "captured_at": utc_now(),
        "path": None,
        "sha256": None,
        "error": args.error or None,
    }
    if args.status == "success":
        if not args.snapshot:
            raise ManifestError("a successful parameter snapshot requires --snapshot")
        snapshot = Path(args.snapshot).absolute()
        if not snapshot.is_file() or snapshot.stat().st_size == 0:
            raise ManifestError(f"parameter snapshot is missing or empty: {snapshot}")
        entry["path"] = str(snapshot)
        entry["sha256"] = sha256_file(snapshot)
    manifest.setdefault("parameters", {})[args.node] = entry
    atomic_write_json(manifest_path, manifest)


def command_mark_running(args):
    manifest_path = Path(args.run_dir) / "run_manifest.json"
    manifest = load_json(manifest_path)
    if manifest.get("status") != "prepared":
        raise ManifestError("run can only enter running state from prepared state")
    manifest["status"] = "running"
    manifest["running_time"] = utc_now()
    atomic_write_json(manifest_path, manifest)


def command_finalize_run(args):
    run_dir = Path(args.run_dir).absolute()
    manifest_path = run_dir / "run_manifest.json"
    manifest = load_json(manifest_path)
    bag_path = Path(manifest["bag"]["path"])
    metadata_present = bag_has_metadata(bag_path)
    parameter_complete = args.parameter_snapshots_complete == "true"
    exit_code = int(args.exit_code)
    if exit_code in (130, 143):
        status = "interrupted"
    elif exit_code != 0 or not parameter_complete or not metadata_present:
        status = "failed"
    else:
        status = "completed"
    manifest.update({
        "end_time": utc_now(),
        "status": status,
        "exit_code": exit_code,
        "parameter_snapshots_complete": parameter_complete,
    })
    manifest["bag"]["metadata_present"] = metadata_present
    manifest["bag"]["present"] = bag_has_payload(bag_path)
    atomic_write_json(manifest_path, manifest)

    if metadata_present and status in {"completed", "interrupted"}:
        atomic_symlink(bag_path, Path(args.latest_bag).absolute())
        manifest = load_json(manifest_path)
        manifest["bag"]["latest_link_updated"] = True
        atomic_write_json(manifest_path, manifest)

    prune_bags(
        run_dir.parent, Path(args.latest_bag).absolute(), run_dir, args.retain_bags
    )


def build_parser():
    parser = argparse.ArgumentParser(description="Navigation build and run manifest helper")
    subparsers = parser.add_subparsers(dest="command", required=True)

    fingerprint = subparsers.add_parser("fingerprint")
    fingerprint.add_argument("--workspace", required=True)
    fingerprint.set_defaults(handler=command_fingerprint)

    source_info = subparsers.add_parser("source-info")
    source_info.add_argument("--workspace", required=True)
    source_info.add_argument("--output")
    source_info.set_defaults(handler=command_source_info)

    record_build = subparsers.add_parser("record-build")
    record_build.add_argument("--workspace", required=True)
    record_build.add_argument("--output", required=True)
    record_build.add_argument("--expected-fingerprint")
    record_build.add_argument("--artifact", action="append", default=[])
    record_build.add_argument("--setting", action="append", default=[])
    record_build.set_defaults(handler=command_record_build)

    verify_build = subparsers.add_parser("verify-build")
    verify_build.add_argument("--workspace", required=True)
    verify_build.add_argument("--manifest", required=True)
    verify_build.add_argument("--artifact", action="append", default=[])
    verify_build.set_defaults(handler=command_verify_build)

    create_run = subparsers.add_parser("create-run")
    create_run.add_argument("--workspace", required=True)
    create_run.add_argument("--runs-root", required=True)
    create_run.add_argument("--latest-bag", required=True)
    create_run.add_argument("--latest-run", required=True)
    create_run.add_argument("--source-fingerprint", required=True)
    create_run.add_argument("--build-manifest", required=True)
    create_run.add_argument("--build-id", required=True)
    create_run.add_argument("--argv", action="append", default=[])
    create_run.add_argument("--setting", action="append", default=[])
    create_run.add_argument("--topic", action="append", default=[])
    create_run.add_argument("--input-file", action="append", default=[])
    create_run.set_defaults(handler=command_create_run)

    record_parameter = subparsers.add_parser("record-parameter")
    record_parameter.add_argument("--run-dir", required=True)
    record_parameter.add_argument("--node", required=True)
    record_parameter.add_argument(
        "--status", choices=("success", "failed", "skipped"), required=True
    )
    record_parameter.add_argument("--snapshot")
    record_parameter.add_argument("--error")
    record_parameter.set_defaults(handler=command_record_parameter)

    mark_running = subparsers.add_parser("mark-running")
    mark_running.add_argument("--run-dir", required=True)
    mark_running.set_defaults(handler=command_mark_running)

    finalize = subparsers.add_parser("finalize-run")
    finalize.add_argument("--run-dir", required=True)
    finalize.add_argument("--latest-bag", required=True)
    finalize.add_argument("--exit-code", required=True, type=int)
    finalize.add_argument(
        "--parameter-snapshots-complete", choices=("true", "false"), required=True
    )
    finalize.add_argument("--retain-bags", type=int, default=3)
    finalize.set_defaults(handler=command_finalize_run)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        args.handler(args)
    except (BagError, ManifestError, OSError, ValueError) as error:
        print(f"navigation manifest error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
