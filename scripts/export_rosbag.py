#!/usr/bin/env python3
"""One-click export of a recorded navigation rosbag.

Mounts the recording volume when it is not mounted, decompresses the raw
sqlite bags into a cache directory, renders the recorded camera stream to an
Ubuntu-playable MP4, and writes a JSON manifest describing every output. No
navigation analysis is performed.

Examples:
  python3 scripts/export_rosbag.py                     # newest run on the volume
  python3 scripts/export_rosbag.py 20260925T120110     # run id prefix
  python3 scripts/export_rosbag.py /path/to/bag        # explicit bag directory
  python3 scripts/export_rosbag.py --no-video          # decompress + manifest only
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import offline_image_track_correction as correction


class ExportError(RuntimeError):
    pass


# Recording volumes, in the same preference order used by
# scripts/start_visual_navigation.sh. Labels come from the NTFS volume labels.
VOLUMES = (
    ("E", Path("/media/hjh/E")),
    ("Data", Path("/media/hjh/Data")),
)
RECORDING_SUBDIR = "rosbag_recording"
DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_FPS = 30.0
# H.264 in MP4 plays in the default Ubuntu viewer; mp4v is the fallback when
# this OpenCV build lacks the H.264 encoder.
CODECS = ("avc1", "mp4v")


def log(message: str) -> None:
    print(f"[export] {message}", flush=True)


def is_mounted(mountpoint: Path) -> bool:
    return os.path.ismount(str(mountpoint))


def volume_device(label: str) -> Path:
    return Path("/dev/disk/by-label") / label


def mount_volume(label: str, mountpoint: Path) -> bool:
    """Mount a labelled volume, prompting for sudo only if udisks refuses."""
    if is_mounted(mountpoint):
        return True
    device = volume_device(label)
    if not device.exists():
        log(f"{label}: device /dev/disk/by-label/{label} is absent, skipping")
        return False

    log(f"{label}: not mounted, mounting {device} at {mountpoint}")
    # udisks creates the mountpoint itself. Pre-creating it makes udisks fall
    # back to a numbered path such as /media/hjh/Data1, which the navigation
    # scripts do not look at, so the directory is only made for the sudo path.
    udisks = shutil.which("udisksctl")
    if udisks:
        result = subprocess.run(
            [udisks, "mount", "-b", str(device)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        if is_mounted(mountpoint):
            log(f"{label}: mounted via udisksctl")
            return True
        log(f"{label}: udisksctl failed: {result.stdout.strip()}")

    try:
        mountpoint.mkdir(parents=True, exist_ok=True)
    except PermissionError:
        subprocess.run(["sudo", "mkdir", "-p", str(mountpoint)], check=False)

    # Windows fast startup leaves NTFS volumes dirty, which makes both udisks
    # and a plain ntfs3 mount refuse. `force` mounts them without touching the
    # data, so it is tried before the repairing ntfsfix fallback.
    for options in (["-o", "force"], []):
        result = subprocess.run(
            ["sudo", "mount", "-t", "ntfs3", *options, str(device), str(mountpoint)],
            stderr=subprocess.STDOUT,
            text=True,
        )
        if is_mounted(mountpoint):
            log(f"{label}: mounted via sudo mount{' ' + ' '.join(options) if options else ''}")
            return True

    log(f"{label}: force mount failed, repairing the dirty volume with ntfsfix")
    subprocess.run(["sudo", "ntfsfix", str(device)], check=False)
    result = subprocess.run(
        ["sudo", "mount", "-t", "ntfs3", str(device), str(mountpoint)],
        stderr=subprocess.STDOUT,
        text=True,
    )
    if is_mounted(mountpoint):
        log(f"{label}: mounted via sudo mount after ntfsfix")
        return True
    log(f"{label}: mount failed (exit {result.returncode})")
    return False


def recording_roots() -> list[Path]:
    return [mountpoint / RECORDING_SUBDIR for _, mountpoint in VOLUMES]


def ensure_recording_volume() -> Path | None:
    """Return a recording root, mounting a volume only if none is ready.

    Already-mounted volumes win, so a mounted Data drive is used without first
    prompting for the password to mount an absent E drive.
    """
    roots = recording_roots()
    for mountpoint, root in zip((mount for _, mount in VOLUMES), roots):
        if is_mounted(mountpoint) and root.is_dir():
            return root
    for (label, mountpoint), root in zip(VOLUMES, roots):
        if mount_volume(label, mountpoint) and root.is_dir():
            return root
    return None


def is_run_dir(path: Path) -> bool:
    return (path / "run_manifest.json").is_file() or (path / "bag").is_dir()


def resolve_bag(target: str | None) -> tuple[Path, Path | None]:
    """Resolve a run id, bag path, or nothing into (bag_dir, run_dir)."""
    if target:
        candidate = Path(target).expanduser()
        if candidate.exists():
            if candidate.is_dir() and candidate.name == "bag":
                return candidate, candidate.parent
            if is_run_dir(candidate):
                return candidate / "bag", candidate
            return candidate, None

    root = ensure_recording_volume()
    if root is None:
        raise ExportError(
            "no recording volume is available; connect the drive or pass --bag"
        )
    runs_root = root / "navigation_runs"
    if not runs_root.is_dir():
        raise ExportError(f"no navigation_runs directory under {root}")

    if target:
        matches = sorted(
            path for path in runs_root.iterdir()
            if path.is_dir() and target in path.name
        )
        if not matches:
            raise ExportError(f"no run under {runs_root} matches '{target}'")
        run_dir = matches[-1]
        if len(matches) > 1:
            log(f"'{target}' matched {len(matches)} runs, using {run_dir.name}")
        return run_dir / "bag", run_dir

    runs = sorted(
        (path for path in runs_root.iterdir() if path.is_dir()),
        key=lambda path: path.stat().st_mtime,
    )
    if not runs:
        raise ExportError(f"no runs under {runs_root}")
    run_dir = runs[-1]
    return run_dir / "bag", run_dir


def image_to_bgr(message: Image) -> np.ndarray | None:
    data = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding == "rgb8":
        rgb = data.reshape((message.height, message.step // 3, 3))[:, : message.width]
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if message.encoding == "bgr8":
        return data.reshape((message.height, message.step // 3, 3))[
            :, : message.width
        ].copy()
    gray = correction.image_to_gray(message)
    if gray is not None:
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return None


def open_writer(path: Path, fps: float, size: tuple[int, int]):
    for codec in CODECS:
        writer = cv2.VideoWriter(
            str(path), cv2.VideoWriter_fourcc(*codec), fps, size
        )
        if writer.isOpened():
            return writer, codec
        writer.release()
    raise ExportError(f"no usable video encoder among {', '.join(CODECS)}")


def render_video(
    db_paths: list[Path],
    image_topic: str,
    output_path: Path,
    fps: float,
    stride: int,
    max_frames: int,
) -> dict:
    """Render the recorded camera stream to a playable video file."""
    writer = None
    codec = None
    frame_size = None
    decoded = 0
    written = 0
    skipped = 0
    first_stamp = None
    last_stamp = None
    try:
        for timestamp_ns, data in correction.iter_topic(db_paths, image_topic):
            if decoded % stride != 0:
                decoded += 1
                continue
            decoded += 1
            message = deserialize_message(data, Image)
            frame = image_to_bgr(message)
            if frame is None:
                skipped += 1
                continue
            if frame_size is None:
                frame_size = (frame.shape[1], frame.shape[0])
                writer, codec = open_writer(output_path, fps, frame_size)
            elif (frame.shape[1], frame.shape[0]) != frame_size:
                frame = cv2.resize(frame, frame_size)
            writer.write(frame)
            written += 1
            if first_stamp is None:
                first_stamp = int(timestamp_ns)
            last_stamp = int(timestamp_ns)
            if max_frames and written >= max_frames:
                break
    finally:
        if writer is not None:
            writer.release()

    if written == 0:
        raise ExportError(f"no decodable images on {image_topic}")

    duration = (
        (last_stamp - first_stamp) * 1.0e-9
        if first_stamp is not None and last_stamp is not None
        else 0.0
    )
    return {
        "path": str(output_path),
        "codec": codec,
        "fps": fps,
        "frames_written": written,
        "frames_skipped": skipped,
        "images_seen": decoded,
        "frame_size": list(frame_size) if frame_size else [],
        "source_duration_s": duration,
        "video_duration_s": written / fps if fps > 0 else 0.0,
        "bytes": output_path.stat().st_size if output_path.exists() else 0,
    }


def write_decompressed_metadata(
    bag_dir: Path, cache_dir: Path, db_paths: list[Path]
) -> Path | None:
    """Write metadata.yaml so the decompressed files form a readable bag.

    The recorded metadata describes the compressed bag; rewriting it drops the
    compression and points at the plain .db3 files, which lets rosbag2 (and
    scripts/analyze_navigation_bag.py) open the cache directory directly.
    """
    source = bag_dir / "metadata.yaml"
    if not source.is_file():
        return None
    try:
        metadata = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return None
    info = (metadata or {}).get("rosbag2_bagfile_information")
    if not isinstance(info, dict):
        return None

    info["compression_format"] = ""
    info["compression_mode"] = "NONE"
    info["relative_file_paths"] = [path.name for path in db_paths]
    files = info.get("files")
    if isinstance(files, list):
        names = {path.name for path in db_paths}
        info["files"] = [
            entry
            for entry in files
            if Path(str(entry.get("path", ""))).name in names
        ]

    target = cache_dir / "metadata.yaml"
    target.write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    return target


def topic_table(db_paths: list[Path]) -> dict[str, dict]:
    table: dict[str, dict] = {}
    for db_path in db_paths:
        rows = correction.topic_table([db_path])
        for name, (topic_type, count) in rows.items():
            entry = table.setdefault(name, {"type": topic_type, "messages": 0})
            entry["messages"] += count
    return table


def build_manifest(
    bag_dir: Path,
    run_dir: Path | None,
    db_paths: list[Path],
    cache_dir: Path,
    video: dict | None,
    image_topic: str,
    metadata_path: Path | None = None,
) -> dict:
    manifest: dict = {
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(
            timespec="seconds"
        ),
        "bag_dir": str(bag_dir),
        "run_dir": str(run_dir) if run_dir else None,
        "run_id": run_dir.name if run_dir else bag_dir.parent.name,
        "decompressed_dir": str(cache_dir),
        "decompressed_metadata": str(metadata_path) if metadata_path else None,
        "decompressed_files": [
            {"path": str(path), "bytes": path.stat().st_size} for path in db_paths
        ],
        "topics": topic_table(db_paths),
        "video": video,
    }
    if run_dir and (run_dir / "run_manifest.json").is_file():
        try:
            source = json.loads((run_dir / "run_manifest.json").read_text())
        except (OSError, json.JSONDecodeError):
            source = {}
        manifest["run"] = {
            "status": source.get("status"),
            "start_time": source.get("start_time"),
            "end_time": source.get("end_time"),
            "exit_code": source.get("exit_code"),
            "odom_topic": (source.get("effective_settings") or {}).get("odom_topic"),
        }
    if video is None and image_topic not in manifest["topics"]:
        manifest["video_note"] = f"{image_topic} is not present in this bag"
    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "target",
        nargs="?",
        default=None,
        help="run id prefix or bag directory (default: newest run on the volume)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="output directory (default: <recording root>/bag_exports/<run-id>)",
    )
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="write every Nth recorded image (default: 1, every frame)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="stop after this many written frames (default: 0, no limit)",
    )
    parser.add_argument(
        "--no-video", action="store_true", help="decompress and write the manifest only"
    )
    parser.add_argument(
        "--mount-only",
        action="store_true",
        help="mount the recording volume and exit",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.stride < 1:
        print("--stride must be at least 1", file=sys.stderr)
        return 2
    if args.fps <= 0.0:
        print("--fps must be positive", file=sys.stderr)
        return 2

    if args.mount_only:
        root = ensure_recording_volume()
        if root is None:
            print("no recording volume could be mounted", file=sys.stderr)
            return 3
        log(f"recording root ready: {root}")
        return 0

    try:
        bag_dir, run_dir = resolve_bag(args.target)
        if not bag_dir.is_dir():
            raise ExportError(f"bag directory does not exist: {bag_dir}")
        run_id = run_dir.name if run_dir else bag_dir.parent.name

        if args.out:
            output_dir = args.out.expanduser()
        else:
            recording_root = next(
                (
                    root for root in recording_roots()
                    if root.is_dir() and root in bag_dir.parents
                ),
                None,
            )
            base = recording_root or bag_dir.parent.parent
            output_dir = base / "bag_exports" / run_id
        output_dir.mkdir(parents=True, exist_ok=True)

        cache_dir = output_dir / "decompressed"
        log(f"bag      : {bag_dir}")
        log(f"output   : {output_dir}")
        log("decompressing raw sqlite bags (sources are never removed)")
        db_paths = correction.decompress_bag_files(bag_dir, cache_dir)
        total = sum(path.stat().st_size for path in db_paths)
        log(f"decompressed {len(db_paths)} file(s), {total / 1e9:.2f} GB")
        metadata_path = write_decompressed_metadata(bag_dir, cache_dir, db_paths)
        if metadata_path is not None:
            log(f"metadata : {metadata_path.name} (cache opens directly as a bag)")

        video = None
        if not args.no_video:
            topics = topic_table(db_paths)
            if args.image_topic in topics:
                video_path = output_dir / f"{run_id}.mp4"
                log(f"rendering {args.image_topic} -> {video_path.name}")
                video = render_video(
                    db_paths,
                    args.image_topic,
                    video_path,
                    args.fps,
                    args.stride,
                    args.max_frames,
                )
                log(
                    f"video    : {video['frames_written']} frames, "
                    f"{video['video_duration_s']:.1f} s, codec {video['codec']}"
                )
            else:
                log(f"{args.image_topic} not in bag, skipping video")

        manifest = build_manifest(
            bag_dir, run_dir, db_paths, cache_dir, video, args.image_topic,
            metadata_path,
        )
        manifest_path = output_dir / "export_manifest.json"
        manifest_path.write_text(
            json.dumps(manifest, indent=2, ensure_ascii=True) + "\n",
            encoding="utf-8",
        )
        log(f"manifest : {manifest_path}")
        print(f"\nExport complete: {output_dir}")
        return 0
    except ExportError as error:
        print(f"export failed: {error}", file=sys.stderr)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())
