#!/usr/bin/env python3
"""只导出导航 rosbag 的相机视频，不依赖 vision_correction。

录制包是 zstd 压缩的，且本机 rosbag2 没有解压插件，所以这里自己解压、
直读 sqlite，把相机话题渲染成 mp4。默认按录制帧率播放（大概 14 Hz），
也可以用 --fps 强制指定，便于按真实节奏回看导航过程。

示例：
  python3 scripts/export_camera_video.py 20261015T115107 --out ~/rosbag_videos
  python3 scripts/export_camera_video.py 20261015T115107 20261015T115005
  python3 scripts/export_camera_video.py /media/hjh/E/.../20261015T115107.../bag
"""

from __future__ import annotations

import argparse
import sqlite3
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

VOLUMES = (Path("/media/hjh/E"), Path("/media/hjh/Data"))
RECORDING_SUBDIR = "rosbag_recording"
DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw"
VIDEO_CODECS = ("avc1", "mp4v")


def log(message: str) -> None:
    print(f"[video] {message}", flush=True)


def recording_roots() -> list[Path]:
    return [
        volume / RECORDING_SUBDIR for volume in VOLUMES
        if (volume / RECORDING_SUBDIR).is_dir()
    ]


def resolve_run(target: str) -> Path:
    """把 run_id 前缀、run 目录或 bag 目录统一解析成 run 目录。"""
    candidate = Path(target).expanduser()
    if candidate.exists():
        if candidate.name == "bag":
            return candidate.parent
        if (candidate / "bag").is_dir() or (candidate / "run_manifest.json").is_file():
            return candidate
        raise FileNotFoundError(f"{candidate} 不是 run 目录或 bag 目录")

    matches: list[Path] = []
    for root in recording_roots():
        runs_root = root / "navigation_runs"
        if not runs_root.is_dir():
            continue
        matches += sorted(
            path for path in runs_root.iterdir()
            if path.is_dir() and target in path.name
        )
    if not matches:
        raise FileNotFoundError(f"没有 run_id 匹配 '{target}'")
    if len(matches) > 1:
        log(f"'{target}' 匹配 {len(matches)} 个 run，取 {matches[-1].name}")
    return matches[-1]


def decompress_bag(bag_dir: Path, cache_dir: Path) -> list[Path]:
    """把 *.db3.zstd 解压到缓存目录，原始压缩包不动，已解压的跳过。"""
    plain = sorted(bag_dir.glob("*.db3"))
    if plain:
        return plain
    compressed = sorted(bag_dir.glob("*.db3.zstd"))
    if not compressed:
        raise FileNotFoundError(f"{bag_dir} 里没有 .db3 或 .db3.zstd")

    cache_dir.mkdir(parents=True, exist_ok=True)
    outputs: list[Path] = []
    for source in compressed:
        target = cache_dir / source.name.removesuffix(".zstd")
        if not target.exists() or target.stat().st_size == 0:
            log(f"解压 {source.name} -> {target}")
            # Ctrl-C 中断的录制包没有 zstd 收尾标记，zstd 会以 premature end
            # 报错退出，但帧数据已经写全，所以按字节流取回并忽略收尾报错。
            with target.open("wb") as stream:
                result = subprocess.run(
                    ["zstd", "-d", "-c", str(source)],
                    stdout=stream,
                    stderr=subprocess.DEVNULL,
                )
            size = target.stat().st_size
            if size == 0:
                raise RuntimeError(f"{source.name} 解压为空")
            if result.returncode != 0:
                log(f"  {source.name} 尾部无收尾标记（中断录制），取回 {size / 1e6:.0f} MB")
        outputs.append(target)
    return outputs


def topic_id(db_path: Path, topic: str) -> int | None:
    with sqlite3.connect(db_path) as conn:
        row = conn.execute("select id from topics where name = ?", (topic,)).fetchone()
    return int(row[0]) if row else None


def iter_frames(db_paths: list[Path], topic: str, timestamps_only: bool):
    for db_path in db_paths:
        db_topic_id = topic_id(db_path, topic)
        if db_topic_id is None:
            continue
        columns = "timestamp" if timestamps_only else "timestamp, data"
        with sqlite3.connect(db_path) as conn:
            cursor = conn.execute(
                f"select {columns} from messages where topic_id = ? order by timestamp",
                (db_topic_id,),
            )
            for row in cursor:
                yield int(row[0]), (None if timestamps_only else row[1])


def frame_to_bgr(message: Image) -> np.ndarray | None:
    encoding = message.encoding.lower()
    data = np.frombuffer(message.data, dtype=np.uint8)
    if encoding in ("rgb8", "bgr8", "rgba8", "bgra8"):
        channels = 4 if encoding in ("rgba8", "bgra8") else 3
        image = data.reshape((message.height, message.step // channels, channels))
        image = image[:, : message.width]
        code = {
            "rgb8": cv2.COLOR_RGB2BGR,
            "bgr8": None,
            "rgba8": cv2.COLOR_RGBA2BGR,
            "bgra8": cv2.COLOR_BGRA2BGR,
        }[encoding]
        return image.copy() if code is None else cv2.cvtColor(image, code)
    if encoding == "mono8":
        gray = data.reshape((message.height, message.step))[:, : message.width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if encoding == "mono16":
        raw = data.view(np.uint16).reshape(
            (message.height, message.step // 2)
        )[:, : message.width]
        return cv2.cvtColor((raw / 256).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    if encoding in ("yuv422_yuy2", "yuyv"):
        yuv = data.reshape((message.height, message.step // 2, 2))[:, : message.width]
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUY2)
    if encoding.startswith("bayer"):
        raw = data.reshape((message.height, message.step))[:, : message.width]
        return cv2.cvtColor(raw, cv2.COLOR_BayerRGGB2BGR)
    return None


def open_writer(path: Path, fps: float, size: tuple[int, int]):
    for codec in VIDEO_CODECS:
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*codec), fps, size)
        if writer.isOpened():
            return writer, codec
        writer.release()
    raise RuntimeError(f"{path.name} 没有可用编码器: {', '.join(VIDEO_CODECS)}")


def frame_rate(stamps: list[int]) -> float:
    if len(stamps) < 2:
        return 30.0
    span = (stamps[-1] - stamps[0]) * 1.0e-9
    return round((len(stamps) - 1) / span, 3) if span > 0 else 30.0


def render(db_paths: list[Path], topic: str, output: Path, fps: float) -> dict:
    writer = None
    codec = None
    size = None
    written = 0
    skipped = 0
    first_stamp = last_stamp = None
    for timestamp_ns, data in iter_frames(db_paths, topic, timestamps_only=False):
        frame = frame_to_bgr(deserialize_message(data, Image))
        if frame is None:
            skipped += 1
            continue
        if size is None:
            size = (frame.shape[1], frame.shape[0])
            writer, codec = open_writer(output, fps, size)
        elif (frame.shape[1], frame.shape[0]) != size:
            frame = cv2.resize(frame, size)
        writer.write(frame)
        written += 1
        if first_stamp is None:
            first_stamp = timestamp_ns
        last_stamp = timestamp_ns
    if writer is not None:
        writer.release()
    if written == 0:
        raise RuntimeError(f"{topic} 里没有可解码的图像")

    return {
        "frames": written,
        "skipped": skipped,
        "size": list(size) if size else [],
        "codec": codec,
        "fps": fps,
        "play_seconds": written / fps,
        "record_seconds": (last_stamp - first_stamp) * 1.0e-9,
        "bytes": output.stat().st_size if output.exists() else 0,
    }


def export_one(target: str, out_dir: Path, topic: str, fps_arg: float) -> None:
    run_dir = resolve_run(target)
    run_id = run_dir.name
    bag_dir = run_dir / "bag"
    log(f"{run_id}: bag {bag_dir}")

    cache_dir = run_dir.parent.parent / "bag_exports" / run_id / "decompressed"
    db_paths = decompress_bag(bag_dir, cache_dir)

    stamps = [stamp for stamp, _ in iter_frames(db_paths, topic, timestamps_only=True)]
    if not stamps:
        raise RuntimeError(f"{topic} 不在 bag 里，可用话题见 metadata.yaml")
    fps = fps_arg if fps_arg > 0 else frame_rate(stamps)

    out_dir.mkdir(parents=True, exist_ok=True)
    output = out_dir / f"{run_id}.mp4"
    log(f"{run_id}: {len(stamps)} 帧，{fps} fps -> {output}")
    info = render(db_paths, topic, output, fps)
    log(
        f"{run_id}: 完成 {info['frames']} 帧，{info['size'][0]}x{info['size'][1]}，"
        f"编码 {info['codec']}，录制 {info['record_seconds']:.1f} s，"
        f"播放 {info['play_seconds']:.1f} s，{info['bytes'] / 1e6:.1f} MB"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("targets", nargs="+", help="run_id 前缀、run 目录或 bag 目录")
    parser.add_argument(
        "--out", type=Path, default=Path("~/rosbag_videos"),
        help="视频输出目录（默认 ~/rosbag_videos）",
    )
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument(
        "--fps", type=float, default=0.0,
        help="播放帧率，默认 0 表示按录制帧率实时播放",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = args.out.expanduser()
    for target in args.targets:
        try:
            export_one(target, out_dir, args.image_topic, args.fps)
        except (FileNotFoundError, RuntimeError) as error:
            print(f"[video] {target} 导出失败: {error}", file=sys.stderr)
            return 4
    log(f"全部完成 -> {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
