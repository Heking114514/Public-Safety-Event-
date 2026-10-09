#!/usr/bin/env python3
"""Export the most recent navigation runs (video, logs, analysis, manifest)."""
import argparse
import subprocess
import sys
from pathlib import Path

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", type=Path, default=Path("/media/hjh/E/rosbag_recording/navigation_runs"))
    p.add_argument("--count", type=int, default=3)
    p.add_argument("--output", type=Path, help="shared export directory")
    p.add_argument("--no-video", action="store_true")
    p.add_argument("--no-analysis", action="store_true")
    args = p.parse_args()
    runs = sorted((x for x in args.root.iterdir() if (x / "bag").is_dir()), key=lambda x: x.stat().st_mtime, reverse=True)[:args.count]
    if len(runs) < args.count:
        print(f"only found {len(runs)} runs", file=sys.stderr)
    script = Path(__file__).with_name("export_rosbag.py")
    for run in reversed(runs):
        cmd = [sys.executable, str(script), str(run)]
        if args.output:
            cmd += ["--out", str(args.output / run.name)]
        if args.no_video: cmd.append("--no-video")
        if args.no_analysis: cmd.append("--no-analysis")
        result = subprocess.run(cmd)
        if result.returncode:
            return result.returncode
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
