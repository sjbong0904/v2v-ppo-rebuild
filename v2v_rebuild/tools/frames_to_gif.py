#!/usr/bin/env python3
"""Convert a recorded PNG frame folder into presentation/media/demo.gif."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DEFAULT_OUT = ROOT / "presentation" / "media" / "demo.gif"


def collect_frames(folder: Path) -> list[Path]:
    frames = sorted(folder.glob("frame_*.png"))
    if not frames:
        frames = sorted(folder.glob("*.png"))
    return frames


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "frames_dir",
        type=Path,
        help="Folder with frame_XXXX.png (from Demo Viewer Record frames)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT,
        help=f"Output GIF path (default: {DEFAULT_OUT})",
    )
    parser.add_argument("--fps", type=float, default=10.0, help="GIF frames per second")
    parser.add_argument("--width", type=int, default=720, help="Max width (keeps aspect)")
    parser.add_argument("--max-frames", type=int, default=180, help="Cap length for README size")
    args = parser.parse_args()

    paths = collect_frames(args.frames_dir)
    if not paths:
        raise SystemExit(f"No PNG frames in {args.frames_dir}")

    if len(paths) > args.max_frames:
        # Uniform subsample
        step = len(paths) / args.max_frames
        paths = [paths[int(i * step)] for i in range(args.max_frames)]

    images: list[Image.Image] = []
    for path in paths:
        im = Image.open(path).convert("RGB")
        if args.width and im.width > args.width:
            h = int(im.height * (args.width / im.width))
            im = im.resize((args.width, h), Image.Resampling.LANCZOS)
        images.append(im)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    duration = max(20, int(1000 / max(args.fps, 1.0)))
    images[0].save(
        args.out,
        save_all=True,
        append_images=images[1:],
        duration=duration,
        loop=0,
        optimize=True,
    )
    size_mb = args.out.stat().st_size / (1024 * 1024)
    print(f"Wrote {args.out} ({len(images)} frames, {size_mb:.2f} MB)")


if __name__ == "__main__":
    main()
