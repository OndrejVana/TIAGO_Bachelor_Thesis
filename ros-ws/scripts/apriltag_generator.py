#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
import imageio.v2 as imageio
from PIL import Image

from moms_apriltag import TagGenerator2


def generate_tag_png(tag_id: int, out_path: Path, family: str = "tag36h11",
                     img_size_px: int = 800, margin_px: int = 80) -> None:
    """
    Generate a v2 AprilTag PNG (e.g., tag36h11) with a white margin around it.
    - img_size_px: final image size (square)
    - margin_px: white border (helps detection/printing)
    """
    tg = TagGenerator2(family)
    tag = tg.generate(tag_id)

    tag_img = Image.fromarray(tag.astype(np.uint8), mode="L")

    inner_size = img_size_px - 2 * margin_px
    if inner_size <= 0:
        raise ValueError("margin_px too large for img_size_px")

    tag_img = tag_img.resize((inner_size, inner_size), resample=Image.NEAREST)

    canvas = Image.new("L", (img_size_px, img_size_px), color=255)
    canvas.paste(tag_img, (margin_px, margin_px))

    out_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.imwrite(str(out_path), np.array(canvas))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", nargs="+", type=int, required=True, help="Tag IDs to generate (e.g. 0 1)")
    ap.add_argument("--out_dir", type=Path, default=Path("./tags_out"))
    ap.add_argument("--family", type=str, default="tag36h11")
    ap.add_argument("--size", type=int, default=800)
    ap.add_argument("--margin", type=int, default=80)
    args = ap.parse_args()

    for tid in args.ids:
        out = args.out_dir / f"{args.family}_{tid}.png"
        generate_tag_png(tid, out, family=args.family, img_size_px=args.size, margin_px=args.margin)
        print(f"Wrote: {out}")


if __name__ == "__main__":
    main()