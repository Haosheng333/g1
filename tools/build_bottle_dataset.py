"""Build a YOLO detection dataset from the auto-labeled video frames
(tools/video_to_dataset.py output): remap the COCO class id to 0 (single
class), drop frames with no detection (these are missed detections on a
visible object, not true background -- keeping them as empty labels would
teach the model those frames have no object, which is wrong), and split into
train/val.
"""

from __future__ import annotations

import random
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent  # the DIP/ project root, not tools/
SRC_DIR = ROOT / "data" / "video_dataset"
OUT_DIR = ROOT / "data" / "bottle_dataset"
VAL_FRACTION = 0.1
SEED = 0
CLASS_NAME = "bottle"


def main() -> None:
    images_dir = SRC_DIR / "images"
    labels_dir = SRC_DIR / "labels"

    usable = []
    for label_path in sorted(labels_dir.glob("*.txt")):
        if label_path.stat().st_size == 0:
            continue  # missed detection, not true background -- drop it
        image_path = images_dir / f"{label_path.stem}.jpg"
        if image_path.exists():
            usable.append((image_path, label_path))

    rng = random.Random(SEED)
    indices = list(range(len(usable)))
    rng.shuffle(indices)
    n_val = max(1, int(len(usable) * VAL_FRACTION))
    val_set = set(indices[:n_val])

    for split in ("train", "val"):
        (OUT_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUT_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)

    for idx, (image_path, label_path) in enumerate(usable):
        split = "val" if idx in val_set else "train"

        lines_out = []
        for line in label_path.read_text().splitlines():
            parts = line.split()
            if not parts:
                continue
            _, cx, cy, w, h = parts
            lines_out.append(f"0 {cx} {cy} {w} {h}")  # remap to class 0

        shutil.copyfile(image_path, OUT_DIR / "images" / split / image_path.name)
        (OUT_DIR / "labels" / split / label_path.name).write_text("\n".join(lines_out) + "\n")

    (OUT_DIR / "bottle_dataset.yaml").write_text(
        f"""path: {OUT_DIR}
train: images/train
val: images/val
nc: 1
names:
  0: {CLASS_NAME}
""",
        encoding="utf-8",
    )

    print(f"{len(usable)} usable frames ({len(val_set)} val), dataset at {OUT_DIR}")


if __name__ == "__main__":
    main()
