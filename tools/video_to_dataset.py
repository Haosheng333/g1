"""Turn a phone video into a reviewable object-detection dataset.

Extracts evenly-spaced frames from a video and runs a pretrained YOLOv8
detector on each frame to produce candidate bboxes (auto-labeling). Since the
target classes (bottle/cup/etc.) are already COCO classes, this gives a
starting label set that a human only needs to review/correct, not draw from
scratch. Writes:

  <out_dir>/images/*.jpg           extracted frames
  <out_dir>/labels/*.txt           YOLO-format bboxes (class cx cy w h, normalized)
  <out_dir>/preview/*.jpg          frames with boxes drawn, for quick human review
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
from ultralytics import YOLO


def extract_evenly_spaced_frame_indices(total_frames: int, n_frames: int) -> list[int]:
    if total_frames <= n_frames:
        return list(range(total_frames))
    step = total_frames / n_frames
    return [int(i * step) for i in range(n_frames)]


def run(video_path: Path, out_dir: Path, n_frames: int, labels: list[str] | None, conf: float) -> None:
    images_dir = out_dir / "images"
    labels_dir = out_dir / "labels"
    preview_dir = out_dir / "preview"
    for d in (images_dir, labels_dir, preview_dir):
        d.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise FileNotFoundError(f"Could not open video: {video_path}")
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    indices = set(extract_evenly_spaced_frame_indices(total_frames, n_frames))

    model = YOLO(str(ROOT / "yolov8n.pt"))
    class_ids = None
    if labels:
        name_to_id = {v: k for k, v in model.names.items()}
        class_ids = [name_to_id[label] for label in labels if label in name_to_id]

    saved, empty = 0, 0
    frame_idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_idx in indices:
            stem = f"frame_{frame_idx:06d}"
            result = model.predict(frame, conf=conf, classes=class_ids, verbose=False)[0]

            h, w = frame.shape[:2]
            lines = []
            preview = frame.copy()
            for box in result.boxes:
                cls_id = int(box.cls.item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx, cy = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
                bw, bh = (x2 - x1) / w, (y2 - y1) / h
                lines.append(f"{cls_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
                cv2.rectangle(preview, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.putText(
                    preview,
                    f"{model.names[cls_id]} {box.conf.item():.2f}",
                    (int(x1), max(0, int(y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                )

            cv2.imwrite(str(images_dir / f"{stem}.jpg"), frame)
            (labels_dir / f"{stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
            cv2.imwrite(str(preview_dir / f"{stem}.jpg"), preview)

            saved += 1
            if not lines:
                empty += 1
        frame_idx += 1

    cap.release()
    print(f"Extracted {saved} frames from {video_path.name} ({total_frames} total frames)")
    print(f"{empty} frames had zero detections -- check those in {preview_dir} first")
    print(f"Review boxes in {preview_dir}, then correct labels in {labels_dir} before training")


ROOT = Path(__file__).resolve().parent.parent  # the DIP/ project root, not tools/


def main() -> None:
    parser = argparse.ArgumentParser(description="Video -> auto-labeled detection dataset")
    parser.add_argument("video", type=str, help="Path to the input video (mp4, mov, etc.)")
    parser.add_argument(
        "--out", type=str, default=str(ROOT / "data" / "video_dataset"), help="Output directory"
    )
    parser.add_argument("--n-frames", type=int, default=500, help="Number of frames to extract")
    parser.add_argument("--labels", type=str, nargs="*", default=None, help="Restrict to these COCO class names")
    parser.add_argument("--conf", type=float, default=0.25, help="Detection confidence threshold")
    args = parser.parse_args()

    run(Path(args.video), Path(args.out), args.n_frames, args.labels, args.conf)


if __name__ == "__main__":
    main()
