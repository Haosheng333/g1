from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent.parent  # the DIP/ project root, not tools/
DATASET_YAML = ROOT / "data" / "bottle_dataset" / "bottle_dataset.yaml"


def main() -> None:
    model = YOLO(str(ROOT / "yolov8n.pt"))
    model.train(
        data=str(DATASET_YAML),
        epochs=100,
        patience=25,
        imgsz=640,
        batch=16,
        device="mps",
        workers=4,
        project=str(ROOT / "runs"),
        name="bottle_detect",
        exist_ok=True,
        verbose=True,
    )


if __name__ == "__main__":
    main()
