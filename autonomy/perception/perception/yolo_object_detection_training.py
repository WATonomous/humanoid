#Trains a YOLO11n model for badminton shuttlecock detection.

from ultralytics import YOLO


def main() -> None:
    model = YOLO("yolo11n.pt")

    model.train(
        # Need to update path after dataset is added
        data="path/to/badminton.yaml",
        epochs=75,
        imgsz=960,
        batch=8,
        device=0,
        project="runs/detect",
        name="yolo11n_shuttlecock",
        pretrained=True,
        single_cls=True,
    )


if __name__ == "__main__":
    main()