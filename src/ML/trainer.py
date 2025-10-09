from ultralytics import YOLO

# Load last checkpoint
model = YOLO("/root/train/runs/detect/train4/weights/best.pt")

# Resume training
results = model.train(
    data="dataset/data.yaml",
    epochs=50,   # total number of epochs you want
    imgsz=640,
    batch=16,
    device="cpu",
    resume=True  # important!
)
