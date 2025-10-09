from ultralytics import YOLO, checks, hub
import torch

# check gpu support
checks()
print(torch.cuda.is_available())
print(torch.cuda.device_count())
print(torch.cuda.get_device_name(0))

def main():
    # Run system checks
    checks()

    # Print GPU status
    print("CUDA available:", torch.cuda.is_available())
    if torch.cuda.is_available():
        print("GPU:", torch.cuda.get_device_name(0))

    # Login to Ultralytics HUB
    hub.login('28fe0dc494fd07c966c0b29ebecd9a48d71bd8a842')

    # Load your model from HUB
    model = YOLO('https://hub.ultralytics.com/models/BvK1L9z8HxgI1qt6W08x')

    # Train on GPU
    results = model.train(
        epochs=50,      # adjust as needed
        imgsz=640,      # adjust as needed
        device=0        # force GPU 0
    )

if __name__ == "__main__":
    main()