from ultralytics import YOLO
import glob
import shutil
from pathlib import Path

# Load YOLOv11l model with pretrained weights
model = YOLO('yollo11n_leaf_v4.pt')
# model = YOLO('../trained_weeds_from_kaggle_best.pt')
# model = YOLO('../yolo11s_coco.pt')



# Train the model with transfer learning

run_name='yollo11n_leaf_v4'

results = model.train(
    data='data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    name=run_name,
    patience=50,
    save=True,
    plots=True
)

# Find and copy the latest best.pt file
best_pt_files = glob.glob('./runs/detect/*/weights/best.pt')
if best_pt_files:
    # Sort by modification time to get the latest
    latest_best_pt = max(best_pt_files, key=lambda x: Path(x).stat().st_mtime)
    destination = f"{run_name}.pt"
    shutil.copy2(latest_best_pt, destination)
    print(f"Copied {latest_best_pt} to {destination}")
else:
    print("No best.pt file found in ./runs/detect/*/weights/")

# Validate the model
metrics = model.val()
