from ultralytics import YOLO
import cv2

# 1. Load your model (either pretrained or your trained weights)
model = YOLO('runs/detect/yolov11_weed_detect/weights/best.pt')  # path to your trained model

# 2. Path to the image you want to run inference on
img_path = '../clover_images/IMG_0784.JPEG'
img = cv2.imread(img_path)
if img is None:
    raise FileNotFoundError(f"Image not found at {img_path}")

height, width = img.shape[:2]
new_width = 640
scale = new_width / width
new_height = int(height * scale)
resized_img = cv2.resize(img, (new_width, new_height))

# Run inference on the resized image
results = model.predict(source=resized_img, conf=0.5, show=False, save=False)

# Visualize results
for result in results:
    annotated_frame = result.plot()  # draws boxes, labels, etc.

    # Show the image until 'q' is pressed
    cv2.imshow("Detection Result", annotated_frame)
    print("Press 'q' to close the window.")
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
# Optionally, save the annotated image
# cv2.imwrite('runs/predict/annotated_gripper.jpg', annotated_frame)

# from ultralytics import YOLO

# detect_model = YOLO("yolo11m.pt")
# pose_model = YOLO("yolo11m-pose.pt")
# for frame in camera_stream:
#     detect_results = detect_model(frame)
#     for box in detect_results[0].boxes:
#         if int(box.cls) == 0:  # "person"
#             crop = frame[int(box.y1):int(box.y2), int(box.x1):int(box.x2)]
#             pose_results = pose_model(crop)
