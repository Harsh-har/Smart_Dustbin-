from ultralytics import YOLO
import cv2

# Load the YOLOv8 model
model = YOLO(r'models\biogas.pt')

# Load the image
image_path = r'C:\Users\harsh\OneDrive\Desktop\r.jpg'
image = cv2.imread(image_path)

# Run inference
results = model(image)

# Define class names (modify as per your model's classes)
class_names = model.names

# Iterate over the detections
for result in results:
    boxes = result.boxes
    for box in boxes:
        # Extract bounding box coordinates
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        # Extract confidence score
        confidence = box.conf[0]

        # Extract class id
        class_id = int(box.cls[0])

        # Draw the bounding box
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Create label text
        label = f"{class_names[class_id]}: {confidence:.2f}"

        # Draw the label
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Display the image with bounding boxes
cv2.imshow('Detected Objects', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
