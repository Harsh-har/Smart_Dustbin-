# import cv2
# import numpy as np
# import requests

# # Define the camera URL
# cam_url = "http://192.168.1.104/cam-hi.jpg"

# # Fetch the image from the URL
# response = requests.get(cam_url, stream=True)
# if response.status_code == 200:
#     img_arr = np.asarray(bytearray(response.content), dtype=np.uint8)
#     frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
# else:
#     print("Failed to fetch image from camera")
#     exit()

# # Define the adjusted region (10% decrease from left and right)
# region_pts = np.array([[160, 150], [170, 370], [390, 350], [380, 170]], np.int32)
# region_pts = region_pts.reshape((-1, 1, 2))

# # Draw the polygon on the image
# cv2.polylines(frame, [region_pts], isClosed=True, color=(0, 255, 255), thickness=2)

# # Show the image with the drawn region
# cv2.imshow("Region Highlighted", frame)
# cv2.waitKey(0)
# cv2.destroyAllWindows()



import cv2
import numpy as np
import requests

# Define the camera URL
cam_url = "http://192.168.1.104/cam-hi.jpg"

# Fetch the image from the URL
response = requests.get(cam_url, stream=True)
if response.status_code == 200:
    img_arr = np.asarray(bytearray(response.content), dtype=np.uint8)
    frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
else:
    print("Failed to fetch image from camera")
    exit()

# Store clicked points
points = []

# Mouse click event function
def draw_circle(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button click
        points.append((x, y))
        print(f"Clicked Coordinate: ({x}, {y})")
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)  # Draw a red circle
        if len(points) > 1:
            cv2.line(frame, points[-2], points[-1], (255, 0, 0), 2)  # Connect points
        cv2.imshow("Click to Draw", frame)

# Create window and set mouse callback
cv2.imshow("Click to Draw", frame)
cv2.setMouseCallback("Click to Draw", draw_circle)

cv2.waitKey(0)
cv2.destroyAllWindows()


# import cv2
# import torch
# import requests
# import numpy as np
# from ultralytics import YOLO  # Ensure YOLOv8 is installed

# # Load YOLOv8 model (Change the path if needed)
# model = YOLO("biogas_model.pt")  # Replace with your actual model path

# # ESP8266 IP Address
# ESP8266_IP = "192.168.1.20"

# # Camera Snapshot URL
# CAMERA_SNAPSHOT_URL = "http://your_camera_ip/capture"

# # Flask API to send the flag
# FLAG_API_URL = f"http://{ESP8266_IP}/send_flag"

# # Define object-to-flag mapping
# OBJECT_FLAGS = {
#     "biogasready": 4,
#     "non_biogasready": 5
# }

# def send_flag(flag):
#     """Send detected flag to the ESP8266 API."""
#     data = {"flag": flag}
#     try:
#         response = requests.post(FLAG_API_URL, json=data)
#         print(f"Flag {flag} sent, Response: {response.text}")
#     except Exception as e:
#         print("Error sending flag:", e)

# def capture_snapshot():
#     """Capture a snapshot from the camera."""
#     try:
#         response = requests.get(CAMERA_SNAPSHOT_URL, stream=True)
#         if response.status_code == 200:
#             image_array = np.asarray(bytearray(response.content), dtype=np.uint8)
#             return cv2.imdecode(image_array, cv2.IMREAD_COLOR)
#         else:
#             print("Failed to capture snapshot")
#             return None
#     except Exception as e:
#         print("Error capturing snapshot:", e)
#         return None

# # Capture a snapshot
# frame = capture_snapshot()
# if frame is not None:
#     # Run YOLO detection
#     results = model(frame)
#     detected_flag = None

#     for r in results:
#         for box in r.boxes:
#             cls = int(box.cls[0])  # Get class index
#             label = model.names[cls]  # Get class label
            
#             if label in OBJECT_FLAGS:
#                 detected_flag = OBJECT_FLAGS[label]
#                 send_flag(detected_flag)
#                 break  # Send only one flag per frame

#     # Display the frame
#     cv2.imshow("Detection", frame)
#     cv2.waitKey(2000)  # Show the frame for 2 seconds before closing

# cv2.destroyAllWindows()
