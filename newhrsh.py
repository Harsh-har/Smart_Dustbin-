import cv2
import requests
import time
import numpy as np
from ultralytics import YOLO

# from exception import CustomException as e
# from logger import logging

import logging

MAX_HEIGHT = 400
MAX_WIDTH = 400

# Initialize YOLO Models
model_poly = YOLO(r"D:\dust3\models\poly_non_poly.pt")  # YOLO model for polythene detection
model_biogas = YOLO(r"D:\dust3\models\biogas.pt")  # YOLO model for biodegradable classification

logging.info("Models have been loaded.")

# ESP8266 Configuration
ESP8266_IP = "192.168.1.200"

class_to_flag = {
    'nonbiogasready': 5,
    'biogasready': 4
}

region_pts = np.array([
[5, 108],
[5, 425],
[113, 467],
[236, 454],
[301, 314],
[328, 169],
[253, 74],
[143, 46],
[59, 64],
[6, 110]
], np.int32).reshape((-1, 1, 2))


def send_esp_command(cmd, expected_response=None, timeout=10):
    """Send a command to ESP8266 and filter duplicate/unexpected responses."""
    url = f"http://{ESP8266_IP}/command?cmd={cmd}"
    try:
        response = requests.get(url, timeout=timeout)
        esp_response = response.text.strip()

        # Print only the "Sent to Arduino" message
        if esp_response.startswith("Command Sent:"):
            print(f"ESP8266 Response: {esp_response}")
            logging.info(f"Sent '{cmd}', ESP Response: {esp_response}")

        # If an expected response is given, validate it
        if expected_response:
            if expected_response in esp_response:
                return True
            else:
                logging.warning(f"Unexpected ESP response for {cmd}: {esp_response}")
                return False
        return True
    except Exception as ex:
        logging.error(f"ESP8266 Communication Error: {ex}")
        return False


def capture_frame(cam_url):
    """Capture a frame from ESP32-CAM without drawing the region."""
    cap = cv2.VideoCapture(cam_url)
    # Wait dynamically for the camera to be ready
    start_time = time.time()
    # while not capture_frame1(cam_2):
    #     if time.time() - start_time > 5:  # Timeout after 5 seconds
    #         print("Camera not ready. Skipping this iteration.")
    #         break
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        print("Failed to grab frame")
        return None
    
    image_path = "temp.jpg"
    cv2.imwrite(image_path, frame)
    logging.info(f"Camera shot '{cam_url}' captured.")
    return frame

def capture_frame1(cam_url):
    """Capture a frame from ESP32-CAM with a highlighted region."""
    cap = cv2.VideoCapture(cam_url)
    time.sleep(2)
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        print("Failed to grab frame")
        return None
    
    cv2.polylines(frame, [region_pts], isClosed=True, color=(0, 255, 255), thickness=2)
    image_path = "temp.jpg"
    cv2.imwrite(image_path, frame)
    logging.info(f"Camera shot '{cam_url}' captured with bounding box")
    return frame


def detect_polythene(image_path):
    """Detect polythene using the YOLO model."""
    results = model_poly(image_path)
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            detected_class = model_poly.names[class_id].lower()
            if detected_class == "polythene":
                return True
    return False


def process_first_layer(cam_1):
    """Handles polythene detection and cutting mechanism."""
    frame = capture_frame(cam_1)
    if frame is None:
        return
    
    polythene_detected = detect_polythene("temp.jpg")
    print('First camera processing complete')

    if polythene_detected:
        print("Polythene detected! Activating cutter and pressure, then opening plate.")
        time.sleep(2)
        send_esp_command('H')  # Hand Pressure Activated
        cutter_response = send_esp_command('C')  # Cutter Complete
        
        if cutter_response:
            print("Cutter operation finished. Proceeding...")
            send_esp_command('O')  # Plate Rotation Completed
        else:
            print("Error: Cutter operation did not complete as expected.")
    else:
        print("No polythene detected! Opening plate only.")
        send_esp_command('O')  # Plate Rotation Completed


def detect_biodegradable(frame):
    """Detect objects in a specific region and classify them."""
    # Save the frame as a temporary image for YOLO processing
    temp_image_path = "temp_detect.jpg"
    cv2.imwrite(temp_image_path, frame)
    time.sleep(10)
    results = model_biogas(temp_image_path)
    detected_classes = []
    region_count = 0
    time.sleep(2)

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            detected_class = model_biogas.names[class_id].lower()
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            width = x2 - x1
            height = y2 - y1

            if width > MAX_WIDTH or height > MAX_HEIGHT :
                print(f"Ignored large bounding box: {(x1, y1, x2, y2)} and class {detected_class} too large with width {width} and height {height}")
                logging.info(f"Ignored large bounding box: {(x1, y1, x2, y2)} and class {detected_class} too large with width {width} and height {height}")
                continue

            if cv2.pointPolygonTest(region_pts, (center_x, center_y), False) >= 0:
                detected_classes.append(detected_class)
                region_count += 1
                # Draw bounding box and label on the frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, detected_class, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                
    # Save the frame with bounding boxes
    cv2.imwrite("detected_objects.jpg", frame)
    logging.info("Detection results saved to 'detected_objects.jpg'")

    print(f"Total objects detected in region: {region_count}, Classes: {detected_classes}")
    return detected_classes, region_count


def process_second_third_layer(cam_2):
    """Handles rotation, detection, and sorting based on classification."""
    send_esp_command("MOVE_SINGLE")
    time.sleep(1)

    for _ in range(4):
        time.sleep(2)
        print("Capturing Image for Object Detection")
        frame = capture_frame1(cam_2)

        if frame is None:
            continue

        detected_classes, region_count = detect_biodegradable(frame)

        if region_count == 0:
            print("No objects detected, moving plate to next position.")
            send_esp_command("MOVE_MULTI")
        elif region_count == 1:
            detected_class = detected_classes[0].lower().replace(" ", "")
            if detected_class in class_to_flag:
                print(f"{detected_class.capitalize()} detected")
                send_esp_command(f"FLAG_{class_to_flag[detected_class]}")
                send_esp_command("MOVE_MULTI")
        elif region_count >= 2:
            bio_count = detected_classes.count("biogasready")
            non_bio_count = detected_classes.count("nonbiogasready")

            if bio_count == region_count:
                print("All objects are biodegradable.")
                send_esp_command("FLAG_4")
                send_esp_command("MOVE_MULTI")
            elif non_bio_count == region_count:
                print("All objects are non-biodegradable.")
                send_esp_command("FLAG_5")
                send_esp_command("MOVE_MULTI")
            elif bio_count > 0 and non_bio_count > 0:
                print("Both biodegradable and non-biodegradable objects detected.")
                send_esp_command("MOVE_SINGLE")
            else:
                print("Mixed objects detected, moving plate quickly.")
                send_esp_command("MOVE_SINGLE")
        # else:
        #     print(f"Unknown classification: {detected_classes}. This might indicate an issue with the YOLO model's class names or unexpected objects in the frame. Please verify the model's configuration and detected classes.")
        #     send_esp_command("MOVE_SINGLE")

    print("✅ Process Completed Successfully!")
def loop(cam_2):
    """Loop to check if something is present on the plate and process accordingly."""
    terminate_loop = False
    no_object_count = 0

    while not terminate_loop:
        print("Capturing Image for Object Detection")
        frame = capture_frame1(cam_2)
        if frame is None:
            print("Failed to capture frame. Retrying...")
            continue

        detected_classes, region_count = detect_biodegradable(frame)

        if region_count == 0:
            no_object_count += 1
            print("No objects detected in the region.")
            if no_object_count >= 3:  # If no objects detected for 3 consecutive checks
                print("Process complete. No objects left on the plate.")
                terminate_loop = True
            else:
                send_esp_command("MOVE_MULTI")
                time.sleep(2)  # Allow time for the plate to move
            continue  # Capture the next frame and recheck

        elif region_count >= 1:
            no_object_count = 0  # Reset the counter if objects are detected
            bio_count = detected_classes.count("biogasready")
            non_bio_count = detected_classes.count("nonbiogasready")

            flag_sent = False  # Flag to prevent multiple executions

            if bio_count == region_count:
                print("All objects are biodegradable.")
                if not flag_sent:
                    send_esp_command("FLAG_4")
                    print("hello gandu")
                    flag_sent = True

            elif non_bio_count == region_count:
                print("All objects are non-biodegradable.")
                if not flag_sent:
                    send_esp_command("FLAG_5")
                    print("hello chutiya")
                    flag_sent = True

            if flag_sent:
                send_esp_command("MOVE_MULTI")

            elif bio_count > 0 and non_bio_count > 0:
                print("Both biodegradable and non-biodegradable objects detected.")
                send_esp_command("MOVE_SINGLE")

    print("✅ loop Completed Successfully!")
