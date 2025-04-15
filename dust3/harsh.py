import cv2
import requests
import time
import numpy as np
from ultralytics import YOLO
import itertools

# from exception import CustomException as e
# from logger import logging

import logging

MAX_HEIGHT = 450
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

        if esp_response.startswith("Command Sent:"):
            print(f"ESP8266 Response: {esp_response}")
            logging.info(f"Sent '{cmd}', ESP Response: {esp_response}")

      
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
    start_time = time.time()

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
    no_object_count = 0

    for _ in itertools.count():
        time.sleep(4) # Wait for the plate to settle after moving
        print("Capturing Image for Object Detection")
        frame = capture_frame1(cam_2)
        if frame is None:
            print("Failed to capture frame. Retrying...")
            continue

        detected_classes, region_count = detect_biodegradable(frame)

        if region_count == 0:
            no_object_count += 1
            print("No objects detected in the region.")
            if no_object_count >= 3:
                print("Process complete. No objects left on the plate.")
                break
            else:
                send_esp_command("MOVE_MULTI")
                time.sleep(2)  # Wait for the plate to move
            continue

        # Reset counter once an object is detected
        no_object_count = 0  
        bio_count = detected_classes.count("biogasready")
        non_bio_count = detected_classes.count("nonbiogasready")

        if bio_count == region_count:
            print("All objects are biodegradable.")
            send_esp_command("FLAG_4")
        elif non_bio_count == region_count:
            print("All objects are non-biodegradable.")
            send_esp_command("FLAG_5")
        elif bio_count > 0 and non_bio_count > 0:
            print("Both biodegradable and non-biodegradable objects detected.")
            send_esp_command("MOVE_SINGLE")

        # Command the plate to move and wait longer for an updated image
        send_esp_command("MOVE_MULTI")
        time.sleep(5) # Increased wait time

        # Flush the camera buffer by opening and releasing the camera
        cap = cv2.VideoCapture(cam_2)
        for _ in range(5):  # Capture and discard several frames to clear the buffer
            ret, _ = cap.read()
            if not ret:
                print("Failed to flush camera buffer.")
        cap.release()
        time.sleep(2) # Give a small delay after flushing

    print("✅ loop Completed Successfully!")





# Camera URLs
cam_1 = "http://192.168.1.103/cam-hi.jpg"
cam_2 = "http://192.168.1.104/cam-hi.jpg"

if __name__ == "__main__":
    Start = time.time()
    
    #================================================================================================================================
    firstlayer_start_time = time.time() 
    print("Starting Garbage Processing...")
    process_first_layer(cam_1)
    FIRST_layer_end_time = time.time()
    
    FIRST_layer_total_time = FIRST_layer_end_time - firstlayer_start_time  # Calculate total time in seconds
    print(f"Total execution time: {FIRST_layer_total_time:.2f} seconds")
    #================================================================================================================================
    
    time.sleep(1)
    
    #================================================================================================================================
    for2layer_start_time = time.time()
    process_second_third_layer(cam_2)
    for2layer_end_time = time.time()
    
    SECOND_layer_total_time = for2layer_end_time - for2layer_start_time  # Calculate total time in seconds
    print(f"Total execution time: {SECOND_layer_total_time:.2f} seconds")
    #================================================================================================================================
    
    #================================================================================================================================
    loopstart = time.time()
    loop(cam_2)
    # checkpoly()
    loop_end_time = time.time()  # Record the end time
    
    loop_total_time = loop_end_time - loopstart  # Calculate total time in seconds
    print(f"Total execution time: {loop_total_time:.2f} seconds")
    #================================================================================================================================


    end = time.time()
    total_time = end - Start  # Calculate total time in seconds
    print(f"Total execution time: {total_time:.2f} seconds")
    print("Garbage Processing Completed!")


