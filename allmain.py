import cv2
import serial
import time
import requests
import numpy as np
from ultralytics import YOLO

from exception import CustomException as e
from logger import logging

MAX_HEIGHT = 400
MAX_WIDTH = 400

# Initialize YOLO Models
model_poly = YOLO(r"models\poly_non_poly.pt")  # YOLO model for polythene detection
model_biogas = YOLO(r"models\biogas.pt")  # YOLO model for biodegradable classification

logging.info("Models have been loaded.")

# Initialize Serial Communication with Arduino
arduino = serial.Serial(port="COM3", baudrate=9600, timeout=1)  # Change COM port if needed
time.sleep(2)  # Wait for connection

logging.info("Arduino connected.")

ESP8266_IP = "192.168.1.17"

class_to_flag = {
    'non-biodegradable': 1,
    'biodegradable': 2,
    'common': 3,
    'nonbiogasready': 5,
    'biogasready': 4
}

region_pts = np.array([[150, 100], [100, 400], [450, 400], [450, 150]], np.int32)
region_pts = region_pts.reshape((-1, 1, 2))

def send_command(cmd):
    """Send a command to Arduino and wait for response"""
    arduino.write(cmd.encode())
    time.sleep(1)
    response = arduino.readline().decode().strip()
    print(f"Arduino Response: {response}")
    logging.info(f"Command '{cmd}' sent to Arduino")

def send_flag(flag):
    """Send a flag value to ESP8266 via HTTP GET request."""
    url = f"http://{ESP8266_IP}/flag?value={flag}"
    try:
        response = requests.get(url, timeout=5)
        print(f"ESP8266 Response: {response.text}")
        logging.info(f"Response '{response.text}' sent.")
    except Exception as e:
        print(f"Error communicating with ESP8266: {e}")

def capture_frame(cam_url):
    """Capture a frame from ESP32-CAM with highlighted region."""
    cap = cv2.VideoCapture(cam_url)
    time.sleep(2)
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        print("Failed to grab frame")
        return None
    
    # cv2.polylines(frame, [region_pts], isClosed=True, color=(0, 255, 255), thickness=2)
    image_path = "temp.jpg"
    cv2.imwrite(image_path, frame)
    logging.info(f"Camera shot '{cam_url}' captured with bounding box")
    return frame

def capture_frame1(cam_url):
    """Capture a frame from ESP32-CAM with highlighted region."""
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
    """Detect polythene using YOLO model."""
    results = model_poly(image_path)
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            detected_class = model_poly.names[class_id].lower()
            if detected_class.lower() == "polythene":
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
        send_command('H')
        time.sleep(0.5)
        send_command('C')
        time.sleep(2)
        send_command('O')
        time.sleep (2)
    else:
        print("No polythene detected! Opening plate only.")
        send_command('O')
        time.sleep (2)
        

def detect_biodegradable(frame):
    """Detect objects in a specific region and classify them."""
    time.sleep(10)
    results = model_biogas(frame)
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


            # Filter out bounding boxes that exceed a given area
            if (x2 - x1) * (y2 - y1) > MAX_HEIGHT * MAX_WIDTH:
                logging.info(f"Ignored large bounding box: {(x1, y1, x2, y2)}")
                continue

            if cv2.pointPolygonTest(region_pts, (center_x, center_y), False) >= 0:
                detected_classes.append(detected_class)
                region_count += 1
                color = (0, 255, 0) 
                if detected_class == "biodegradable":
                    color = (0, 255, 0)
                elif detected_class == "nonbiogasready":
                    color = (0, 0, 255)
                else :
                    color = (255, 0, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, detected_class, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                
    print(f"Total objects detected in region: {region_count}")

     # If no objects are detected in the ROI, move the plate to the next position
    if region_count == 3:
        send_command("MOVE_MULTI")
        print("No objects detected in region, moving plate to next position.")

    return detected_classes, region_count


def process_second_third_layer(cam_2):
    """Handles rotation, detection, and sorting based on classification."""
    for _ in range(3):
        print("Capturing Image for Object Detection")
        frame = capture_frame1(cam_2)
        if frame is None:
            continue

        detected_classes, region_count = detect_biodegradable(frame)

        if region_count == 0:
            print("No objects detected, moving plate slowly to next position.")
            send_command("STOP")
            send_command("MOVE_MULTI")
            time.sleep(2)
        
        elif region_count >= 2:
            bio_count = detected_classes.count("biodegradable")
            non_bio_count = detected_classes.count("nonbiogasready")
            
            if bio_count == region_count:
                print("All objects are biodegradable, sending flag and moving plate slowly.")
                send_flag(class_to_flag["biogasready"])
                send_command("MOVE_MULTI")
                time.sleep(2)
            elif non_bio_count == region_count:
                print("All objects are non-biodegradable, sending flag and moving plate slowly.")
                send_flag(class_to_flag["nonbiogasready"])
                send_command("MOVE_MULTI")
                time.sleep(2)
            else:
                print("Mixed objects detected, moving plate quickly to next position.")
                send_command("MOVE_SINGLE")
                time.sleep(2)
        else:
            print("Objects detected, sending flag and moving plate.")
            detected_class = detected_classes[0].lower().replace(" ", "").lower()
            if detected_class in class_to_flag:
                send_flag(class_to_flag[detected_class])
            else:
                print(f"Unknown class detected: {detected_class}")
            time.sleep(1)
            send_command("MOVE_SINGLE")
            time.sleep(6)
            send_command("MOVE_MULTI")
    
    print("✅ Process Completed Successfully!")


cam_1 = "http://192.168.1.103/cam-hi.jpg"
cam_2 = "http://192.168.1.104/cam-hi.jpg"

if __name__ == "__main__":
    print("Starting Garbage Processing...")
    process_first_layer(cam_1)
    time.sleep(2)
    process_second_third_layer(cam_2)