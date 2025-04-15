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

logging.info("models been read.")

# Initialize Serial Communication with Arduino
arduino = serial.Serial(port="COM7", baudrate=9600, timeout=1)  # Change COM port if needed
time.sleep(2)  # Wait for connection

logging.info("arduino got connected.")

ESP8266_IP = "192.168.1.31"

class_to_flag = {
    'non-biodegradable': 1,
    'biodegradable': 2,
    'common': 3,
    'nonbiogasready': 5,
    'biogasready': 4
}

def send_command(cmd):
    """Send a command to Arduino and wait for response"""
    arduino.write(cmd.encode())
    time.sleep(1)
    response = arduino.readline().decode().strip()
    print(f"Arduino Response: {response}")
    logging.info(f"Command '{cmd}' been sent to arduino")

def send_flag(flag):
    """Send a flag value to ESP8266 via HTTP GET request."""
    url = f"http://{ESP8266_IP}/flag?value={flag}"
    try:
        response = requests.get(url, timeout=5)
        print(f"ESP8266 Response: {response.text}")
        logging.info("response '{response.text}' been sent.")
    except Exception as e:
        print(f"Error communicating with ESP8266: {e}")

def capture_frame(cam_url):
    """Capture a frame from ESP32-CAM."""
    cap = cv2.VideoCapture(cam_url)
    time.sleep(2)
    
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        print("Failed to grab frame")
        return None
    
    image_path = "temp.jpg"
    cv2.imwrite(image_path, frame)
    logging.info("camera shot '{cam_url}' done successfully")
    return frame

def detect_polythene(image_path):
    """Run YOLO detection to check for polythene."""
    results = model_poly(image_path)
    
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            detected_class = model_poly.names[class_id]

            logging.info("Class detected: {detected_class}")
            
            print(f"Detected: {detected_class}")
            if detected_class.lower() == "polythene":
                return True
    
    return False

def process_first_layer(cam_1):

    
    """Handles polythene detection and cutting mechanism."""
    frame = capture_frame(cam_1)
    if frame is None:
        return
    
    polythene_detected = detect_polythene("temp.jpg")
    print('first camera did its work')
    
    if polythene_detected:
        print("Polythene detected! Activating cutter and hand pressure, then opening plate.")
        send_command('H')
        time.sleep(1)
        send_command('C')
        time.sleep(1)
        send_command('O')
    else:
        print("Non-polythene detected! Opening plate only.")
        send_command('O')

def detect_biodegradable(frame):
    """Run YOLO detection to classify garbage as biodegradable or non-biodegradable."""
    results = model_biogas(frame)
    detected_flags = []
    
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            detected_class = model_biogas.names[class_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            width = x2 - x1
            height = y2 - y1

            if width > MAX_WIDTH or height > MAX_HEIGHT:
                print(f"Ignoring {detected_class} (Too Large: {width}x{height})")
                continue

            normalized_class = detected_class.lower().replace(" ", "-")
            print(f"Detected: {detected_class}")
            flag = class_to_flag.get(normalized_class, None)
            if flag is not None:
                detected_flags.append(flag)
    
    if detected_flags:

        selected_flag = min(detected_flags)
        send_flag(selected_flag)
        print(f"Sending Flag: {selected_flag}")
        logging.info(f'{selected_flag} been sent')
    else:
        print("No valid detections found.")

    return detected_flags

def process_second_third_layer(cam_2):
    """Handles rotation, detection, and sorting."""
    for _ in range(4):
        print("Activating Robotic Hand")
        send_command("MOVE")
        time.sleep(1)

        print("Capturing Image for Object Detection")
        frame = capture_frame(cam_2)
        if frame is None:
            continue

        print("Running Object Detection")
        detected_class = detect_biodegradable(frame)

        print("Rotating Third Layer Hand Motor Based on Detected Class")
        send_command(f"HAND_MOTOR:{detected_class}")
        time.sleep(2)

        print("Returning Third Layer Hand Motor to Initial State")
        send_command("HAND_MOTOR:RESET")
        time.sleep(1)

    print("✅ Process Completed Successfully!")

cam_1 = "http://192.168.1.103/cam-hi.jpg"
cam_2 = "http://192.168.1.104/cam-hi.jpg"

if __name__ == "__main__":

    print("Starting Garbage Processing...")
    process_first_layer(cam_1)
    time.sleep(2)
    print("Starting Sorting Mechanism...")
    process_second_third_layer(cam_2)