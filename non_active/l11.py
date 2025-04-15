import cv2
import serial
import time
from ultralytics import YOLO  # Import YOLO from Ultralytics
import os


poly_model_path = 'models\poly_non_poly.pt'
# Initialize YOLO Model
model = YOLO(poly_model_path)  # Load your trained YOLO model

# Initialize Serial Communication with Arduino
arduino = serial.Serial(port="COM8", baudrate=9600, timeout=1)  # Change COM port if needed
time.sleep(2)  # Wait for connection


def send_command(cmd):
    """Send a command to Arduino and wait for response"""
    arduino.write(cmd.encode())  # Send command
    time.sleep(1)  # Wait for action to complete
    response = arduino.readline().decode().strip()  # Read response
    print(f"Arduino Response: {response}")


def capture_frame():
    """Capture a frame from ESP32-CAM."""
    stream_url = "http://192.168.1.103/cam-hi.jpg"  # ESP32-CAM Stream URL
    cap = cv2.VideoCapture(stream_url)
    time.sleep(2)  # Allow camera to stabilize
    
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        print("Failed to grab frame")
        return None
    
    cv2.imwrite("temp.jpg", frame)  # Save image for processing
    return "temp.jpg"


def detect_polythene(image_path):
    """Run YOLO detection on the captured image to detect polythene."""
    results = model(image_path)  # Run inference
    
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            confidence = box.conf[0].item()
            detected_class = model.names[class_id]
            
            print(f"Detected: {detected_class} (Confidence: {confidence:.2f})")
            
            if detected_class.lower() == "polythene":
                return True
    
    return False


def process_garbage():
    """Complete process of detecting and handling garbage."""
    image_path = capture_frame()
    if image_path is None:
        return
    
    polythene_detected = detect_polythene(image_path)
    
    if polythene_detected:
        print("Polythene detected! Activating cutter and hand pressure, then opening plate.")
        send_command('H')  # Hand Pressure
        send_command('C')  # Cutter
        send_command('O')  # Open Plate (DC Motor)
    else:
        print("Non-polythene detected! Opening plate only.")
        send_command('O')  # Open Plate (DC Motor)
    
    # Indicate that first layer process is done
    with open("done.txt", "w") as f:
        f.write("done")


if __name__ == "__main__":
    process_garbage()
