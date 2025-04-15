import cv2
import serial
import time
from ultralytics import YOLO
import os



class GarbageProcessor:
    """Handles the first layer mechanism (detecting and processing polythene)."""
    
    def __init__(self, arduino_port="COM6", baudrate=9600, model_path="models\poly_non_poly.pt"):
        self.arduino = serial.Serial(port=arduino_port, baudrate=baudrate, timeout=1)
        self.model = YOLO(model_path)
        time.sleep(2)  # Allow Arduino to initialize

    def send_command(self, cmd):
        """Send a command to Arduino and wait for response."""
        self.arduino.write(cmd.encode())
        time.sleep(1)
        response = self.arduino.readline().decode().strip()
        print(f"Arduino Response: {response}")

    def capture_frame(self):
        """Capture a frame from ESP32-CAM."""
        stream_url = "http://192.168.1.103/cam-hi.jpg"
        cap = cv2.VideoCapture(stream_url)
        time.sleep(2)  # Allow camera to stabilize
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            print("Failed to grab frame")
            return None
        
        cv2.imwrite("temp.jpg", frame)  # Save image for processing
        return "temp.jpg"

    def detect_polythene(self, image_path):
        """Run YOLO detection on the captured image to detect polythene."""
        results = self.model(image_path)
        
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = box.conf[0].item()
                detected_class = self.model.names[class_id]
                
                print(f"Detected: {detected_class} (Confidence: {confidence:.2f})")
                
                if detected_class.lower() == "polythene":
                    return True
        
        return False

    def process_garbage(self):
        """Complete process of detecting and handling garbage."""
        image_path = self.capture_frame()
        if image_path is None:
            return
        
        polythene_detected = self.detect_polythene(image_path)
        
        if polythene_detected:
            print("Polythene detected! Activating cutter and hand pressure, then opening plate.")
            self.send_command('H')  # Hand Pressure
            self.send_command('C')  # Cutter
            self.send_command('O')  # Open Plate (DC Motor)
        else:
            print("Non-polythene detected! Opening plate only.")
            self.send_command('O')  # Open Plate (DC Motor)
        
        # Indicate that first layer process is done
        with open("done.txt", "w") as f:
            f.write("done")
