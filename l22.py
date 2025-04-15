import cv2
import serial
import time
import requests
import numpy as np
from ultralytics import YOLO

MAX_HEIGHT = 400
MAX_WIDTH = 400

class SortingMechanism:
    """Handles the second and third layers (rotating plate, detection, and sorting)."""
    
    def __init__(self, arduino_port="COM6", baudrate=9600, model_path="models\biogas.pt", esp8266_ip="192.168.1.28"):
        self.arduino = serial.Serial(port=arduino_port, baudrate=baudrate, timeout=1)
        self.model = YOLO(model_path)
        self.esp8266_ip = esp8266_ip
        self.rotation_angles = [0, 90, 180, 270]
        time.sleep(2)  # Allow Arduino to initialize
        
        # Mapping class names to flag values
        self.class_to_flag = {
            'non-biodegradable': 1,
            'biodegradable': 2,
            'common': 3,
            'nonbiogasready': 5,
            'biogasready': 4
        }

    def send_command(self, cmd):
        """Send a command to Arduino and print its response."""
        self.arduino.write(f"{cmd}\n".encode())  # Ensure newline for Arduino parsing
        time.sleep(1)
        response = self.arduino.readline().decode().strip()
        print(f"🟢 Arduino Response: {response}")

    def send_flag(self, flag):
        """Send a flag value to ESP8266 via HTTP GET request."""
        url = f"http://{self.esp8266_ip}/flag?value={flag}"
        try:
            response = requests.get(url, timeout=5)
            print(f"🟢 ESP8266 Response: {response.text}")
        except Exception as e:
            print(f"❌ Error communicating with ESP8266: {e}")

    def capture_image(self):
        """Capture an image from ESP32-CAM and return the frame."""
        cam_url = 'http://192.168.1.104/cam-hi.jpg'
        cap = cv2.VideoCapture(cam_url)
        time.sleep(1)
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            print("❌ Error: Could not capture frame from camera.")
            return None

        timestamp = int(time.time())
        image_filename = f"captured_image_{timestamp}.jpg"
        cv2.imwrite(image_filename, frame)
        print(f"📸 Image captured and saved as '{image_filename}'.")
        return frame

    def detect_objects(self, frame):
        """Run YOLO detection and return detected class & confidence."""
        results = self.model(frame)
        detected_flags = []

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = box.conf[0].item()
                detected_class = self.model.names.get(class_id, "Unknown")
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                width = x2 - x1
                height = y2 - y1

                if width > MAX_WIDTH or height > MAX_HEIGHT:
                    print(f"⚠ Ignoring {detected_class} (Too Large: {width}x{height})")
                    continue

                normalized_class = detected_class.lower().replace(" ", "-")
                color = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = f"{detected_class} {confidence:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                print(f"✅ Detected: {detected_class} | Confidence: {confidence:.2f}")
                if confidence > 0.4:
                    flag = self.class_to_flag.get(normalized_class, None)
                    if flag is not None:
                        detected_flags.append(flag)

        if detected_flags:
            selected_flag = min(detected_flags)
            self.send_flag(selected_flag)
            print(f"🟢 Sending Flag: {selected_flag}")
        else:
            print("⚠ No valid detections found.")

        return detected_flags

    def process_sorting(self):
        """Main loop to rotate plate, capture images, detect objects, and activate robotic hand."""
        for _ in range(4):
            print("\n✋ --- Activating Robotic Hand ---")
            self.send_command("MOVE")
            time.sleep(1)

            print("\n📸 --- Capturing New Image for Object Detection ---")
            frame = self.capture_image()
            if frame is None:
                continue

            print("\n🔍 --- Running Object Detection ---")
            detected_class = self.detect_objects(frame)

            print("\n🤖 --- Rotating Third Layer Hand Motor Based on Detected Class ---")
            self.send_command(f"HAND_MOTOR:{detected_class}")
            time.sleep(2)

            print("\n🔄 --- Returning Third Layer Hand Motor to Initial State ---")
            self.send_command("HAND_MOTOR:RESET")
            time.sleep(1)

        print("✅ Process Completed Successfully!")