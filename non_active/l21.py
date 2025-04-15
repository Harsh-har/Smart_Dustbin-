import cv2
import serial
import time
from ultralytics import YOLO


biogas_path = "models\biogas.pt"
# --------------------------
# Setup: Load YOLO Model & Arduino Communication
# --------------------------
model = YOLO(biogas_path)  # Load YOLO model

# Initialize serial communication with Arduino (update port if needed)
arduino = serial.Serial(port="COM6", baudrate=9600, timeout=1)
time.sleep(2)  # Allow Arduino to initialize

# Define angles (Only 0° and 90°)
rotation_angles = [0, 90]

def send_command(cmd):
    """Send a command to Arduino and print its response."""
    arduino.write(f"{cmd}\n".encode())  # Ensure newline for Arduino parsing
    time.sleep(1)  # Allow time for Arduino to execute
    response = arduino.readline().decode().strip()
    print(f"🟢 Arduino Response: {response}")

def capture_image():
    """Capture an image from ESP32-CAM and return the frame."""
    cam_url = 'http://192.168.1.104/cam-hi.jpg'
    cap = cv2.VideoCapture(cam_url)
    time.sleep(1)  # Give time for the camera to adjust
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        print("❌ Error: Could not capture frame from camera.")
        return None

    image_filename = "captured_image.jpg"
    cv2.imwrite(image_filename, frame)
    print(f"📸 Image captured and saved as '{image_filename}'.")
    return frame

def detect_objects(frame):
    """Run YOLO detection and return detected class & confidence."""
    results = model(frame)
    detected_objects = []

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            confidence = box.conf[0].item()
            detected_class = model.names[class_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            color = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"{detected_class} {confidence:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            print(f"✅ Detected: {detected_class} | Confidence: {confidence:.2f}")

            if confidence > 0.5:  # Consider object detected if confidence > 50%
                detected_objects.append(detected_class)

    result_filename = "detection_result.jpg"
    cv2.imwrite(result_filename, frame)
    print(f"📂 Detection result saved as '{result_filename}'.")
    return detected_objects

def main():
    """Main loop to rotate plate between 0° and 90°, detect objects, and activate robotic hand."""
    for angle in rotation_angles:
        print(f"\n🔄 --- Moving Plate to {angle}° ---")
        send_command(f"ROTATE:{angle}")  # Command to rotate plate
        time.sleep(3)  # Wait for plate to move

        print("\n📸 --- Capturing Image ---")
        frame = capture_image()
        if frame is None:
            continue  # Skip to next iteration if image capture fails

        print("\n🔍 --- Running Object Detection ---")
        detected_objects = detect_objects(frame)

        if detected_objects:
            print("\n🤖 --- Activating Robotic Hand ---")
            send_command("MOVE")  # Trigger robotic hand
            time.sleep(3)  # Allow time for hand movement

        print("\n🔄 --- Moving Plate to Next Position ---")
        send_command("NEXT_POSITION")
        time.sleep(2)  # Allow plate to move

    print("✅ Process Completed Successfully!")

if _name_ == "_main_":
    main()