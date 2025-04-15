import cv2
import serial
import time
import requests
import numpy as np
from ultralytics import YOLO
import logging

# Configure logging for debugging and info messages
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Maximum allowed bounding box area to filter out overly large detections
MAX_HEIGHT = 400
MAX_WIDTH = 400

# Initialize YOLO models
model_poly = YOLO(r"poly_non_poly.pt")  # YOLO model for polythene detection
model_biogas = YOLO(r"biogas.pt")         # YOLO model for biodegradable classification
logging.info("YOLO models have been loaded successfully.")

# Initialize Serial Communication with Arduino
try:
    arduino = serial.Serial(port="COM10", baudrate=9600, timeout=1)  # Change port as needed
    time.sleep(2)  # Wait for the serial connection to initialize
    logging.info("Arduino connected on COM10.")
except Exception as e:
    logging.error(f"Error connecting to Arduino: {e}")
    exit(1)

# IP address for the ESP8266 module
ESP8266_IP = "192.168.1.17"

# Mapping from detected class to flag values for the ESP8266/Arduino system (keys in lowercase)
class_to_flag = {
    'non-biodegradable': 1,
    'biodegradable': 2,
    'common': 3,
    'nonbiogasready': 5,
    'biogasready': 4
}

# Define the region of interest (ROI) as a polygon (modify points as needed)
region_pts = np.array([[150, 100], [100, 400], [450, 400], [450, 150]], np.int32)
region_pts = region_pts.reshape((-1, 1, 2))

def send_command(cmd):
    """Send a command to Arduino and log the response."""
    try:
        arduino.write((cmd + "\n").encode())
        time.sleep(1)
        response = arduino.readline().decode().strip()
        print(f"Arduino Response: {response}")
        logging.info(f"Sent command '{cmd}' to Arduino, received response: {response}")
    except Exception as e:
        logging.error(f"Error sending command '{cmd}' to Arduino: {e}")

def send_flag(flag):
    """Send a flag value to ESP8266 via HTTP GET request."""
    url = f"http://{ESP8266_IP}/flag?value={flag}"
    try:
        response = requests.get(url, timeout=5)
        print(f"ESP8266 Response: {response.text}")
        logging.info(f"Flag '{flag}' sent to ESP8266, received response: {response.text}")
    except Exception as e:
        print(f"Error communicating with ESP8266: {e}")
        logging.error(f"Error sending flag '{flag}' to ESP8266: {e}")

def capture_frame(cam_url):
    """Capture a frame from an ESP32-CAM without drawing a bounding box."""
    cap = cv2.VideoCapture(cam_url)
    time.sleep(2)  # Allow the camera stream to initialize
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        logging.error(f"Failed to capture frame from {cam_url}")
        return None
    
    # Save the image temporarily for model input if needed
    cv2.imwrite("temp.jpg", frame)
    logging.info(f"Captured frame from {cam_url} without annotation.")
    return frame

def capture_frame_with_box(cam_url):
    """Capture a frame from an ESP32-CAM and draw the ROI bounding box."""
    cap = cv2.VideoCapture(cam_url)
    time.sleep(2)
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        logging.error(f"Failed to capture frame from {cam_url}")
        return None
    
    cv2.polylines(frame, [region_pts], isClosed=True, color=(0, 255, 255), thickness=2)
    cv2.imwrite("temp.jpg", frame)
    logging.info(f"Captured frame from {cam_url} with ROI annotation.")
    return frame

def detect_polythene(image_path):
    """Detect polythene objects using the YOLO model."""
    results = model_poly(image_path)
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            detected_class = model_poly.names[class_id].lower()  # Normalize to lowercase
            if detected_class == "polythene":
                return True
    return False

def process_first_layer(cam_1):
    """
    Process the first layer by detecting polythene.
    If detected, activate cutter and hand pressure, then open the plate.
    Otherwise, only open the plate.
    """
    frame = capture_frame(cam_1)
    if frame is None:
        return
    
    polythene_detected = detect_polythene("temp.jpg")
    print('First camera processing complete.')
    
    if polythene_detected:
        print("Polythene detected! Activating cutter and hand pressure, then opening plate.")
        send_command('H')
        time.sleep(0.5)
        send_command('C')
        time.sleep(0.5)
        send_command('O')
    else:
        print("No polythene detected! Opening plate only.")
        send_command('O')

def detect_biodegradable(frame):
    """
    Detect objects in a specific region using the biodegradable YOLO model,
    annotate the frame, and count objects inside the defined ROI.
    """
    results = model_biogas(frame)
    detected_classes = []
    region_count = 0
    
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            detected_class = model_biogas.names[class_id].lower()  # Normalize to lowercase
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Filter out bounding boxes that exceed a given area
            if (x2 - x1) * (y2 - y1) > MAX_HEIGHT * MAX_WIDTH:
                logging.info(f"Ignored large bounding box: {(x1, y1, x2, y2)}")
                continue

            # Check if the center point is inside the ROI
            if cv2.pointPolygonTest(region_pts, (center_x, center_y), False) >= 0:
                detected_classes.append(detected_class)
                region_count += 1
                # Use green for biogasready, red for nonbiogasready, blue otherwise.
                if detected_class == "biogasready":
                    color = (0, 255, 0)
                elif detected_class == "nonbiogasready":
                    color = (0, 0, 255)
                else:
                    color = (255, 0, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, detected_class, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                
    print(f"Total objects detected in region: {region_count}")
    
    # If no objects are detected in the ROI, move the plate to the next position
    if region_count == 0:
        send_command("MOVE")
        print("No objects detected in region, moving plate to next position.")
    
    return detected_classes, region_count

def process_second_third_layer(cam_2):
    """
    For each cycle, move the plate, capture a frame with ROI, detect objects,
    then decide the appropriate action based on the classification.
    """
    # Number of cycles can be adjusted as needed
    for _ in range(2):
        print("Starting new detection cycle...")
        time.sleep(3)  # Delay for system stabilization
        send_command("MOVE")
        time.sleep(2)
        
        frame = capture_frame_with_box(cam_2)
        if frame is None:
            continue

        detected_classes, region_count = detect_biodegradable(frame)

        if region_count == 0:
            print("No objects detected, moving plate to next position.")
            time.sleep(3)
            send_command("MOVE")
        elif region_count >= 2:
            bio_count = detected_classes.count("biogasready")
            non_bio_count = detected_classes.count("nonbiogasready")
            
            if bio_count == region_count:
                print("All detected objects are biodegradable, activating hand to bio bucket.")
                send_flag(class_to_flag["biogasready"])
                time.sleep(3)
                send_command("STOP")
            elif non_bio_count == region_count:
                print("All detected objects are non-biodegradable, activating hand to non-bio bucket.")
                send_flag(class_to_flag["nonbiogasready"])
                time.sleep(3)
                send_command("STOP")
            elif bio_count == True and non_bio_count == True:
                print("Both objects are present in region, rotating motor at full speed.")
                send_command("MOVE_fullspeed")
            else:
                print("Mixed objects detected, moving plate quickly.")
                send_command("MOVE")
        else:
            # For a single detected object, send the corresponding flag
            detected_key = detected_classes[0].replace(" ", "").lower()  # Remove spaces and convert to lowercase
            flag_value = class_to_flag.get(detected_key)
            if flag_value is not None:
                print(f"Detected '{detected_classes[0]}', sending flag {flag_value}.")
                send_flag(flag_value)
                time.sleep(3)
                send_command("STOP")
            # elif bio_count == True and non_bio_count == True:
            #     print("Both objects are present in region, rotating motor at full speed.")
            #     send_command("MOVE")    
            else:
                print(f"Detected '{detected_classes[0]}' but no flag mapping found.")
            time.sleep(3)
            # send_command("MOVE")
        
        print("✅ Process cycle completed successfully!")
        # (Optional) Uncomment the following to display the annotated frame for debugging:
        # cv2.imshow("Detection", frame)
        # cv2.waitKey(1)
    # cv2.destroyAllWindows()
send_command("STOP_NetMotor")

def main():
    # Update these camera URLs as required for your network
    cam_1 = "http://192.168.1.103/cam-hi.jpg"
    cam_2 = "http://192.168.1.104/cam-hi.jpg"
    
    print("Starting Garbage Processing...")
    logging.info("Starting first layer processing.")
    process_first_layer(cam_1)
    time.sleep(2)
    logging.info("Starting second and third layer processing.")
    process_second_third_layer(cam_2)
    
if _name_ == "_main_":
    main()

# -------------------------
# #include <Servo.h>

# // ---------------------
# // Hardware Definitions
# // ---------------------

# // Hand Pressure Servo
# Servo handPressure;
# #define HAND_PRESSURE_PIN 6

# // Cutter Motor
# #define cutterMotorEnable 22
# #define cutterMotorInput1 24
# #define cutterMotorInput2 26

# // Opening Plate Motor
# #define openingPlateMotorEnable 32
# #define openingPlateMotorInput1 28
# #define openingPlateMotorInput2 30
# #define ENCODER_A 2

# // Servo Motors for Robotic Arm
# Servo servoBase;
# Servo servoElbow;

# // Gate Servo (for flag handling)
# Servo myServo;

# // Initial and target angles for the arm servos
# int baseStart = 80;
# int elbowStart = 180;
# int baseTarget = 0;
# int elbowTarget = -135;

# // Motor Driver for Net Motor (Plate Motor)
# int netMotor_IN1 = 7;
# int netMotor_IN2 = 8;
# int ena = 11;

# // Motor Driver for Flag Motor
# const int motorPin1 = 3;
# const int motorPin2 = 4;
# const int enablePin = 5;

# // Encoder Variables for Opening Plate Motor
# volatile int encoderCount = 0;
# int pulsesPerRotation = 100;
# int targetRotations = 2;

# // Motor Speed for Flag Motor
# int motorSpeed = 110;

# // ---------------------
# // Interrupt Service Routine for Encoder
# // ---------------------
# void countPulses() {
#     encoderCount++;
# }

# // ---------------------
# // Setup Function
# // ---------------------
# void setup() {
#     Serial.begin(9600);
#     Serial1.begin(9600); // Used for ESP8266 communication (if available)

#     // Attach Servos
#     handPressure.attach(HAND_PRESSURE_PIN);
#     handPressure.write(130);

#     servoBase.attach(10);
#     servoElbow.attach(9);
#     servoBase.write(baseStart);
#     servoElbow.write(elbowStart);

#     // Gate servo (for flag-related actions)
#     myServo.attach(52);
#     myServo.write(0);  // Initialize servo to 0 degrees
#     delay(1000);

#     // Setup Cutter Motor Pins
#     pinMode(cutterMotorEnable, OUTPUT);
#     pinMode(cutterMotorInput1, OUTPUT);
#     pinMode(cutterMotorInput2, OUTPUT);

#     // Setup Opening Plate Motor Pins
#     pinMode(openingPlateMotorEnable, OUTPUT);
#     pinMode(openingPlateMotorInput1, OUTPUT);
#     pinMode(openingPlateMotorInput2, OUTPUT);

#     // Setup Net Motor (Plate Motor) Pins
#     pinMode(netMotor_IN1, OUTPUT);
#     pinMode(netMotor_IN2, OUTPUT);
#     pinMode(ena, OUTPUT);
#     analogWrite(ena, 255); // Full speed

#     // Setup Flag Motor Pins
#     pinMode(motorPin1, OUTPUT);
#     pinMode(motorPin2, OUTPUT);
#     pinMode(enablePin, OUTPUT);

#     // Setup Encoder
#     pinMode(ENCODER_A, INPUT_PULLUP);
#     attachInterrupt(digitalPinToInterrupt(ENCODER_A), countPulses, RISING);

#     Serial.println("🚀 System Ready. Waiting for commands...");
# }

# // ---------------------
# // Main Loop
# // ---------------------
# void loop() {
#     if (Serial.available()) {
#         String command = Serial.readStringUntil('\n');
#         command.trim();

#         if (command == "H") {
#             activateHandPressure();
#         }
#         else if (command == "C") {
#             activateCutter();
#         }
#         else if (command == "O") {
#             // Rotate opening plate in one direction, then reverse after a delay.
#             rotateOpeningPlate(true, 150);
#             delay(2000);
#             rotateOpeningPlate(false, 150);
#         }
#         else if (command == "MOVE") {
#             Serial.println("Starting MOVE sequence...");
#             rotateNetFullSpeed();
#             for (int i = 0; i < 4; i++) {
#                 int flag = receiveFlagFromSerial1();
#                 // If no flag is received, use a default (3 = common/default)
#                 if (flag == 0) {
#                     flag = 3;
#                 }
#                 executeFlagAction(flag);
#                 // rotateNetHalfSpeed();
#             }
#         }
#         else if (command == "MOVE_fullspeed") {
#             finalStopPlate();
#             Serial.println("Rotating Motor at Full Speed.");
#         }
#         else if (command == "STOP_NetMotor") {
#             stopNetMotorPlate();
#             Serial.println("STOP command received. Net motor plate movement disabled.");
#         }
#         e   
#         }

#     }
# }

# // ---------------------
# // Command Functions
# // ---------------------

# // Activate hand pressure by moving the hand pressure servo.
# void activateHandPressure() {
#     Serial.println("Hand Pressure Activated");
#     handPressure.write(13);
#     delay(500);
# }

# // Activate cutter motor: run cutter for 2 seconds then reset.
# void activateCutter() {
#     Serial.println("Cutter Motor Activated");
#     digitalWrite(cutterMotorEnable, HIGH);
#     digitalWrite(cutterMotorInput1, HIGH);
#     digitalWrite(cutterMotorInput2, LOW);
#     delay(2000);
#     digitalWrite(cutterMotorEnable, LOW);
#     handPressure.write(130);
#     Serial.println("Cutter Motor Stopped");
# }

# // Rotate opening plate using encoder feedback with dynamic braking.
# void rotateOpeningPlate(bool clockwise, int durationMs) {
#     encoderCount = 0;
#     int targetPulses = pulsesPerRotation * targetRotations;
#     Serial.println(clockwise ? "Rotating Opening Plate Clockwise" : "Rotating Opening Plate Counterclockwise");

#     // Start motor in desired direction
#     digitalWrite(openingPlateMotorEnable, HIGH);
#     digitalWrite(openingPlateMotorInput1, (clockwise ? HIGH : LOW));
#     digitalWrite(openingPlateMotorInput2, (clockwise ? LOW : HIGH));

#     unsigned long startTime = millis();
#     while ((millis() - startTime) < durationMs && encoderCount < targetPulses) {
#         // Waiting until either duration elapses or target pulses reached
#     }

#     // Stop the motor
#     digitalWrite(openingPlateMotorEnable, LOW);
#     // Dynamic braking: briefly reverse motor polarity
#     digitalWrite(openingPlateMotorInput1, (clockwise ? LOW : HIGH));
#     digitalWrite(openingPlateMotorInput2, (clockwise ? HIGH : LOW));
#     delay(100); // Braking delay (adjust as needed)
#     // Ensure all motor pins are LOW to fully stop the motor
#     digitalWrite(openingPlateMotorInput1, LOW);
#     digitalWrite(openingPlateMotorInput2, LOW);

#     handPressure.write(130);
#     Serial.println("Opening Plate Stopped");
# }

# // ---------------------
# // Plate (Net Motor) Functions
# // ---------------------

# // Rotate plate at full speed for 2 seconds.
# void rotateNetFullSpeed() {
#     Serial.println("Rotating Plate at full speed...");
#     digitalWrite(netMotor_IN1, HIGH);
#     digitalWrite(netMotor_IN2, LOW);
#     delay(2000);
#     digitalWrite(netMotor_IN1, LOW);
#     digitalWrite(netMotor_IN2, LOW);
# }

# // Rotate plate at half speed: run briefly then pause.
# void rotateNetHalfSpeed() {
#     Serial.println("Rotating Plate at 50% speed...");
#     analogWrite(ena, 60); // Set motor to half speed
#     digitalWrite(netMotor_IN1, HIGH);
#     digitalWrite(netMotor_IN2, LOW);
#     delay(400);
#     digitalWrite(netMotor_IN1, LOW);
#     digitalWrite(netMotor_IN2, LOW);
#     delay(6000);
#     analogWrite(ena, 255); // Restore full speed
# }

# // Final function to stop all plate movement completely.
# void finalStopPlate() {
#     // Stop net motor outputs
#     digitalWrite(netMotor_IN1, LOW);
#     digitalWrite(netMotor_IN2, LOW);
#     analogWrite(ena, 0);
#     // Ensure opening plate motor outputs are LOW
#     digitalWrite(openingPlateMotorEnable, LOW);
#     digitalWrite(openingPlateMotorInput1, LOW);
#     digitalWrite(openingPlateMotorInput2, LOW);
# }

# // Function to stop only the net motor (plate motor)
# void stopNetMotorPlate() {
#     digitalWrite(netMotor_IN1, LOW);
#     digitalWrite(netMotor_IN2, LOW);
#     analogWrite(ena, 0); // Stops PWM signal so the motor doesn't run
# }

# // Function to stop only the net motor (plate motor)
# void fullstopNetMotorPlate() {
#     digitalWrite(netMotor_IN1, LOW);
#     digitalWrite(netMotor_IN2, LOW);
#     analogWrite(ena, 0); // Stops PWM signal so the motor doesn't run
# }


# // ---------------------
# // ESP8266 Flag Reception
# // ---------------------

# // Try to receive a flag from Serial1 with a 2-second timeout.
# int receiveFlagFromSerial1() {
#     unsigned long startTime = millis();
#     while (!Serial1.available() && (millis() - startTime < 2000)) {
#         // Waiting for flag data from ESP8266
#     }
#     if (Serial1.available()) {
#         String receivedData = Serial1.readStringUntil('\n');
#         receivedData.trim();
#         String flagStr = "";
#         for (char c : receivedData) {
#             if (isDigit(c))
#                 flagStr += c;
#         }
#         if (flagStr.length() > 0) {
#             return flagStr.toInt();
#         }
#     }
#     return 0; // No valid flag received
# }

# // ---------------------
# // Flag Action Functions
# // ---------------------

# // Execute an action based on the received flag.
# void executeFlagAction(int flag) {
#     if (flag == 5) {  // e.g., nonbiogasready
#         rotateLeft(motorSpeed);
#         delay(1400);
#         stopMotor();
#         myServo.write(145); // Adjust servo to 145 degrees
#         delay(500);
#         moveServosSync(baseTarget, elbowTarget, 5);
#         delay(500);
#         moveServosSync(baseStart, elbowStart, 5);
#         delay(500);
#         myServo.write(10);  // Reset servo position
#         rotateRight(motorSpeed);
#         delay(1600);
#         stopMotor();
#     } else if (flag == 4) {  // e.g., biogasready
#         rotateLeft(motorSpeed);
#         delay(600);
#         stopMotor();
#         myServo.write(145);
#         moveServosSync(baseTarget, elbowTarget, 5);
#         delay(500);
#         moveServosSync(baseStart, elbowStart, 5);
#         delay(500);
#         myServo.write(10);
#         delay(500);
#         rotateRight(motorSpeed);
#         delay(700);
#         stopMotor();
#     } else if (flag == 3) {
#         Serial.println("Default flag action (Common) executed.");
#         // You can add default behavior here if desired.
#     } else {
#         Serial.print("Unknown flag received: ");
#         Serial.println(flag);
#     }
# }

# // Synchronized movement for the robotic arm servos.
# void moveServosSync(int baseAngle, int elbowAngle, int speed) {
#     servoBase.write(baseAngle);
#     servoElbow.write(elbowAngle);
#     delay(1000);
# }

# // ---------------------
# // Flag Motor Control Functions
# // ---------------------

# void rotateRight(int speed) {
#     digitalWrite(motorPin1, HIGH);
#     digitalWrite(motorPin2, LOW);
#     analogWrite(enablePin, speed);
# }

# void rotateLeft(int speed) {
#     digitalWrite(motorPin1, LOW);
#     digitalWrite(motorPin2, HIGH);
#     analogWrite(enablePin, speed);
# }

# void stopMotor() {
#     digitalWrite(motorPin1, LOW);
#     digitalWrite(motorPin2, LOW);
#     analogWrite(enablePin, 0);
# }