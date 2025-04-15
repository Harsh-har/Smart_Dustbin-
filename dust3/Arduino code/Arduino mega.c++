
#include <Servo.h>

// Hand Pressure Servo
Servo handPressure;
#define HAND_PRESSURE_PIN 6

// Cutter Motor Pins
#define cutterMotorEnable 22
#define cutterMotorInput1 24
#define cutterMotorInput2 26

// Opening Plate Motor Pins
#define openingPlateMotorEnable 32
#define openingPlateMotorInput1 28
#define openingPlateMotorInput2 30
#define ENCODER_A 2

// Servo motors for robotic arm
Servo servoBase;
Servo servoElbow;
int baseStart = 90;
int elbowStart = 180;
int baseTarget = 8;
int elbowTarget = -135;

// Servo motor for gate
Servo myServo;

// Motor driver for Net Motor
int netMotor_IN1 = 7;
int netMotor_IN2 = 8;
int ena = 11;

// Motor driver for Flag Motor
const int motorPin1 = 3;
const int motorPin2 = 4;
const int enablePin = 5;

// Encoder Variables
volatile int encoderCount = 0;
int pulsesPerRotation = 100;
int targetRotations = 2;

// Motor Speed
int motorSpeed = 110;

// Encoder ISR Function
void countPulses() {
    encoderCount++;
}

void setup() {
    Serial.begin(9600);
    Serial.println("🚀 System Ready. Waiting for commands...");

    // Attach servos
    handPressure.attach(HAND_PRESSURE_PIN);
    handPressure.write(130);
    servoBase.attach(10);
    servoElbow.attach(9);
    servoBase.write(baseStart);
    servoElbow.write(elbowStart);

    // Gate servo
    myServo.attach(52); 
    myServo.write(0); 
    delay(1000); 

    // Setup Cutter Motor
    pinMode(cutterMotorEnable, OUTPUT);
    pinMode(cutterMotorInput1, OUTPUT);
    pinMode(cutterMotorInput2, OUTPUT);

    // Setup Opening Plate Motor
    pinMode(openingPlateMotorEnable, OUTPUT);
    pinMode(openingPlateMotorInput1, OUTPUT);
    pinMode(openingPlateMotorInput2, OUTPUT);

    // Setup Net Motor
    pinMode(netMotor_IN1, OUTPUT);
    pinMode(netMotor_IN2, OUTPUT);
    pinMode(ena, OUTPUT);
    digitalWrite(ena, HIGH);

    // Setup Flag Motor
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(enablePin, OUTPUT);

    // Setup Encoder
    pinMode(ENCODER_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), countPulses, RISING);

    
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
         Serial.print("Received Command: ");
         Serial.println(command);

        if (command == "H") {
            activateHandPressure();
            delay(500);
        } 
        else if (command == "C") {
            activateCutter();
            delay(1000);
        } 
        else if (command == "O") {
            // Perform clockwise rotation, then counterclockwise rotation, then send final confirmation.
            rotateOpeningPlate(true, 200);
            delay(1000);
            rotateOpeningPlate(false, 200);
            delay(1000);
            Serial.println("Plate Rotation Completed");
        } 
        else if (command == "MOVE_SINGLE") {
            Serial.println("MOVE_SINGLE WITH SPEED ...");
            rotateNetFullSpeed();
        }
      
        else if (command == "MOVE_MULTI") {
            Serial.println("MOVE_MULTI WITHOUT SPEED");
                     rotateNetHalfSpeed();
                    

        }
      
     else if (command == "FLAG_4") {
          
        executeFlagAction(4);
        }
      
        else if (command == "FLAG_5") {
            executeFlagAction(5);
            

        }
        
    }
}






void activateHandPressure() {
    handPressure.write(13);
    Serial.println("Hand Pressure Activated");
}

void activateCutter() {
    // Start cutter
    digitalWrite(cutterMotorEnable, HIGH);
    digitalWrite(cutterMotorInput1, HIGH);
    digitalWrite(cutterMotorInput2, LOW);
    delay(2000);  // Cutter active period
    
    // Stop cutter
    digitalWrite(cutterMotorEnable, LOW);
    delay(2000);  // Ensure complete stop

    // Send final confirmation message
    Serial.println("Cutter Complete");
}

void rotateOpeningPlate(bool clockwise, int durationMs) {
    encoderCount = 0;
    int targetPulses = pulsesPerRotation * targetRotations;
    
    // Print rotation direction
    Serial.println(clockwise ? "Rotating Opening Plate Clockwise" : "Rotating Opening Plate Counterclockwise");

    digitalWrite(openingPlateMotorEnable, HIGH);
    digitalWrite(openingPlateMotorInput1, clockwise ? HIGH : LOW);
    digitalWrite(openingPlateMotorInput2, clockwise ? LOW : HIGH);

    unsigned long startTime = millis();
    while ((millis() - startTime) < durationMs && encoderCount < targetPulses) {
        // Wait for the plate to rotate the required amount or for duration to expire
    }

    digitalWrite(openingPlateMotorEnable, LOW);
    handPressure.write(130);
    
    Serial.println("Opening Plate Stopped");
}

void rotateNetFullSpeed() {
    Serial.println("Rotating plate at full speed...");
    digitalWrite(netMotor_IN1, HIGH);
    digitalWrite(netMotor_IN2, LOW);
    delay(2000);
    digitalWrite(netMotor_IN1, LOW);
    digitalWrite(netMotor_IN2, LOW);
}

void rotateNetHalfSpeed() {
    Serial.println("Rotating plate at 50% speed...");
    analogWrite(ena, 70);
    digitalWrite(netMotor_IN1, HIGH);
    digitalWrite(netMotor_IN2, LOW);
    delay(400);
    digitalWrite(netMotor_IN1, LOW);
    digitalWrite(netMotor_IN2, LOW);
    delay(3000);
    analogWrite(ena, 170);
}


// Executing action based on flag
void executeFlagAction(int flag) {
    if (flag == 5) {
        rotateLeft(motorSpeed);
        delay(1710);
        stopMotor();
        myServo.write(145); // Move servo to 45 degrees
        delay(500);
        moveServosSync(baseTarget, elbowTarget, 5);
        delay(500);
        moveServosSync(baseStart, elbowStart, 5);
        delay(500);

        myServo.write(10);  // Move servo back to 0 degrees
    
        rotateRight(motorSpeed);
        delay(1730);
        stopMotor();
    } else if (flag == 4) {
        rotateLeft(motorSpeed);
        delay(640);
        stopMotor();
       
        myServo.write(145);  // Move servo back to 0 degrees

        moveServosSync(baseTarget, elbowTarget, 5);
        delay(500);
        moveServosSync(baseStart, elbowStart, 5);
        delay(500);

        myServo.write(10);  // Move servo back to 0 degrees

        delay(500);

        rotateRight(motorSpeed);
        delay(650);
        stopMotor();
    }
}

void moveServosSync(int baseAngle, int elbowAngle, int speed) {
    servoBase.write(baseAngle);
    servoElbow.write(elbowAngle);
    delay(1000);
}

void rotateRight(int speed) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, speed);
}

void rotateLeft(int speed) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(enablePin, speed);
}

void stopMotor() {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, 0);
}



