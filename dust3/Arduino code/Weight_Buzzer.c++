#include "HX711.h"
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#define DT_NON_BIO     A1
#define SCK_NON_BIO    A0
#define DT_BIO         A3
#define SCK_BIO        A2
#define DT_COMMON      A5
#define SCK_COMMON     A4

const float MAX_CAPACITY = 300.0;
const float ALERT_THRESHOLD = 75.0; // 75% threshold

HX711 scaleNonBio, scaleBio, scaleCommon;

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

void setup() {
    Serial.begin(115200);
    mySoftwareSerial.begin(9600);

    if (!myDFPlayer.begin(mySoftwareSerial, false)) {
        Serial.println(F("DFPlayer initialization failed!"));
        while (true) { delay(0); }
    }
    myDFPlayer.volume(30);
    
    scaleNonBio.begin(DT_NON_BIO, SCK_NON_BIO);
    scaleBio.begin(DT_BIO, SCK_BIO);
    scaleCommon.begin(DT_COMMON, SCK_COMMON);

    // Adjust scale factors for better accuracy
    scaleNonBio.set_scale(150.f);
    scaleBio.set_scale(150.f);
    scaleCommon.set_scale(150.f);

    scaleNonBio.tare();
    scaleBio.tare();
    scaleCommon.tare();

    Serial.println("All Weight Sensors Initialized");
}

void loop() {
    if (!scaleNonBio.is_ready()) {
        Serial.println("Non-Biodegradable HX711 not ready!");
        return;
    }
    if (!scaleBio.is_ready()) {
        Serial.println("Biodegradable HX711 not ready!");
        return;
    }
    if (!scaleCommon.is_ready()) {
        Serial.println("Common HX711 not ready!");
        return;
    }

    // Get raw readings
    long nonBioWeight = scaleNonBio.get_units(10);
    long bioWeight = scaleBio.get_units(10);
    long commonWeight = scaleCommon.get_units(10);
    long liquidWeight = 50; // Dummy data for liquid weight

    // Debug raw values
    Serial.print("Raw Non-Bio Weight: "); Serial.println(nonBioWeight);
    Serial.print("Raw Bio Weight: "); Serial.println(bioWeight);
    Serial.print("Raw Common Weight: "); Serial.println(commonWeight);

    // Calculate percentage values
    float nonBioPercentage = (nonBioWeight / MAX_CAPACITY) * 100.0;
    float bioPercentage = (bioWeight / MAX_CAPACITY) * 100.0;
    float commonPercentage = (commonWeight / MAX_CAPACITY) * 100.0;
    float liquidPercentage = (liquidWeight / MAX_CAPACITY) * 100.0;

    // Print formatted values
    Serial.print("Non-Biodegradable: "); Serial.print(nonBioPercentage, 2); Serial.print("% | ");
    Serial.print("Biodegradable: "); Serial.print(bioPercentage, 2); Serial.print("% | ");
    Serial.print("Common: "); Serial.print(commonPercentage, 2); Serial.print("% | ");
    Serial.print("Liquid: "); Serial.print(liquidPercentage, 2); Serial.println("%");

    // Alert if any bin is full
    if (bioPercentage >= ALERT_THRESHOLD) {
        Serial.println("Biodegradable waste full! Playing alert audio 001...");
        myDFPlayer.play(1);
    } else if (nonBioPercentage >= ALERT_THRESHOLD) {
        Serial.println("Non-Biodegradable waste full! Playing alert audio 002...");
        myDFPlayer.play(2);
    } else if (commonPercentage >= ALERT_THRESHOLD) {
        Serial.println("Common waste full! Playing alert audio 003...");
        myDFPlayer.play(3);
    }

    delay(1000);
}
