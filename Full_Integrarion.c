/*
  Full Integration:
  - Pitot tube (analog airspeed sensor)
  - Two HX711 load cells (force sensors)
  - SD card logging
  
  Logs data to "data.csv" in format:
  Time(ms),Airspeed(m/s),Force1,Force2
*/

#include <HX711_ADC.h>
#include <SD.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// --------------------- Pin Assignments ---------------------
const int airspeedPin = A0;      // Pitot tube analog signal
const int chipSelect = 10;       // SD card CS pin (use 53 for Mega)

// HX711 connections
const int HX711_dout_1 = 4;
const int HX711_sck_1  = 5;
const int HX711_dout_2 = 6;
const int HX711_sck_2  = 7;

// --------------------- Sensor Constants ---------------------
const float RHO = 1.204;          // Air density (kg/mÂ³)
const int OFFSET_SAMPLES = 10;    // For Pitot calibration
const int AVG_SAMPLES = 20;       // For Pitot averaging
const int ZERO_SPAN = 2;
const float PITOT_SCALE = 10000.0; // Pressure scale factor (tune experimentally)

// HX711 calibration values (tune per load cell)
const float CAL_1 = 696.0;
const float CAL_2 = 733.0;

// --------------------- Globals ---------------------
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1);
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2);
File myFile;
int pitotOffset = 0;
unsigned long t = 0;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Initializing system...");

  // ---------- Initialize SD card ----------
  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card ready.");

  // ---------- Initialize HX711 load cells ----------
  LoadCell_1.begin();
  LoadCell_2.begin();

  unsigned long stabilizingtime = 2000;
  bool _tare = true;
  byte rdy1 = 0, rdy2 = 0;

  while ((rdy1 + rdy2) < 2) {
    if (!rdy1) rdy1 = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!rdy2) rdy2 = LoadCell_2.startMultiple(stabilizingtime, _tare);
  }

  if (LoadCell_1.getTareTimeoutFlag()) Serial.println("HX711 #1 timeout!");
  if (LoadCell_2.getTareTimeoutFlag()) Serial.println("HX711 #2 timeout!");

  LoadCell_1.setCalFactor(CAL_1);
  LoadCell_2.setCalFactor(CAL_2);
  Serial.println("Load cells ready.");

  // ---------- Calibrate Pitot tube ----------
  Serial.println("Calibrating Pitot tube...");
  for (int i = 0; i < OFFSET_SAMPLES; i++) {
    pitotOffset += analogRead(airspeedPin) - (1023 / 2);
    delay(50);
  }
  pitotOffset /= OFFSET_SAMPLES;
  Serial.print("Pitot offset calibrated: ");
  Serial.println(pitotOffset);

  // ---------- Prepare SD file ----------
  myFile = SD.open("data.csv", FILE_WRITE);
  if (myFile) {
    myFile.println("Time(ms),Airspeed(m/s),Force1,Force2");
    myFile.close();
    Serial.println("Logging started: data.csv");
  } else {
    Serial.println("Error opening data.csv");
    while (1);
  }
}

// ------------------------------------------------------------
void loop() {
  static bool newDataReady = false;
  LoadCell_1.update();
  LoadCell_2.update();
  newDataReady = LoadCell_1.update();

  if (newDataReady) {
    // ----- Read airspeed -----
    float adc_avg = 0.0;
    for (int i = 0; i < AVG_SAMPLES; i++) {
      adc_avg += analogRead(airspeedPin) - pitotOffset;
    }
    adc_avg /= AVG_SAMPLES;

    float velocity = 0.0;
    if (adc_avg > 512 - ZERO_SPAN && adc_avg < 512 + ZERO_SPAN) {
      velocity = 0;
    } else {
      float pressure_ratio = (adc_avg / 1023.0) - 0.5;
      if (adc_avg < 512)
        velocity = -sqrt((-PITOT_SCALE * pressure_ratio) / RHO);
      else
        velocity = sqrt((PITOT_SCALE * pressure_ratio) / RHO);
    }

    // ----- Read forces -----
    float force1 = LoadCell_1.getData();
    float force2 = LoadCell_2.getData();

    // ----- Print to Serial -----
    Serial.print("Airspeed: "); Serial.print(velocity, 2); Serial.print(" m/s");
    Serial.print("\tForce1: "); Serial.print(force1, 2);
    Serial.print("\tForce2: "); Serial.println(force2, 2);

    // ----- Log to SD -----
    myFile = SD.open("data.csv", FILE_WRITE);
    if (myFile) {
      myFile.print(millis());
      myFile.print(",");
      myFile.print(velocity, 2);
      myFile.print(",");
      myFile.print(force1, 2);
      myFile.print(",");
      myFile.println(force2, 2);
      myFile.close();
    } else {
      Serial.println("SD write error!");
    }

    newDataReady = false;
    delay(200); // 5 Hz logging rate
  }

  // Optional: manual tare via serial
  if (Serial.available()) {
    char inByte = Serial.read();
    if (inByte == 't') {
      LoadCell_1.tareNoDelay();
      LoadCell_2.tareNoDelay();
    }
  }

  if (LoadCell_1.getTareStatus()) Serial.println("Tare load cell 1 complete");
  if (LoadCell_2.getTareStatus()) Serial.println("Tare load cell 2 complete");
}
