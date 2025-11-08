/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-web-bluetooth/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <elapsedMillis.h>
#include <Wire.h>
#include "data_struct.h"

#define I2C_ADDR_THIS 0x42

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;


uint32_t value = 0;
const int builtInLED = 8;
const int buttonPin = 4;
const int RED_LED = 3;
const int YELLOW_LED = 2;
const int GREEN_LED = 1;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"


volatile bool I2CdataReceived = false;
StateData I2Cstatedata;
CommandData I2CTxCmd;
// int I2C_packet = 0;

// I2C Event Handlers
void I2CReceiveEvent(int numBytes) {
    if (numBytes < (int)(sizeof(StateData) + 1)) {
        // Not enough bytes; discard
        while (Wire.available()) Wire.read();
        return;
    }

    // Read sensor data byte by byte
    uint8_t *ptr = (uint8_t *)&I2Cstatedata;
    for (size_t i = 0; i < sizeof(StateData); i++) {
        if (Wire.available()) {
            ptr[i] = Wire.read();
        }
    }

    // Read checksum
    uint8_t receivedChecksum = 0;
    if (Wire.available()) {
        receivedChecksum = Wire.read();
    }

    // Flush any extra bytes
    while (Wire.available()) Wire.read();

    // Verify checksum
    uint8_t calcChecksum = computeChecksum(I2Cstatedata);
    if (receivedChecksum == calcChecksum) {
        I2CdataReceived = true;
    } else {
        Serial.print("Checksum mismatch: got ");
        Serial.print(receivedChecksum);
        Serial.print(", expected ");
        Serial.println(calcChecksum);
    }
}


void I2CRequestEvent() {
  return;
    Wire.write((uint8_t *)&I2CTxCmd, sizeof(CommandData));
    Wire.write(computeChecksum(I2CTxCmd));
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    std::string value = pLedCharacteristic->getValue();
    if (value.length() >= 8) { // Check if we received at least 8 bytes (2 float values)
      // Extract the two float values from the received data
      float value1, value2;
      memcpy(&value1, value.data(), sizeof(float));
      memcpy(&value2, value.data() + sizeof(float), sizeof(float));
      
      Serial.print("Received values - Value1: ");
      Serial.print(value1);
      Serial.print(", Value2: ");
      Serial.println(value2);

      // Here you can process the received float values
      // For example, you could use them to set power limits or other parameters
      if (value1 >= 0 && value1 <= 255) {
        I2CTxCmd.setMaxPower = static_cast<uint8_t>(value1);
      }
    }
  }
};

void BLESetup(){
  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
                      SENSOR_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
                      LED_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");
  delay(1000);
  pinMode(builtInLED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);
    
  // I2C Setup
  Wire.begin((int)I2C_ADDR_THIS);
  Wire.onReceive(I2CReceiveEvent);
  Wire.onRequest(I2CRequestEvent);

  BLESetup();
}

void BLELoop() {
  // notify changed value
  if (deviceConnected) {
    // Create a buffer to hold the three float values (12 bytes total)
    uint8_t buffer[12];
    
    // For debugging
    Serial.println("Current state values:");
    Serial.print("Voltage: "); Serial.print(I2Cstatedata.batteryVoltage);
    Serial.print(" Current: "); Serial.print(I2Cstatedata.batteryCurrent);
    Serial.print(" Power: "); Serial.println(I2Cstatedata.batteryPower);
    
    // Copy the float values from the StateData struct
    float *voltagePtr = (float *)&buffer[0];
    float *currentPtr = (float *)&buffer[4];
    float *powerPtr = (float *)&buffer[8];
    
    *voltagePtr = I2Cstatedata.batteryVoltage;
    *currentPtr = I2Cstatedata.batteryCurrent;
    *powerPtr = I2Cstatedata.batteryPower;
    
    // Set the value and notify
    pSensorCharacteristic->setValue(buffer, sizeof(buffer));
    pSensorCharacteristic->notify();
    
    Serial.print("Notified values - Voltage: ");
    Serial.print(I2Cstatedata.batteryVoltage);
    Serial.print("V, Current: ");
    Serial.print(I2Cstatedata.batteryCurrent);
    Serial.print("A, Power: ");
    Serial.print(I2Cstatedata.batteryPower);
    Serial.println("W");
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}

void I2CLoop(){
  static unsigned long lastI2CRxTime = 0;
  if(millis() - lastI2CRxTime > 5000) {
    lastI2CRxTime = millis();
    Serial.println("No I2C data received in last 5 seconds");
  }

  if(I2CdataReceived) {
    lastI2CRxTime = millis();
    
    // The BLELoop will handle sending the data when I2CdataReceived is true
    // Reset the flag after BLELoop has processed it
    // I2CdataReceived = false;
  }
}

void checkButton(){

  if(digitalRead(buttonPin) == LOW){

    // Output LEDs to indicate battery voltage level
    // > 12.5V : Green, 11.5-12.5V : Yellow, <11.5V : Red
    if(I2Cstatedata.batteryVoltage > 12.5){
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
    } else if(I2Cstatedata.batteryVoltage > 11.5){
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
    } else {
      digitalWrite(RED_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
    }
  } else {
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
  }

}

void loop() {
  static elapsedMillis BLETimer = 0;
  if(BLETimer >= 500 && I2CdataReceived){
    I2CdataReceived = false;
    BLETimer = 0;
    BLELoop();
  }
  
  // Check for I2C updates
  I2CLoop();

  static elapsedMillis buttonTimer = 0;
  if(buttonTimer >= 100){
    buttonTimer = 0;
    checkButton();
  }

  // static elapsedMillis debugTimer = 0;
  // if(debugTimer >= 1000){
  //   debugTimer = 0;
  //   Serial.print("Button state: ");
  //   Serial.println(digitalRead(buttonPin));
  //   digitalWrite(builtInLED, digitalRead(buttonPin) == LOW ? HIGH : LOW);
  // }


}