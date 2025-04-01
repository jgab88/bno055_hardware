#include <Arduino.h>
#include <Wire.h>
#include "vl53l5cx_api.h"
#include "vl53l5cx_interface.h"


// Define the VL53L5CX I2C address
#define VL53L5CX_ADDR (0x29)

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial port to connect
  
  delay(1000);
  Serial.println("VL53L5CX Debug Tool");
  Serial.println("-------------------");
  
  // Setup I2C buses
  Wire.begin();  // SCL:19, SDA:18
  Wire.setClock(100000);  // 100kHz
  
  Wire1.begin(); // SCL2:16, SDA2:17
  Wire1.setClock(100000); // 100kHz
  
  // I2C Scanner
  Serial.println("Scanning I2C bus 0 (Wire)...");
  scanI2C(Wire);
  
  Serial.println("Scanning I2C bus 1 (Wire1)...");
  scanI2C(Wire1);
  
  // Try to communicate with VL53L5CX on both buses
  Serial.println("Testing VL53L5CX communication on Wire...");
  testVL53L5CX(Wire);
  
  Serial.println("Testing VL53L5CX communication on Wire1...");
  testVL53L5CX(Wire1);
}

void loop() {
  // Nothing to do in the loop
  delay(1000);
}

void scanI2C(TwoWire &wire) {
  byte error, address;
  int nDevices = 0;

  for(address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    error = wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("Scan complete");
}

void testVL53L5CX(TwoWire &wire) {
  // Try to read the model ID and revision ID registers (common for most ST sensors)
  wire.beginTransmission(VL53L5CX_ADDR);
  wire.write(0x00); // Model ID register
  wire.endTransmission(false);
  
  uint8_t available = wire.requestFrom(VL53L5CX_ADDR, 2); // Request 2 bytes
  
  if (available == 2) {
    uint8_t model_id = wire.read();
    uint8_t revision_id = wire.read();
    
    Serial.print("VL53L5CX Model ID: 0x");
    Serial.print(model_id, HEX);
    Serial.print(", Revision ID: 0x");
    Serial.println(revision_id, HEX);
    
    if (model_id == 0xF0 && revision_id == 0x02) {
      Serial.println("VL53L5CX Identified correctly!");
    } else {
      Serial.println("Device responded but doesn't match expected VL53L5CX ID");
    }
  } else {
    Serial.print("Device did not respond. Requested 2 bytes, received ");
    Serial.println(available);
  }
}