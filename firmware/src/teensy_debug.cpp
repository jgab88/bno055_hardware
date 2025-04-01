#include <Arduino.h>
#include <Wire.h>

// Include the VL53L5CX API header - wrap with extern "C" for C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif
#include "vl53l5cx_api.h"
#ifdef __cplusplus
}
#endif

// Settings
#define TOF_I2C_ADDR 0x29 // Default I2C address for VL53L5CX
#define TOF_LPN_PIN  14   // LPN pin for reset, set to 0 if not connected

// Global variables
VL53L5CX_Configuration Dev;
VL53L5CX_ResultsData Results;

// Platform function prototypes
extern "C" {
  uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
  uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
  uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
  uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
  uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs);
  void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size);
}

void scanI2C(TwoWire &wire) {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning I2C bus...");
  
  for(address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    error = wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  
  // Blink LED to show we're starting
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  // Wait for serial connection to be established
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 3000)) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
  
  Serial.println("\nVL53L5CX TOF Sensor Test");
  Serial.println("========================");
  
  // Scan I2C buses to see what's connected
  Serial.println("\nScanning Wire...");
  Wire.begin();
  Wire.setClock(100000); // 100kHz
  scanI2C(Wire);
  
  Serial.println("\nScanning Wire1...");
  Wire1.begin();
  Wire1.setClock(100000); // 100kHz
  scanI2C(Wire1);
  
  // Try different configurations for finding the sensor
  Serial.println("\nTrying different configurations to find VL53L5CX...");

  // Try Wire at 0x29
  Serial.print("Trying Wire at address 0x29: ");
  Wire.beginTransmission(0x29);
  uint8_t error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("Device found!");
  } else {
    Serial.print("Not found, error: ");
    Serial.println(error);
  }

  // Try Wire at 0x28
  Serial.print("Trying Wire at address 0x28: ");
  Wire.beginTransmission(0x28);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("Device found!");
  } else {
    Serial.print("Not found, error: ");
    Serial.println(error);
  }

  // Try Wire1 at 0x29
  Serial.print("Trying Wire1 at address 0x29: ");
  Wire1.beginTransmission(0x29);
  error = Wire1.endTransmission();
  if (error == 0) {
    Serial.println("Device found!");
  } else {
    Serial.print("Not found, error: ");
    Serial.println(error);
  }

  // Try Wire1 at 0x28
  Serial.print("Trying Wire1 at address 0x28: ");
  Wire1.beginTransmission(0x28);
  error = Wire1.endTransmission();
  if (error == 0) {
    Serial.println("Device found!");
  } else {
    Serial.print("Not found, error: ");
    Serial.println(error);
  }
  
  // Reset sensor if LPN pin is defined
  if (TOF_LPN_PIN > 0) {
    Serial.println("\nResetting VL53L5CX sensor via LPN pin...");
    pinMode(TOF_LPN_PIN, OUTPUT);
    digitalWrite(TOF_LPN_PIN, LOW);
    delay(100);
    digitalWrite(TOF_LPN_PIN, HIGH);
    delay(200);
  }
  
  // Initialize sensor platform with I2C address
  Serial.println("\nInitializing VL53L5CX sensor...");
  Dev.platform.address = TOF_I2C_ADDR << 1; // ST driver uses 8-bit address format
  
  // Check if sensor is alive
  uint8_t isAlive = 0;
  uint8_t status = vl53l5cx_is_alive(&Dev, &isAlive);
  
  if (status) {
    Serial.print("Error checking if sensor is alive. Status: ");
    Serial.println(status);
    
    // Try with the device at 0x28 on Wire
    Serial.println("\nTrying with device at 0x28 on Wire...");
    Dev.platform.address = 0x28 << 1;
    status = vl53l5cx_is_alive(&Dev, &isAlive);
    
    if (status) {
      Serial.print("Error checking if sensor at 0x28 is alive. Status: ");
      Serial.println(status);
    } else if (!isAlive) {
      Serial.println("Sensor at 0x28 is not responding!");
    } else {
      Serial.println("Sensor at 0x28 is alive!");
    }
  } else if (!isAlive) {
    Serial.println("Sensor is not responding!");
    
    // Try with the device at 0x28 on Wire
    Serial.println("\nTrying with device at 0x28 on Wire...");
    Dev.platform.address = 0x28 << 1;
    status = vl53l5cx_is_alive(&Dev, &isAlive);
    
    if (status) {
      Serial.print("Error checking if sensor at 0x28 is alive. Status: ");
      Serial.println(status);
    } else if (!isAlive) {
      Serial.println("Sensor at 0x28 is not responding!");
    } else {
      Serial.println("Sensor at 0x28 is alive!");
    }
  } else {
    Serial.println("Sensor is alive!");
    
    // Initialize sensor
    Serial.println("Initializing VL53L5CX...");
    status = vl53l5cx_init(&Dev);
    
    if (status) {
      Serial.print("Initialization failed! Status: ");
      Serial.println(status);
    } else {
      Serial.print("VL53L5CX initialized! Version: ");
      Serial.println(VL53L5CX_API_REVISION);
      
      // Configure sensor
      Serial.println("Configuring sensor...");
      
      // Set resolution to 4x4
      status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
      if (status) {
        Serial.print("Failed to set resolution. Status: ");
        Serial.println(status);
      } else {
        Serial.println("Resolution set to 4x4");
      }
      
      // Set ranging frequency to 5Hz
      status = vl53l5cx_set_ranging_frequency_hz(&Dev, 5);
      if (status) {
        Serial.print("Failed to set ranging frequency. Status: ");
        Serial.println(status);
      } else {
        Serial.println("Ranging frequency set to 5Hz");
      }
      
      // Start ranging
      Serial.println("Starting ranging...");
      status = vl53l5cx_start_ranging(&Dev);
      
      if (status) {
        Serial.print("Failed to start ranging. Status: ");
        Serial.println(status);
      } else {
        Serial.println("Ranging started successfully!");
      }
    }
  }
  
  Serial.println("\nSetup complete!");
}

void loop() {
  // Blink LED to show we're running
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Check if new data is ready
  uint8_t isReady = 0;
  uint8_t status = vl53l5cx_check_data_ready(&Dev, &isReady);
  
  if (status == 0 && isReady) {
    // Data is ready, read it
    status = vl53l5cx_get_ranging_data(&Dev, &Results);
    
    if (status == 0) {
      // Get timestamp
      unsigned long timestamp = millis();
      
      // Print header
      Serial.print("\n===== Frame ");
      Serial.print(Dev.streamcount);
      Serial.print(" @ ");
      Serial.print(timestamp);
      Serial.println("ms =====");
      
      // Get resolution
      uint8_t resolution;
      vl53l5cx_get_resolution(&Dev, &resolution);
      
      // Print data for each zone
      for (uint8_t i = 0; i < resolution; i++) {
        Serial.print("Zone ");
        Serial.print(i);
        Serial.print(": Status=");
        Serial.print(Results.target_status[i]);
        Serial.print(", Distance=");
        Serial.print(Results.distance_mm[i]);
        Serial.println(" mm");
      }
    } else {
      Serial.print("Error getting ranging data. Status: ");
      Serial.println(status);
    }
  }
  
  // Small delay to avoid CPU hogging
  delay(10);
}

// Platform function implementations
uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {
  Wire.beginTransmission((uint8_t)((p_platform->address) >> 1));
  Wire.write((uint8_t)(RegisterAdress >> 8));
  Wire.write((uint8_t)(RegisterAdress & 0xFF));
  uint8_t status = Wire.endTransmission(false);
  
  if (status != 0) {
    return status;
  }
  
  // Use explicit casts to avoid ambiguity
  Wire.requestFrom((uint8_t)((p_platform->address) >> 1), (uint8_t)1);
  
  if (Wire.available()) {
    *p_value = Wire.read();
  } else {
    return 1; // Error - no data available
  }
  
  return 0;
}

uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {
  Wire.beginTransmission((uint8_t)((p_platform->address) >> 1));
  Wire.write((uint8_t)(RegisterAdress >> 8));
  Wire.write((uint8_t)(RegisterAdress & 0xFF));
  Wire.write(value);
  return Wire.endTransmission();
}

uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
  uint8_t status = 0;
  uint32_t i = 0;
  
  while (i < size && status == 0) {
    uint32_t chunk_size = size - i;
    if (chunk_size > 32) chunk_size = 32; // I2C buffer size limit for Teensy
    
    Wire.beginTransmission((uint8_t)((p_platform->address) >> 1));
    Wire.write((uint8_t)(RegisterAdress >> 8));
    Wire.write((uint8_t)(RegisterAdress & 0xFF));
    
    for (uint32_t j = 0; j < chunk_size; j++) {
      Wire.write(p_values[i + j]);
    }
    
    status = Wire.endTransmission(true);
    i += chunk_size;
    RegisterAdress += chunk_size;
  }
  
  return status;
}

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
  uint8_t status = 0;
  uint32_t i = 0;
  
  Wire.beginTransmission((uint8_t)((p_platform->address) >> 1));
  Wire.write((uint8_t)(RegisterAdress >> 8));
  Wire.write((uint8_t)(RegisterAdress & 0xFF));
  status = Wire.endTransmission(false);
  
  if (status != 0) {
    return status;
  }
  
  while (i < size) {
    uint32_t chunk_size = size - i;
    if (chunk_size > 32) chunk_size = 32; // I2C buffer size limit for Teensy
    
    // Use explicit cast to avoid ambiguity
    Wire.requestFrom((uint8_t)((p_platform->address) >> 1), (uint8_t)chunk_size);
    
    uint32_t j = 0;
    while (Wire.available() && j < chunk_size) {
      p_values[i + j] = Wire.read();
      j++;
    }
    
    if (j != chunk_size) {
      return 1; // Error - not enough data read
    }
    
    i += chunk_size;
  }
  
  return status;
}

uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs) {
  delay(TimeMs);
  return 0;
}

void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size) {
  uint32_t i, tmp;
  
  for(i = 0; i < size; i = i + 4) 
  {
    tmp = (
      (buffer[i] << 24) |
      (buffer[i+1] << 16) |
      (buffer[i+2] << 8) |
      (buffer[i+3])
    );
    
    memcpy(&(buffer[i]), &tmp, 4);
  }
}