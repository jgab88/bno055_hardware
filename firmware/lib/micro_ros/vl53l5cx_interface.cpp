#include "vl53l5cx_interface.h"

// Note: All the platform functions are implemented in vl53l5cx_platform.cpp

// Implement VL53L5CXInterface class
VL53L5CXInterface::VL53L5CXInterface(uint8_t address, uint8_t lpn_pin)
  : _lpn_pin(lpn_pin), _address(address), _data_ready(false)
{
  _dev.platform.address = address << 1; // ST driver uses 8-bit address format
}

void VL53L5CXInterface::init()
{
  // More robust reset sequence
  if (_lpn_pin > 0) {
    pinMode(_lpn_pin, OUTPUT);
    // Make sure the sensor starts from a clean state
    digitalWrite(_lpn_pin, HIGH);
    delay(10);
    digitalWrite(_lpn_pin, LOW);
    delay(100);  // Longer delay for complete power down
    digitalWrite(_lpn_pin, HIGH);
    delay(200);  // Longer delay to let the sensor boot up completely
  }
  
  // Check if the sensor is alive
  uint8_t isAlive = 0;
  uint8_t status = vl53l5cx_is_alive(&_dev, &isAlive);
  if (!isAlive || status) {
    // Try i2c scan to see what addresses respond on Wire1
    for(uint8_t addr = 0x08; addr <= 0x77; addr++) {
      Wire1.beginTransmission(addr);
      if (Wire1.endTransmission() == 0) {
        // Found a device at this address
        Serial.print("Found I2C device at address 0x");
        Serial.println(addr, HEX);
        
        // If we find a device at the expected address, but isAlive failed,
        // there might be an issue with the sensor specific protocol
        if (addr == (_address >> 1)) {
          Serial.println("Device found at VL53L5CX address but not responding to protocol");
        }
      }
    }
    
    Serial.print("VL53L5CX not detected at address 0x");
    Serial.println(_address, HEX);
    return;
  }
  
  Serial.println("VL53L5CX sensor found!");
  
  // Initialize sensor with more detailed error checking
  status = vl53l5cx_init(&_dev);
  if (status) {
    Serial.print("VL53L5CX initialization failed, status: ");
    Serial.println(status);
    return;
  }
  
  Serial.print("VL53L5CX ULD ready! (Version: ");
  Serial.print(VL53L5CX_API_REVISION);
  Serial.println(")");
  
  // Use a significant delay after initialization
  delay(1000);
  
  // Configure the sensor with more detailed error checking
  status = vl53l5cx_set_ranging_mode(&_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
  if (status) {
    Serial.print("Failed to set ranging mode, status: ");
    Serial.println(status);
    return;
  }
  
  // Try a lower frequency (5Hz instead of 10Hz) to give more time per reading
  status = vl53l5cx_set_ranging_frequency_hz(&_dev, 5);
  if (status) {
    Serial.print("Failed to set ranging frequency, status: ");
    Serial.println(status);
    return;
  }
  
  // Stick with 4x4 resolution for simplicity
  status = vl53l5cx_set_resolution(&_dev, VL53L5CX_RESOLUTION_4X4);
  if (status) {
    Serial.print("Failed to set resolution, status: ");
    Serial.println(status);
    return;
  }
  
  // Store resolution for later use
  status = vl53l5cx_get_resolution(&_dev, &_resolution);
  if (status) {
    Serial.print("Failed to get resolution, status: ");
    Serial.println(status);
    return;
  }
  
  // Additional configuration to try: set integration time explicitly
  status = vl53l5cx_set_integration_time_ms(&_dev, 20);
  if (status) {
    Serial.print("Failed to set integration time, status: ");
    Serial.println(status);
  }
  
  // Start ranging
  status = vl53l5cx_start_ranging(&_dev);
  if (status) {
    Serial.print("Failed to start ranging, status: ");
    Serial.println(status);
    return;
  }
  
  Serial.println("VL53L5CX ranging started");
  
  // Another delay to ensure ranging has started
  delay(200);
}

void VL53L5CXInterface::update()
{
  uint8_t isReady = 0;
  uint8_t status = vl53l5cx_check_data_ready(&_dev, &isReady);
  
  if (isReady) {
    status = vl53l5cx_get_ranging_data(&_dev, &_results);
    if (status == 0) {
      _data_ready = true;
      
      // Debug output - uncomment this if you can capture serial output
      /*
      Serial.println("VL53L5CX Data:");
      for (uint8_t i = 0; i < _resolution; i++) {
        Serial.print("Zone ");
        Serial.print(i);
        Serial.print(": Status=");
        Serial.print(_results.target_status[i]);
        Serial.print(", Distance=");
        Serial.print(_results.distance_mm[i]);
        Serial.println(" mm");
      }
      */
    } else {
      Serial.print("Error getting ranging data, status: ");
      Serial.println(status);
    }
  }
}

sensor_msgs__msg__Range VL53L5CXInterface::getRangeData(uint8_t zone_idx)
{
  sensor_msgs__msg__Range range_msg;
  
  // Basic ROS Range message setup
  range_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range_msg.field_of_view = 0.471239; // 27 degrees in radians for the VL53L5CX
  range_msg.min_range = 0.04f;        // 4cm minimum range
  range_msg.max_range = 4.0f;         // 4m maximum range
  
  // Center zone is zone 5 for 4x4 grid
  if (zone_idx >= getZoneCount()) {
    zone_idx = _resolution == 16 ? 5 : 28; // Default to center zone
  }
  
  // Only use valid measurements
  if (_data_ready) {
    uint8_t status = _results.target_status[zone_idx];
    int16_t distance = _results.distance_mm[zone_idx];
    
    // Target status values:
    // 5 = Ranging data valid
    // 9 = Valid data with large pulse
    // 0 = Supported by some firmware versions
    if ((status == 5 || status == 9 || status == 0) && distance > 0) {
      range_msg.range = distance / 1000.0f; // Convert mm to meters
    } else {
      // Try all zones to find any valid measurement
      for (uint8_t i = 0; i < getZoneCount(); i++) {
        status = _results.target_status[i];
        distance = _results.distance_mm[i];
        if ((status == 5 || status == 9 || status == 0) && distance > 0) {
          range_msg.range = distance / 1000.0f; // Convert mm to meters
          break;
        }
      }
      
      // If still no valid measurement
      if (range_msg.range <= 0.0f) {
        range_msg.range = 0.0f;
      }
    }
  } else {
    range_msg.range = 0.0f; // No data available
  }
  
  return range_msg;
}

uint8_t VL53L5CXInterface::getTargetStatus(uint8_t zone_idx)
{
  if (_data_ready && zone_idx < getZoneCount()) {
    return _results.target_status[zone_idx];
  }
  return 255; // Invalid
}

int16_t VL53L5CXInterface::getDistanceMm(uint8_t zone_idx)
{
  if (_data_ready && zone_idx < getZoneCount()) {
    return _results.distance_mm[zone_idx];
  }
  return 0;
}

uint8_t VL53L5CXInterface::getResolution()
{
  return _resolution;
}

uint8_t VL53L5CXInterface::getZoneCount()
{
  return _resolution;
}