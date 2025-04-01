#ifndef VL53L5CX_INTERFACE_H
#define VL53L5CX_INTERFACE_H

#include <Wire.h>
#include <sensor_msgs/msg/range.h>

// Make sure we handle the C/C++ linkage properly
#ifdef __cplusplus
extern "C" {
#endif
#include "vl53l5cx_api.h"
#ifdef __cplusplus
}
#endif

class VL53L5CXInterface {
public:
  VL53L5CXInterface(uint8_t address = 0x29, uint8_t lpn_pin = 0);
  
  /**
   * Initialize the VL53L5CX sensor
   */
  void init();
  
  /**
   * Update sensor data - call this periodically
   */
  void update();
  
  /**
   * Get data in ROS message format for publishing
   * @param zone_idx Index of the zone (0-15 for 4x4 grid, 0-63 for 8x8 grid)
   * @return Range message with distance data
   */
  sensor_msgs__msg__Range getRangeData(uint8_t zone_idx = 0);
  
  /**
   * Get target status for a specific zone
   * @param zone_idx Index of the zone
   * @return Target status code (5 or 9 means valid data)
   */
  uint8_t getTargetStatus(uint8_t zone_idx = 0);
  
  /**
   * Get raw distance measurement for a specific zone
   * @param zone_idx Index of the zone
   * @return Distance in millimeters
   */
  int16_t getDistanceMm(uint8_t zone_idx = 0);
  
  /**
   * Get the resolution of the sensor (16 for 4x4, 64 for 8x8)
   * @return Resolution (number of zones)
   */
  uint8_t getResolution();
  
  /**
   * Get the number of zones available
   * @return Number of zones (16 for 4x4 grid, 64 for 8x8 grid)
   */
  uint8_t getZoneCount();

private:
  VL53L5CX_Configuration _dev;      // Sensor configuration
  VL53L5CX_ResultsData _results;    // Results data from sensor
  uint8_t _lpn_pin;                 // Pin used for sensor reset (Low Power Nin)
  uint8_t _resolution;              // Current sensor resolution
  uint8_t _address;                 // I2C address
  bool _data_ready;                 // Flag indicating if new data is available
};

#endif  // VL53L5CX_INTERFACE_H