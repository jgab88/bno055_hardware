#ifndef BNO055_INTERFACE_H
#define BNO055_INTERFACE_H

#include <Adafruit_BNO055.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

class BNO055Interface {
public:
  BNO055Interface(uint8_t address = BNO055_ADDRESS_A);
  void init();
  void update();
  sensor_msgs__msg__Imu getIMUData();
  sensor_msgs__msg__MagneticField getMagneticFieldData();

private:
  Adafruit_BNO055 bno;
  sensors_event_t linearAccel;
  sensors_event_t angVelocity;
  sensors_event_t magneticField;
  imu::Quaternion quat;
};

#endif  // BNO055_INTERFACE_H