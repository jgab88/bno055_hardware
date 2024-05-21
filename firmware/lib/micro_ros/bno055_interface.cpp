#include "bno055_interface.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

BNO055Interface::BNO055Interface(uint8_t address) : bno(55, address, &Wire) {}

void BNO055Interface::init() {
  if (!bno.begin()) {
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(10);
}

void BNO055Interface::update() {
  bno.getEvent(&linearAccel, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&angVelocity, Adafruit_BNO055::VECTOR_GYROSCOPE);
  quat = bno.getQuat();
  bno.getEvent(&magneticField, Adafruit_BNO055::VECTOR_MAGNETOMETER);
}

sensor_msgs__msg__Imu BNO055Interface::getIMUData() {
  sensor_msgs__msg__Imu imu_msg;
  
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();

  imu_msg.angular_velocity.x = angVelocity.gyro.x;
  imu_msg.angular_velocity.y = angVelocity.gyro.y;
  imu_msg.angular_velocity.z = angVelocity.gyro.z;

  imu_msg.linear_acceleration.x = linearAccel.acceleration.x;
  imu_msg.linear_acceleration.y = linearAccel.acceleration.y;
  imu_msg.linear_acceleration.z = linearAccel.acceleration.z;

  return imu_msg;
}

sensor_msgs__msg__MagneticField BNO055Interface::getMagneticFieldData() {
  sensor_msgs__msg__MagneticField mag_msg;

  mag_msg.magnetic_field.x = magneticField.magnetic.x;
  mag_msg.magnetic_field.y = magneticField.magnetic.y;
  mag_msg.magnetic_field.z = magneticField.magnetic.z;

  return mag_msg;
}