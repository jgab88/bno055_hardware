#include <micro_ros_platformio.h>
#include <Wire.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <Arduino.h>
#include <string.h>

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  if (timer != NULL) {
    // Get IMU data
    imu::Quaternion quat = bno.getQuat();
    imu_msg.orientation.x = quat.x();
    imu_msg.orientation.y = quat.y();
    imu_msg.orientation.z = quat.z();
    imu_msg.orientation.w = quat.w();

    imu::Vector<3> ang_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_msg.angular_velocity.x = ang_vel.x();
    imu_msg.angular_velocity.y = ang_vel.y();
    imu_msg.angular_velocity.z = ang_vel.z();

    imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu_msg.linear_acceleration.x = lin_accel.x();
    imu_msg.linear_acceleration.y = lin_accel.y();
    imu_msg.linear_acceleration.z = lin_accel.z();

    // Get magnetic field data
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    mag_msg.magnetic_field.x = mag.x();
    mag_msg.magnetic_field.y = mag.y();
    mag_msg.magnetic_field.z = mag.z();

    // Add timestamps
    imu_msg.header.stamp.sec = micros() / 1000000;
    imu_msg.header.stamp.nanosec = (micros() % 1000000) * 1000;

    mag_msg.header.stamp.sec = micros() / 1000000;
    mag_msg.header.stamp.nanosec = (micros() % 1000000) * 1000;

    delay(100);

    rcl_ret_t rc;
    rc = rcl_publish(&imu_publisher, &imu_msg, NULL);
    if (rc != RCL_RET_OK) {
      Serial.print("Error publishing IMU data: ");
    }

    rc = rcl_publish(&mag_publisher, &mag_msg, NULL);
    if (rc != RCL_RET_OK) {
      Serial.print("Error publishing Mag");
    }
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  // create init_options
  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) {
    Serial.print("Error initializing RCL support: ");
    return false;
  }

  // create node
  rc = rclc_node_init_default(&node, "bno055_publisher", "", &support);
  if (rc != RCL_RET_OK) {
    Serial.print("Error creating node: ");
    Serial.print(rcl_get_error_string().str);
    rclc_support_fini(&support);
    return false;
  }

  // create IMU publisher
  rc = rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");
  if (rc != RCL_RET_OK) {
    Serial.print("Error creating IMU publisher: ");
    Serial.print(rcl_get_error_string().str);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    return false;
  }

  // create magnetic field publisher
  rc = rclc_publisher_init_best_effort(&mag_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "imu/mag");
  if (rc != RCL_RET_OK) {
    Serial.print("Error creating MAG publisher: ");
    Serial.print(rcl_get_error_string().str);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    return false;
  }

  // create timer
  const unsigned int timer_timeout = 100;
  rc = rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);
  if (rc != RCL_RET_OK) {
    Serial.print("Error creating timer: ");
    Serial.print(rcl_get_error_string().str);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    return false;
  }

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t rc;
  rc = rcl_publisher_fini(&imu_publisher, &node);
  if (rc != RCL_RET_OK) {
    Serial.print("Error finalizing IMU publisher: ");
    Serial.print(rcl_get_error_string().str);
  }

  rc = rcl_publisher_fini(&mag_publisher, &node);
  if (rc != RCL_RET_OK) {
    Serial.print("Error finalizing magnetometer publisher: ");
    Serial.print(rcl_get_error_string().str);
  }

  rc = rcl_timer_fini(&timer);
  if (rc != RCL_RET_OK) {
    Serial.print("Error finalizing timer: ");
    Serial.print(rcl_get_error_string().str);
  }

  rclc_executor_fini(&executor);

  rc = rcl_node_fini(&node);
  if (rc != RCL_RET_OK) {
    Serial.print("Error finalizing node: ");
    Serial.print(rcl_get_error_string().str);
  }

  rc = rclc_support_fini(&support);
  if (rc != RCL_RET_OK) {
    Serial.print("Error finalizing RCL support: ");
    Serial.print(rcl_get_error_string().str);
  }
}

void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  pinMode(LED_PIN, OUTPUT);
  state = WAITING_AGENT;

  // Initialize BNO055
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // Calibrate the sensor
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // Print the calibration status
  Serial.print("Calibration Status: ");
  Serial.print("Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  // Prompt the user to perform the calibration
  while (!system && !gyro && !accel && !mag) {
    Serial.println("Please perform the calibration routine.");
    bno.getCalibration(&system, &gyro, &accel, &mag);
    delay(500);
  }

  Serial.println("Calibration complete!");
  delay(200);
}

void loop()
{
  switch (state)
  {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}
