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
#include <nav_msgs/msg/odometry.h>
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

// Encoder pins and parameters
const int encoderPinA = 5;
const int encoderPinB = 6;
volatile long encoderCount = 0;
const float TICKS_PER_REVOLUTION = 1024.0; // Replace with your encoder's PPR
const float WHEEL_DIAMETER = 0.1;          // Replace with your wheel diameter in meters
const float METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REVOLUTION;
unsigned long previousTime = 0;
long previousCount = 0;

// ROS entities
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_publisher_t odom_publisher;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
nav_msgs__msg__Odometry odom_msg;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void FASTRUN encoderISR() {
    int a = digitalRead(encoderPinA);
    int b = digitalRead(encoderPinB);
    
    if (a == b) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  if (timer != NULL) {
    rcl_ret_t rc;
    unsigned long currentTime = micros();
    
    // Get IMU data
    imu::Quaternion quat = bno.getQuat();
    if (quat.w() == 0 && quat.x() == 0 && quat.y() == 0 && quat.z() == 0) {
      Serial.println("Warning: Invalid quaternion data");
    }
    
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

    // Calculate encoder-based odometry
    long currentCount = encoderCount;
    float deltaTime = (currentTime - previousTime) / 1000000.0; // Convert to seconds
    long deltaTicks = currentCount - previousCount;
    float distance = deltaTicks * METERS_PER_TICK;
    float velocity = distance / deltaTime;

    // Get fresh timestamp
    currentTime = micros();
    uint32_t sec = currentTime / 1000000;
    uint32_t nanosec = (currentTime % 1000000) * 1000;

    // Update all message headers with timestamps
    imu_msg.header.stamp.sec = sec;
    imu_msg.header.stamp.nanosec = nanosec;
    mag_msg.header.stamp.sec = sec;
    mag_msg.header.stamp.nanosec = nanosec;
    odom_msg.header.stamp.sec = sec;
    odom_msg.header.stamp.nanosec = nanosec;

    // Set frame IDs
    imu_msg.header.frame_id.data = (char*)"imu_link";
    imu_msg.header.frame_id.size = strlen("imu_link");
    mag_msg.header.frame_id.data = (char*)"imu_link";
    mag_msg.header.frame_id.size = strlen("imu_link");
    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.size = strlen("odom");
    odom_msg.child_frame_id.data = (char*)"base_link";
    odom_msg.child_frame_id.size = strlen("base_link");

    // Update odometry message
    odom_msg.pose.pose.position.x = currentCount * METERS_PER_TICK;
    odom_msg.twist.twist.linear.x = velocity;

    // Publish all messages with error checking
    rc = rcl_publish(&imu_publisher, &imu_msg, NULL);
    if (rc != RCL_RET_OK) {
      Serial.print("Failed to publish IMU: ");
      Serial.println(rc);
    }

    rc = rcl_publish(&mag_publisher, &mag_msg, NULL);
    if (rc != RCL_RET_OK) {
      Serial.print("Failed to publish MAG: ");
      Serial.println(rc);
    }

    rc = rcl_publish(&odom_publisher, &odom_msg, NULL);
    if (rc != RCL_RET_OK) {
      Serial.print("Failed to publish ODOM: ");
      Serial.println(rc);
    }

    // Update previous values for next iteration
    previousTime = currentTime;
    previousCount = currentCount;
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  
  // create init_options
  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) {
    Serial.println("Failed to initialize RCL support");
    return false;
  }

  // create node
  rc = rclc_node_init_default(&node, "teensy_sensors_node", "", &support);
  if (rc != RCL_RET_OK) {
    Serial.println("Failed to create node");
    return false;
  }

  // create IMU publisher
  rc = rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data");
  if (rc != RCL_RET_OK) {
    Serial.println("Failed to create IMU publisher");
    return false;
  }

  // create magnetic field publisher
  rc = rclc_publisher_init_best_effort(
    &mag_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "imu/mag");
  if (rc != RCL_RET_OK) {
    Serial.println("Failed to create MAG publisher");
    return false;
  }

  // create odometry publisher
  rc = rclc_publisher_init_best_effort(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "wheel_odom");
  if (rc != RCL_RET_OK) {
    Serial.println("Failed to create ODOM publisher");
    return false;
  }

  // create timer
  const unsigned int timer_timeout = 100;
  rc = rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback);
  if (rc != RCL_RET_OK) {
    Serial.println("Failed to create timer");
    return false;
  }

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (rc != RCL_RET_OK) {
    Serial.println("Failed to initialize executor");
    return false;
  }
  
  rc = rclc_executor_add_timer(&executor, &timer);
  if (rc != RCL_RET_OK) {
    Serial.println("Failed to add timer to executor");
    return false;
  }
  
  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&mag_publisher, &node);
  rcl_publisher_fini(&odom_publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  
  Serial.println("Entities destroyed");
}

void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize encoder pins
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
  
  state = WAITING_AGENT;
  Serial.println("Starting initialization...");

  // Initialize BNO055
  if (!bno.begin())
  {
    Serial.println("No BNO055 detected. Check wiring or I2C address!");
    while (1);
  }

  // Calibrate the sensor
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.println("\nCalibration Status:");
  Serial.print("Sys:"); Serial.print(system, DEC);
  Serial.print(" Gyro:"); Serial.print(gyro, DEC);
  Serial.print(" Accel:"); Serial.print(accel, DEC);
  Serial.print(" Mag:"); Serial.println(mag, DEC);

  // Wait for initial calibration
  while (system < 1 && gyro < 1 && accel < 1 && mag < 1) {
    Serial.println("Please perform calibration routine...");
    bno.getCalibration(&system, &gyro, &accel, &mag);
    delay(500);
  }

  Serial.println("Basic calibration complete!");
  Serial.println("Initialization complete!");
}

void loop()
{
  static states previous_state = WAITING_AGENT;
  
  switch (state)
  {
    case WAITING_AGENT:
      if (previous_state != WAITING_AGENT) {
        Serial.println("State: WAITING_AGENT");
        previous_state = WAITING_AGENT;
      }
      EXECUTE_EVERY_N_MS(500, {
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      });
      break;
      
    case AGENT_AVAILABLE:
      if (previous_state != AGENT_AVAILABLE) {
        Serial.println("State: AGENT_AVAILABLE");
        previous_state = AGENT_AVAILABLE;
      }
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        Serial.println("Failed to create entities");
        destroy_entities();
      }
      break;
      
    case AGENT_CONNECTED:
      if (previous_state != AGENT_CONNECTED) {
        Serial.println("State: AGENT_CONNECTED");
        previous_state = AGENT_CONNECTED;
      }
      EXECUTE_EVERY_N_MS(200, {
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
        if (state == AGENT_DISCONNECTED) {
          Serial.println("Agent ping failed");
        }
      });
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
      
    case AGENT_DISCONNECTED:
      if (previous_state != AGENT_DISCONNECTED) {
        Serial.println("State: AGENT_DISCONNECTED");
        previous_state = AGENT_DISCONNECTED;
      }
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