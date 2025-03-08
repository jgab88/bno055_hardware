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
#include <geometry_msgs/msg/twist_with_covariance_stamped.h>
#include <std_msgs/msg/string.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <Arduino.h>
#include <string.h>
#include <std_msgs/msg/float32.h> // Sonar height message

#define TRIG_PIN 9  // Sonar Trigger Pin
#define ECHO_PIN 10 // Sonar Echo Pin

#define LED_PIN 13
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
    }                            \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

// Frame ID definitions
#define ODOM_FRAME_ID "odom"
#define BASE_FRAME_ID "base_link"
#define IMU_FRAME_ID "imu_link"

// Encoder pins and parameters
const int encoderPinA = 5;
const int encoderPinB = 6;
volatile long encoderCount = 0;
const float TICKS_PER_REVOLUTION = 1024.0;
const float WHEEL_DIAMETER = 0.1;
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
rcl_publisher_t wheel_odom_publisher;
rcl_publisher_t wheel_vel_publisher;
rcl_publisher_t debug_publisher;
rcl_publisher_t sonar_height_publisher;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
nav_msgs__msg__Odometry wheel_odom_msg;
geometry_msgs__msg__TwistWithCovarianceStamped wheel_vel_msg;
std_msgs__msg__String debug_msg;
std_msgs__msg__Float32 sonar_height_msg;

char debug_string_buffer[100];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Function to measure distance using HC-SR04
float measureSonarDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  // Convert to meters (speed of sound = 343m/s)
  float distance = (duration * 0.000343) / 2.0;
  
  // Limit the range to reasonable values (0.02m to 4m)
  return constrain(distance, 0.02, 4.0);
}

// Debug function
void publish_debug(const char *msg)
{
  if (state == AGENT_CONNECTED)
  {
    snprintf(debug_string_buffer, sizeof(debug_string_buffer), "%s", msg);
    debug_msg.data.data = debug_string_buffer;
    debug_msg.data.size = strlen(debug_string_buffer);
    rcl_ret_t rc = rcl_publish(&debug_publisher, &debug_msg, NULL);
    (void)rc; // Avoid unused variable warning
  }
}

void FASTRUN encoderISR()
{
  int a = digitalRead(encoderPinA);
  int b = digitalRead(encoderPinB);

  if (a == b)
  {
    encoderCount++;
  }
  else
  {
    encoderCount--;
  }
}

void init_messages()
{
  // Initialize wheel odometry message
  static char odom_frame[] = ODOM_FRAME_ID;
  static char base_frame[] = BASE_FRAME_ID;
  static char imu_frame[] = IMU_FRAME_ID;

  // Initialize wheel odometry message
  wheel_odom_msg.header.frame_id.data = odom_frame;
  wheel_odom_msg.header.frame_id.size = strlen(odom_frame);
  wheel_odom_msg.child_frame_id.data = base_frame;
  wheel_odom_msg.child_frame_id.size = strlen(base_frame);

  // Initialize pose with zeros and orientation.w = 1
  memset(&wheel_odom_msg.pose, 0, sizeof(wheel_odom_msg.pose));
  wheel_odom_msg.pose.pose.orientation.w = 1.0;

  // Initialize IMU message
  imu_msg.header.frame_id.data = imu_frame;
  imu_msg.header.frame_id.size = strlen(imu_frame);

  // Initialize magnetometer message
  mag_msg.header.frame_id.data = imu_frame;
  mag_msg.header.frame_id.size = strlen(imu_frame);

  // Initialize wheel velocity message
  wheel_vel_msg.header.frame_id.data = base_frame;
  wheel_vel_msg.header.frame_id.size = strlen(base_frame);

  publish_debug("Messages initialized");
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    if (timer != NULL) {
        unsigned long currentTime = micros();
        rcl_ret_t rc;
        char debug_buf[100];
        
        // Measure sonar height
        float height = measureSonarDistance();
        sonar_height_msg.data = height;

        // Get IMU data
        imu::Quaternion quat = bno.getQuat();
        if (quat.w() == 0 && quat.x() == 0 && quat.y() == 0 && quat.z() == 0) {
            publish_debug("Warning: Invalid quaternion data");
            return;
        }

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

        // Update all message timestamps
        imu_msg.header.stamp.sec = sec;
        imu_msg.header.stamp.nanosec = nanosec;
        mag_msg.header.stamp.sec = sec;
        mag_msg.header.stamp.nanosec = nanosec;
        wheel_odom_msg.header.stamp.sec = sec;
        wheel_odom_msg.header.stamp.nanosec = nanosec;
        wheel_vel_msg.header.stamp.sec = sec;
        wheel_vel_msg.header.stamp.nanosec = nanosec;

        // Update IMU message - manually copy quaternion values
        imu_msg.orientation.w = quat.w();
        imu_msg.orientation.x = quat.x();
        imu_msg.orientation.y = quat.y();
        imu_msg.orientation.z = quat.z();

        // Update angular velocity
        imu::Vector<3> ang_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu_msg.angular_velocity.x = ang_vel.x();
        imu_msg.angular_velocity.y = ang_vel.y();
        imu_msg.angular_velocity.z = ang_vel.z();

        // Update linear acceleration
        imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu_msg.linear_acceleration.x = lin_accel.x();
        imu_msg.linear_acceleration.y = lin_accel.y();
        imu_msg.linear_acceleration.z = lin_accel.z();

        // Update magnetometer message
        imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        mag_msg.magnetic_field.x = mag.x();
        mag_msg.magnetic_field.y = mag.y();
        mag_msg.magnetic_field.z = mag.z();

        // Update wheel odometry message (position only)
        wheel_odom_msg.pose.pose.position.x = currentCount * METERS_PER_TICK;
        wheel_odom_msg.pose.pose.position.y = 0.0;
        wheel_odom_msg.pose.pose.position.z = 0.0;
        
        // Don't set orientation in wheel odometry
        wheel_odom_msg.pose.pose.orientation.w = 1.0;
        wheel_odom_msg.pose.pose.orientation.x = 0.0;
        wheel_odom_msg.pose.pose.orientation.y = 0.0;
        wheel_odom_msg.pose.pose.orientation.z = 0.0;

        // Update wheel velocity message
        wheel_vel_msg.twist.twist.linear.x = velocity;
        wheel_vel_msg.twist.twist.linear.y = 0.0;
        wheel_vel_msg.twist.twist.linear.z = 0.0;
        
        // Copy angular velocity from IMU
        wheel_vel_msg.twist.twist.angular.x = ang_vel.x();
        wheel_vel_msg.twist.twist.angular.y = ang_vel.y();
        wheel_vel_msg.twist.twist.angular.z = ang_vel.z();

        // Set covariances
        // Wheel odometry position covariance (only X position is measured)
        wheel_odom_msg.pose.covariance[0] = 0.01;   // x
        wheel_odom_msg.pose.covariance[7] = 0.1;    // y
        wheel_odom_msg.pose.covariance[14] = 0.1;   // z

        // Wheel velocity covariance
        wheel_vel_msg.twist.covariance[0] = 0.01;   // linear x
        wheel_vel_msg.twist.covariance[7] = 0.1;    // linear y
        wheel_vel_msg.twist.covariance[14] = 0.1;   // linear z
        wheel_vel_msg.twist.covariance[21] = 0.01;  // angular x
        wheel_vel_msg.twist.covariance[28] = 0.01;  // angular y
        wheel_vel_msg.twist.covariance[35] = 0.01;  // angular z

        // Debug output
        snprintf(debug_buf, sizeof(debug_buf), 
                "Publishing - pos: %.3f m, vel: %.3f m/s", 
                wheel_odom_msg.pose.pose.position.x,
                wheel_vel_msg.twist.twist.linear.x,
                height);
        publish_debug(debug_buf);

        // Publish all messages
        rc = rcl_publish(&imu_publisher, &imu_msg, NULL);
        if (rc != RCL_RET_OK) {
            publish_debug("Failed to publish IMU data");
        }

        rc = rcl_publish(&mag_publisher, &mag_msg, NULL);
        if (rc != RCL_RET_OK) {
            publish_debug("Failed to publish MAG data");
        }

        rc = rcl_publish(&wheel_odom_publisher, &wheel_odom_msg, NULL);
        if (rc != RCL_RET_OK) {
            publish_debug("Failed to publish wheel odometry");
        }

        rc = rcl_publish(&wheel_vel_publisher, &wheel_vel_msg, NULL);
        if (rc != RCL_RET_OK) {
            publish_debug("Failed to publish wheel velocity");
        }

        rc = rcl_publish(&sonar_height_publisher, &sonar_height_msg, NULL);
        if (rc != RCL_RET_OK) {
            publish_debug("Failed to publish sonar height");
        }

        // Update previous values
        previousTime = currentTime;
        previousCount = currentCount;
    }
}

bool create_entities()
{
  publish_debug("Creating entities...");

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_sensors_node", "", &support));

  // Create publishers
  RCCHECK(rclc_publisher_init_default(
      &imu_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data"));

  RCCHECK(rclc_publisher_init_default(
      &mag_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
      "imu/mag"));

  RCCHECK(rclc_publisher_init_default(
      &wheel_odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "wheel/odometry"));

  RCCHECK(rclc_publisher_init_default(
      &wheel_vel_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistWithCovarianceStamped),
      "wheel/velocity"));

  RCCHECK(rclc_publisher_init_default(
      &debug_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "teensy_debug"));

      // Create sonar height publisher
  RCCHECK(rclc_publisher_init_default(
      &sonar_height_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "sonar/height"));

  // Create timer
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // Create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize messages
  init_messages();

  publish_debug("All entities created successfully");
  return true;
}

void destroy_entities() {
    publish_debug("Starting entity cleanup...");
    rcl_ret_t rc;
    
    // Set context timeout to 0 for immediate cleanup
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // Clean up publishers
    rc = rcl_publisher_fini(&imu_publisher, &node);
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to cleanup IMU publisher");
        delay(10); // Small delay to allow debug message to send
    }

    rc = rcl_publisher_fini(&mag_publisher, &node);
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to cleanup magnetometer publisher");
        delay(10);
    }

    rc = rcl_publisher_fini(&wheel_odom_publisher, &node);
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to cleanup wheel odometry publisher");
        delay(10);
    }

    rc = rcl_publisher_fini(&wheel_vel_publisher, &node);
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to cleanup wheel velocity publisher");
        delay(10);
    }

    // Clean up sonar publisher
    rc = rcl_publisher_fini(&sonar_height_publisher, &node);
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to cleanup sonar publisher");
        delay(10);
    }

    // Final debug message before we destroy debug publisher
    publish_debug("All other publishers cleaned up, destroying debug publisher now");
    delay(20); // Slightly longer delay for final message

    // Clean up debug publisher last
    rc = rcl_publisher_fini(&debug_publisher, &node);

    // Clean up remaining entities
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
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

  // Setup HC-SR04 pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  state = WAITING_AGENT;

  // Initialize BNO055
  if (!bno.begin())
  {
    // We can't use publish_debug here as entities aren't created yet
    while (1)
    {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }

  // Basic calibration check
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // Wait for basic calibration
  while (system < 1 && gyro < 1 && accel < 1 && mag < 1)
  {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    delay(100);
  }
}

void loop()
{
  static states previous_state = WAITING_AGENT;
  static unsigned long last_state_print = 0;
  unsigned long current_time = millis();

  switch (state)
  {
  case WAITING_AGENT:
    if (previous_state != WAITING_AGENT)
    {
      publish_debug("State: WAITING_AGENT");
      previous_state = WAITING_AGENT;
    }
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;

  case AGENT_AVAILABLE:
    if (previous_state != AGENT_AVAILABLE)
    {
      publish_debug("State: AGENT_AVAILABLE");
      previous_state = AGENT_AVAILABLE;
    }
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      publish_debug("Failed to create entities");
      destroy_entities();
    }
    break;

  case AGENT_CONNECTED:
    if (previous_state != AGENT_CONNECTED)
    {
      publish_debug("State: AGENT_CONNECTED");
      previous_state = AGENT_CONNECTED;
    }

    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

      // Periodic debug messages
      if (current_time - last_state_print > 5000)
      { // Every 5 seconds
        publish_debug("System running normally");
        last_state_print = current_time;
      }
    }
    break;

  case AGENT_DISCONNECTED:
    if (previous_state != AGENT_DISCONNECTED)
    {
      publish_debug("State: AGENT_DISCONNECTED");
      previous_state = AGENT_DISCONNECTED;
    }
    destroy_entities();
    state = WAITING_AGENT;
    break;

  default:
    break;
  }

  digitalWrite(LED_PIN, state == AGENT_CONNECTED);
}