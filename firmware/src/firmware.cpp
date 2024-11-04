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
#include <std_msgs/msg/string.h>
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

// Frame ID buffers
char odom_frame_id_buffer[20];
char base_frame_id_buffer[20];
char imu_frame_id_buffer[20];

// ROS entities
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t debug_publisher;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__String debug_msg;
char debug_string_buffer[100];

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Debug function declaration
void publish_debug(const char* msg) {
    if (state == AGENT_CONNECTED) {
        snprintf(debug_string_buffer, sizeof(debug_string_buffer), "%s", msg);
        debug_msg.data.data = debug_string_buffer;
        debug_msg.data.size = strlen(debug_string_buffer);
        rcl_ret_t rc = rcl_publish(&debug_publisher, &debug_msg, NULL);
        (void)rc; // Avoid unused variable warning
    }
}

void FASTRUN encoderISR() {
    int a = digitalRead(encoderPinA);
    int b = digitalRead(encoderPinB);
    
    if (a == b) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

void init_odom_msg() {
    // Zero out the entire message first
    memset(&odom_msg, 0, sizeof(nav_msgs__msg__Odometry));

    // Initialize strings with static buffers
    static char odom_frame[] = "odom";
    static char base_frame[] = "base_link";
    
    // Set up header frame_id
    odom_msg.header.frame_id.data = odom_frame;
    odom_msg.header.frame_id.size = strlen(odom_frame);
    odom_msg.header.frame_id.capacity = sizeof(odom_frame);

    // Set up child frame_id
    odom_msg.child_frame_id.data = base_frame;
    odom_msg.child_frame_id.size = strlen(base_frame);
    odom_msg.child_frame_id.capacity = sizeof(base_frame);

    // Initialize pose and twist with zeros
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    // Initialize covariance matrices with small non-zero values on diagonal
    for(size_t i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
        odom_msg.twist.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
    }
    
    publish_debug("Odometry message initialized");
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  if (timer != NULL) {
    unsigned long currentTime = micros();
    rcl_ret_t rc;
    char debug_buf[100];
    
    // Get IMU data
    imu::Quaternion quat = bno.getQuat();
    if (quat.w() == 0 && quat.x() == 0 && quat.y() == 0 && quat.z() == 0) {
        publish_debug("Warning: Invalid quaternion data");
        return;
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

    snprintf(debug_buf, sizeof(debug_buf), "Odom calc: count=%ld delta=%ld dist=%.3f vel=%.3f", 
             currentCount, deltaTicks, distance, velocity);
    publish_debug(debug_buf);

    // Get fresh timestamp
    currentTime = micros();
    uint32_t sec = currentTime / 1000000;
    uint32_t nanosec = (currentTime % 1000000) * 1000;

    // Update all headers with timestamps
    imu_msg.header.stamp.sec = sec;
    imu_msg.header.stamp.nanosec = nanosec;
    mag_msg.header.stamp.sec = sec;
    mag_msg.header.stamp.nanosec = nanosec;
    odom_msg.header.stamp.sec = sec;
    odom_msg.header.stamp.nanosec = nanosec;

    // Update odometry message
    odom_msg.pose.pose.position.x = currentCount * METERS_PER_TICK;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;
    
    odom_msg.twist.twist.linear.x = velocity;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Initialize covariance matrices
    for(size_t i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.01;   // x
    odom_msg.pose.covariance[7] = 0.01;   // y
    odom_msg.pose.covariance[14] = 0.01;  // z
    odom_msg.pose.covariance[21] = 0.01;  // rotation about X axis
    odom_msg.pose.covariance[28] = 0.01;  // rotation about Y axis
    odom_msg.pose.covariance[35] = 0.01;  // rotation about Z axis

    snprintf(debug_buf, sizeof(debug_buf), "Publishing odom: x=%.3f v=%.3f", 
             odom_msg.pose.pose.position.x, odom_msg.twist.twist.linear.x);
    publish_debug(debug_buf);

    // Pre-publish verification
    snprintf(debug_buf, sizeof(debug_buf), "Pre-pub check: frame_id=%s child=%s", 
             odom_msg.header.frame_id.data, 
             odom_msg.child_frame_id.data);
    publish_debug(debug_buf);

    // Publish messages with error checking
    rc = rcl_publish(&imu_publisher, &imu_msg, NULL);
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to publish IMU data");
    }

    rc = rcl_publish(&mag_publisher, &mag_msg, NULL);
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to publish MAG data");
    }

    rc = rcl_publish(&odom_publisher, &odom_msg, NULL);
    if (rc != RCL_RET_OK) {
        const char* error_string = rcutils_get_error_string().str;
        rcutils_reset_error();
        
        snprintf(debug_buf, sizeof(debug_buf), 
                "ODOM publish error %ld: %s", 
                (long)rc, error_string);
        publish_debug(debug_buf);
    }

    // Update previous values for next iteration
    previousTime = currentTime;
    previousCount = currentCount;
  }
}

bool create_entities()
{
    publish_debug("Creating entities...");
    
    allocator = rcl_get_default_allocator();

    // Create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    publish_debug("Support initialized");

    // Create node
    RCCHECK(rclc_node_init_default(&node, "teensy_sensors_node", "", &support));
    publish_debug("Node initialized");

    // Create IMU publisher
    rcl_ret_t rc = rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data");
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to create IMU publisher");
        return false;
    }
    publish_debug("IMU publisher created");

    // Create magnetometer publisher
    rc = rclc_publisher_init_default(
        &mag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "imu/mag");
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to create MAG publisher");
        return false;
    }
    publish_debug("MAG publisher created");

// Initialize odometry message with static frame IDs
    static char odom_frame[] = "odom";
    static char base_frame[] = "base_link";
    
    odom_msg.header.frame_id.data = odom_frame;
    odom_msg.header.frame_id.size = strlen(odom_frame);
    odom_msg.child_frame_id.data = base_frame;
    odom_msg.child_frame_id.size = strlen(base_frame);
    
    // Initialize pose and twist with zeros
    memset(&odom_msg.pose, 0, sizeof(odom_msg.pose));
    memset(&odom_msg.twist, 0, sizeof(odom_msg.twist));
    
    // Default orientation (no rotation)
    odom_msg.pose.pose.orientation.w = 1.0;
    
    publish_debug("Initializing odometry publisher...");
    
    // Use the simpler rclc publisher initialization
    rc = rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "wheel_odom");
    
    if (rc != RCL_RET_OK) {
        char error_buf[100];
        snprintf(error_buf, sizeof(error_buf), 
                "Failed to create ODOM publisher, error: %ld", (long)rc);
        publish_debug(error_buf);
        return false;
    }
    
    // Verify publisher handle
    if (!rcl_publisher_is_valid(&odom_publisher)) {
        publish_debug("Odometry publisher handle invalid");
        return false;
    }
    
    publish_debug("Odometry publisher created successfully");

    // Create debug publisher
    rc = rclc_publisher_init_default(
        &debug_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "teensy_debug");
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to create debug publisher");
        return false;
    }
    publish_debug("Debug publisher created");

    // Create timer
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
    publish_debug("Timer created");

    // Create executor
    executor = rclc_executor_get_zero_initialized_executor();
    rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to initialize executor");
        return false;
    }
    
    rc = rclc_executor_add_timer(&executor, &timer);
    if (rc != RCL_RET_OK) {
        publish_debug("Failed to add timer to executor");
        return false;
    }
    publish_debug("Executor initialized");

    // Initialize odometry message
    odom_msg.header.frame_id.capacity = strlen("odom") + 1;
    odom_msg.child_frame_id.capacity = strlen("base_link") + 1;
    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.size = strlen("odom");
    odom_msg.child_frame_id.data = (char*)"base_link";
    odom_msg.child_frame_id.size = strlen("base_link");

    publish_debug("All entities created successfully");
    return true;
}

void destroy_entities()
{
    publish_debug("Destroying entities...");
    
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // Cast to void to explicitly ignore return values
    (void)rcl_publisher_fini(&imu_publisher, &node);
    (void)rcl_publisher_fini(&mag_publisher, &node);
    (void)rcl_publisher_fini(&odom_publisher, &node);
    (void)rcl_publisher_fini(&debug_publisher, &node);
    (void)rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
    
    publish_debug("Entities destroyed");
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

    // Initialize BNO055
    if (!bno.begin())
    {
        // We can't use publish_debug here as entities aren't created yet
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    // Basic calibration check
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    
    // Wait for basic calibration
    while (system < 1 && gyro < 1 && accel < 1 && mag < 1) {
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
            if (previous_state != WAITING_AGENT) {
                publish_debug("State: WAITING_AGENT");
                previous_state = WAITING_AGENT;
            }
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
            
        case AGENT_AVAILABLE:
            if (previous_state != AGENT_AVAILABLE) {
                publish_debug("State: AGENT_AVAILABLE");
                previous_state = AGENT_AVAILABLE;
            }
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                publish_debug("Failed to create entities");
                destroy_entities();
            }
            break;
            
        case AGENT_CONNECTED:
            if (previous_state != AGENT_CONNECTED) {
                publish_debug("State: AGENT_CONNECTED");
                previous_state = AGENT_CONNECTED;
            }
            
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                
                // Periodic debug messages
                if (current_time - last_state_print > 5000) {  // Every 5 seconds
                    publish_debug("System running normally");
                    last_state_print = current_time;
                }
            }
            break;
            
        case AGENT_DISCONNECTED:
            if (previous_state != AGENT_DISCONNECTED) {
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