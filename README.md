Notes on IMU_Project:

As of 5.26.24:
I have the sensor data being published by a microros environment running on a teensy4.1. I used platformio as my ide. Here is was is currently being published:

	imu/data =  imu::Quaternion quat = bno.getQuat();
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
			
	imu/mag =   imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
			mag_msg.magnetic_field.x = mag.x();
			mag_msg.magnetic_field.y = mag.y();
			mag_msg.magnetic_field.z = mag.z();


Host environment:
Host is running on Ununtu 20.04 on a Ros2 Humble environment. 
Workspace = bno055_hardware
Workspace tree:
+-- build #colcon generated#
+-- firmware #firmware for teensy41#
¦   +-- build
¦   ¦   +-- project.checksum
¦   ¦   +-- teensy41
¦   ¦       +-- idedata.json
¦   +-- firmware
¦   +-- include
¦   ¦   +-- README
¦   +-- lib
¦   ¦   +-- Adafruit_BNO055
¦   ¦   ¦   +-- Adafruit_BNO055.cpp
¦   ¦   ¦   +-- Adafruit_BNO055.h
¦   ¦   ¦   +-- assets
¦   ¦   ¦   ¦   +-- board.jpg
¦   ¦   ¦   +-- code-of-conduct.md
¦   ¦   ¦   +-- examples
¦   ¦   ¦   ¦   +-- bunny
¦   ¦   ¦   ¦   ¦   +-- bunny.ino
¦   ¦   ¦   ¦   +-- position
¦   ¦   ¦   ¦   ¦   +-- position.ino
¦   ¦   ¦   ¦   +-- rawdata
¦   ¦   ¦   ¦   ¦   +-- rawdata.ino
¦   ¦   ¦   ¦   +-- read_all_data
¦   ¦   ¦   ¦   ¦   +-- read_all_data.ino
¦   ¦   ¦   ¦   +-- restore_offsets
¦   ¦   ¦   ¦   ¦   +-- restore_offsets.ino
¦   ¦   ¦   ¦   +-- sensorapi
¦   ¦   ¦   ¦   ¦   +-- sensorapi.ino
¦   ¦   ¦   ¦   +-- webserial_3d
¦   ¦   ¦   ¦       +-- webserial_3d.ino
¦   ¦   ¦   +-- examples_processing
¦   ¦   ¦   +-- library.properties
¦   ¦   ¦   +-- LICENSE
¦   ¦   ¦   +-- OBJLoader
¦   ¦   ¦   ¦   +-- OBJLoader.txt
¦   ¦   ¦   ¦   +-- OBJLoader.zip
¦   ¦   ¦   +-- README.md
¦   ¦   ¦   +-- utility
¦   ¦   ¦       +-- imumaths.h
¦   ¦   ¦       +-- matrix.h
¦   ¦   ¦       +-- quaternion.h
¦   ¦   ¦       +-- vector.h
¦   ¦   +-- micro_ros
¦   ¦   ¦   +-- bno055_interface.cpp
¦   ¦   ¦   +-- bno055_interface.h
¦   ¦   +-- README
¦   +-- platformio.ini
¦   +-- src
¦   ¦   +-- firmware.cpp
¦   +-- test
¦       +-- README
+-- install #colcon generated#
+-- log #colcon generated#
+-- micro_ros_agent #standard gitclone#
+-- src
¦   +-- imu_test #ros2 enviroment#
¦       +-- imu_test
¦       ¦   +-- imu_3d_visualization.py
¦       ¦   +-- __init__.py
¦       +-- launch
¦       ¦   +-- imu_3d_visualization.launch.py
¦       ¦   +-- imu_test.launch.txt
¦       +-- package.xml
¦       +-- resource
¦       ¦   +-- imu_test
¦       +-- rviz
¦       ¦   +-- robotmodel.rviz
¦       +-- setup.cfg
¦       +-- setup.py
¦       +-- test
¦       ¦   +-- test_copyright.py
¦       ¦   +-- test_flake8.py
¦       ¦   +-- test_pep257.py
¦       +-- urdf
¦           +-- robotmodel.urdf
			
Steps for launching project:
--Make sure to source /opt/ros/humble/setup.sh--
1. Initiate Microros on the microcontroller side:
	Firmware Dir: jg@farmbot:~/bno055_hardware/firmware
	Firmware Launch Script: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
--wait for micro_ros to finish loading, you should see "create_publisher" and "create_datawriter"--
Initiate Host side:
	Package Dir: jg@farmbot:~/bno055_hardware
--Make sure to source package with source install/setup.sh--
	Package Launch Script: ros2 launch imu_test imu_3d_visualization.launch.py
--You should see rviz come up with the object representing orientation from the physical imu chip--

TODO_JG
5.26.24
Now that we have the sensor publishing data and the host ros2 enviroment listening and parsing the data
accordingly. We need to start implementing sensor fusion and other algorithms to generate positional 
data so that we can move the object in 3D space. Currently, the ros2 package (imu_test) is using the imu/data
for for orientation transformations, linear acceleration, and angular velocity (gyroscope), we need to utilize 
the imu/mag that is being published by the microcontroller and use sensor fusion algorithms to generate directional 
movement based on the earth magnetic field and acceleration information - later, we can start to experiment with 
fusing/combining gyroscope and accelerometer data with the magnetometer data.
Steps:
1. Integrate the magnetometer data
2. Test
2. Implement sensor fusion with gyroscope, accelerometer and magnetometer.
	-Madgwick filter	
	-Mahony filter
3. Test
4. Document.
			
				
			