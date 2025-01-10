#include "MPU9250.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>

MPU9250 mpu;

// ROS node handle
ros::NodeHandle nh;

// IMU message
sensor_msgs/Imu imu_msg;

// ROS publisher
ros::Publisher imu_pub("imu/data", &imu_msg);

// Configuration constants
const uint8_t MPU_ADDRESS = 0x68;
const uint32_t UPDATE_INTERVAL_MS = 25;  // 40Hz update rate
const float MAGNETIC_DECLINATION = 0.62;  // Adjust for your location

// Status variables
bool isSensorInitialized = false;
uint32_t lastUpdateTime = 0;

void setupMPU() {
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(MPU_ADDRESS, setting)) {
        nh.logwarn("MPU connection failed!");
        isSensorInitialized = false;
        return;
    }
    
    // Apply calibration values (replace with your calibrated values)
    mpu.setMagBias(-153.835, -127.095, -381.37);
    mpu.setMagScale(1.00788, 1.0275, 0.9666);
    mpu.setMagneticDeclination(MAGNETIC_DECLINATION);
    
    isSensorInitialized = true;
    nh.loginfo("MPU9250 initialized successfully");
}

void setup() {
    // Initialize ROS node
    nh.initNode();
    nh.advertise(imu_pub);

    // Wait for ROSserial to be ready
    while(!nh.connected()) {
        nh.spinOnce();
    }

    Wire.begin();
    delay(2000);  // Allow time for sensor to stabilize
    
    setupMPU();
    
    // Set up the IMU message headers
    imu_msg.header.frame_id = "imu_link";
    
    // Initialize covariance matrices (adjust these values based on your sensor's characteristics)
    for(int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    // Set diagonal elements with estimated variances
    imu_msg.orientation_covariance[0] = 0.0025;  // X
    imu_msg.orientation_covariance[4] = 0.0025;  // Y
    imu_msg.orientation_covariance[8] = 0.0025;  // Z
    
    imu_msg.angular_velocity_covariance[0] = 0.02;  // X
    imu_msg.angular_velocity_covariance[4] = 0.02;  // Y
    imu_msg.angular_velocity_covariance[8] = 0.02;  // Z
    
    imu_msg.linear_acceleration_covariance[0] = 0.04;  // X
    imu_msg.linear_acceleration_covariance[4] = 0.04;  // Y
    imu_msg.linear_acceleration_covariance[8] = 0.04;  // Z
}

void loop() {
    if (!isSensorInitialized) {
        nh.logwarn("Sensor not initialized! Retrying setup...");
        setupMPU();
        delay(5000);
        return;
    }

    if (mpu.update()) {
        uint32_t currentTime = millis();
        if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
            publishImuData();
            lastUpdateTime = currentTime;
        }
    }

    nh.spinOnce();
}

void publishImuData() {
    // Update header timestamp
    imu_msg.header.stamp = nh.now();
    
    // Get quaternion orientation
    float q[4];
    mpu.getQuaternion(q[0], q[1], q[2], q[3]);
    
    // Set orientation
    imu_msg.orientation.w = q[0];
    imu_msg.orientation.x = q[1];
    imu_msg.orientation.y = q[2];
    imu_msg.orientation.z = q[3];
    
    // Set angular velocity (in rad/sec)
    imu_msg.angular_velocity.x = mpu.getGyroX() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = mpu.getGyroY() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = mpu.getGyroZ() * DEG_TO_RAD;
    
    // Set linear acceleration (in m/s^2)
    imu_msg.linear_acceleration.x = mpu.getAccX() * 9.81;
    imu_msg.linear_acceleration.y = mpu.getAccY() * 9.81;
    imu_msg.linear_acceleration.z = mpu.getAccZ() * 9.81;
    
    // Publish the message
    imu_pub.publish(&imu_msg);
}
