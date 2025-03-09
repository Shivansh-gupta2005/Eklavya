#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

// MPU9250 I2C address
#define MPU9250_ADDR 0x68
#define MAG_ADDR 0x0C

// ROS node handle
ros::NodeHandle nh;

// IMU message
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

// Calibration variables
float accel_offset[3] = {0, 0, 0};
float gyro_offset[3] = {0, 0, 0};
float mag_offset[3] = {0, 0, 0};
float mag_scale[3] = {1, 1, 1};

// Orientation offset for zeroing
float q_offset[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// Sensor data
float accel[3], gyro[3], mag[3];

// Orientation data
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // Quaternion
float q_calibrated[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Calibrated quaternion
unsigned long lastUpdate;
const float sampleRate = 100.0f;            // Hz

void writeByte(uint8_t address, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void readBytes(uint8_t address, uint8_t reg, uint8_t count, uint8_t* data) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address, count);
    for(uint8_t i = 0; i < count; i++) {
        data[i] = Wire.read();
    }
}

void initMPU9250() {
    Wire.begin();
    delay(100);
    
    // Wake up MPU9250
    writeByte(MPU9250_ADDR, 0x6B, 0x00);
    delay(100);
    
    // Configure accelerometer (±2g)
    writeByte(MPU9250_ADDR, 0x1C, 0x00);
    
    // Configure gyroscope (±250°/s)
    writeByte(MPU9250_ADDR, 0x1B, 0x00);
    
    // Enable bypass mode for magnetometer
    writeByte(MPU9250_ADDR, 0x37, 0x02);
    delay(100);
}

void calibrateSensors() {
    nh.loginfo("Starting calibration... Keep sensor still and level!");
    delay(2000);
    
    // Collect samples for calibration
    const int numSamples = 1000;
    float accel_sum[3] = {0};
    float gyro_sum[3] = {0};
    float mag_sum[3] = {0};
    
    for(int i = 0; i < numSamples; i++) {
        uint8_t buffer[14];
        readBytes(MPU9250_ADDR, 0x3B, 14, buffer);
        
        // Process accelerometer
        float accel_temp[3];
        for(int j = 0; j < 3; j++) {
            accel_temp[j] = (float)((int16_t)((buffer[j*2] << 8) | buffer[j*2+1])) / 16384.0f;
            accel_sum[j] += accel_temp[j];
        }
        
        // Process gyroscope
        float gyro_temp[3];
        for(int j = 0; j < 3; j++) {
            gyro_temp[j] = (float)((int16_t)((buffer[j*2+8] << 8) | buffer[j*2+9])) / 131.0f;
            gyro_sum[j] += gyro_temp[j];
        }
        
        // Process magnetometer (if needed)
        uint8_t mag_buffer[7];
        readBytes(MAG_ADDR, 0x03, 7, mag_buffer);
        
        float mag_temp[3];
        mag_temp[0] = (float)((int16_t)(mag_buffer[1] << 8 | mag_buffer[0])) * 0.15f;
        mag_temp[1] = (float)((int16_t)(mag_buffer[3] << 8 | mag_buffer[2])) * 0.15f;
        mag_temp[2] = (float)((int16_t)(mag_buffer[5] << 8 | mag_buffer[4])) * 0.15f;
        
        for(int j = 0; j < 3; j++) {
            mag_sum[j] += mag_temp[j];
        }
        
        delay(5);
    }
    
    // Calculate accelerometer offsets
    accel_offset[0] = accel_sum[0] / numSamples;  // X should be 0
    accel_offset[1] = accel_sum[1] / numSamples;  // Y should be 0
    accel_offset[2] = accel_sum[2] / numSamples - 1.0f;  // Z should be 1.0 (normalized gravity)
    
    // Calculate gyroscope offsets
    for(int i = 0; i < 3; i++) {
        gyro_offset[i] = gyro_sum[i] / numSamples;  // All should be 0
    }
    
    // Calculate magnetometer offsets
    for(int i = 0; i < 3; i++) {
        mag_offset[i] = mag_sum[i] / numSamples;
    }
    
    // Initialize and capture orientation offset
    updateSensors();
    updateQuaternion();
    
    // Store the inverse of the initial orientation as the offset
    q_offset[0] = q[0];
    q_offset[1] = -q[1];
    q_offset[2] = -q[2];
    q_offset[3] = -q[3];
    
    // Log offset information
    char buffer[50];
    
    sprintf(buffer, "Accel offsets: %.3f, %.3f, %.3f", accel_offset[0], accel_offset[1], accel_offset[2]);
    nh.loginfo(buffer);
    
    sprintf(buffer, "Gyro offsets: %.3f, %.3f, %.3f", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
    nh.loginfo(buffer);
    
    nh.loginfo("Calibration complete!");
    lastUpdate = micros();
}

void updateSensors() {
    uint8_t buffer[14];
    readBytes(MPU9250_ADDR, 0x3B, 14, buffer);
    
    // Read accelerometer (apply calibration)
    for(int i = 0; i < 3; i++) {
        accel[i] = ((float)((int16_t)((buffer[i*2] << 8) | buffer[i*2+1])) / 16384.0f) - accel_offset[i];
    }
    
    // Read gyroscope (apply calibration)
    for(int i = 0; i < 3; i++) {
        gyro[i] = ((float)((int16_t)((buffer[i*2+8] << 8) | buffer[i*2+9])) / 131.0f) - gyro_offset[i];
    }
    
    // Convert gyro values from degrees/sec to radians/sec for quaternion update
    for(int i = 0; i < 3; i++) {
        gyro[i] = gyro[i] * (PI / 180.0f);
    }
}

void updateQuaternion() {
    float dt = (float)(micros() - lastUpdate) / 1000000.0f;
    lastUpdate = micros();
    
    // Quaternion derivative from gyroscope
    float qDot[4];
    qDot[0] = 0.5f * (-q[1]*gyro[0] - q[2]*gyro[1] - q[3]*gyro[2]);
    qDot[1] = 0.5f * (q[0]*gyro[0] + q[2]*gyro[2] - q[3]*gyro[1]);
    qDot[2] = 0.5f * (q[0]*gyro[1] - q[1]*gyro[2] + q[3]*gyro[0]);
    qDot[3] = 0.5f * (q[0]*gyro[2] + q[1]*gyro[1] - q[2]*gyro[0]);
    
    // Integrate to get quaternion
    for(int i = 0; i < 4; i++) {
        q[i] += qDot[i] * dt;
    }
    
    // Normalize quaternion
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for(int i = 0; i < 4; i++) {
        q[i] /= norm;
    }
    
    // Apply calibration offset to quaternion (quaternion multiplication)
    // q_calibrated = q_offset * q
    q_calibrated[0] = q_offset[0]*q[0] - q_offset[1]*q[1] - q_offset[2]*q[2] - q_offset[3]*q[3];
    q_calibrated[1] = q_offset[0]*q[1] + q_offset[1]*q[0] + q_offset[2]*q[3] - q_offset[3]*q[2];
    q_calibrated[2] = q_offset[0]*q[2] - q_offset[1]*q[3] + q_offset[2]*q[0] + q_offset[3]*q[1];
    q_calibrated[3] = q_offset[0]*q[3] + q_offset[1]*q[2] - q_offset[2]*q[1] + q_offset[3]*q[0];
}

void updateOrientation() {
    updateQuaternion();
}

void publishImuData() {
    // Update header
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = nh.now();
    
    // Set orientation quaternion (using the calibrated quaternion)
    imu_msg.orientation.w = q_calibrated[0];
    imu_msg.orientation.x = q_calibrated[1];
    imu_msg.orientation.y = q_calibrated[2];
    imu_msg.orientation.z = q_calibrated[3];
    
    // Set angular velocity (gyro data in rad/s)
    imu_msg.angular_velocity.x = gyro[0];
    imu_msg.angular_velocity.y = gyro[1];
    imu_msg.angular_velocity.z = gyro[2];
    
    // Set linear acceleration in m/s²
    imu_msg.linear_acceleration.x = accel[0] * 9.8f;
    imu_msg.linear_acceleration.y = accel[1] * 9.8f;
    imu_msg.linear_acceleration.z = accel[2] * 9.8f;
    
    // Set covariance matrices (if unknown, set first element to -1)
    for(int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    // Diagonal elements represent variance
    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[8] = 0.0025;
    
    imu_msg.angular_velocity_covariance[0] = 0.002;
    imu_msg.angular_velocity_covariance[4] = 0.002;
    imu_msg.angular_velocity_covariance[8] = 0.002;
    
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[8] = 0.04;
    
    // Publish the message
    imu_pub.publish(&imu_msg);
}

void setup() {
    // Initialize ROS node
    nh.initNode();
    nh.advertise(imu_pub);
    
    while(!nh.connected()) {
        nh.spinOnce();
        delay(100);
    }
    
    nh.loginfo("Initializing MPU9250...");
    initMPU9250();
    calibrateSensors();
}

void loop() {
    updateSensors();
    updateOrientation();
    publishImuData();
    
    nh.spinOnce();
    delay(10);  // 100Hz update rate
}
