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

// Sensor data
float accel[3], gyro[3], mag[3];

// Orientation data
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // Quaternion
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
    nh.loginfo("Starting calibration... Keep sensor still!");
    delay(2000);
    
    float accel_sum[3] = {0}, gyro_sum[3] = {0};
    float mag_min[3] = {99999, 99999, 99999};
    float mag_max[3] = {-99999, -99999, -99999};
    
    // Collect 1000 samples
    for(int i = 0; i < 1000; i++) {
        uint8_t buffer[14];
        readBytes(MPU9250_ADDR, 0x3B, 14, buffer);
        
        // Process accelerometer
        for(int j = 0; j < 3; j++) {
            float accel_temp = (float)((int16_t)((buffer[j*2] << 8) | buffer[j*2+1])) / 16384.0f;
            accel_sum[j] += accel_temp;
        }
        
        // Process gyroscope
        for(int j = 0; j < 3; j++) {
            float gyro_temp = (float)((int16_t)((buffer[j*2+8] << 8) | buffer[j*2+9])) / 131.0f;
            gyro_sum[j] += gyro_temp;
        }
        
        // Read magnetometer
        uint8_t mag_buffer[7];
        readBytes(MAG_ADDR, 0x03, 7, mag_buffer);
        
        float mag_temp[3];
        mag_temp[0] = (float)((int16_t)(mag_buffer[1] << 8 | mag_buffer[0])) * 0.15f;
        mag_temp[1] = (float)((int16_t)(mag_buffer[3] << 8 | mag_buffer[2])) * 0.15f;
        mag_temp[2] = (float)((int16_t)(mag_buffer[5] << 8 | mag_buffer[4])) * 0.15f;
        
        for(int j = 0; j < 3; j++) {
            mag_min[j] = min(mag_min[j], mag_temp[j]);
            mag_max[j] = max(mag_max[j], mag_temp[j]);
        }
        
        delay(5);
    }
    
    // Calculate offsets
    for(int i = 0; i < 3; i++) {
        accel_offset[i] = accel_sum[i] / 1000.0f;
        gyro_offset[i] = gyro_sum[i] / 1000.0f;
        mag_offset[i] = (mag_max[i] + mag_min[i]) / 2.0f;
        mag_scale[i] = (mag_max[i] - mag_min[i]) / 2.0f;
    }
    
    nh.loginfo("Calibration complete!");
    lastUpdate = micros();
}

void updateSensors() {
    uint8_t buffer[14];
    readBytes(MPU9250_ADDR, 0x3B, 14, buffer);
    
    // Read accelerometer (convert to m/s^2)
    for(int i = 0; i < 3; i++) {
        accel[i] = (((float)((int16_t)((buffer[i*2] << 8) | buffer[i*2+1])) / 16384.0f) - accel_offset[i]) * 9.81;
    }
    
    // Read gyroscope (convert to rad/s)
    for(int i = 0; i < 3; i++) {
        gyro[i] = (((float)((int16_t)((buffer[i*2+8] << 8) | buffer[i*2+9])) / 131.0f) - gyro_offset[i]) * PI / 180.0f;
    }
    
    // Read magnetometer
    uint8_t mag_buffer[7];
    readBytes(MAG_ADDR, 0x03, 7, mag_buffer);
    
    mag[0] = ((float)((int16_t)(mag_buffer[1] << 8 | mag_buffer[0])) * 0.15f - mag_offset[0]) / mag_scale[0];
    mag[1] = ((float)((int16_t)(mag_buffer[3] << 8 | mag_buffer[2])) * 0.15f - mag_offset[1]) / mag_scale[1];
    mag[2] = ((float)((int16_t)(mag_buffer[5] << 8 | mag_buffer[4])) * 0.15f - mag_offset[2]) / mag_scale[2];
}

void updateOrientation() {
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
}

void publishImuData() {
    // Update header
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = nh.now();
    
    // Set orientation quaternion
    imu_msg.orientation.w = q[0];
    imu_msg.orientation.x = q[1];
    imu_msg.orientation.y = q[2];
    imu_msg.orientation.z = q[3];
    
    // Set angular velocity (gyro data)
    imu_msg.angular_velocity.x = gyro[0];
    imu_msg.angular_velocity.y = gyro[1];
    imu_msg.angular_velocity.z = gyro[2];
    
    // Set linear acceleration
    imu_msg.linear_acceleration.x = accel[0];
    imu_msg.linear_acceleration.y = accel[1];
    imu_msg.linear_acceleration.z = accel[2];
    
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
