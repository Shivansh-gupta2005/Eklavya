#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

// MPU9250 I2C address
#define MPU9250_ADDR 0x68
#define MAG_ADDR 0x0C

// ESP32 I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

// ROS communication setup for ESP32
class ESP32Hardware {
  public:
    ESP32Hardware() {}

    void init() {
      // Configure ESP32 Serial port for ROS
      Serial.begin(57600);  // Changed to 57600 to match rosserial default
    }

    int read() {
      return Serial.read();
    }

    void write(uint8_t* data, int length) {
      Serial.write(data, length);
    }

    unsigned long time() {
      return millis();
    }

    bool connected() {
      return Serial;
    }
} ;

ros::NodeHandle_<ESP32Hardware> nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

// Sensor variables
float accel_offset[3] = {0, 0, 0};
float gyro_offset[3] = {0, 0, 0};
float mag_offset[3] = {0, 0, 0};
float mag_scale[3] = {1, 1, 1};
float accel[3], gyro[3], mag[3];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
unsigned long lastUpdate;

// Basic I2C functions
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
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    delay(100);
    
    writeByte(MPU9250_ADDR, 0x6B, 0x00);  // Wake up
    delay(100);
    writeByte(MPU9250_ADDR, 0x1C, 0x00);  // Accelerometer ±2g
    writeByte(MPU9250_ADDR, 0x1B, 0x00);  // Gyroscope ±250°/s
    writeByte(MPU9250_ADDR, 0x37, 0x02);  // Enable magnetometer
    delay(100);
}

void calibrateSensors() {
    nh.loginfo("Starting calibration...");
    float accel_sum[3] = {0}, gyro_sum[3] = {0};
    float mag_min[3] = {99999, 99999, 99999};
    float mag_max[3] = {-99999, -99999, -99999};
    
    for(int i = 0; i < 1000; i++) {
        uint8_t buffer[14];
        readBytes(MPU9250_ADDR, 0x3B, 14, buffer);
        
        for(int j = 0; j < 3; j++) {
            float accel_temp = (float)((int16_t)((buffer[j*2] << 8) | buffer[j*2+1])) / 16384.0f;
            float gyro_temp = (float)((int16_t)((buffer[j*2+8] << 8) | buffer[j*2+9])) / 131.0f;
            accel_sum[j] += accel_temp;
            gyro_sum[j] += gyro_temp;
        }
        
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
    
    for(int i = 0; i < 3; i++) {
        accel_offset[i] = accel_sum[i] / 1000.0f;
        gyro_offset[i] = gyro_sum[i] / 1000.0f;
        mag_offset[i] = (mag_max[i] + mag_min[i]) / 2.0f;
        mag_scale[i] = (mag_max[i] - mag_min[i]) / 2.0f;
    }
    
    lastUpdate = micros();
    nh.loginfo("Calibration done!");
}

void updateSensors() {
    uint8_t buffer[14];
    readBytes(MPU9250_ADDR, 0x3B, 14, buffer);
    
    for(int i = 0; i < 3; i++) {
        accel[i] = (((float)((int16_t)((buffer[i*2] << 8) | buffer[i*2+1])) / 16384.0f) - accel_offset[i]) * 9.81;
        gyro[i] = (((float)((int16_t)((buffer[i*2+8] << 8) | buffer[i*2+9])) / 131.0f) - gyro_offset[i]) * PI / 180.0f;
    }
}

void updateOrientation() {
    float dt = (float)(micros() - lastUpdate) / 1000000.0f;
    lastUpdate = micros();
    
    float qDot[4];
    qDot[0] = 0.5f * (-q[1]*gyro[0] - q[2]*gyro[1] - q[3]*gyro[2]);
    qDot[1] = 0.5f * (q[0]*gyro[0] + q[2]*gyro[2] - q[3]*gyro[1]);
    qDot[2] = 0.5f * (q[0]*gyro[1] - q[1]*gyro[2] + q[3]*gyro[0]);
    qDot[3] = 0.5f * (q[0]*gyro[2] + q[1]*gyro[1] - q[2]*gyro[0]);
    
    for(int i = 0; i < 4; i++) {
        q[i] += qDot[i] * dt;
    }
    
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for(int i = 0; i < 4; i++) {
        q[i] /= norm;
    }
}

void publishImuData() {
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = nh.now();
    
    imu_msg.orientation.w = q[0];
    imu_msg.orientation.x = q[1];
    imu_msg.orientation.y = q[2];
    imu_msg.orientation.z = q[3];
    
    imu_msg.angular_velocity.x = gyro[0];
    imu_msg.angular_velocity.y = gyro[1];
    imu_msg.angular_velocity.z = gyro[2];
    
    imu_msg.linear_acceleration.x = accel[0];
    imu_msg.linear_acceleration.y = accel[1];
    imu_msg.linear_acceleration.z = accel[2];
    
    // Set constant covariance
    for(int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    
    imu_pub.publish(&imu_msg);
}

void setup() {
    // Initialize ROS node
    nh.initNode();
    nh.advertise(imu_pub);
    
    // Wait for ROS connection
    while(!nh.connected()) {
        nh.spinOnce();
        delay(100);
    }
    
    initMPU9250();
    calibrateSensors();
}

void loop() {
    if (nh.connected()) {
        updateSensors();
        updateOrientation();
        publishImuData();
        nh.spinOnce();
        delay(10);  // 100Hz
    } else {
        nh.initNode();
        delay(1000);
    }
}
