#include <Wire.h>

// MPU9250 I2C address
#define MPU9250_ADDR 0x68
#define MAG_ADDR 0x0C

// Calibration variables
float accel_offset[3] = {0, 0, 0};
float gyro_offset[3] = {0, 0, 0};
float mag_offset[3] = {0, 0, 0};
float mag_scale[3] = {1, 1, 1};

// Sensor data
float accel[3], gyro[3], mag[3];

// Orientation data
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // Quaternion
float euler[3] = {0.0f, 0.0f, 0.0f};       // Roll, pitch, yaw
unsigned long lastUpdate;
const float sampleRate = 100.0f;            // Hz

// Function to normalize angles to specified ranges
float normalizeAngle(float angle, float min_angle, float max_angle) {
    float range = max_angle - min_angle;
    float normalized = fmod(angle - min_angle, range);
    if (normalized < 0) normalized += range;
    return normalized + min_angle;
}

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
    Serial.println("Starting calibration... Keep sensor still!");
    delay(2000);
    
    // Collect 1000 samples with the sensor in a known orientation
    float accel_sum[3] = {0};
    
    for(int i = 0; i < 1000; i++) {
        uint8_t buffer[14];
        readBytes(MPU9250_ADDR, 0x3B, 14, buffer);
        
        // Process accelerometer
        for(int j = 0; j < 3; j++) {
            float accel_temp = (float)((int16_t)((buffer[j*2] << 8) | buffer[j*2+1])) / 16384.0f;
            accel_sum[j] += accel_temp;
        }
        
        delay(5);
    }
    
    // Calculate offsets assuming the sensor was flat with Z-axis up
    for(int i = 0; i < 3; i++) {
        accel_offset[i] = accel_sum[i] / 1000.0f;
    }
    
    // Adjust Z-axis offset to reflect expected gravity
    if (accel_offset[2] > 0) {
        // Sensor was likely flat with Z-axis up
        accel_offset[2] -= 9.8f; // Adjust for gravity
    } else {
        // Sensor might have been inverted
        accel_offset[2] += 9.8f; // Adjust for inverted gravity
    }
    
    Serial.println("Calibration complete!");
    lastUpdate = micros();
}


void updateSensors() {
    uint8_t buffer[14];
    readBytes(MPU9250_ADDR, 0x3B, 14, buffer);
    
    // Read accelerometer
    for(int i = 0; i < 3; i++) {
        accel[i] = ((float)((int16_t)((buffer[i*2] << 8) | buffer[i*2+1])) / 16384.0f) - accel_offset[i];
    }
    
    // Read gyroscope
    for(int i = 0; i < 3; i++) {
        gyro[i] = ((float)((int16_t)((buffer[i*2+8] << 8) | buffer[i*2+9])) / 131.0f) - gyro_offset[i];
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
    
    // Convert gyro values to rad/sec
    float gx = gyro[0] * PI / 180.0f;
    float gy = gyro[1] * PI / 180.0f;
    float gz = gyro[2] * PI / 180.0f;
    
    // Quaternion derivative from gyroscope
    float qDot[4];
    qDot[0] = 0.5f * (-q[1]*gx - q[2]*gy - q[3]*gz);
    qDot[1] = 0.5f * (q[0]*gx + q[2]*gz - q[3]*gy);
    qDot[2] = 0.5f * (q[0]*gy - q[1]*gz + q[3]*gx);
    qDot[3] = 0.5f * (q[0]*gz + q[1]*gy - q[2]*gx);
    
    // Integrate to get quaternion
    for(int i = 0; i < 4; i++) {
        q[i] += qDot[i] * dt;
    }
    
    // Normalize quaternion
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for(int i = 0; i < 4; i++) {
        q[i] /= norm;
    }
    
    // Convert to Euler angles (in degrees)
    float roll = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2])) * 180.0f/PI;
    float pitch = asin(2*(q[0]*q[2] - q[3]*q[1])) * 180.0f/PI;
    float yaw = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3])) * 180.0f/PI;
    
    // Normalize angles to proper ranges
    euler[0] = normalizeAngle(roll, -180.0f, 180.0f);     // Roll: -180 to +180
    euler[1] = normalizeAngle(pitch, -90.0f, 90.0f);      // Pitch: -90 to +90
    euler[2] = normalizeAngle(yaw + 180.0f, 0.0f, 360.0f); // Yaw: 0 to 360
}

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);
    
    Serial.println("Initializing MPU9250...");
    initMPU9250();
    calibrateSensors();
}

void loop() {
    updateSensors();
    updateOrientation();
    
    // Print Euler angles
    Serial.print("Roll: "); Serial.print(euler[0], 1);
    Serial.print(" Pitch: "); Serial.print(euler[1], 1);
    Serial.print(" Yaw: "); Serial.print(euler[2], 1);

    Serial.print("Acceleration in z:"); Serial.print(accel[2]);
    
    // Print quaternion
    Serial.print(" Quaternion: ");
    Serial.print(q[0], 3); Serial.print(", ");
    Serial.print(q[1], 3); Serial.print(", ");
    Serial.print(q[2], 3); Serial.print(", ");
    Serial.println(q[3], 3);
    
    delay(10);  // 100Hz update rate
}
