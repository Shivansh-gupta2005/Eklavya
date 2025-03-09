#include <Wire.h>

// MPU9250 I2C address
#define MPU9250_ADDR 0x68
#define MAG_ADDR 0x0C

// Calibration variables
float accel_offset[3] = {0, 0, 0};
float gyro_offset[3] = {0, 0, 0};
float mag_offset[3] = {0, 0, 0};
float mag_scale[3] = {1, 1, 1};

// Orientation offset for zeroing
float q_offset[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float euler_offset[3] = {0.0f, 0.0f, 0.0f};

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
    Serial.println("Starting calibration... Keep sensor still and level!");
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
        
        // Process magnetometer
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
    accel_offset[2] = accel_sum[2] / numSamples - 1.0f;  // Z should be 1.0 (9.8/9.8)
    
    // Calculate gyroscope offsets
    for(int i = 0; i < 3; i++) {
        gyro_offset[i] = gyro_sum[i] / numSamples;  // All should be 0
    }
    
    // Calculate magnetometer offsets
    for(int i = 0; i < 3; i++) {
        mag_offset[i] = mag_sum[i] / numSamples;
    }
    
    // Initialize quaternions and capture orientation offset
    updateSensors();
    updateQuaternion();
    
    // Store the inverse of the initial orientation as an offset
    q_offset[0] = q[0];
    q_offset[1] = -q[1];
    q_offset[2] = -q[2];
    q_offset[3] = -q[3];
    
    // Store Euler angles offset
    for(int i = 0; i < 3; i++) {
        euler_offset[i] = euler[i];
    }
    
    Serial.println("Calibration complete!");
    Serial.print("Accel offsets: ");
    Serial.print(accel_offset[0]); Serial.print(", ");
    Serial.print(accel_offset[1]); Serial.print(", ");
    Serial.println(accel_offset[2]);
    
    Serial.print("Gyro offsets: ");
    Serial.print(gyro_offset[0]); Serial.print(", ");
    Serial.print(gyro_offset[1]); Serial.print(", ");
    Serial.println(gyro_offset[2]);
    
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
    
    // Read magnetometer (apply calibration)
    uint8_t mag_buffer[7];
    readBytes(MAG_ADDR, 0x03, 7, mag_buffer);
    
    for(int i = 0; i < 3; i++) {
        int idx[3] = {0, 2, 4}; // X, Y, Z indices
        mag[i] = (((float)((int16_t)(mag_buffer[idx[i]+1] << 8 | mag_buffer[idx[i]])) * 0.15f) - mag_offset[i]) / mag_scale[i];
    }
}

// New function to update quaternion without Euler angles immediately
void updateQuaternion() {
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
}

void updateOrientation() {
    // First update the quaternion
    updateQuaternion();
    
    // Apply the offset quaternion (compensate for initial orientation)
    float q_temp[4];
    // Quaternion multiplication: q_result = q_offset * q
    q_temp[0] = q_offset[0]*q[0] - q_offset[1]*q[1] - q_offset[2]*q[2] - q_offset[3]*q[3];
    q_temp[1] = q_offset[0]*q[1] + q_offset[1]*q[0] + q_offset[2]*q[3] - q_offset[3]*q[2];
    q_temp[2] = q_offset[0]*q[2] - q_offset[1]*q[3] + q_offset[2]*q[0] + q_offset[3]*q[1];
    q_temp[3] = q_offset[0]*q[3] + q_offset[1]*q[2] - q_offset[2]*q[1] + q_offset[3]*q[0];
    
    // Convert corrected quaternion to Euler angles (in degrees)
    float roll = atan2(2*(q_temp[0]*q_temp[1] + q_temp[2]*q_temp[3]), 1 - 2*(q_temp[1]*q_temp[1] + q_temp[2]*q_temp[2])) * 180.0f/PI;
    float pitch = asin(2*(q_temp[0]*q_temp[2] - q_temp[3]*q_temp[1])) * 180.0f/PI;
    float yaw = atan2(2*(q_temp[0]*q_temp[3] + q_temp[1]*q_temp[2]), 1 - 2*(q_temp[2]*q_temp[2] + q_temp[3]*q_temp[3])) * 180.0f/PI;
    
    // Normalize angles to proper ranges
    euler[0] = normalizeAngle(roll, -180.0f, 180.0f);     // Roll: -180 to +180
    euler[1] = normalizeAngle(pitch, -90.0f, 90.0f);      // Pitch: -90 to +90
    euler[2] = normalizeAngle(yaw, 0.0f, 360.0f);         // Yaw: 0 to 360
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
    
    // Convert acceleration to m/s² for display
    float acc_ms2[3];
    for(int i = 0; i < 3; i++) {
        acc_ms2[i] = accel[i] * 9.8f;
    }
    
    // Print Euler angles
    Serial.print("Roll: "); Serial.print(euler[0], 1);
    Serial.print(" Pitch: "); Serial.print(euler[1], 1);
    Serial.print(" Yaw: "); Serial.print(euler[2], 1);
    
    // Print gyroscope data (should be near zero when static)
    Serial.print(" | Gyro (°/s): ");
    Serial.print(gyro[0], 1); Serial.print(", ");
    Serial.print(gyro[1], 1); Serial.print(", ");
    Serial.print(gyro[2], 1);
    
    // Print accelerometer data (should be near 0,0,9.8 when level)
    Serial.print(" | Accel (m/s²): ");
    Serial.print(acc_ms2[0], 2); Serial.print(", ");
    Serial.print(acc_ms2[1], 2); Serial.print(", ");
    Serial.print(acc_ms2[2], 2);
    
    // Print quaternion if desired
    Serial.print(" | Quaternion: ");
    Serial.print(q_temp[0], 3); Serial.print(", ");
    Serial.print(q_temp[1], 3); Serial.print(", ");
    Serial.print(q_temp[2], 3); Serial.print(", ");
    Serial.println(q_temp[3], 3);
    
    delay(10);  // 100Hz update rate
}
