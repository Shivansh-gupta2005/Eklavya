#include <Wire.h>
#include <ros.h>
#include <IntervalTimer.h>
#include <TinyGPS++.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <LiquidCrystal_I2C.h>


// Create two interval timers
IntervalTimer encoderTimer;
IntervalTimer imuTimer;
IntervalTimer gpsTimer;

LiquidCrystal_I2C lcd(0x27, 16, 2);

TinyGPSPlus gps;
#define gpsSerial Serial1

// MPU9250 I2C address
#define MPU9250_ADDR 0x68
#define MAG_ADDR 0x0C

// Encoder pins
#define ENCODER1_PIN_A 21
#define ENCODER1_PIN_B 23
#define ENCODER2_PIN_A 2
#define ENCODER2_PIN_B 3
#define ENCODER3_PIN_A 4
#define ENCODER3_PIN_B 5
#define ENCODER4_PIN_A 16
#define ENCODER4_PIN_B 17

// Motor pins
#define DIR1 9
#define PWM1 8
#define DIR2 7
#define PWM2 6
#define DIR3 14
#define PWM3 15
#define DIR4 20
#define PWM4 22

// Global variables for encoders
volatile int encoder1Ticks = 0;
volatile int encoder2Ticks = 0;
volatile int encoder3Ticks = 0;
volatile int encoder4Ticks = 0;

// Global variables for IMU
float accel[3], gyro[3], mag[3];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float accel_offset[3] = {0, 0, 0};
float gyro_offset[3] = {0, 0, 0};
float mag_offset[3] = {0, 0, 0};
float mag_scale[3] = {1, 1, 1};
unsigned long lastUpdate;

// ROS node handle
ros::NodeHandle nh;

sensor_msgs::NavSatFix navsat_msg;
ros::Publisher navsat_pub("navsatfix", &navsat_msg);

// ROS messages
sensor_msgs::Imu imu_msg;
std_msgs::Int32 encoder1_msg;
std_msgs::Int32 encoder2_msg;
std_msgs::Int32 encoder3_msg;
std_msgs::Int32 encoder4_msg;

// ROS publishers
ros::Publisher imu_pub("imu/data", &imu_msg);
ros::Publisher encoder1_pub("encoder1", &encoder1_msg);
ros::Publisher encoder2_pub("encoder2", &encoder2_msg);
ros::Publisher encoder3_pub("encoder3", &encoder3_msg);
ros::Publisher encoder4_pub("encoder4", &encoder4_msg);


void commandCallback(const std_msgs::String& cmd_msg);
ros::Subscriber<std_msgs::String> sub("robot_command", commandCallback);

// Function declarations
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

// Encoder ISR functions
void encoder1ISR() {
    int stateA = digitalRead(ENCODER1_PIN_A);
    int stateB = digitalRead(ENCODER1_PIN_B);
    if (stateA == stateB) encoder1Ticks++; else encoder1Ticks--;
}

void encoder2ISR() {
    int stateA = digitalRead(ENCODER2_PIN_A);
    int stateB = digitalRead(ENCODER2_PIN_B);
    if (stateA == stateB) encoder2Ticks++; else encoder2Ticks--;
}

void encoder3ISR() {
    int stateA = digitalRead(ENCODER3_PIN_A);
    int stateB = digitalRead(ENCODER3_PIN_B);
    if (stateA == stateB) encoder3Ticks++; else encoder3Ticks--;
}

void encoder4ISR() {
    int stateA = digitalRead(ENCODER4_PIN_A);
    int stateB = digitalRead(ENCODER4_PIN_B);
    if (stateA == stateB) encoder4Ticks++; else encoder4Ticks--;
}

// Timer callback functions
void encoderCallback() {
    encoder1_msg.data = encoder1Ticks;
    encoder2_msg.data = encoder2Ticks;
    encoder3_msg.data = encoder3Ticks;
    encoder4_msg.data = encoder4Ticks;
    
    encoder1_pub.publish(&encoder1_msg);
    encoder2_pub.publish(&encoder2_msg);
    encoder3_pub.publish(&encoder3_msg);
    encoder4_pub.publish(&encoder4_msg);
}  

void imuCallback() {
    
    updateSensors();
    updateOrientation();
    publishImuData();
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

void publishEncoderData() {
  // Publish encoder tick counts
  encoder1_msg.data = encoder1Ticks;
  encoder2_msg.data = encoder2Ticks;
  encoder3_msg.data = encoder3Ticks;
  encoder4_msg.data = encoder4Ticks;

  encoder1_pub.publish(&encoder1_msg);
  encoder2_pub.publish(&encoder2_msg);
  encoder3_pub.publish(&encoder3_msg);
  encoder4_pub.publish(&encoder4_msg);
}

void moveforward() {
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, HIGH);
  analogWrite(PWM1, 1023);  
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 1023);  
}

void moveleft() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);
  digitalWrite(DIR3, HIGH);
  digitalWrite(DIR4, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 1023);  
  analogWrite(PWM3, 1023);  
  analogWrite(PWM4, 0);
}

void movebackward() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, LOW);
  analogWrite(PWM1, 1023);  
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 1023); 
}

void moveright() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 1023);  
  analogWrite(PWM3, 1023);  
  analogWrite(PWM4, 0);
}

void spinleft() {
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, HIGH);
  digitalWrite(DIR4, LOW);
  analogWrite(PWM1, 1023);  
  analogWrite(PWM2, 1023);  
  analogWrite(PWM3, 1023);  
  analogWrite(PWM4, 1023);  
}

void spinright() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, HIGH);
  analogWrite(PWM1, 1023);
  analogWrite(PWM2, 1023);  
  analogWrite(PWM3, 1023);  
  analogWrite(PWM4, 1023);  
}

void diagonalforward(){
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, HIGH);
  digitalWrite(DIR3, HIGH);
  digitalWrite(DIR4, HIGH);
  analogWrite(PWM1, 1023);
  analogWrite(PWM2, 1023);  
  analogWrite(PWM3, 1023);  
  analogWrite(PWM4, 1023);  
}


void diagonalbackward() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, LOW);
  analogWrite(PWM1, 1023);
  analogWrite(PWM2, 1023);  
  analogWrite(PWM3, 1023);  
  analogWrite(PWM4, 1023);  
}


void stopmotors() {
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 0);
}

void commandCallback(const std_msgs::String& cmd_msg) {
  char command = cmd_msg.data[0];
  
  switch(command) {
    case 'w': case 'W': moveforward(); break;
    case 's': case 'S': movebackward(); break;
    case 'a': case 'A': moveleft(); break;
    case 'd': case 'D': moveright(); break;
    case 'q': case 'Q': spinleft(); break;
    case 'e': case 'E': spinright(); break;
    case 'g': case 'G': diagonalforward(); break;
    case 'h': case 'H': diagonalbackward(); break;
    case 'x': case 'X': default: stopmotors(); break;
  }
}

void gpsCallback() {
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c);
    }
    
    if (gps.location.isUpdated()) {
        navsat_msg.header.stamp = nh.now();
        navsat_msg.header.frame_id = "gps_link";
        
        navsat_msg.latitude = gps.location.lat();
        navsat_msg.longitude = gps.location.lng();
        navsat_msg.altitude = gps.altitude.meters();
        
        navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        navsat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        
        navsat_pub.publish(&navsat_msg);
    }
}

void setup() {

    lcd.init();
    lcd.backlight();
    
    lcd.setCursor(4, 0);
    lcd.print("Eklavya");
    
    lcd.setCursor(0, 1);
    lcd.print("Robo Club IITK");
    
    gpsSerial.begin(9600);// Initialize ROS node
    nh.initNode();
    
    // Advertise all topics
    nh.advertise(imu_pub);
    nh.advertise(encoder1_pub);
    nh.advertise(encoder2_pub);
    nh.advertise(encoder3_pub);
    nh.advertise(encoder4_pub);
    nh.advertise(navsat_pub);
    nh.subscribe(sub);
    
    // Configure encoder pins
    pinMode(ENCODER1_PIN_A, INPUT);
    pinMode(ENCODER1_PIN_B, INPUT);
    pinMode(ENCODER2_PIN_A, INPUT);
    pinMode(ENCODER2_PIN_B, INPUT);
    pinMode(ENCODER3_PIN_A, INPUT);
    pinMode(ENCODER3_PIN_B, INPUT);
    pinMode(ENCODER4_PIN_A, INPUT);
    pinMode(ENCODER4_PIN_B, INPUT);


    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR3, OUTPUT);
    pinMode(PWM3, OUTPUT);
    pinMode(DIR4, OUTPUT);
    pinMode(PWM4, OUTPUT);
    stopmotors();

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_PIN_A), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_PIN_A), encoder4ISR, CHANGE);
    
    
    // Initialize IMU
    initMPU9250();
    calibrateSensors();
    lastUpdate = micros();
    
    // Start interval timers
    encoderTimer.begin(encoderCallback, 100000);  // 100ms = 10Hz for encoders
    imuTimer.begin(imuCallback, 10000);   
    gpsTimer.begin(gpsCallback, 10);        // 10ms = 100Hz for IMU
}

void loop() {
    nh.spinOnce();
     // Give time for other tasks
}
