#include <Wire.h>
#include <ros.h>
#include <IntervalTimer.h>
#include <TinyGPS++.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <LiquidCrystal_I2C.h>

// Create interval timers
IntervalTimer encoderTimer;
IntervalTimer imuTimer;
IntervalTimer gpsTimer;
IntervalTimer odometryTimer;

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

// Motor pins - using pins from the second file
#define DIR1 9
#define PWM1 8
#define DIR2 7
#define PWM2 6
#define DIR3 14
#define PWM3 15
#define DIR4 20
#define PWM4 22

// Robot parameters
#define WHEEL_RADIUS 0.076 // meters
#define ROBOT_RADIUS 0.268  // meters (distance from center to wheel)
#define MAX_PWM 255
#define GEAR_RATIO 50.9
#define ENCODER_PPR 714 // Pulses per revolution

// Global variables for encoders
volatile int encoder1Ticks = 0;
volatile int encoder2Ticks = 0;
volatile int encoder3Ticks = 0;
volatile int encoder4Ticks = 0;

volatile int prev_encoder1Ticks = 0;
volatile int prev_encoder2Ticks = 0;
volatile int prev_encoder3Ticks = 0;
volatile int prev_encoder4Ticks = 0;

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



// Odometry variables
float x = 0.0;
float y = 0.0;
float theta = 0.0;
float vx = 0.0;
float vy = 0.0;
float vtheta = 0.0;
unsigned long last_odom_time = 0;

// Wheel velocities
float v1 = 0, v2 = 0, v3 = 0, v4 = 0; // Individual wheel velocities

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
// nav_msgs::Odometry odom_msg;

// ROS publishers
ros::Publisher imu_pub("imu/data", &imu_msg);
ros::Publisher encoder1_pub("encoder1", &encoder1_msg);
ros::Publisher encoder2_pub("encoder2", &encoder2_msg);
ros::Publisher encoder3_pub("encoder3", &encoder3_msg);
ros::Publisher encoder4_pub("encoder4", &encoder4_msg);
// ros::Publisher odom_pub("odom", &odom_msg);

// // TF broadcaster
// tf::TransformBroadcaster tf_broadcaster;


void cmdVelCallback(const geometry_msgs::Twist& twist_msg);


ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", cmdVelCallback);

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



// // Convert encoder ticks to distance
// float ticksToDistance(int ticks) {
//     return (ticks * 2.0 * PI * WHEEL_RADIUS) / (ENCODER_PPR);
// }

// void updateOdometry() {
//     unsigned long current_time = millis();
//     float dt = (current_time - last_odom_time) / 1000.0; // Convert to seconds
    
//     if (dt <= 0) return;
    
//     last_odom_time = current_time;
    
//     // Calculate wheel velocities based on encoder changes
//     int delta_ticks1 = encoder1Ticks - prev_encoder1Ticks;
//     int delta_ticks2 = encoder2Ticks - prev_encoder2Ticks;
//     int delta_ticks3 = encoder3Ticks - prev_encoder3Ticks;
//     int delta_ticks4 = encoder4Ticks - prev_encoder4Ticks;
    
//     prev_encoder1Ticks = encoder1Ticks;
//     prev_encoder2Ticks = encoder2Ticks;
//     prev_encoder3Ticks = encoder3Ticks;
//     prev_encoder4Ticks = encoder4Ticks;
    
//     float delta_dist1 = ticksToDistance(delta_ticks1);
//     float delta_dist2 = ticksToDistance(delta_ticks2);
//     float delta_dist3 = ticksToDistance(delta_ticks3);
//     float delta_dist4 = ticksToDistance(delta_ticks4);
    
//     float wheel_v1 = delta_dist1 / dt;
//     float wheel_v2 = delta_dist2 / dt;
//     float wheel_v3 = delta_dist3 / dt;
//     float wheel_v4 = delta_dist4 / dt;
    
//     // Calculate robot velocities
//     vx = (wheel_v1 + wheel_v3) / 2.0;
//     vy = (wheel_v2 + wheel_v4) / 2.0;
//     vtheta = (-wheel_v1 + wheel_v2 + wheel_v3 - wheel_v4) / (2.0 * ROBOT_RADIUS);
    
//     // Integrate velocities to update position and orientation in the robot's frame
//     float delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
//     float delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
//     float delta_theta = vtheta * dt;
    
//     x += delta_x;
//     y += delta_y;
//     theta += delta_theta;
    
//     // Normalize angle to [-pi, pi]
//     while (theta > PI) theta -= 2.0 * PI;
//     while (theta < -PI) theta += 2.0 * PI;
// }

// void publishOdometry() {
//     // Create quaternion from yaw
//     geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);
    
//     // Publish transform first
//     geometry_msgs::TransformStamped t;
//     t.header.stamp = nh.now();
//     t.header.frame_id = "odom";
//     t.child_frame_id = "base_link";
    
//     t.transform.translation.x = x;
//     t.transform.translation.y = y;
//     t.transform.translation.z = 0.0;
//     t.transform.rotation = odom_quat;
    
//     tf_broadcaster.sendTransform(t);
    
//     // Publish odometry message
//     odom_msg.header.stamp = nh.now();
//     odom_msg.header.frame_id = "odom";
//     odom_msg.child_frame_id = "base_link";
    
//     // Set position
//     odom_msg.pose.pose.position.x = x;
//     odom_msg.pose.pose.position.y = y;
//     odom_msg.pose.pose.position.z = 0.0;
//     odom_msg.pose.pose.orientation = odom_quat;
    
//     // Set velocity
//     odom_msg.twist.twist.linear.x = vx;
//     odom_msg.twist.twist.linear.y = vy;
//     odom_msg.twist.twist.angular.z = vtheta;
    
//     // Publish odometry
//     odom_pub.publish(&odom_msg);
// }

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
    imu_msg.orientation_covariance[0] = 0.00;
    imu_msg.orientation_covariance[4] = 0.00;
    imu_msg.orientation_covariance[8] = 0.00;
    
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;
    
    imu_msg.linear_acceleration_covariance[0] = 1;
    imu_msg.linear_acceleration_covariance[4] = 1;
    imu_msg.linear_acceleration_covariance[8] = 1;
    
    // Publish the message
    imu_pub.publish(&imu_msg);
}

void imuCallback() {
    updateSensors();
    updateOrientation();
    publishImuData();
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

// Convert velocity to PWM using the quadratic formula
int velocityToPWM(float velocity) {
  // Convert from linear velocity to RPM
  float rpm = abs(velocity) / (2 * PI * WHEEL_RADIUS) * 60;
  
  // Use the quadratic formula: PWM = (-a + sqrt(a² + 4*b*rpm)) / (2*b)
  const float a = 0.481;              // Linear coefficient
  const float b = 4.87e-5;            // Quadratic coefficient
  
  float discriminant = (a * a) + (4 * b * rpm);
  float pwm = 0;
  
  if (discriminant >= 0) {
    pwm = (-a + sqrt(discriminant)) / (2 * b);
  }
  
  // Constrain PWM values to valid range
  pwm = constrain(pwm, 0, MAX_PWM);
  
  return (int)pwm;
}


void cmdVelCallback(const geometry_msgs::Twist& twist_msg) {
  // Extract linear and angular velocities
  float linear_x = twist_msg.linear.x;
  float linear_y = twist_msg.linear.y;
  float angular_z = twist_msg.angular.z;
  
  // Calculate wheel velocities using mecanum wheel kinematics
  v1 = linear_x - angular_z * ROBOT_RADIUS; // Front Left
  v2 = linear_y + angular_z * ROBOT_RADIUS; // Front Right
  v3 = linear_x - angular_z * ROBOT_RADIUS; // Rear Left
  v4 = linear_y - angular_z * ROBOT_RADIUS; // Rear Right
  
  // Convert velocity to PWM values
  int pwm1 = velocityToPWM(v1);
  int pwm2 = velocityToPWM(v2);
  int pwm3 = velocityToPWM(v3);
  int pwm4 = velocityToPWM(v4);
  
  // Set motor directions
  digitalWrite(DIR1, v1 >= 0 ? HIGH : LOW);
  digitalWrite(DIR2, v2 >= 0 ? HIGH : LOW);
  digitalWrite(DIR3, v3 >= 0 ? HIGH : LOW);
  digitalWrite(DIR4, v4 >= 0 ? HIGH : LOW);
  
  // Apply PWM values to motors
  analogWrite(PWM1, pwm1);
  analogWrite(PWM2, pwm2);
  analogWrite(PWM3, pwm3);
  analogWrite(PWM4, pwm4);
}

void setup() {
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    
    lcd.setCursor(4, 0);
    lcd.print("Eklavya");
    
    lcd.setCursor(0, 1);
    lcd.print("Robo Club IITK");
    
    // Initialize serial
    Serial.begin(9600);
    gpsSerial.begin(9600);
    
    // Initialize ROS node
    nh.initNode();
    
    // Advertise all topics
    nh.advertise(imu_pub);
    nh.advertise(encoder1_pub);
    nh.advertise(encoder2_pub);
    nh.advertise(encoder3_pub);
    nh.advertise(encoder4_pub);
    nh.advertise(navsat_pub);
    // nh.advertise(odom_pub);
    

    nh.subscribe(twist_sub);
    
    // // Initialize TF broadcaster
    // tf_broadcaster.init(nh);
    
    // Configure encoder pins
    pinMode(ENCODER1_PIN_A, INPUT);
    pinMode(ENCODER1_PIN_B, INPUT);
    pinMode(ENCODER2_PIN_A, INPUT);
    pinMode(ENCODER2_PIN_B, INPUT);
    pinMode(ENCODER3_PIN_A, INPUT);
    pinMode(ENCODER3_PIN_B, INPUT);
    pinMode(ENCODER4_PIN_A, INPUT);
    pinMode(ENCODER4_PIN_B, INPUT);

    // Configure motor pins
    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR3, OUTPUT);
    pinMode(PWM3, OUTPUT);
    pinMode(DIR4, OUTPUT);
    pinMode(PWM4, OUTPUT);
    

    // Attach interrupts to encoder pins
    attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), encoder2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER3_PIN_A), encoder3ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER4_PIN_A), encoder4ISR, CHANGE);
    
    // Initialize IMU
    initMPU9250();
    calibrateSensors();
    lastUpdate = micros();
    
    encoderTimer.begin(encoderCallback, 100000);  // 100ms = 10Hz for encoders
    imuTimer.begin(imuCallback, 10000);   
    gpsTimer.begin(gpsCallback, 10);        // 10ms = 100Hz for IMU
}

void loop() {
    nh.spinOnce();
     // Give time for other tasks
}
