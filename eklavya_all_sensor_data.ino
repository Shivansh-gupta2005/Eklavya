#include <Arduino.h>
#include <ros.h>
#include <Wire.h>
#include <MPU9250.h>
#include <Kalman.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <TinyGPS++.h>
#include <util/atomic.h>

// Encoder Connections (from Task 1)
#define ENCA1 2
#define ENCB1 3
#define ENCA2 4
#define ENCB2 5
#define ENCA3 16
#define ENCB3 17
#define ENCA4 21
#define ENCB4 23

// Global variables for encoder positions
volatile int posi1 = 0, posi2 = 0, posi3 = 0, posi4 = 0;

// IMU Setup
MPU9250 mpu;
Kalman kalmanYaw, kalmanPitch, kalmanRoll;
const float Q_angle = 0.001;
const float Q_bias = 0.003;
const float R_measure = 0.03;
float angleYaw, anglePitch, angleRoll;

// GPS Setup
TinyGPSPlus gps;
#define gpsSerial Serial1

// ROS node handle
ros::NodeHandle nh;

// ROS Publishers
// Encoders (from Task 1)
nav_msgs::Odometry odom_msg1, odom_msg2, odom_msg3, odom_msg4;
ros::Publisher odom_pub1("encoder1/odom", &odom_msg1);
ros::Publisher odom_pub2("encoder2/odom", &odom_msg2);
ros::Publisher odom_pub3("encoder3/odom", &odom_msg3);
ros::Publisher odom_pub4("encoder4/odom", &odom_msg4);

// IMU and GPS
sensor_msgs::Imu imu_msg;
sensor_msgs::NavSatFix navsat_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);
ros::Publisher navsat_pub("navsatfix", &navsat_msg);

// Timer objects
IntervalTimer encoderTimer;  // Task 1 timer
IntervalTimer imuTimer;      // Task 2 timer
IntervalTimer gpsTimer;      // Task 3 timer

// Time tracking
unsigned long prev_time = 0;

void setup() {
    // Initialize Serial and wait for connection
    Serial.begin(115200);
    while (!Serial);

    // Initialize I2C and MPU9250
    Wire.begin();
    if (!mpu.setup(0x68)) {
        Serial.println("MPU connection failed!");
        while (1);
    }

    // Initialize Kalman filters
    kalmanYaw.setAngle(0);
    kalmanPitch.setAngle(0);
    kalmanRoll.setAngle(0);
    kalmanYaw.setQangle(Q_angle);
    kalmanYaw.setQbias(Q_bias);
    kalmanYaw.setRmeasure(R_measure);
    kalmanPitch.setQangle(Q_angle);
    kalmanPitch.setQbias(Q_bias);
    kalmanPitch.setRmeasure(R_measure);
    kalmanRoll.setQangle(Q_angle);
    kalmanRoll.setQbias(Q_bias);
    kalmanRoll.setRmeasure(R_measure);

    // Initialize GPS Serial
    gpsSerial.begin(9600);

    // Initialize ROS node and publishers
    nh.initNode();
    // Encoder publishers
    nh.advertise(odom_pub1);
    nh.advertise(odom_pub2);
    nh.advertise(odom_pub3);
    nh.advertise(odom_pub4);
    // IMU and GPS publishers
    nh.advertise(imu_pub);
    nh.advertise(navsat_pub);

    // Initialize encoder pins and interrupts
    setupEncoders();
    
    // Initialize messages
    initializeOdomMsgs();

    // Start the timers
    // Task 1: Encoder reading (50Hz)
    if (!encoderTimer.begin(encoderTimerISR, 20000)) {
        Serial.println("Encoder timer failed!");
    }
    // Task 2: IMU reading (40Hz)
    if (!imuTimer.begin(readIMUTask, 25000)) {
        Serial.println("IMU timer failed!");
    }
    // Task 3: GPS reading (10Hz)
    if (!gpsTimer.begin(readGPSTask, 100000)) {
        Serial.println("GPS timer failed!");
    }
}

void loop() {
    nh.spinOnce();
}

// [Previous encoder functions remain the same]
void setupEncoders() {
    // Set encoder pins as inputs
    pinMode(ENCA1, INPUT); pinMode(ENCB1, INPUT);
    pinMode(ENCA2, INPUT); pinMode(ENCB2, INPUT);
    pinMode(ENCA3, INPUT); pinMode(ENCB3, INPUT);
    pinMode(ENCA4, INPUT); pinMode(ENCB4, INPUT);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA3), readEncoder3, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA4), readEncoder4, RISING);
}

// Task 1: Encoder Timer ISR
void encoderTimerISR() {
    int pos1, pos2, pos3, pos4;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pos1 = posi1; pos2 = posi2;
        pos3 = posi3; pos4 = posi4;
    }

    unsigned long current_time = millis();
    double dt = (current_time - prev_time) / 1000.0;
    prev_time = current_time;

    updateOdomMsg(odom_msg1, pos1, "encoder1", dt);
    updateOdomMsg(odom_msg2, pos2, "encoder2", dt);
    updateOdomMsg(odom_msg3, pos3, "encoder3", dt);
    updateOdomMsg(odom_msg4, pos4, "encoder4", dt);

    odom_pub1.publish(&odom_msg1);
    odom_pub2.publish(&odom_msg2);
    odom_pub3.publish(&odom_msg3);
    odom_pub4.publish(&odom_msg4);
}

// Task 2: IMU Timer ISR
void readIMUTask() {
    if (mpu.update()) {
        float gyroX = mpu.getGyroX();
        float gyroY = mpu.getGyroY();
        float gyroZ = mpu.getGyroZ();

        float dt = 0.025;
        angleYaw = kalmanYaw.getAngle(mpu.getYaw(), gyroX, dt);
        anglePitch = kalmanPitch.getAngle(mpu.getPitch(), gyroY, dt);
        angleRoll = kalmanRoll.getAngle(mpu.getRoll(), gyroZ, dt);

        // Convert to quaternion
        float q[4];
        float cy = cos(angleYaw * 0.5);
        float sy = sin(angleYaw * 0.5);
        float cp = cos(anglePitch * 0.5);
        float sp = sin(anglePitch * 0.5);
        float cr = cos(angleRoll * 0.5);
        float sr = sin(angleRoll * 0.5);

        q[0] = cr * cp * cy + sr * sp * sy;
        q[1] = sr * cp * cy - cr * sp * sy;
        q[2] = cr * sp * cy + sr * cp * sy;
        q[3] = cr * cp * sy - sr * sp * cy;

        imu_msg.header.stamp = nh.now();
        imu_msg.orientation.x = q[1];
        imu_msg.orientation.y = q[2];
        imu_msg.orientation.z = q[3];
        imu_msg.orientation.w = q[0];
        imu_msg.angular_velocity.x = gyroX;
        imu_msg.angular_velocity.y = gyroY;
        imu_msg.angular_velocity.z = gyroZ;

        imu_pub.publish(&imu_msg);
    }
}

// Task 3: GPS Timer ISR
void readGPSTask() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        navsat_msg.latitude = gps.location.lat();
        navsat_msg.longitude = gps.location.lng();
        navsat_msg.altitude = gps.altitude.meters();
        navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        navsat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        
        navsat_pub.publish(&navsat_msg);
    }
}
void initializeOdomMsgs() {
  // Initialize common fields for all odometry messages
  odom_msg1.header.frame_id = "odom";
  odom_msg2.header.frame_id = "odom";
  odom_msg3.header.frame_id = "odom";
  odom_msg4.header.frame_id = "odom";

  // Initialize child frames
  odom_msg1.child_frame_id = "encoder1";
  odom_msg2.child_frame_id = "encoder2";
  odom_msg3.child_frame_id = "encoder3";
  odom_msg4.child_frame_id = "encoder4";

  // Initialize covariance matrices (if needed)
  for(int i = 0; i < 36; i++) {
    odom_msg1.pose.covariance[i] = 0.0;
    odom_msg2.pose.covariance[i] = 0.0;
    odom_msg3.pose.covariance[i] = 0.0;
    odom_msg4.pose.covariance[i] = 0.0;
    
    odom_msg1.twist.covariance[i] = 0.0;
    odom_msg2.twist.covariance[i] = 0.0;
    odom_msg3.twist.covariance[i] = 0.0;
    odom_msg4.twist.covariance[i] = 0.0;
  }
}

void updateOdomMsg(nav_msgs::Odometry &msg, int encoder_ticks, const char* frame_id, double dt) {
  // Update header
  msg.header.stamp = nh.now();
  
  // Convert encoder ticks to distance (modify these calculations based on your encoder specs)
  const float TICKS_PER_REV = 1440.0; // Modify based on your encoder
  const float WHEEL_RADIUS = 0.05;    // Modify based on your wheel radius in meters
  
  float distance = (2.0 * PI * WHEEL_RADIUS * encoder_ticks) / TICKS_PER_REV;
  float velocity = distance / dt;

  // Update position (just using x for linear distance)
  msg.pose.pose.position.x = distance;
  msg.pose.pose.position.y = 0.0;
  msg.pose.pose.position.z = 0.0;
  
  // Set orientation to identity quaternion
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.0;
  msg.pose.pose.orientation.w = 1.0;
  
  // Update velocity
  msg.twist.twist.linear.x = velocity;
  msg.twist.twist.linear.y = 0.0;
  msg.twist.twist.linear.z = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = 0.0;
}

// Interrupt functions for encoders
void readEncoder1() {
  int b = digitalRead(ENCB1);
  if(b > 0) {
    posi1++;
  }
  else {
    posi1--;
  }
}

void readEncoder2() {
  int b = digitalRead(ENCB2);
  if(b > 0) {
    posi2++;
  }
  else {
    posi2--;
  }
}

void readEncoder3() {
  int b = digitalRead(ENCB3);
  if(b > 0) {
    posi3++;
  }
  else {
    posi3--;
  }
}

void readEncoder4() {
  int b = digitalRead(ENCB4);
  if(b > 0) {
    posi4++;
  }
  else {
    posi4--;
  }
}
