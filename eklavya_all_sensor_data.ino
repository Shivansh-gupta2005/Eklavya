#include <Arduino.h>
#include <IntervalTimer.h>
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <util/atomic.h>
#include "MPU9250.h"
#include <TinyGPS++.h>
#include <Wire.h>

// Create IntervalTimer objects
IntervalTimer timer1;
IntervalTimer timer2;
IntervalTimer timer3;

volatile int posi1 = 0;
volatile int posi2 = 0;
volatile int posi3 = 0;
volatile int posi4 = 0;

// ROS node handle
ros::NodeHandle nh;

// Odometry messages for each encoder
nav_msgs::Odometry odom_msg1;
nav_msgs::Odometry odom_msg2;
nav_msgs::Odometry odom_msg3;
nav_msgs::Odometry odom_msg4;

// Publishers for odometry messages
ros::Publisher odom_pub1("encoder1/odom", &odom_msg1);
ros::Publisher odom_pub2("encoder2/odom", &odom_msg2);
ros::Publisher odom_pub3("encoder3/odom", &odom_msg3);
ros::Publisher odom_pub4("encoder4/odom", &odom_msg4);

// === Task 2: IMU Variables ===
MPU9250 mpu;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

const uint8_t MPU_ADDRESS = 0x68;
const float MAGNETIC_DECLINATION = 0.62;
bool isSensorInitialized = false;

// === Task 3: GPS Variables ===
TinyGPSPlus gps;
sensor_msgs::NavSatFix nav_msg;
ros::Publisher gps_pub("gps/fix", &nav_msg);
#define gpsSerial Serial1

const unsigned long TASK1_INTERVAL = 10000;    // 10ms for encoder publishing
const unsigned long TASK2_INTERVAL = 25000;    // 25ms (40Hz) for IMU
const unsigned long TASK3_INTERVAL = 1000000;  // 1000ms (1Hz) for GPS

// Buffer for GPS data
const int GPS_BUFFER_SIZE = 128;
char gpsBuffer[GPS_BUFFER_SIZE];
int gpsBufferIndex = 0;

// Task 1 variables
unsigned long prev_encoder_time = 0;

// === Motor Control Variables ===
#define DIR1 9
#define PWM1 8
#define DIR2 7
#define PWM2 6
#define DIR3 14
#define PWM3 15
#define DIR4 20
#define PWM4 22

// Current sensor configuration
const int currentPin = 41;
const float vRef = 3.3;
const float vcc = 3.3;
const float quiescent = vcc / 2;
const float sensitivity = 0.04;
const int PWM_MAX = 255;

// ROS subscriber for motor commands
ros::Subscriber<std_msgs::String> command_sub("robot_command", commandCallback);

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

    // Initialize covariance matrices
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
    
    // Convert encoder ticks to distance
    const float TICKS_PER_REV = 1440.0;
    const float WHEEL_RADIUS = 0.05;
    
    float distance = (2.0 * PI * WHEEL_RADIUS * encoder_ticks) / TICKS_PER_REV;
    float velocity = distance / dt;

    // Update position
    msg.pose.pose.position.x = distance;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;
    
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;
    
    msg.twist.twist.linear.x = velocity;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.linear.z = 0.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = 0.0;
}

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
    
    // Apply calibration values
    mpu.setMagBias(-153.835, -127.095, -381.37);
    mpu.setMagScale(1.00788, 1.0275, 0.9666);
    mpu.setMagneticDeclination(MAGNETIC_DECLINATION);
    
    isSensorInitialized = true;
    nh.loginfo("MPU9250 initialized successfully");
}

void initializeImuMsg() {
    imu_msg.header.frame_id = "imu_link";
    
    for(int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    
    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[8] = 0.0025;
    
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[8] = 0.02;
    
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[8] = 0.04;
}

void initializeGPSMsg() {
    nav_msg.header.frame_id = "gps";
    nav_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    nav_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

// Task 1: Encoder reading and publishing callback
void task1Callback() {
    // Read encoder positions
    int pos1, pos2, pos3, pos4;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pos1 = posi1;
        pos2 = posi2;
        pos3 = posi3;
        pos4 = posi4;
    }

    unsigned long current_time = millis();
    double dt = (current_time - prev_encoder_time) / 1000.0;
    prev_encoder_time = current_time;

    // Update and publish odometry messages
    updateOdomMsg(odom_msg1, pos1, "encoder1", dt);
    updateOdomMsg(odom_msg2, pos2, "encoder2", dt);
    updateOdomMsg(odom_msg3, pos3, "encoder3", dt);
    updateOdomMsg(odom_msg4, pos4, "encoder4", dt);

    // Publish messages
    odom_pub1.publish(&odom_msg1);
    odom_pub2.publish(&odom_msg2);
    odom_pub3.publish(&odom_msg3);
    odom_pub4.publish(&odom_msg4);
}


// Task 2: IMU reading and publishing callback
void task2Callback() {
    if (!isSensorInitialized) {
        return;
    }

    if (mpu.update()) {
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
}

// Task 3: GPS reading and publishing callback
void task3Callback() {
    // Process any available GPS data
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        if (gps.encode(c)) {
            if (gps.location.isUpdated()) {
                // Update header timestamp
                nav_msg.header.stamp = nh.now();
                
                // Set fix status
                nav_msg.status.status = gps.location.isValid() ? 
                    sensor_msgs::NavSatStatus::STATUS_FIX : 
                    sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                
                // Set position data
                nav_msg.latitude = gps.location.lat();
                nav_msg.longitude = gps.location.lng();
                nav_msg.altitude = gps.altitude.meters();
                
                // Set position covariance using HDOP
                float hdop = gps.hdop.value() / 100.0;
                nav_msg.position_covariance[0] = hdop * hdop;
                nav_msg.position_covariance[4] = hdop * hdop;
                nav_msg.position_covariance[8] = (hdop * 2) * (hdop * 2);
                
                // Publish the message
                gps_pub.publish(&nav_msg);
            }
        }
    }
}

void readEncoder1() {
    int b = digitalRead(ENCB1);
    if(b > 0) {
        posi1++;
    } else {
        posi1--;
    }
}

void readEncoder2() {
    int b = digitalRead(ENCB2);
    if(b > 0) {
        posi2++;
    } else {
        posi2--;
    }
}

void readEncoder3() {
    int b = digitalRead(ENCB3);
    if(b > 0) {
        posi3++;
    } else {
        posi3--;
    }
}

void readEncoder4() {
    int b = digitalRead(ENCB4);
    if(b > 0) {
        posi4++;
    } else {
        posi4--;
    }
}

// Motor control functions
void moveforward() {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, LOW);
    digitalWrite(DIR4, HIGH);
    analogWrite(PWM1, PWM_MAX);
    analogWrite(PWM2, 0);
    analogWrite(PWM3, 0);
    analogWrite(PWM4, PWM_MAX);
}

void moveleft() {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, HIGH);
    digitalWrite(DIR3, HIGH);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM1, 0);
    analogWrite(PWM2, PWM_MAX);
    analogWrite(PWM3, PWM_MAX);
    analogWrite(PWM4, 0);
}

void movebackward() {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, LOW);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM1, PWM_MAX);
    analogWrite(PWM2, 0);
    analogWrite(PWM3, 0);
    analogWrite(PWM4, PWM_MAX);
}

void moveright() {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, LOW);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM1, 0);
    analogWrite(PWM2, PWM_MAX);
    analogWrite(PWM3, PWM_MAX);
    analogWrite(PWM4, 0);
}

void spinleft() {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, LOW);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM1, PWM_MAX);
    analogWrite(PWM2, PWM_MAX);
    analogWrite(PWM3, PWM_MAX);
    analogWrite(PWM4, PWM_MAX);
}

void spinright() {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);
    digitalWrite(DIR3, HIGH);
    digitalWrite(DIR4, HIGH);
    analogWrite(PWM1, PWM_MAX);
    analogWrite(PWM2, PWM_MAX);
    analogWrite(PWM3, PWM_MAX);
    analogWrite(PWM4, PWM_MAX);
}

void stopmotors() {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    analogWrite(PWM3, 0);
    analogWrite(PWM4, 0);
}

float readCurrent() {
    int sensorValue = analogRead(currentPin);
    float voltage = (sensorValue * vRef) / 4095.0;
    return (voltage - quiescent) / sensitivity;
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
        case 'x': case 'X': default: stopmotors(); break;
    }
}

void setup() {
    // Initialize ROS node
    nh.initNode();
    
    // Advertise publishers and subscribe to commands
    nh.advertise(odom_pub1);
    nh.advertise(odom_pub2);
    nh.advertise(odom_pub3);
    nh.advertise(odom_pub4);
    nh.advertise(imu_pub);
    nh.advertise(gps_pub);
    nh.subscribe(command_sub);
    
    // Initialize motor pins
    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR3, OUTPUT);
    pinMode(PWM3, OUTPUT);
    pinMode(DIR4, OUTPUT);
    pinMode(PWM4, OUTPUT);
    
    // Initialize motors to stopped state
    stopmotors();
    
    // Setup current sensor
    analogReadResolution(12);
    
    // Initialize I2C and other sensors
    Wire.begin();
    delay(2000);
    
    // Initialize GPS serial
    gpsSerial.begin(9600);
    
        // Set encoder pins as inputs
    pinMode(ENCA1, INPUT);
    pinMode(ENCB1, INPUT);
    pinMode(ENCA2, INPUT);
    pinMode(ENCB2, INPUT);
    pinMode(ENCA3, INPUT);
    pinMode(ENCB3, INPUT);
    pinMode(ENCA4, INPUT);
    pinMode(ENCB4, INPUT);
    
    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA3), readEncoder3, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA4), readEncoder4, RISING);

    // Initialize  messages
    initializeOdomMsgs();
    initializeImuMsg();
    initializeGPSMsg();
    
    // Start all timer tasks
    if (!timer1.begin(task1Callback, TASK1_INTERVAL)) {
        nh.logwarn("Timer 1 initialization failed!");
    }
    if (!timer2.begin(task2Callback, TASK2_INTERVAL)) {
        nh.logwarn("Timer 2 initialization failed!");
    }
    if (!timer3.begin(task3Callback, TASK3_INTERVAL)) {
        nh.logwarn("Timer 3 initialization failed!");
    }
}

void loop() {
    // Read and log current sensor
    static unsigned long lastCurrentLog = 0;
    if (millis() - lastCurrentLog >= 100) {  // Log current every 100ms
        float current = readCurrent();
        char current_str[10];
        dtostrf(current, 4, 2, current_str);
        char log_msg[20];
        sprintf(log_msg, "Current: %sA", current_str);
        nh.loginfo(log_msg);
        lastCurrentLog = millis();
    }
    
    // Check IMU initialization status
    static unsigned long lastCheckTime = 0;
    if (!isSensorInitialized && (millis() - lastCheckTime > 5000)) {
        nh.logwarn("IMU not initialized! Retrying setup...");
        setupMPU();
        lastCheckTime = millis();
    }
    
    nh.spinOnce();
}
