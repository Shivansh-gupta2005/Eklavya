#include <Wire.h>
#include <MPU9250.h>
#include <Kalman.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <nav_msgs/Odometry.h>
#include <IntervalTimer.h>
#include<util/atomic.h>
#include <std_msgs/String.h>

// Encoder Connections
#define ENCA1 2
#define ENCB1 3
#define ENCA2 4
#define ENCB2 5
#define ENCA3 16
#define ENCB3 17
#define ENCA4 21
#define ENCB4 23

#define DIR1 9
#define PWM1 8
#define DIR2 7
#define PWM2 6
#define DIR3 14
#define PWM3 15
#define DIR4 20
#define PWM4 22

// Global variables for encoder positions
volatile int posi1 = 0;
volatile int posi2 = 0;
volatile int posi3 = 0;
volatile int posi4 = 0;

// IMU Setup
MPU9250 mpu;
Kalman kalmanYaw, kalmanPitch, kalmanRoll;
const float Q_angle = 0.001;   // Process noise covariance for angle
const float Q_bias = 0.003;    // Process noise covariance for bias
const float R_measure = 0.03;  // Measurement noise covariance

float angleYaw, anglePitch, angleRoll;

// GPS Setup
TinyGPSPlus gps;  // Create a GPS object
#define gpsSerial Serial1  // Connect NEO-M8P-2's TX to Teensy's RX1 and RX to TX1

// ROS Node Handle
ros::NodeHandle nh;

void commandCallback(const std_msgs::String& cmd_msg);
ros::Subscriber<std_msgs::String> sub("robot_command", commandCallback);


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

// Time tracking
unsigned long prev_time = 0;

// ROS Publisher
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);

sensor_msgs::NavSatFix navsat_msg;
ros::Publisher navsat_pub("navsatfix", &navsat_msg);

// Timers
IntervalTimer encoderTimer;
IntervalTimer imuTimer;
IntervalTimer gpsTimer;

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

void updateOdomMsg(nav_msgs::Odometry &msg, float encoder_ticks, const char* frame_id, double dt) {
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
    
    msg.twist.twist.linear.x = encoder_ticks;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.linear.z = 0.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = 0.0;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);  // Wait for the serial connection to establish

    // Initialize MPU9250
    Wire.begin();
    if (!mpu.setup(0x68)) {
        Serial.println("MPU connection failed. Check your connection.");
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

    // Start UART communication with NEO-M8P-2
    gpsSerial.begin(9600);  // Set baud rate to 9600

    // Initialize ROS node
    nh.initNode();
    nh.advertise(imu_pub);
    nh.advertise(navsat_pub);
    // Advertise publishers
  nh.advertise(odom_pub1);
  nh.advertise(odom_pub2);
  nh.advertise(odom_pub3);
  nh.advertise(odom_pub4);
  nh.subscribe(sub);
  
  // Set encoder pins as inputs
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  pinMode(ENCA3, INPUT);
  pinMode(ENCB3, INPUT);
  pinMode(ENCA4, INPUT);
  pinMode(ENCB4, INPUT);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA3), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA4), readEncoder4, RISING);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(PWM4, OUTPUT);
  
  // Initialize all motors to stopped state
  stopmotors();

  // Time tracking
unsigned long prev_time = 0;

  // Initialize messages
  initializeOdomMsgs();

    // Start timers
    encoderTimer.begin(readEncoderTask,10*1000);
    imuTimer.begin(readIMUTask, 25 * 1000);  // IMU update every 25 ms
    gpsTimer.begin(readGPSTask, 30 * 1000); // GPS update every 100 ms

    Serial.println("Initialization complete.");
}

void loop() {
    nh.spinOnce();  // Process incoming ROS messages
}

// Function to read IMU data and publish it
void readIMUTask() {
    if (mpu.update()) {
        float gyroX = mpu.getGyroX();
        float gyroY = mpu.getGyroY();
        float gyroZ = mpu.getGyroZ();

        // Update Kalman filters
        float dt = 0.025; // Time interval in seconds (25 milliseconds)
        angleYaw = kalmanYaw.getAngle(mpu.getYaw(), gyroX, dt);
        anglePitch = kalmanPitch.getAngle(mpu.getPitch(), gyroY, dt);
        angleRoll = kalmanRoll.getAngle(mpu.getRoll(), gyroZ, dt);

        // Convert Euler angles to quaternion
        float q[4];
        float cy = cos(angleYaw * 0.5);
        float sy = sin(angleYaw * 0.5);
        float cp = cos(anglePitch * 0.5);
        float sp = sin(anglePitch * 0.5);
        float cr = cos(angleRoll * 0.5);
        float sr = sin(angleRoll * 0.5);

        q[0] = cr * cp * cy + sr * sp * sy; // q[0] = w
        q[1] = sr * cp * cy - cr * sp * sy; // q[1] = x
        q[2] = cr * sp * cy + sr * cp * sy; // q[2] = y
        q[3] = cr * cp * sy - sr * sp * cy; // q[3] = z

        // Fill the IMU message
        imu_msg.header.stamp = nh.now();
        imu_msg.orientation.x = q[1];
        imu_msg.orientation.y = q[2];
        imu_msg.orientation.z = q[3];
        imu_msg.orientation.w = q[0];
        imu_msg.angular_velocity.x = gyroX;
        imu_msg.angular_velocity.y = gyroY;
        imu_msg.angular_velocity.z = gyroZ;

        // Publish IMU message
        imu_pub.publish(&imu_msg);
    }
}

// Function to read GPS data and publish it
void readGPSTask() {
    // Check if GPS data is available
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c);  // Feed the characters to the GPS object
    }

    // Publish GPS data if it's ready
    if (gps.location.isUpdated()) {
        // Fill NavSatFix message with GPS data
        navsat_msg.latitude = gps.location.lat();
        navsat_msg.longitude = gps.location.lng();
        navsat_msg.altitude = gps.altitude.meters();

        // Set position covariance and its type (optional)
        navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        // Fill status field
        navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        navsat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        // Publish the NavSatFix message
        navsat_pub.publish(&navsat_msg);

        // Print GPS data to Serial Monitor for debugging
        Serial.print("Latitude: ");
        Serial.print(navsat_msg.latitude, 6);  
        Serial.print(" Longitude: ");
        Serial.println(navsat_msg.longitude, 6);
        Serial.print("Altitude: ");
        Serial.print(navsat_msg.altitude);
        Serial.println(" m");
    }
}

// Task 1: Encoder reading and publishing callback
void readEncoderTask() {
    // Read encoder positions
    int pos1, pos2, pos3, pos4;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pos1 = posi1;
        pos2 = posi2;
        pos3 = posi3;
        pos4 = posi4;
    }

 // Get current time
  unsigned long current_time = millis();
  double dt = (current_time - prev_time) / 1000.0; // Convert to seconds
  prev_time = current_time;

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


void moveforward() {
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, HIGH);
  analogWrite(PWM1, 255);  // Changed from 1024 to 255
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 255);  // Changed from 1024 to 255
}

void moveleft() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);
  digitalWrite(DIR3, HIGH);
  digitalWrite(DIR4, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 255);  // Changed from 1024 to 255
  analogWrite(PWM3, 255);  // Changed from 1024 to 255
  analogWrite(PWM4, 0);
}

void movebackward() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, LOW);
  analogWrite(PWM1, 255);  // Changed from 1024 to 255
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 255);  // Changed from 1024 to 255
}

void moveright() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 255);  // Changed from 1024 to 255
  analogWrite(PWM3, 255);  // Changed from 1024 to 255
  analogWrite(PWM4, 0);
}

void spinleft() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, LOW);
  analogWrite(PWM1, 255);  // Changed from 1024 to 255
  analogWrite(PWM2, 255);  // Changed from 1024 to 255
  analogWrite(PWM3, 255);  // Changed from 1024 to 255
  analogWrite(PWM4, 255);  // Changed from 1024 to 255
}

void spinright() {
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, HIGH);
  digitalWrite(DIR3, HIGH);
  digitalWrite(DIR4, HIGH);
  analogWrite(PWM1, 255);  // Changed from 1024 to 255
  analogWrite(PWM2, 255);  // Changed from 1024 to 255
  analogWrite(PWM3, 255);  // Changed from 1024 to 255
  analogWrite(PWM4, 255);  // Changed from 1024 to 255
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
    case 'x': case 'X': default: stopmotors(); break;
  }
}
