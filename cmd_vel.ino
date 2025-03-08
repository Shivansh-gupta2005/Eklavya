#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
// // Define pins for encoder channels
// #define ENCODER1_PIN_A 21
// #define ENCODER1_PIN_B 23
// #define ENCODER2_PIN_A 2
// #define ENCODER2_PIN_B 3
// #define ENCODER3_PIN_A 4
// #define ENCODER3_PIN_B 5
// #define ENCODER4_PIN_A 16
// #define ENCODER4_PIN_B 17
#define DIR1 9
#define PWM1 8
#define DIR2 7
#define PWM2 6
#define DIR3 14
#define PWM3 15
#define DIR4 20
#define PWM4 22
// Bot parameters (adjust based on your robot dimensions)
#define WHEEL_RADIUS 0.076 // meters
#define ROBOT_RADIUS 0.29 // meters (distance from center to wheel)
#define MAX_PWM 255
#define MAX_RPM 128 // Maximum RPM your motors can achieve
// RPM to PWM conversion parameters (rpm = 0.5119*pwm - 3.10)
// Rearranging to PWM = (RPM + 3.10) / 0.5119
#define RPM_TO_PWM_SLOPE 0.4575

// volatile int encoder1Ticks = 0; // Encoder 1 tick count
// volatile int encoder2Ticks = 0; // Encoder 2 tick count
// volatile int encoder3Ticks = 0; // Encoder 3 tick count
// volatile int encoder4Ticks = 0; // Encoder 4 tick count
float v1 = 0, v2 = 0, v3 = 0, v4 = 0; // Individual wheel velocities
ros::NodeHandle nh;
// // ROS topics for each encoder
// std_msgs::Int32 encoder1_msg;
// std_msgs::Int32 encoder2_msg;
// std_msgs::Int32 encoder3_msg;
// std_msgs::Int32 encoder4_msg;
// ros::Publisher encoder1_pub("encoder1", &encoder1_msg);
// ros::Publisher encoder2_pub("encoder2", &encoder2_msg);
// ros::Publisher encoder3_pub("encoder3", &encoder3_msg);
// ros::Publisher encoder4_pub("encoder4", &encoder4_msg);
// Callback for cmd_vel subscribed topic
void cmdVelCallback(const geometry_msgs::Twist& twist);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);
// Interrupt Service Routine for Encoder 1 pin A
// void encoder1ISR() {
// int stateA = digitalRead(ENCODER1_PIN_A);
// int stateB = digitalRead(ENCODER1_PIN_B);
// if (stateA == stateB) {
// encoder1Ticks++;
// } else {
// encoder1Ticks--;
// }
// }
// // Interrupt Service Routine for Encoder 2 pin A
// void encoder2ISR() {
// int stateA = digitalRead(ENCODER2_PIN_A);
// int stateB = digitalRead(ENCODER2_PIN_B);
// if (stateA == stateB) {
// encoder2Ticks++;
// } else {
// encoder2Ticks--;
// }
// }
// // Interrupt Service Routine for Encoder 3 pin A
// void encoder3ISR() {
// int stateA = digitalRead(ENCODER3_PIN_A);
// int stateB = digitalRead(ENCODER3_PIN_B);
// if (stateA == stateB) {
// encoder3Ticks++;
// } else {
// encoder3Ticks--;
// }
// }
// // Interrupt Service Routine for Encoder 4 pin A
// void encoder4ISR() {
// int stateA = digitalRead(ENCODER4_PIN_A);
// int stateB = digitalRead(ENCODER4_PIN_B);
// if (stateA == stateB) {
// encoder4Ticks++;
// } else {
// encoder4Ticks--;
// }
// }
// Timer-based callback function for publishing encoder data
// void publishEncoderData() {
// // Publish encoder tick counts
// encoder1_msg.data = encoder1Ticks;
// encoder2_msg.data = encoder2Ticks;
// encoder3_msg.data = encoder3Ticks;
// encoder4_msg.data = encoder4Ticks;
// encoder1_pub.publish(&encoder1_msg);
// encoder2_pub.publish(&encoder2_msg);
// encoder3_pub.publish(&encoder3_msg);
// encoder4_pub.publish(&encoder4_msg);
// }
// Convert velocity to PWM using the empirical formula
int velocityToPWM(float velocity) {
 // Convert from normalized velocity to RPM
float rpm = velocity/0.076;
 // Use the formula: PWM = (RPM + 3.10) / 0.5119
int pwm = rpm  RPM_TO_PWM_SLOPE;
 // Ensure minimum PWM to overcome static friction if velocity is non-zero
 // if (velocity != 0 && pwm < 20) {
 // pwm = 20;
 // }
 // Constrain PWM values
return pwm;
}
// Apply wheel velocities to motors
void applyWheelVelocities() {
 // Convert velocity to PWM using the empirical formula
int pwm1 = velocityToPWM(v1);
int pwm2 = velocityToPWM(v2);
int pwm3 = velocityToPWM(v3);
int pwm4 = velocityToPWM(v4);
 // Set motor directions based on velocity sign
digitalWrite(DIR1, v1 >= 0 ? HIGH : LOW);
digitalWrite(DIR2, v2 >= 0 ? HIGH : LOW);
digitalWrite(DIR3, v3 >= 0 ? HIGH : LOW);
digitalWrite(DIR4, v4 >= 0 ? HIGH : LOW);
 // Apply PWM values
analogWrite(PWM1, pwm1);
analogWrite(PWM2, pwm2);
analogWrite(PWM3, pwm3);
analogWrite(PWM4, pwm4);
}
// Callback for cmd_vel topic
void cmdVelCallback(const geometry_msgs::Twist& twist) {
 // Extract linear and angular velocities from the Twist message
float linear_x = twist.linear.x; // Forward/backward movement
float linear_y = twist.linear.y; // Left/right movement
float angular_z = twist.angular.z; // Rotation
 // 1: Front Left, 2: Front Right, 3: Rear Left, 4: Rear Right
 v1 = linear_x - angular_z * ROBOT_RADIUS; // Front Left
 v2 = linear_y + angular_z * ROBOT_RADIUS; // Front Right
 v3 = linear_x - angular_z * ROBOT_RADIUS; // Rear Left
 v4 = - linear_y + angular_z * ROBOT_RADIUS; // Rear Right
 // Normalize velocities if any exceeds maximum
applyWheelVelocities();
}
void stopmotors() {
 v1 = v2 = v3 = v4 = 0;
analogWrite(PWM1, 0);
analogWrite(PWM2, 0);
analogWrite(PWM3, 0);
analogWrite(PWM4, 0);
}
void setup() {
 // Initialize serial communication
Serial.begin(9600);
 // Initialize ROS node handle
nh.initNode();
 // Subscribe to cmd_vel
nh.subscribe(cmd_vel_sub);
pinMode(DIR1, OUTPUT);
pinMode(PWM1, OUTPUT);
pinMode(DIR2, OUTPUT);
pinMode(PWM2, OUTPUT);
pinMode(DIR3, OUTPUT);
pinMode(PWM3, OUTPUT);
pinMode(DIR4, OUTPUT);
pinMode(PWM4, OUTPUT);
stopmotors();
Serial.println("ROSserial cmd_vel node initialized.");
}
void loop() {
nh.spinOnce();
delay(100);
}
