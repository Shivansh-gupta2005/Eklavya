#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <util/atomic.h>

// Encoder Connections
#define ENCA1 2
#define ENCB1 3
#define ENCA2 4
#define ENCB2 5
#define ENCA3 16
#define ENCB3 17
#define ENCA4 21
#define ENCB4 23

// Global variables for encoder positions
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

// Time tracking
unsigned long prev_time = 0;

void setup() {
  // Initialize ROS node
  nh.initNode();
  
  // Advertise publishers
  nh.advertise(odom_pub1);
  nh.advertise(odom_pub2);
  nh.advertise(odom_pub3);
  nh.advertise(odom_pub4);
  
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

  // Initialize messages
  initializeOdomMsgs();
}

void loop() {
  // Read encoder positions
  int pos1, pos2, pos3, pos4;
  
  // Read in atomic block to prevent interrupt conflicts
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

  nh.spinOnce();
  delay(10); // Small delay for stability
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
