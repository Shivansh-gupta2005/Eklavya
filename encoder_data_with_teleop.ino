#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <util/atomic.h>


// Motor Pins
#define DIR1 9
#define PWM1 8
#define DIR2 7
#define PWM2 6
#define DIR3 14
#define PWM3 15
#define DIR4 20
#define PWM4 22

// Encoder Pins
#define ENCA1 2
#define ENCB1 3
#define ENCA2 4
#define ENCB2 5
#define ENCA3 16
#define ENCB3 17
#define ENCA4 21
#define ENCB4 23

// Flags for interrupt handling
volatile bool command_received = false;
volatile char current_command = 'x';  // Default to stop
volatile bool encoder_updated = false;

// Timing variables
unsigned long last_command_time = 0;
unsigned long last_encoder_publish = 0;
const unsigned long COMMAND_TIMEOUT = 100;  // 100ms timeout
const unsigned long ENCODER_PUBLISH_INTERVAL = 50;  // 50ms between publishes

// Global variables for encoder positions with better interrupt handling
volatile long encoder_counts[4] = {0, 0, 0, 0};
volatile bool encoder_states[4] = {false, false, false, false};

// ROS node handle and messages (same as before)
ros::NodeHandle nh;
nav_msgs::Odometry odom_msg1;
nav_msgs::Odometry odom_msg2;
nav_msgs::Odometry odom_msg3;
nav_msgs::Odometry odom_msg4;

// Publishers
ros::Publisher odom_pub1("encoder1/odom", &odom_msg1);
ros::Publisher odom_pub2("encoder2/odom", &odom_msg2);
ros::Publisher odom_pub3("encoder3/odom", &odom_msg3);
ros::Publisher odom_pub4("encoder4/odom", &odom_msg4);

// Improved command callback with flag
void commandCallback(const std_msgs::String& cmd_msg) {
  current_command = cmd_msg.data[0];
  command_received = true;
  last_command_time = millis();
}

ros::Subscriber<std_msgs::String> sub("robot_command", &commandCallback);

// Motor control functions with interrupt safety
void setMotorState(int dir_pin, int pwm_pin, bool direction, int speed) {
  noInterrupts();  // Disable interrupts briefly
  digitalWrite(dir_pin, direction);
  analogWrite(pwm_pin, speed);
  interrupts();    // Re-enable interrupts
}

// Updated movement functions with interrupt safety
void moveforward() {
  setMotorState(DIR1, PWM1, HIGH, 1023);
  setMotorState(DIR2, PWM2, LOW, 0);
  setMotorState(DIR3, PWM3, LOW, 0);
  setMotorState(DIR4, PWM4, HIGH, 1023);
}

void moveleft() {
    setMotorState(DIR1, PWM1, LOW, 0);
  setMotorState(DIR2, PWM2, HIGH, 1023);
  setMotorState(DIR3, PWM3, HIGH, 1023);
  setMotorState(DIR4, PWM4, LOW, 0);

}

void movebackward() {
   setMotorState(DIR1, PWM1, LOW, 1023);
  setMotorState(DIR2, PWM2, LOW, 0);
  setMotorState(DIR3, PWM3, LOW, 0);
  setMotorState(DIR4, PWM4, LOW, 1023);

 
}

void moveright() {
  setMotorState(DIR1, PWM1, HIGH, 0);
  setMotorState(DIR2, PWM2, LOW, 1023);
  setMotorState(DIR3, PWM3, LOW, 1023);
  setMotorState(DIR4, PWM4, HIGH, 0);
}

void spinleft() {
  setMotorState(DIR1, PWM1, HIGH, 1023);
  setMotorState(DIR2, PWM2, LOW, 1023);
  setMotorState(DIR3, PWM3, HIGH, 1023);
  setMotorState(DIR4, PWM4, LOW, 1023); 
}

void spinright() {
  setMotorState(DIR1, PWM1, LOW, 1023);
  setMotorState(DIR2, PWM2, HIGH, 1023);
  setMotorState(DIR3, PWM3, LOW, 1023);
  setMotorState(DIR4, PWM4, HIGH, 1023);  
}

void diagonalforward(){
  setMotorState(DIR1, PWM1, HIGH, 1023);
  setMotorState(DIR2, PWM2, HIGH, 1023);
  setMotorState(DIR3, PWM3, HIGH, 1023);
  setMotorState(DIR4, PWM4, HIGH, 1023); 
}


void diagonalbackward() {
  setMotorState(DIR1, PWM1, LOW, 1023);
  setMotorState(DIR2, PWM2, LOW, 1023);
  setMotorState(DIR3, PWM3, LOW, 1023);
  setMotorState(DIR4, PWM4, LOW, 1023);   
}

void stopmotors() {
  noInterrupts();
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM4, 0);
  interrupts();
}

// Improved interrupt handlers with state checking
void handleEncoder(int index, int encA_pin, int encB_pin) {
  // Only process if state has changed
  bool new_state = digitalRead(encA_pin);
  if (new_state != encoder_states[index]) {
    encoder_states[index] = new_state;
    if (new_state) {  // RISING edge
      if (digitalRead(encB_pin)) {
        encoder_counts[index]++;
      } else {
        encoder_counts[index]--;
      }
      encoder_updated = true;
    }
  }
}

void readEncoder1() { handleEncoder(0, ENCA1, ENCB1); }
void readEncoder2() { handleEncoder(1, ENCA2, ENCB2); }
void readEncoder3() { handleEncoder(2, ENCA3, ENCB3); }
void readEncoder4() { handleEncoder(3, ENCA4, ENCB4); }

void setup() {
  // Initialize ROS node
  nh.initNode();
  nh.advertise(odom_pub1);
  nh.advertise(odom_pub2);
  nh.advertise(odom_pub3);
  nh.advertise(odom_pub4);
  nh.subscribe(sub);
  
  // Pin setup with interrupt safety
  noInterrupts();
  
  // Motor pins
  pinMode(DIR1, OUTPUT); pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT); pinMode(PWM2, OUTPUT);
  pinMode(DIR3, OUTPUT); pinMode(PWM3, OUTPUT);
  pinMode(DIR4, OUTPUT); pinMode(PWM4, OUTPUT);
  
  // Encoder pins with pullup
  pinMode(ENCA1, INPUT_PULLUP); pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP); pinMode(ENCB2, INPUT_PULLUP);
  pinMode(ENCA3, INPUT_PULLUP); pinMode(ENCB3, INPUT_PULLUP);
  pinMode(ENCA4, INPUT_PULLUP); pinMode(ENCB4, INPUT_PULLUP);
  
  interrupts();

  // Attach interrupts with CHANGE mode for better reliability
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA3), readEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA4), readEncoder4, CHANGE);

  initializeOdomMsgs();
  stopmotors();
}

void loop() {
  unsigned long current_time = millis();
  
  // Handle command with timeout
  if (command_received) {
    switch(current_command) {
      case 'w': case 'W': moveforward(); break;
      case 's': case 'S': movebackward(); break;
      case 'a': case 'A': moveleft(); break;
      case 'd': case 'D': moveright(); break;
      case 'q': case 'Q': spinleft(); break;
      case 'e': case 'E': spinright(); break;
      case 'g': case 'G': diagonalforward(); break;
      case 'h': case 'H': diagonalbackward(); break;
      default: stopmotors(); break;
    }
    command_received = false;
  }
  
  // Command timeout check
  if (current_time - last_command_time > COMMAND_TIMEOUT) {
    stopmotors();
  }

  // Publish encoder data at fixed interval
  if (encoder_updated && (current_time - last_encoder_publish > ENCODER_PUBLISH_INTERVAL)) {
    long counts[4];
    
    // Atomic block for reading encoder counts
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      for(int i = 0; i < 4; i++) {
        counts[i] = encoder_counts[i];
      }
      encoder_updated = false;
    }
    
    // Calculate dt for velocity
    double dt = (current_time - last_encoder_publish) / 1000.0;
    
    // Update and publish odometry
    updateOdomMsg(odom_msg1, counts[0], "encoder1", dt);
    updateOdomMsg(odom_msg2, counts[1], "encoder2", dt);
    updateOdomMsg(odom_msg3, counts[2], "encoder3", dt);
    updateOdomMsg(odom_msg4, counts[3], "encoder4", dt);
    
    odom_pub1.publish(&odom_msg1);
    odom_pub2.publish(&odom_msg2);
    odom_pub3.publish(&odom_msg3);
    odom_pub4.publish(&odom_msg4);
    
    last_encoder_publish = current_time;
  }

  nh.spinOnce();
  
  // Small delay to prevent overwhelming the system
  delay(1);
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
  const float TICKS_PER_REV = 1440.0; // Adjust based on your encoder
  const float WHEEL_RADIUS = 0.05;    // Adjust based on your wheel radius in meters
  
  float distance = (2.0 * PI * WHEEL_RADIUS * encoder_ticks) / TICKS_PER_REV;
  float velocity = distance / dt;

  // Update position
  msg.pose.pose.position.x = distance;
  msg.pose.pose.position.y = 0.0;
  msg.pose.pose.position.z = 0.0;
  
  // Set orientation (identity quaternion)
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
