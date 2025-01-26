#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <TimerOne.h> // Timer library for handling periodic tasks

// Define pins for encoder channels
#define ENCODER1_PIN_A 21
#define ENCODER1_PIN_B 23
#define ENCODER2_PIN_A 2
#define ENCODER2_PIN_B 3
#define ENCODER3_PIN_A 4
#define ENCODER3_PIN_B 5
#define ENCODER4_PIN_A 16
#define ENCODER4_PIN_B 17

#define DIR1 9
#define PWM1 8
#define DIR2 7
#define PWM2 6
#define DIR3 14
#define PWM3 15
#define DIR4 20
#define PWM4 22

volatile int encoder1Ticks = 0; // Encoder 1 tick count
volatile int encoder2Ticks = 0; // Encoder 2 tick count
volatile int encoder3Ticks = 0; // Encoder 3 tick count
volatile int encoder4Ticks = 0; // Encoder 4 tick count

ros::NodeHandle nh;

// ROS topics for each encoder
std_msgs::Int32 encoder1_msg;
std_msgs::Int32 encoder2_msg;
std_msgs::Int32 encoder3_msg;
std_msgs::Int32 encoder4_msg;

ros::Publisher encoder1_pub("encoder1", &encoder1_msg);
ros::Publisher encoder2_pub("encoder2", &encoder2_msg);
ros::Publisher encoder3_pub("encoder3", &encoder3_msg);
ros::Publisher encoder4_pub("encoder4", &encoder4_msg);

void commandCallback(const std_msgs::String& cmd_msg);
ros::Subscriber<std_msgs::String> sub("robot_command", commandCallback);

// Interrupt Service Routine for Encoder 1 pin A
void encoder1ISR() {
  int stateA = digitalRead(ENCODER1_PIN_A);
  int stateB = digitalRead(ENCODER1_PIN_B);

  if (stateA == stateB) {
    encoder1Ticks++;
  } else {
    encoder1Ticks--;
  }
}

// Interrupt Service Routine for Encoder 2 pin A
void encoder2ISR() {
  int stateA = digitalRead(ENCODER2_PIN_A);
  int stateB = digitalRead(ENCODER2_PIN_B);

  if (stateA == stateB) {
    encoder2Ticks++;
  } else {
    encoder2Ticks--;
  }
}

// Interrupt Service Routine for Encoder 3 pin A
void encoder3ISR() {
  int stateA = digitalRead(ENCODER3_PIN_A);
  int stateB = digitalRead(ENCODER3_PIN_B);

  if (stateA == stateB) {
    encoder3Ticks++;
  } else {
    encoder3Ticks--;
  }
}

// Interrupt Service Routine for Encoder 4 pin A
void encoder4ISR() {
  int stateA = digitalRead(ENCODER4_PIN_A);
  int stateB = digitalRead(ENCODER4_PIN_B);

  if (stateA == stateB) {
    encoder4Ticks++;
  } else {
    encoder4Ticks--;
  }
}

// Timer-based callback function for publishing encoder data
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

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize ROS node handle
  nh.initNode();

  // Advertise ROS topics
  nh.advertise(encoder1_pub);
  nh.advertise(encoder2_pub);
  nh.advertise(encoder3_pub);
  nh.advertise(encoder4_pub);
  nh.subscribe(sub);

  // Configure encoder pins as input
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
  
  // Initialize all motors to stopped state
  stopmotors();

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_PIN_A), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_PIN_A), encoder4ISR, CHANGE);

  // // Initialize the timer to call publishEncoderData every 100ms
  // Timer1.initialize(100000); // 100,000 microseconds = 100ms
  // Timer1.attachInterrupt(publishEncoderData);

  Serial.println("ROSserial encoder node initialized.");
}

void loop() {
  publishEncoderData();
  nh.spinOnce(); // Handle ROS communication
  delay(100);
}
