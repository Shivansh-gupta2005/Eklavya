#include <ros.h>
#include <std_msgs/String.h>

#define DIR1 9
#define PWM1 8
#define DIR2 7
#define PWM2 6
#define DIR3 14
#define PWM3 15
#define DIR4 20
#define PWM4 22

// ROS setup
ros::NodeHandle nh;
void commandCallback(const std_msgs::String& cmd_msg);
ros::Subscriber<std_msgs::String> sub("robot_command", commandCallback);

void setup() {
  Serial.begin(9600);
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
 
  
  // ROS initialization
  nh.initNode();
  nh.subscribe(sub);
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

void loop() {

  
  nh.spinOnce();
  delay(100);
}
