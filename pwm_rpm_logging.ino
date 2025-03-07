#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#define ENCODER_PIN_A 4  
#define ENCODER_PIN_B 5
#define MOTOR_PWM_PIN 8 
#define MOTOR_DIR_PIN 9

volatile int encoder_ticks = 0;
float rpm = 0;
int pwm_value = 0;
int max_pwm = 1023;   
int pwm_step = 5;    
int delay_time = 500; 

ros::NodeHandle nh;
std_msgs::Float32MultiArray motor_data;
ros::Publisher motor_pub("motor_data", &motor_data);


void encoderISR() {
    if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B)) {
        encoder_ticks++;
    } else {
        encoder_ticks--;
    }
}


void calculateRPM() {
    static unsigned long last_time = 0;
    unsigned long current_time = millis();
    float time_difference = (current_time - last_time) / 1000.0; 

    if (time_difference >= 0.1) { 
        rpm = (encoder_ticks / 400.0) * 60.0; 
        encoder_ticks = 0;
        last_time = current_time;
    }
}

void setMotorSpeed(int pwm) {
    pwm_value = constrain(pwm, 0, max_pwm); 
    digitalWrite(MOTOR_DIR_PIN, HIGH);  
    analogWrite(MOTOR_PWM_PIN, pwm_value);
}

void setup() {
    nh.initNode();
    nh.advertise(motor_pub);

    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);
}

void loop() {
    for (pwm_value = 0; pwm_value <= max_pwm; pwm_value += pwm_step) {
        setMotorSpeed(pwm_value);
        delay(delay_time);  
        calculateRPM();    


        motor_data.data_length = 2;
        motor_data.data = (float*)malloc(2 * sizeof(float));
        motor_data.data[0] = rpm;
        motor_data.data[1] = pwm_value;
        motor_pub.publish(&motor_data);
        free(motor_data.data);

        nh.spinOnce();
    }

    setMotorSpeed(0);
    while (true); 
}
