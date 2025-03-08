#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#define ENCODER_A 4    // Encoder Channel A
#define ENCODER_B 5    // Encoder Channel B
#define MOTOR_PWM_PIN 8    // PWM pin for motor control
#define MOTOR_DIR_PIN 9    // Motor direction control

volatile int encoderCount = 0;
unsigned long prevTime = 0;
float rpm = 0;
int pwm_value = 0;
const int max_pwm = 1023;   // Max PWM for 10-bit resolution
const int pwm_step = 5;     // Step size for increasing PWM
const int delay_time = 500; // Delay for motor stabilization (ms)

ros::NodeHandle nh;
std_msgs::Float32MultiArray rpm_pwm_msg;
ros::Publisher rpm_pwm_pub("rpm_pwm_data", &rpm_pwm_msg);

void  encoderISR() {
    static int lastBState = 0;
    int aState = digitalRead(ENCODER_A);
    int bState = digitalRead(ENCODER_B);

    if (aState != lastBState) { // Detects pulse change
        if (bState != aState) {
            encoderCount++;
        } else {
            encoderCount--;
        }
    }
    lastBState = aState;
}


// Function to calculate RPM
void calculateRPM() {
      unsigned long currentTime = millis();
    float timeDiff = (currentTime - prevTime) / 1000.0;
    rpm = (encoderCount / 714.0) * 60.0 / timeDiff;  // Assuming 360 CPR encoder
    encoderCount = 0;
    prevTime = currentTime;
}

// Function to set motor speed
void setMotorSpeed(int pwm) {
    pwm_value = constrain(pwm, 0, max_pwm); // Ensure PWM is within range
    digitalWrite(MOTOR_DIR_PIN, HIGH);  // Forward direction
    analogWrite(MOTOR_PWM_PIN, pwm_value);
}

void setup() {
    nh.initNode();
    nh.advertise(rpm_pwm_pub);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

    // Initialize MultiArray structure
    rpm_pwm_msg.data_length = 2; // Two elements: [PWM, RPM]
    rpm_pwm_msg.data = (float*)malloc(2 * sizeof(float));
}

void loop() {
    // Increase PWM from 0 to 1023
    for (pwm_value = 0; pwm_value <= max_pwm; pwm_value += pwm_step) {
        setMotorSpeed(pwm_value);
        delay(delay_time);  // Allow motor to stabilize
        calculateRPM();     // Measure RPM

        // Publish RPM & PWM as a Float32MultiArray
        rpm_pwm_msg.data[0] = pwm_value;
        rpm_pwm_msg.data[1] = rpm;
        rpm_pwm_pub.publish(&rpm_pwm_msg);

        nh.spinOnce();
    }

    // Stop motor instantly when PWM reaches 1023
    setMotorSpeed(0);
    delay(500); // Short delay to register stop

    // Publish final stop state
    rpm_pwm_msg.data[0] = 0;
    rpm_pwm_msg.data[1] = 0;
    rpm_pwm_pub.publish(&rpm_pwm_msg);
    nh.spinOnce();

    while (true); // Halt execution after finishing PWM sweep
}
