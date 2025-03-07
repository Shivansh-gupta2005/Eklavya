#define ENCODER_A 2  // Encoder Channel A
#define ENCODER_B 3  // Encoder Channel B
#define MOTOR_PWM 9  // PWM Pin for motor speed control
#define MOTOR_DIR 8  // Motor direction control pin
#define KNOWN_RPM 96  // Replace with actual max RPM

volatile long pulseCount = 0; // Tracks encoder pulses
volatile bool direction = true; // True = Forward, False = Backward
unsigned long prevTime = 0;
float ppr = 0.0;
int pwmValue = 1023;  // Max PWM

void encoderISR() {
    if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
        pulseCount--;  // Counterclockwise
        direction = false;
    } else {
        pulseCount++;  // Clockwise
        direction = true;
    }
}

void setup() {
    Serial.begin(115200);
    
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

    // Start motor at max PWM in forward direction
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, pwmValue);
}

void loop() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - prevTime;

    if (elapsedTime >= 1000) {  // Every 1 second
        Serial.print("Pulse Count: ");
        Serial.println(pulseCount);

        // Calculate PPR using formula: PPR = (Pulse Count per sec * 60) / Known RPM
        ppr = (pulseCount * 60.0) / KNOWN_RPM;

        Serial.print("Calculated PPR: ");
        Serial.println(ppr);

        pulseCount = 0;  // Reset count for the next measurement
        prevTime = currentTime;
    }
}
