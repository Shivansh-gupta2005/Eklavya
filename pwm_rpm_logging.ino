const int pwmPin = 9;   // PWM pin for motor control
const int encA = 2;     // Encoder A pin
const int encB = 3;     // Encoder B pin

volatile long encoderCount = 0;
unsigned long prevMillis = 0;
float rpm = 0;

// PWM range
const int pwmMin = 5;
const int pwmMax = 655535;
const int pwmStep = 10;
const int interval = 100000; // Interval for RPM calculation (ms)

// Interrupt function for encoder counting
void encoderISR() {
    if (digitalRead(encA) == digitalRead(encB)) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(pwmPin, OUTPUT);
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encA), encoderISR, CHANGE);

    // Wait for Python script to start listening
    while (!Serial) {
        delay(10);
    }

    Serial.println("PWM,RPM"); // CSV header
}

void loop() {
    for (int pwm = pwmMin; pwm <= pwmMax; pwm += pwmStep) {
        analogWrite(pwmPin, pwm);
        encoderCount = 0;
        prevMillis = millis();

        while (millis() - prevMillis < interval);

        // Calculate RPM (adjust '20' for your encoder's pulses per revolution)
        rpm = (encoderCount / 714.0) * 60.0;

        // Print data in CSV format
        Serial.print(pwm);
        Serial.print(",");
        Serial.println(rpm);

        delay(500); // Small delay before next step
    }

    analogWrite(pwmPin, 0); // Stop motor
    Serial.println("Done"); // Signal completion
    while (1); // Halt further execution
}
