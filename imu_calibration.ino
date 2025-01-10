#include "MPU9250.h"

MPU9250 mpu;
const uint8_t MPU_ADDRESS = 0x68;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!initializeSensor()) {
        while (1) {
            Serial.println("MPU connection failed. Check your connections.");
            delay(5000);
        }
    }

    // Wait for serial command to start calibration
    Serial.println("\nType 'c' to start calibration");
    while (!Serial.available() || Serial.read() != 'c') {
        delay(100);
    }

    // Full calibration process
    performFullCalibration();
}

bool initializeSensor() {
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    return mpu.setup(MPU_ADDRESS, setting);
}

void performFullCalibration() {
    // Step 1: Accel/Gyro Calibration
    calibrateAccelGyro();
    
    // Step 2: Magnetometer Calibration
    calibrateMagnetometer();
    
    // Print final calibration values
    printCalibrationValues();
}

void calibrateAccelGyro() {
    Serial.println("\n=== Accelerometer and Gyroscope Calibration ===");
    Serial.println("Place the sensor on a flat, stable surface.");
    Serial.println("Don't move the sensor until calibration is complete!");
    Serial.println("Calibration will start in 5 seconds...");
    
    for (int i = 5; i > 0; i--) {
        Serial.print(i); Serial.println(" seconds remaining...");
        delay(1000);
    }
    
    mpu.calibrateAccelGyro();
    Serial.println("Accel/Gyro calibration complete!");
}

void calibrateMagnetometer() {
    Serial.println("\n=== Magnetometer Calibration ===");
    Serial.println("Wave the sensor in a figure-8 pattern while rotating it.");
    Serial.println("Continue this motion for about 30 seconds.");
    Serial.println("Type 's' when ready to start, 'f' to finish.");
    
    // Wait for start command
    while (!Serial.available() || Serial.read() != 's') {
        delay(100);
    }

    float mag_x, mag_y, mag_z;
    float mag_x_min = 32767, mag_x_max = -32768;
    float mag_y_min = 32767, mag_y_max = -32768;
    float mag_z_min = 32767, mag_z_max = -32768;
    
    Serial.println("Collecting magnetometer data... Move the sensor!");
    
    // Collect data until user sends 'f'
    while (!Serial.available() || Serial.read() != 'f') {
        if (mpu.update()) {
            mag_x = mpu.getMagX();
            mag_y = mpu.getMagY();
            mag_z = mpu.getMagZ();
            
            // Update min/max values
            mag_x_min = min(mag_x_min, mag_x);
            mag_x_max = max(mag_x_max, mag_x);
            mag_y_min = min(mag_y_min, mag_y);
            mag_y_max = max(mag_y_max, mag_y);
            mag_z_min = min(mag_z_min, mag_z);
            mag_z_max = max(mag_z_max, mag_z);
            
            // Print current values
            Serial.print("Mag X: "); Serial.print(mag_x);
            Serial.print(" Y: "); Serial.print(mag_y);
            Serial.print(" Z: "); Serial.println(mag_z);
            delay(100);
        }
    }
    
    // Calculate magnetometer bias and scale
    float mag_bias_x = (mag_x_max + mag_x_min) / 2;
    float mag_bias_y = (mag_y_max + mag_y_min) / 2;
    float mag_bias_z = (mag_z_max + mag_z_min) / 2;
    
    float mag_scale_x = (mag_x_max - mag_x_min) / 2;
    float mag_scale_y = (mag_y_max - mag_y_min) / 2;
    float mag_scale_z = (mag_z_max - mag_z_min) / 2;
    
    float avg_scale = (mag_scale_x + mag_scale_y + mag_scale_z) / 3;
    
    mpu.setMagBias(mag_bias_x, mag_bias_y, mag_bias_z);
    mpu.setMagScale(avg_scale/mag_scale_x, avg_scale/mag_scale_y, avg_scale/mag_scale_z);
}

void printCalibrationValues() {
    Serial.println("\n=== Calibration Values ===");
    Serial.println("Copy these values into your main sketch:");
    Serial.println("\n// Magnetometer Bias");
    Serial.print("mpu.setMagBias(");
    Serial.print(mpu.getMagBiasX()); Serial.print(", ");
    Serial.print(mpu.getMagBiasY()); Serial.print(", ");
    Serial.print(mpu.getMagBiasZ()); Serial.println(");");
    
    Serial.println("\n// Magnetometer Scale");
    Serial.print("mpu.setMagScale(");
    Serial.print(mpu.getMagScaleX(), 6); Serial.print(", ");
    Serial.print(mpu.getMagScaleY(), 6); Serial.print(", ");
    Serial.print(mpu.getMagScaleZ(), 6); Serial.println(");");
    
    Serial.println("\nCalibration complete! Save these values!");
}

void loop() {
    // Nothing to do in loop for calibration sketch
    delay(1000);
}
