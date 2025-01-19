#include <HardwareSerial.h>

// Constants for RTK and Position Status
const uint8_t COVARIANCE_TYPE_UNKNOWN = 0;
const uint8_t COVARIANCE_TYPE_APPROXIMATED = 1;
const uint8_t COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
const uint8_t COVARIANCE_TYPE_KNOWN = 3;

const uint8_t GPS_FIX_NONE = 0;
const uint8_t GPS_FIX_SINGLE = 1;
const uint8_t GPS_FIX_DGPS = 2;
const uint8_t GPS_FIX_RTK_FLOAT = 5;
const uint8_t GPS_FIX_RTK_FIXED = 4;

// GNSS Data Structure
struct GNSSData {
    // Position Data
    double latitude;
    double longitude;
    double altitude;
    float ground_speed;    // in m/s
    float ground_course;   // in degrees
    
    // Time Data
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
    
    // Accuracy Data
    float hdop;           // Horizontal dilution of precision
    float pdop;           // Position dilution of precision
    float vdop;           // Vertical dilution of precision
    float position_covariance[9];  // 3x3 covariance matrix
    uint8_t position_covariance_type;
    
    // Satellite Data
    uint8_t satellites_visible;
    uint8_t satellites_used;
    
    // Fix Data
    uint8_t fix_type;
    uint8_t fix_quality;
    bool valid_fix;
    
    // RTK Data
    uint8_t rtk_status;
    float baseline_length;    // RTK baseline length
    float baseline_course;    // RTK baseline course
    
    // Signal Quality
    float carrier_noise_ratio;
    float signal_strength;
};

GNSSData gnssData;

// UBX Configuration Messages
const uint8_t UBX_CFG_RTK[] = {
    0xB5, 0x62, 0x06, 0x71, 0x28, 0x00, // UBX-CFG-TMODE3
    0x00, 0x00,                         // Version
    0x00, 0x00,                         // Reserved1
    0x02, 0x00,                         // Flags (Survey-In Mode)
    0x00, 0x00, 0x00, 0x00,            // ecefXOrLat
    0x00, 0x00, 0x00, 0x00,            // ecefYOrLon
    0x00, 0x00, 0x00, 0x00,            // ecefZOrAlt
    0xF4, 0x01, 0x00, 0x00,            // Fixed Position Accuracy
    0x00, 0x00, 0x00, 0x00,            // Survey-In Min Duration
    0x00, 0x00, 0x00, 0x00,            // Survey-In Position Accuracy
    0x00, 0x00, 0x00, 0x00,            // Reserved2
    0x7E, 0x3B                          // Checksum
};

void setup() {
    // Initialize Serial for GNSS
    Serial.begin(38400);
    
    // Initialize GNSS Data
    initializeGNSSData();
    
    // Configure GNSS Module
    configureGNSS();
    
    Serial.println("RTK GNSS System Initialized");
}

void loop() {
    static char buffer[128];
    static uint8_t bufferIndex = 0;
    
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n') {
            buffer[bufferIndex] = '\0';
            parseNMEAMessage(buffer);
            bufferIndex = 0;
        } else if (bufferIndex < sizeof(buffer) - 1) {
            buffer[bufferIndex++] = c;
        }
        
        // Process UBX messages if available
        processUBXMessages();
    }
    
    // Print data every second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 1000) {
        printGNSSData();
        lastPrint = millis();
    }
}

void initializeGNSSData() {
    memset(&gnssData, 0, sizeof(GNSSData));
    gnssData.position_covariance_type = COVARIANCE_TYPE_UNKNOWN;
    gnssData.fix_type = GPS_FIX_NONE;
}

void configureGNSS() {
    // Reset GNSS module to default settings
    sendUBXMessage("\xB5\x62\x06\x04\x04\x00\xFF\xFF\x00\x00\x0E\x61");
    delay(1000);
    
    // Configure GNSS Navigation Rate (1 Hz)
    sendUBXMessage("\xB5\x62\x06\x08\x06\x00\xE8\x03\x01\x00\x01\x00\x01\x39");
    
    // Enable RTCM3 messages for RTK
    sendUBXMessage("\xB5\x62\x06\x01\x08\x00\xF5\x05\x01\x01\x01\x01\x01\x01\x13\x56");
    
    // Enable required NMEA messages
    // GGA - Global positioning system fix data
    sendUBXMessage("\xB5\x62\x06\x01\x08\x00\xF0\x00\x01\x01\x01\x01\x01\x01\x04\x4B");
    // RMC - Recommended minimum specific GNSS data
    sendUBXMessage("\xB5\x62\x06\x01\x08\x00\xF0\x04\x01\x01\x01\x01\x01\x01\x08\x4F");
    // GST - GNSS pseudorange error statistics
    sendUBXMessage("\xB5\x62\x06\x01\x08\x00\xF0\x07\x01\x01\x01\x01\x01\x01\x0B\x52");
    // GSV - GNSS satellites in view
    sendUBXMessage("\xB5\x62\x06\x01\x08\x00\xF0\x03\x01\x01\x01\x01\x01\x01\x07\x4E");
    
    // Configure RTK settings
    sendUBXMessage(UBX_CFG_RTK, sizeof(UBX_CFG_RTK));
    
    delay(1000);
}

void parseNMEAMessage(const char* message) {
    if (strstr(message, "$GNGGA")) {
        parseGGA(message);
    }
    else if (strstr(message, "$GNRMC")) {
        parseRMC(message);
    }
    else if (strstr(message, "$GNGST")) {
        parseGST(message);
    }
    else if (strstr(message, "$GNGSV")) {
        parseGSV(message);
    }
}

void parseGGA(const char* message) {
    char* ptr = strdup(message);
    char* token = strtok(ptr, ",");
    
    // Skip to time field
    token = strtok(NULL, ",");
    if (token) {
        // Parse time
        gnssData.hour = (token[0] - '0') * 10 + (token[1] - '0');
        gnssData.minute = (token[2] - '0') * 10 + (token[3] - '0');
        gnssData.second = (token[4] - '0') * 10 + (token[5] - '0');
    }
    
    // Latitude
    token = strtok(NULL, ",");
    if (token) {
        double lat = atof(token);
        token = strtok(NULL, ","); // N/S indicator
        gnssData.latitude = (lat / 100.0) * (token[0] == 'S' ? -1 : 1);
    }
    
    // Longitude
    token = strtok(NULL, ",");
    if (token) {
        double lon = atof(token);
        token = strtok(NULL, ","); // E/W indicator
        gnssData.longitude = (lon / 100.0) * (token[0] == 'W' ? -1 : 1);
    }
    
    // Fix quality
    token = strtok(NULL, ",");
    if (token) {
        gnssData.fix_quality = atoi(token);
        gnssData.rtk_status = (gnssData.fix_quality == GPS_FIX_RTK_FIXED) ? 2 :
                             (gnssData.fix_quality == GPS_FIX_RTK_FLOAT) ? 1 : 0;
    }
    
    // Number of satellites
    token = strtok(NULL, ",");
    if (token) {
        gnssData.satellites_used = atoi(token);
    }
    
    // HDOP
    token = strtok(NULL, ",");
    if (token) {
        gnssData.hdop = atof(token);
    }
    
    // Altitude
    token = strtok(NULL, ",");
    if (token) {
        gnssData.altitude = atof(token);
    }
    
    free(ptr);
    
    // Update covariance type based on fix quality
    switch (gnssData.fix_quality) {
        case GPS_FIX_RTK_FIXED:
            gnssData.position_covariance_type = COVARIANCE_TYPE_KNOWN;
            break;
        case GPS_FIX_RTK_FLOAT:
            gnssData.position_covariance_type = COVARIANCE_TYPE_APPROXIMATED;
            break;
        default:
            gnssData.position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
}

void parseRMC(const char* message) {
    char* ptr = strdup(message);
    char* token = strtok(ptr, ",");
    
    // Skip to status field
    token = strtok(NULL, ",");
    token = strtok(NULL, ",");
    
    if (token) {
        gnssData.valid_fix = (token[0] == 'A');
    }
    
    // Skip to speed field (already got position from GGA)
    for (int i = 0; i < 4; i++) {
        token = strtok(NULL, ",");
    }
    
    // Speed in knots
    if (token) {
        gnssData.ground_speed = atof(token) * 0.514444; // Convert knots to m/s
    }
    
    // Course
    token = strtok(NULL, ",");
    if (token) {
        gnssData.ground_course = atof(token);
    }
    
    free(ptr);
}

void parseGST(const char* message) {
    char* ptr = strdup(message);
    char* token = strtok(ptr, ",");
    
    // Skip to RMS value
    for (int i = 0; i < 2; i++) {
        token = strtok(NULL, ",");
    }
    
    if (token) {
        float std_dev_lat = atof(strtok(NULL, ",")); // Latitude std dev
        float std_dev_lon = atof(strtok(NULL, ",")); // Longitude std dev
        float std_dev_alt = atof(strtok(NULL, ",")); // Altitude std dev
        
        // Update position covariance matrix (diagonal elements)
        gnssData.position_covariance[0] = std_dev_lat * std_dev_lat;
        gnssData.position_covariance[4] = std_dev_lon * std_dev_lon;
        gnssData.position_covariance[8] = std_dev_alt * std_dev_alt;
    }
    
    free(ptr);
}

void parseGSV(const char* message) {
    char* ptr = strdup(message);
    char* token = strtok(ptr, ",");
    
    // Skip to number of satellites field
    for (int i = 0; i < 3; i++) {
        token = strtok(NULL, ",");
    }
    
    if (token) {
        gnssData.satellites_visible = atoi(token);
    }
    
    free(ptr);
}

void processUBXMessages() {
    // Process any incoming UBX messages
    while (Serial.available() >= 2) {
        if (Serial.read() == 0xB5) { // UBX sync char 1
            if (Serial.read() == 0x62) { // UBX sync char 2
                processUBXPayload();
            }
        }
    }
}

void processUBXPayload() {
    // Read UBX message class and ID
    if (Serial.available() >= 2) {
        uint8_t messageClass = Serial.read();
        uint8_t messageId = Serial.read();
        
        // Process based on message type
        switch (messageClass) {
            case 0x01: // NAV messages
                processNAVMessage(messageId);
                break;
            // Add other message classes as needed
        }
    }
}

void processNAVMessage(uint8_t messageId) {
    // Process specific NAV messages
    switch (messageId) {
        case 0x14: // NAV-HPPOSLLH
            processHighPrecisionPosition();
            break;
        // Add other NAV message types as needed
    }
}

void processHighPrecisionPosition() {
    // Skip length bytes
    if (Serial.available() >= 2) {
        Serial.read(); // Length LSB
        Serial.read(); // Length MSB
        
        // Read high precision position data
        // Implementation depends on specific needs
    }
}

// ... [Previous code remains the same until sendUBXMessage functions] ...

// Modified sendUBXMessage functions to handle both char* and uint8_t*
void sendUBXMessage(const uint8_t* msg, size_t len) {
    for (size_t i = 0; i < len; i++) {
        Serial.write(msg[i]);
    }
}

void sendUBXMessage(const char* msg) {
    while (*msg) {
        Serial.write(*msg++);
    }
}

void printGNSSData() {
    Serial.println("\n=== GNSS Status Update ===");
    
    // Position
    Serial.print("Position (Lat, Lon, Alt): ");
    Serial.print(gnssData.latitude, 8); Serial.print(", ");
    Serial.print(gnssData.longitude, 8); Serial.print(", ");
    Serial.println(gnssData.altitude, 2);
    
    // Speed and Course
    Serial.print("Ground Speed (m/s): "); Serial.println(gnssData.ground_speed, 2);
    Serial.print("Ground Course (deg): "); Serial.println(gnssData.ground_course, 2);
    
    // Fix Status
    Serial.print("Fix Type: ");
    switch (gnssData.fix_quality) {
        case GPS_FIX_NONE:
            Serial.println("No Fix");
            break;
        case GPS_FIX_SINGLE:
            Serial.println("Single Fix");
            break;
        case GPS_FIX_DGPS:
            Serial.println("DGPS");
            break;
        case GPS_FIX_RTK_FLOAT:
            Serial.println("RTK Float");
            break;
        case GPS_FIX_RTK_FIXED:
            Serial.println("RTK Fixed");
            break;
        default:
            Serial.println("Unknown");
    }
    
    // RTK Status
    Serial.print("RTK Status: ");
    switch (gnssData.rtk_status) {
        case 0:
            Serial.println("No RTK");
            break;
        case 1:
            Serial.println("RTK Float");
            break;
        case 2:
            Serial.println("RTK Fixed");
            break;
    }
    
    // Satellite Information
    Serial.print("Satellites (Visible/Used): ");
    Serial.print(gnssData.satellites_visible);
    Serial.print("/");
    Serial.println(gnssData.satellites_used);
    
    // DOP Values
    Serial.print("HDOP: "); Serial.println(gnssData.hdop, 2);
    Serial.print("PDOP: "); Serial.println(gnssData.pdop, 2);
    Serial.print("VDOP: "); Serial.println(gnssData.vdop, 2);
    
    // Position Covariance
    Serial.println("Position Covariance Matrix:");
    for (int i = 0; i < 9; i++) {
        Serial.print(gnssData.position_covariance[i], 6);
        Serial.print(i % 3 == 2 ? "\n" : ", ");
    }
    
    Serial.print("Covariance Type: ");
    switch (gnssData.position_covariance_type) {
        case COVARIANCE_TYPE_UNKNOWN:
            Serial.println("Unknown");
            break;
        case COVARIANCE_TYPE_APPROXIMATED:
            Serial.println("Approximated");
            break;
        case COVARIANCE_TYPE_DIAGONAL_KNOWN:
            Serial.println("Diagonal Known");
            break;
        case COVARIANCE_TYPE_KNOWN:
            Serial.println("Known");
            break;
    }
    
    Serial.println("================================\n");
}
