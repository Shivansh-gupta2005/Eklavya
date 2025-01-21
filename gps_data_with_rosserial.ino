#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

// Define serial ports
#define GNSS_SERIAL Serial1  // Teensy hardware serial port connected to NEO-M8P-2
#define DEBUG_SERIAL Serial  // USB serial for debugging

// ROS node handle
ros::NodeHandle nh;

// NavSatFix message
sensor_msgs::NavSatFix nav_msg;

// Publisher
ros::Publisher gps_pub("gps/fix", &nav_msg);

// Buffer for NMEA messages
const int BUFFER_SIZE = 512;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

// Parsing helpers
char *token;
char *saveptr;

void setup() {
  // Initialize debug serial
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) {}  // Wait for USB Serial
  
  // Initialize GNSS serial
  GNSS_SERIAL.begin(38400);
  
  // Initialize ROS
  nh.initNode();
  nh.advertise(gps_pub);
  
  DEBUG_SERIAL.println("RTK GNSS Initialization...");
  
  // Configure NEO-M8P-2 for RTK
  const char enableRTCM[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x00, // CFG-PRT
    0x14, 0x00, // Length
    0x01, 0x00, // Port 1 (UART1)
    0x00, 0x00, // Reserved
    0xD0, 0x08, // TX ready
    0x00, 0x00, // Reserved
    0x00, 0xC2, // UART mode (8N1)
    0x00, 0x08, // Baud rate (38400)
    0x00, 0x00,
    0x07, 0x00, // Input protocols (RTCM3, NMEA, UBX)
    0x03, 0x00, // Output protocols (RTCM3, NMEA)
    0x00, 0x00,
    0x00, 0x00
  };
  
  // Send configuration
  for (int i = 0; i < sizeof(enableRTCM); i++) {
    GNSS_SERIAL.write(enableRTCM[i]);
  }
}

void loop() {
  // Read GNSS data
  while (GNSS_SERIAL.available()) {
    char c = GNSS_SERIAL.read();
    
    if (bufferIndex < BUFFER_SIZE - 1) {
      buffer[bufferIndex++] = c;
      
      if (c == '\n') {
        buffer[bufferIndex] = '\0';
        processGNSSData();
        bufferIndex = 0;
      }
    } else {
      bufferIndex = 0;
    }
  }
  
  nh.spinOnce();
}

void processGNSSData() {
  if (strstr(buffer, "$GNGGA") != NULL) {
    parseGGA();
  }
}

void parseGGA() {
  char *ptr = buffer;
  int field = 0;
  float lat = 0, lon = 0, alt = 0;
  int fix_quality = 0;
  int num_sats = 0;
  float hdop = 0;
  
  // Parse GGA message fields
  while ((token = strtok_r(ptr, ",", &saveptr)) != NULL) {
    ptr = NULL;
    
    switch(field) {
      case 2: // Latitude
        lat = convertNMEAToDecimal(token, saveptr);
        break;
      case 4: // Longitude
        lon = convertNMEAToDecimal(token, saveptr);
        break;
      case 6: // Fix quality
        fix_quality = atoi(token);
        break;
      case 7: // Number of satellites
        num_sats = atoi(token);
        break;
      case 8: // HDOP
        hdop = atof(token);
        break;
      case 9: // Altitude
        alt = atof(token);
        break;
    }
    field++;
  }
  
  // Populate ROS message
  nav_msg.header.stamp = nh.now();
  nav_msg.header.frame_id = "gps";
  
  // Set status based on RTK fix quality
  nav_msg.status.status = fix_quality >= 4 ? sensor_msgs::NavSatStatus::STATUS_GBAS_FIX : 
                         fix_quality >= 1 ? sensor_msgs::NavSatStatus::STATUS_FIX : 
                         sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  nav_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  
  nav_msg.latitude = lat;
  nav_msg.longitude = lon;
  nav_msg.altitude = alt;
  
  // Set position covariance based on HDOP
  float covariance = hdop * hdop * 4.0; // Approximate covariance from HDOP
  for(int i = 0; i < 9; i++) {
    nav_msg.position_covariance[i] = 0.0;
  }
  nav_msg.position_covariance[0] = covariance; // East
  nav_msg.position_covariance[4] = covariance; // North
  nav_msg.position_covariance[8] = covariance * 2; // Up (usually less accurate)
  
  nav_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  
  // Publish the message
  gps_pub.publish(&nav_msg);
}

float convertNMEAToDecimal(char* nmeaPos, char* nextToken) {
  float decimal = 0;
  
  if (strlen(nmeaPos) > 0) {
    float degrees = atof(nmeaPos) / 100;
    int deg = (int)degrees;
    float min = (degrees - deg) * 100;
    decimal = deg + (min / 60);
    
    // Get N/S or E/W indicator
    char dir = strtok_r(NULL, ",", &nextToken)[0];
    if (dir == 'S' || dir == 'W') {
      decimal = -decimal;
    }
  }
  
  return decimal;
}
