#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ros.h>
#include <sensor_msgs/NavSatFix.h>

TinyGPSPlus gps;  // Create a GPS object
#define gpsSerial Serial1  // Connect NEO-M8P-2's TX to Teensy's RX1 and RX to TX1

// ROS node handle
ros::NodeHandle nh;

// Create NavSatFix message
sensor_msgs::NavSatFix nav_msg;

// Create publisher
ros::Publisher gps_pub("gps/fix", &nav_msg);

void setup() {
  // Initialize ROS node
  nh.initNode();
  nh.advertise(gps_pub);
  
  // Start UART communication with NEO-M8P-2
  gpsSerial.begin(9600);  // Default baud rate for NEO-M8P-2
  
  // Initialize NavSatFix message
  nav_msg.header.frame_id = "gps";
  nav_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  nav_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

void loop() {
  // Check if GPS data is available
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }
  
  // If we have new GPS data, publish it
  if (gps.location.isUpdated()) {
    // Update header timestamp
    nav_msg.header.stamp = nh.now();
    
    // Set fix status
    nav_msg.status.status = gps.location.isValid() ? 
      sensor_msgs::NavSatStatus::STATUS_FIX : 
      sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    
    // Set position data
    nav_msg.latitude = gps.location.lat();
    nav_msg.longitude = gps.location.lng();
    nav_msg.altitude = gps.altitude.meters();
    
    // Set position covariance using HDOP
    // HDOP gives us an idea of the horizontal position accuracy
    float hdop = gps.hdop.value() / 100.0; // Convert HDOP to meters (approximate)
    nav_msg.position_covariance[0] = hdop * hdop; // XX
    nav_msg.position_covariance[4] = hdop * hdop; // YY
    nav_msg.position_covariance[8] = (hdop * 2) * (hdop * 2); // ZZ, typically less accurate
    
    // Publish the message
    gps_pub.publish(&nav_msg);
  }
  
  nh.spinOnce();
}
