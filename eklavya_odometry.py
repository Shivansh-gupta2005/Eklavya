import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Quaternion, Point, Pose, PoseWithCovariance, TwistWithCovariance, Vector3, Twist
import tf2_ros

class RobotOdometryPublisher:
    def __init__(self):
        # Initialize node
        rospy.init_node("robot_odometry_publisher")

        # Robot parameters
        self.wheel_radius = 0.076  # meters
        self.robot_radius = 0.268   # distance from center to wheel
        self.ppr = 714             # pulses per revolution
        self.gear_ratio = 50.9
        self.wheel_positions = [
            [1, -1],   # Front Left: +x, -y
            [1, 1],    # Front Right: +x, +y
            [-1, -1],  # Rear Left: -x, -y
            [-1, 1]    # Rear Right: -x, +y
        ]

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()

        # Encoder data
        self.encoder_ticks = [0, 0, 0, 0]
        self.prev_encoder_ticks = [0, 0, 0, 0]

        # Wheel velocities
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Subscribers for encoder data
        rospy.Subscriber('/encoder1', Int32, lambda msg: self.encoder_callback(msg, 0))
        rospy.Subscriber('/encoder2', Int32, lambda msg: self.encoder_callback(msg, 1))
        rospy.Subscriber('/encoder3', Int32, lambda msg: self.encoder_callback(msg, 2))
        rospy.Subscriber('/encoder4', Int32, lambda msg: self.encoder_callback(msg, 3))

        rospy.loginfo("Robot Odometry Publisher initialized")

    def encoder_callback(self, msg, wheel_index):
        self.prev_encoder_ticks[wheel_index] = self.encoder_ticks[wheel_index]
        self.encoder_ticks[wheel_index] = msg.data
        self.update_odometry()

    def ticks_to_distance(self, ticks):
        # Convert encoder ticks to distance traveled
        return ticks * (2 * math.pi * self.wheel_radius) / (self.ppr * self.gear_ratio)

    def update_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        if dt <= 0:
            return
            
        self.last_time = current_time

        # Calculate wheel velocities based on encoder tick changes
        for i in range(4):
            delta_ticks = self.encoder_ticks[i] - self.prev_encoder_ticks[i]
            delta_distance = self.ticks_to_distance(delta_ticks)
            self.wheel_velocities[i] = delta_distance / dt if dt > 0 else 0.0

        # Corrected kinematic equations for Mecanum wheels
        vx = (self.wheel_velocities[0] + self.wheel_velocities[2]) / 2.0
        vy = (self.wheel_velocities[1] + self.wheel_velocities[3]) / 2.0
        vtheta = (-self.wheel_velocities[0] + self.wheel_velocities[1] + self.wheel_velocities[2] - self.wheel_velocities[3]) / (4.0 * self.robot_radius)

        # Update robot position using basic integration
        # Account for robot orientation in global frame
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = vtheta * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize angle to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish odometry
        self.publish_odometry(vx, vy, vtheta)

    def publish_odometry(self, vx, vy, vtheta):
        # Create and populate Odometry message
        odom = Odometry()
        odom.header.stamp = self.last_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set position
        odom.pose.pose.position = Point(self.x, self.y, 0.0)
        
        # Convert orientation to quaternion
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        # Set twist
        odom.twist.twist = Twist(
            Vector3(vx, vy, 0),
            Vector3(0, 0, vtheta)
        )

        # Set covariance matrices (example values, should be tuned)
        # 6x6 covariance matrices: [x, y, z, roll, pitch, yaw]
        # Diagonal elements represent variance of each variable
        pose_covariance = [0.01, 0, 0, 0, 0, 0,
                           0, 0.01, 0, 0, 0, 0,
                           0, 0, 0.01, 0, 0, 0,
                           0, 0, 0, 0.01, 0, 0,
                           0, 0, 0, 0, 0.01, 0,
                           0, 0, 0, 0, 0, 0.01]
                           
        twist_covariance = [0.01, 0, 0, 0, 0, 0,
                            0, 0.01, 0, 0, 0, 0,
                            0, 0, 0.01, 0, 0, 0,
                            0, 0, 0, 0.01, 0, 0,
                            0, 0, 0, 0, 0.01, 0,
                            0, 0, 0, 0, 0, 0.01]
                            
        odom.pose.covariance = pose_covariance
        odom.twist.covariance = twist_covariance

        # Publish odometry message
        self.odom_pub.publish(odom)

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.last_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

if __name__ == "__main__":
    try:
        robot_odom = RobotOdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
