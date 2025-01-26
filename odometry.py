import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, PoseWithCovariance, TwistWithCovariance

class WheelOdometryPublisher:
    def __init__(self, wheel_name):
        self.wheel_name = wheel_name
        self.encoder_ticks = 0
        self.previous_ticks = 0
        self.previous_time = rospy.Time.now()

        # Gear ratio and wheel radius
        self.ppr = 714
        self.gear_ratio = 50.9
        self.wheel_radius = 0.076  # meters (6.5 cm)

        # Initialize publisher for odometry data
        self.odom_pub = rospy.Publisher(f"/odometry/{self.wheel_name}", Odometry, queue_size=10)

        # Initialize subscriber for encoder data
        rospy.Subscriber(f"/encoder{self.wheel_name[-1]}", Int32, self.encoder_callback)

    def encoder_callback(self, msg):
        self.previous_ticks = self.encoder_ticks
        self.encoder_ticks = msg.data
        self.publish_odometry()

    def publish_odometry(self):
        # Create an Odometry message
        odom_msg = Odometry()

        # Header setup
        odom_msg.header = Header()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = f"{self.wheel_name}_frame"

        # Set child frame ID
        odom_msg.child_frame_id = f"{self.wheel_name}_link"

        # Calculate distance traveled per tick
        ticks_to_distance = (2 * 3.141592653589793 * self.wheel_radius) / self.ppr
        distance = self.encoder_ticks * ticks_to_distance

        # Calculate velocity
        current_time = rospy.Time.now()
        time_delta = (current_time - self.previous_time).to_sec()
        self.previous_time = current_time

        if time_delta > 0:
            velocity = ((self.encoder_ticks - self.previous_ticks) * ticks_to_distance*self.gear_ratio) / time_delta
        else:
            velocity = 0.0

        # Set the position based on encoder ticks
        odom_msg.pose = PoseWithCovariance()
        odom_msg.pose.pose.position.x = distance
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(0, 0, 0, 1)
        # Covariance (example values, adjust as needed)
        odom_msg.pose.covariance = [0.0] * 36

        # Set the twist
        odom_msg.twist = TwistWithCovariance()
        odom_msg.twist.twist.linear.x = velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.twist.covariance = [0.0] * 36

        # Publish odometry data
        self.odom_pub.publish(odom_msg)

if __name__ == "__main__":
    rospy.init_node("wheel_odometry_publisher")

    # Create instances for each wheel
    wheels = [
        WheelOdometryPublisher("wheel1"),
        WheelOdometryPublisher("wheel2"),
        WheelOdometryPublisher("wheel3"),
        WheelOdometryPublisher("wheel4"),
    ]

    # Spin to keep the script running
    rospy.spin()


