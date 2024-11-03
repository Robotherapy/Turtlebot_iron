import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import sin, cos
import math

class Tb3OdomTf(Node):
    def __init__(self):
        super().__init__('tb3_odom_tf')

        # Create a subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initial pose and velocity
        self.x = 0.0  # robot's x position
        self.y = 0.0  # robot's y position
        self.yaw = 0.0  # robot's yaw angle (orientation)

        self.linear_velocity = 0.0  # linear velocity (from cmd_vel)
        self.angular_velocity = 0.0  # angular velocity (from cmd_vel)

        # Timer to periodically broadcast the transform (every 100 ms or 10 Hz)
        self.timer = self.create_timer(0.1, self.broadcast_transform)

        # Last time update for time difference calculation
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        # Update linear and angular velocities from the cmd_vel topic
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def broadcast_transform(self):
        # Get the current time and calculate the time delta
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9  # Convert from nanoseconds to seconds
        self.last_time = current_time

        # Calculate odometry using the provided equations

        # 1. delta_yaw = yaw_vel * delta_time
        delta_yaw = self.angular_velocity * delta_time

        # 2. robot_yaw = robot_yaw + delta_yaw
        self.yaw += delta_yaw

        # 3. robot_x = robot_x + (x_vel * cos(robot_yaw)) * delta_time
        self.x += self.linear_velocity * cos(self.yaw) * delta_time

        # 4. robot_y = robot_y + (x_vel * sin(robot_yaw)) * delta_time
        self.y += self.linear_velocity * sin(self.yaw) * delta_time

        # Create a TransformStamped message to broadcast the transform
        t = TransformStamped()

        # Set the header with frame information
        t.header.stamp = current_time.to_msg()  # Use the current time
        t.header.frame_id = 'world'  # Parent frame (world)
        t.child_frame_id = 'base_link'  # Child frame (base_link)

        # Set translation (x, y, z) -> For a 2D robot, z is always 0
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Set the rotation as a quaternion (from Euler angles)
        # We only rotate around the Z-axis in a 2D plane, so roll and pitch are 0
        q = self.euler_to_quaternion(0, 0, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (roll, pitch, yaw) to a quaternion"""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it
    tb3_odom_tf = Tb3OdomTf()
    rclpy.spin(tb3_odom_tf)

    # Shutdown and clean up
    tb3_odom_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
