#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import euler_from_quaternion

# Desired distance from the landmark (meters)
DESIRED_DISTANCE = 1.0

class TurtlebotFollower(Node):
    def __init__(self):
        super().__init__('turtlebot_follower')

        # Create a publisher to send velocity commands to the TurtleBot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 50)

        # Create a TF buffer and listener to get the transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Set up a timer to periodically call the control loop
        self.timer = self.create_timer(0.5, self.control_loop)  # 10 Hz

    def control_loop(self):
        twist = Twist()  # Create a new Twist message with zero velocities to stop by default
        twist.linear.x = 0.0
        twist.angular.z= 0.0        

        try:
            # Set a timeout of 0.2 seconds to lookup the transform
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('base_link', 'tagStandard41h12:5', now, rclpy.duration.Duration(seconds=0.2))

            # Extract translation (position) data
            distance_x = transform.transform.translation.x
            distance_y = transform.transform.translation.y

            # Compute the Euclidean distance to the landmark
            current_distance = math.sqrt(distance_x**2 + distance_y**2)

            # Calculate the error in distance (how far we are from the desired distance)
            distance_error = current_distance - DESIRED_DISTANCE

            # Calculate the angular error
            angle_to_landmark = math.atan2(distance_y, distance_x)

            # Proportional controller for linear speed (move forward or backward)
            if abs(distance_error) > 0.1:  # Only move if we're farther than 10 cm from the target
                twist.linear.x = 0.5 * distance_error  # Proportional control for speed
            else:
                self.get_logger().info('Stopping: Robot is close enough to the tag.')
                twist.linear.x = 0.0  # Stop when close enough to the target
                twist.angular.z= 0.0

            # Clamp the linear velocity to avoid moving too fast
            twist.linear.x = max(min(twist.linear.x, 0.3), -0.3)  # Limit speed between -0.3 and 0.3 m/s

            # Proportional controller for angular speed (rotate toward the landmark)
            twist.angular.z = 2.0 * angle_to_landmark

            # Clamp the angular velocity
            twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)  # Limit turning speed

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Stop the robot if the transform lookup fails (couldn't find the tag)
            self.get_logger().warn(f'Tag lost. Stopping the robot: {str(e)}')
            twist = Twist()   # Set all velocities to zero if tag is lost

        finally:
            # Publish the velocity command (either moving or stopping)
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    follower = TurtlebotFollower()

    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
