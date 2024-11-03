import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTwistNode(Node):
    def __init__(self):
        super().__init__('joy_twist_node')
        
        # Subscription to /joy topic
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Publisher to /cmd_vel topic
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for the dead man's switch
        self.timeout_duration = 1.0  # Timeout in seconds (adjust as needed)
        self.timer = self.create_timer(self.timeout_duration, self.deadman_check)
        
        # To store the time when the last joystick message was received
        self.last_joy_time = self.get_clock().now()

        # Button index that needs to be pressed (e.g., button 0)
        self.required_button = 0

        # Logging to confirm node startup
        self.get_logger().info('JoyTwistNode with Button Hold and Dead Man\'s Switch has been started')

    def joy_callback(self, joy_msg):
        # Update the last time a joystick message was received
        self.last_joy_time = self.get_clock().now()

        # Check if the required button is pressed
        if joy_msg.buttons[self.required_button] == 1:
            # Create a Twist message
            twist = Twist()

            # Map joystick axes to linear and angular velocities
            # Here, we assume axes[1] controls linear.x and axes[0] controls angular.z
            twist.linear.x = joy_msg.axes[1]  # Left stick vertical axis
            twist.angular.z = joy_msg.axes[0]  # Left stick horizontal axis

            # Log the velocities for debug purposes
            self.get_logger().info(f'Linear: {twist.linear.x}, Angular: {twist.angular.z}')

            # Publish the Twist message to /cmd_vel
            self.twist_publisher.publish(twist)
        else:
            # If the button is not pressed, stop the robot
            self.stop_robot()

    def deadman_check(self):
        # Calculate the time elapsed since the last joystick message
        time_since_last_joy = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9

        if time_since_last_joy > self.timeout_duration:
            # If the timeout has passed, send zero velocities to stop the robot
            self.get_logger().warn('No joystick input received, stopping the robot')
            self.stop_robot()

    def stop_robot(self):
        # Create a zero-velocity Twist message
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        # Log the robot stop action
        self.get_logger().warn('Stopping the robot due to inactive button or timeout')

        # Publish the zero-velocity message to stop the robot
        self.twist_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the node
    node = JoyTwistNode()
    
    # Spin the node to keep it alive
    rclpy.spin(node)
    
    # Cleanup on shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
