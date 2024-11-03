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

        # Button index that needs to be pressed to allow movement (e.g., button 0)
        self.required_button = 0

        # Button index for the stop button (e.g., button 1)
        self.stop_button = 1

        # Flag to track whether the stop button has been pressed
        self.stopped = False

        # Logging to confirm node startup
        self.get_logger().info('JoyTwistNode with Button Hold, Stop Button, and Dead Man\'s Switch has been started')

    def joy_callback(self, joy_msg):
        # Update the last time a joystick message was received
        self.last_joy_time = self.get_clock().now()

        # Check if the stop button is pressed
        if joy_msg.buttons[self.stop_button] == 1:
            self.get_logger().warn('Stop button pressed, stopping the robot')
            self.stopped = True  # Set the stop flag
            self.stop_robot()  # Stop the robot immediately
            return  # Exit the callback early to ignore further joystick input
        
        # If the stop button is not pressed, reset the stop flag
        if self.stopped:
            self.get_logger().info('Stop button released, robot ready to move again')
            self.stopped = False

        # Check if the required button (e.g., button 0) is pressed
        if joy_msg.buttons[self.required_button] == 1 and not self.stopped:
            # Create a Twist message
            twist = Twist()

            # Map joystick axes to linear and angular velocities
            twist.linear.x = joy_msg.axes[1]  # Left stick vertical axis
            twist.angular.z = joy_msg.axes[0]  # Left stick horizontal axis

            # Log the velocities for debug purposes
            self.get_logger().info(f'Linear: {twist.linear.x}, Angular: {twist.angular.z}')

            # Publish the Twist message to /cmd_vel
            self.twist_publisher.publish(twist)
        else:
            # If the required button is not pressed or stop button is active, stop the robot
            self.stop_robot()

    def deadman_check(self):
        # Calculate the time elapsed since the last joystick message
        time_since_last_joy = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9

        if time_since_last_joy > self.timeout_duration and not self.stopped:
            # If the timeout has passed, send zero velocities to stop the robot
            self.get_logger().warn('No joystick input received, stopping the robot')
            self.stop_robot()

    def stop_robot(self):
        # Create a zero-velocity Twist message
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        # Log the robot stop action
        self.get_logger().warn('Stopping the robot')

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
