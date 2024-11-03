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
        
        # Logging to confirm node startup
        self.get_logger().info('JoyTwistNode has been started')

    def joy_callback(self, joy_msg):
        # Create a Twist message
        twist = Twist()

        # Map joystick axes to linear and angular velocities
        # Here, we assume axes[1] controls linear.x and axes[0] controls angular.z
        # Modify according to your joystick setup
        twist.linear.x = joy_msg.axes[1]  # Left stick vertical axis
        twist.angular.z = joy_msg.axes[0]  # Left stick horizontal axis

        # Log the velocities for debug purposes
        self.get_logger().info(f'Linear: {twist.linear.x}, Angular: {twist.angular.z}')

        # Publish the Twist message to /cmd_vel
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
