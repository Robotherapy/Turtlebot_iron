import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Int8, String
import time

class RefereeNode(Node):
    def __init__(self):
        super().__init__('referee_node')
        
        self.start_time = None
        self.data_log = []
        self.initial_pose = None
        self.task_completed = False

        self.create_timer(1.0, self.countdown_callback)
        self.get_logger().info('Referee is on the field.')

        self.pose_subscription = self.create_subscription(PoseStamped, 'detected_marker_pose', self.pose_callback, 10)
        self.id_subscription = self.create_subscription(Int8, 'detected_marker_id', self.id_callback, 10)
        self.image_subscription = self.create_subscription(String, 'detected_image', self.image_callback, 10)
        self.robot_pose_subscription = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.robot_pose_callback, 10)

    def countdown_callback(self):
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('Starting 5-second countdown...')
            return

        self.elapsed_time = time.time() - self.start_time
        if self.elapsed_time < 5:
            self.get_logger().info(f'Countdown: {5 - int(self.elapsed_time)} seconds')
        else:
            self.destroy_timer(self.countdown_callback)

    def pose_callback(self, msg):
        if not self.task_completed:
            self.data_log.append({'pose': msg})
            self.get_logger().info(f'pose: {msg} in {self.elapsed_time} seconds')

    def id_callback(self, msg):
        if not self.task_completed:
            self.data_log.append({'marker_id': msg.data})
            self.get_logger().info(f'marker_id: {msg.data} in {self.elapsed_time} seconds')

    def image_callback(self, msg):
        if not self.task_completed:
            self.data_log.append({'image': msg.data})
            self.get_logger().info(f'image: {msg.data} in {self.elapsed_time} seconds')

    def robot_pose_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose
            self.get_logger().info(f"Initial pose: {self.initial_pose}")
            return

        if self.is_initial_pose(msg.pose.pose):
            self.task_completed = True
            self.log_data()
            self.destroy_node()
            rclpy.shutdown()

    def is_initial_pose(self, pose):
        return (pose.position == self.initial_pose.position and
                pose.orientation == self.initial_pose.orientation)

    def log_data(self):
        end_time = time.time()
        duration = int(end_time - self.start_time)
        self.get_logger().info(f"Task completed in {duration} seconds")

        # Calculate points
    
        mode = self.get_parameter('mode').value

        if mode == 'teleop':
            mode_multiplier = 1
        elif mode == 'partial_auto':
            mode_multiplier = 2
        elif mode == 'full_auto':
            mode_multiplier = 5
        else:
            mode_multiplier = 1  # Default to teleop if unknown mode
        print(f"Mode: {mode}, Mode Multiplier: {mode_multiplier}")

        num_poses = sum(1 for entry in self.data_log if 'pose' in entry)
        num_ids = sum(1 for entry in self.data_log if 'marker_id' in entry)
        num_images = sum(1 for entry in self.data_log if 'image' in entry)
        total_points = (num_poses + num_ids + num_images) * mode_multiplier

        self.get_logger().info(f"Total points: {total_points}")
        self.get_logger().info("Data log:")
        for entry in self.data_log:
            self.get_logger().info(str(entry))

def main():
    rclpy.init()
    referee_node = RefereeNode()

    referee_node.declare_parameter("mode", 'teleop')
   

    try:
        rclpy.spin(referee_node)
    except KeyboardInterrupt:
        referee_node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        if not referee_node.task_completed:
            referee_node.log_data()
        referee_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)
