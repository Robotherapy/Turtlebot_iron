#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations

class TagPosePublisher(Node):
    def __init__(self):
        super().__init__('tag_pose_publisher')

        # Create publishers for tag pose and ID
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_marker_pose', 10)
        self.id_pub = self.create_publisher(Int8, '/detected_marker_id', 10)

        # Create a subscriber to the /detections topic
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)

        # Create a TF buffer and listener to transform pose to 'map' frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Tag pose publisher node has been started.')

    def detection_callback(self, msg):
        if not msg.detections:
            self.get_logger().info('No tags detected.')
            return

        for detection in msg.detections:
            # Extract the ID
            tag_id = detection.id

            # Create a PoseStamped message for the tag pose
            tag_pose = PoseStamped()
            tag_pose.header.stamp = self.get_clock().now().to_msg()
            tag_pose.header.frame_id = 'camera_link'  # Pose is in 'camera_link' frame

            # Transform homography to pose
            homography = detection.homography
            tag_pose.pose.position.x = homography[2]  # Extract x from homography
            tag_pose.pose.position.y = homography[5]  # Extract y from homography
            tag_pose.pose.position.z = 0.0  # Homography does not provide z

            # Convert homography to orientation (quaternion) if needed, placeholder here
            tag_pose.pose.orientation.x = 0.0
            tag_pose.pose.orientation.y = 0.0
            tag_pose.pose.orientation.z = 0.0
            tag_pose.pose.orientation.w = 1.0

            try:
                # Transform the tag pose to the 'map' frame
                transform = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
                
                # Manually perform the transformation if the direct method fails
                transformed_pose = self.perform_pose_transform(tag_pose, transform)
                self.pose_pub.publish(transformed_pose)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'Transform error: {str(e)}')

            # Publish the tag ID
            tag_id_msg = Int8()
            tag_id_msg.data = tag_id
            self.id_pub.publish(tag_id_msg)

            self.get_logger().info(f'Published pose and ID for tag {tag_id}')

    def perform_pose_transform(self, pose, transform):
        # Convert PoseStamped to a format suitable for transformation
        pose_stamped = PoseStamped()
        pose_stamped.header = pose.header
        pose_stamped.pose = pose.pose

        # Apply the transformation manually
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Convert quaternion to rotation matrix
        rotation_matrix = tf_transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        
        # Convert PoseStamped to matrix form
        pose_matrix = tf_transformations.translation_matrix([
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z
        ])

        # Apply the transformation matrix
        transformed_matrix = tf_transformations.concatenate_matrices(rotation_matrix, pose_matrix)
        
        # Extract the transformed pose
        transformed_pose = PoseStamped()
        transformed_pose.header.stamp = self.get_clock().now().to_msg()
        transformed_pose.header.frame_id = 'base_link'

        transformed_pose.pose.position.x = transformed_matrix[0, 3] + translation.x
        transformed_pose.pose.position.y = transformed_matrix[1, 3] + translation.y
        transformed_pose.pose.position.z = transformed_matrix[2, 3] + translation.z

        # Extract and set orientation
        q = tf_transformations.quaternion_from_matrix(transformed_matrix)
        transformed_pose.pose.orientation.x = q[0]
        transformed_pose.pose.orientation.y = q[1]
        transformed_pose.pose.orientation.z = q[2]
        transformed_pose.pose.orientation.w = q[3]

        return transformed_pose

def main(args=None):
    rclpy.init(args=args)
    tag_pose_publisher = TagPosePublisher()

    try:
        rclpy.spin(tag_pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tag_pose_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
