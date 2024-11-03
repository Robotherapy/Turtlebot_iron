#!/usr/bin/env python3

import rclpy
import tf2_ros

import tf_transformations
from geometry_msgs.msg import TransformStamped, Twist

left_wheel_yaw = -3.14/2
right_wheel_yaw = 3.14/2
x_vel = 0.0
yaw_vel = 0.0
AXIS_L = 0.15
WHEEL_R = 0.03

def calculate_wheel_rotation():
    global x_vel, yaw_vel, AXIS_L, WHEEL_R
    left = 1 / WHEEL_R * (x_vel - (AXIS_L / 2) * yaw_vel)
    right = 1 / WHEEL_R * (x_vel + (AXIS_L / 2) * yaw_vel)
    return left, right

def calculate_wheel_position():
    global left_wheel_yaw, right_wheel_yaw, now, mydynamictf
    left_wheel_vel, right_wheel_vel = calculate_wheel_rotation()
    left_wheel_yaw = left_wheel_yaw + (mydynamictf.get_clock().now().nanoseconds / (10 ** 9) - now) * left_wheel_vel
    right_wheel_yaw = right_wheel_yaw + (mydynamictf.get_clock().now().nanoseconds / (10 ** 9) - now) * right_wheel_vel
    now = mydynamictf.get_clock().now().nanoseconds / (10 ** 9)
    return left_wheel_yaw, right_wheel_yaw

def quaternion_from_euler(roll, pitch, yaw):
    #calculate quaternions from euler angles
    quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    t = TransformStamped()
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t.transform.rotation

def calculate_tf():
    global mydynamictf, left_wheel_yaw, right_wheel_yaw
    t = TransformStamped()
    #getting actual position
    left_wheel_yaw, right_wheel_yaw = calculate_wheel_position()
    #calculating transform
    stamp = mydynamictf.get_clock().now().to_msg()
    t.header.stamp = stamp
    t.header.frame_id = "base_link"
    t.transform.translation.x = 0.10
    t.transform.translation.z = -0.03

    return t

def sendtransform():
    global mydynamictf
    base_link_to_wheel = calculate_tf()
    br = tf2_ros.transform_broadcaster.TransformBroadcaster(mydynamictf)

    base_link_to_wheel.child_frame_id = "left_front_wheel"
    base_link_to_wheel.transform.translation.y = 0.1
    base_link_to_wheel.transform.rotation = quaternion_from_euler(-1.57, left_wheel_yaw, 0.0)
    br.sendTransform(base_link_to_wheel)

    base_link_to_wheel.child_frame_id = "right_front_wheel"
    base_link_to_wheel.transform.translation.y = -0.1
    base_link_to_wheel.transform.rotation = quaternion_from_euler(1.57, right_wheel_yaw, 0.0)
    br.sendTransform(base_link_to_wheel)

def get_robot_velocity(msg):
    global x_vel, yaw_vel
    x_vel = msg.linear.x
    yaw_vel = msg.angular.z

def main():
    global mydynamictf, now
    rclpy.init()
    mydynamictf = rclpy.create_node('my_dynamic_tf')
    mydynamictf.create_subscription(Twist, '/cmd_vel', get_robot_velocity, 10)    
    mydynamictf.create_timer(1.0 / 1000.0, sendtransform)
    now = mydynamictf.get_clock().now().nanoseconds / (10 ** 9)
    try:
        rclpy.spin(mydynamictf)
    except KeyboardInterrupt:
        pass    
    mydynamictf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
