#!/usr/bin/env python3

import rclpy
from std_srvs.srv import Empty
from turtlesim.msg import Pose

def callback(msg):
    if not 3.0 < msg.x < 7.0:
        reset()
    if not 3.0 < msg.y < 7.0:
        reset()

def reset():
    global reset_service
    while not reset_service.wait_for_service(timeout_sec=1.0):
        print('service not available, trying again...')
    reset_service.call_async(Empty.Request())

def main():
    global reset_service
    rclpy.init()
    myfirstsrvclient = rclpy.create_node('myfirstserviceclient')
    myfirstsrvclient.create_subscription(Pose, '/turtle1/pose', callback, 10)
    reset_service = myfirstsrvclient.create_client(Empty, 'reset')
    try:
        rclpy.spin(myfirstsrvclient)
    except KeyboardInterrupt:
        pass
    myfirstsrvclient.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()