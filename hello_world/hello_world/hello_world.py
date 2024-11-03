#!/usr/bin/env python3


import rclpy

from time import sleep


def main():

    rclpy.init()

    node = rclpy.create_node('Hello_World')
    
    t=0

    while rclpy.ok():

        try:

            rclpy.spin_once(node, timeout_sec=1.0)

        except KeyboardInterrupt:

            pass

        print('Hello ROS2 World!')
        print(t)  
        t+=1
        sleep(1.0) 

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()