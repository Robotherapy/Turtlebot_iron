#!/usr/bin/env python3


import rclpy

from time import sleep


def main():

    rclpy.init()

    node = rclpy.create_node('hello_param_node')

    node.declare_parameter('freq', 1.0)
    node.declare_parameter('text', 'Hello')
    t=0
    while rclpy.ok():

        try:

            rclpy.spin_once(node, timeout_sec=1.0)

        except KeyboardInterrupt:

            pass
        
        print(node.get_parameter('text').value)
        print(t)  
        t+=1

        sleep(node.get_parameter('freq').value)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()