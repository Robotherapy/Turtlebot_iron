#!/usr/bin/env python3


import rclpy

from time import sleep


def main():

    rclpy.init()

    node = rclpy.create_node('my_param_node')

    node.declare_parameter('my_param', 13)

    while rclpy.ok():

        try:

            rclpy.spin_once(node, timeout_sec=1.0)

        except KeyboardInterrupt:

            pass

        print(node.get_parameter('my_param').value)

        sleep(1.0)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()