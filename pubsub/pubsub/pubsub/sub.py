#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriberClass(Node):

    def __init__(self):
        super().__init__('myfirstsubscriber')
        self.subscription = self.create_subscription(
            String, 'myfirsttopic', self.mysubcallback, 10)

    def mysubcallback(self, msg):
        print('You said: ', msg.data)

def main():
    rclpy.init()
    subscribernode = MySubscriberClass()
    try:
        rclpy.spin(subscribernode)
    except KeyboardInterrupt:
        pass
    subscribernode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()