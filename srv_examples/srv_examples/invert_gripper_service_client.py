#!/usr/bin/env python3

import rclpy
from std_srvs.srv import SetBool

def main():
    rclpy.init()
    gripper_service_client = rclpy.create_node('gripper_client_node')
    gripper_service = gripper_service_client.create_client(SetBool, 'open_gripper')

    while not gripper_service.wait_for_service(timeout_sec=1.0):
        print("Service not available, trying again...")

    req = SetBool.Request()
    # initially request to open the gripper
    req.data = True
    # invert gripper result initially false
    inverted = False
    
    while(inverted == False):
        future = gripper_service.call_async(req)
        #Use spin_until_future_complete to wait for the ROS2 service server
        rclpy.spin_until_future_complete(gripper_service_client, future)
        try:
            result = future.result()
        except KeyboardInterrupt:
            pass
        inverted = result.success
        if(inverted==True):
            print('The gripper has been inverted.')
        else:
            req.data = False

    gripper_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
