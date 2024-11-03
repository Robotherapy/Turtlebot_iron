#!/usr/bin/env python3

import rclpy
from std_srvs.srv import SetBool

gripper_opened = True

def open_gripper_callback(request, response):
        global gripper_opened
        if request.data:
            if gripper_opened:
                response.success = False
                response.message = 'Opening gripper failed. Gripper was already opened.'
            else:
                response.success = True
                response.message = 'Opening gripper succeeded. Gripper is open now.'
                print('The gripper is open.')
        else:
            if gripper_opened:
                response.success = True
                response.message = 'Closing gripper succeeded. Gripper is closed now.'
                print('The gripper is closed.')
            else:
                response.success = False
                response.message = 'Closing gripper failed. Gripper was already closed.'
        if response.success:
            gripper_opened = not gripper_opened
        return response

def main():
    rclpy.init()
    myfirstsrvserver = rclpy.create_node('myfirstserviceserver')
    myfirstsrvserver.create_service(SetBool, 'open_gripper', open_gripper_callback)
    print('The gripper is open.')
    try:
        rclpy.spin(myfirstsrvserver)
    except KeyboardInterrupt:
        pass
    myfirstsrvserver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()