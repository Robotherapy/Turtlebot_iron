#!/usr/bin/env python3

from math import sqrt
import sys
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion

# Define 6 points (x, y, z)
points = [
    {"x": 2.89, "y": 0.26, "z": -0.0},
   
    {"x": 1.89, "y": -2.05, "z": -0.0},
    
    {"x": 3.54, "y": -2.93, "z": -0.0},
   
    {"x": 0.82, "y": -2.25, "z": 0.0},
   
    {"x": 2.1, "y": -2.07, "z": 0.0},
   
    {"x": 2.41, "y": 0.56, "z": 0.0},
    {"x": 0.0, "y": 0.0, "z": 0.0},
   
]

def main():
    global auto_chaos
    global nav_to_pose_client

    rclpy.init()

    auto_chaos = rclpy.create_node('auto_goals')

    # Create Action Client object with desired message type and action name
    nav_to_pose_client = ActionClient(auto_chaos, NavigateToPose, 'navigate_to_pose')

    # Wait for the action server to come up
    while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):
        print("Server still not available; waiting...")

    # Navigate through each point in the array
    for point in points:
        position = generatePosition(point)
        orientation = generateOrientation()
        goal_handle = sendGoal(position, orientation)
        status = checkResult(goal_handle)
        if status != GoalStatus.STATUS_SUCCEEDED:
            print("Failed to reach one of the goals. Exiting.")
            break

    nav_to_pose_client.destroy()
    auto_chaos.destroy_node()
    rclpy.shutdown()

def sendGoal(position, orientation):
    """Create action goal object and send to action server, with feedback callback"""
    global auto_chaos
    global nav_to_pose_client

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"

    goal.pose.header.stamp = auto_chaos.get_clock().now().to_msg()

    goal.pose.pose.position = position
    goal.pose.pose.orientation = orientation

    print("Sending new goal => X: " + str(goal.pose.pose.position.x) + " Y: " + str(goal.pose.pose.position.y))

    # Send goal and attach feedback callback
    send_goal_future = nav_to_pose_client.send_goal_async(goal, feedback_callback=feedbackCallback)
    rclpy.spin_until_future_complete(auto_chaos, send_goal_future)

    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        print("Goal was rejected")
        nav_to_pose_client.destroy()
        auto_chaos.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    print("Goal Accepted!")

    return goal_handle

def feedbackCallback(feedback_msg):
    """Callback to process feedback from the action server"""
    feedback = feedback_msg.feedback
    # Extract current position feedback (assumed structure based on the action definition)
    current_pose = feedback.current_pose.pose
    print(f"Feedback: Robot is at position X: {current_pose.position.x:.2f}, Y: {current_pose.position.y:.2f}")

def checkResult(goal_handle):
    """Check for task completion while blocking further execution"""
    get_result_future = goal_handle.get_result_async()

    rclpy.spin_until_future_complete(auto_chaos, get_result_future)

    status = get_result_future.result().status

    if status == GoalStatus.STATUS_SUCCEEDED:
        print("Reached Goal!!!")

    return status

def generatePosition(point):
    """Convert predefined point values to geometry_msgs/Point"""
    position = Point()
    position.x = point["x"]
    position.y = point["y"]
    position.z = point["z"]
    return position

def generateOrientation():
    """Generate a fixed orientation (for simplicity)"""
    quat = Quaternion()
    quat.w = 1.0
    quat.x = 0.0
    quat.y = 0.0
    quat.z = 0.0
    return quat

if __name__ == '__main__':
    main() 