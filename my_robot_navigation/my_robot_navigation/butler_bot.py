#!/usr/bin/env python3

from math import sqrt
import sys
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion
from time import time, sleep

# Define the home, kitchen, and table positions
home = {"x": 0.0, "y": 0.0, "z": 0.0}
kitchen = {"x": 1.5, "y": 2.0, "z": 0.0}  # Example position for kitchen
tables = {
    "table1": {"x": 2.0, "y": 3.0, "z": 0.0},
    "table2": {"x": 2.5, "y": -2.5, "z": 0.0},
    "table3": {"x": -1.5, "y": 3.5, "z": 0.0},
}

# Define robot's operational logic for each scenario
def main():
    global auto_chaos, nav_to_pose_client
    rclpy.init()

    auto_chaos = rclpy.create_node('butler_robot')

    # Create Action Client object
    nav_to_pose_client = ActionClient(auto_chaos, NavigateToPose, 'navigate_to_pose')

    # Wait for the action server to be available
    while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):
        print("Waiting for action server to be available...")

    # Example scenario: Handling multiple orders with some cancellations
    orders = ["table1", "table2", "table3"]
    handleOrders(orders)

    nav_to_pose_client.destroy()
    auto_chaos.destroy_node()
    rclpy.shutdown()

def handleOrders(orders):
    """Handle the orders and manage the robot's actions."""
    for order in orders:
        if not executeTask(order):
            print(f"Task for {order} was canceled or timed out.")
            break
    print("Completed all orders or task was interrupted.")

def executeTask(order):
    """Execute task for delivering food to a specific table."""
    global auto_chaos, nav_to_pose_client

    # Start at home position
    print(f"Received order for {order}. Moving to kitchen...")
    if not moveTo(kitchen):
        return False

    print(f"Collecting food from the kitchen for {order}...")
    if not waitForConfirmation("kitchen"):
        return False

    print(f"Moving to {order} for food delivery...")
    if not moveTo(tables[order]):
        return False

    if not waitForConfirmation(order):
        return False

    print(f"Task completed for {order}. Returning to home.")
    return moveTo(home)

def moveTo(position):
    """Move the robot to a specified position."""
    position = generatePosition(position)
    orientation = generateOrientation()
    goal_handle = sendGoal(position, orientation)
    status = checkResult(goal_handle)

    if status != GoalStatus.STATUS_SUCCEEDED:
        print(f"Failed to reach position at X: {position.x}, Y: {position.y}")
        return False
    return True

def waitForConfirmation(location):
    """Wait for confirmation at a location (either table or kitchen)."""
    start_time = time()
    while time() - start_time < 30:  # Timeout after 30 seconds
        print(f"Waiting for confirmation at {location}...")
        sleep(5)  # Wait for a few seconds to simulate checking for confirmation
        # Add condition to check if confirmation is received (this would be a sensor or user input in a real robot)
        if confirmationReceived(location):
            print(f"Confirmation received at {location}.")
            return True
    print(f"No confirmation received at {location}. Returning to home position.")
    return False

def confirmationReceived(location):
    """Simulate confirmation reception (to be replaced with actual confirmation logic)."""
    # Placeholder: simulate that only table1 confirms orders
    return location == "table1" or location == "kitchen"

def sendGoal(position, orientation):
    """Create and send a goal to navigate to a position."""
    global auto_chaos, nav_to_pose_client

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = auto_chaos.get_clock().now().to_msg()
    goal.pose.pose.position = position
    goal.pose.pose.orientation = orientation

    print(f"Sending goal to position => X: {goal.pose.pose.position.x} Y: {goal.pose.pose.position.y}")

    send_goal_future = nav_to_pose_client.send_goal_async(goal, feedback_callback=feedbackCallback)
    rclpy.spin_until_future_complete(auto_chaos, send_goal_future)

    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        print("Goal was rejected.")
        return None

    print("Goal Accepted!")
    return goal_handle

def feedbackCallback(feedback_msg):
    """Callback to process feedback from the action server."""
    feedback = feedback_msg.feedback
    current_pose = feedback.current_pose.pose
    print(f"Feedback: Robot at position X: {current_pose.position.x:.2f}, Y: {current_pose.position.y:.2f}")

def checkResult(goal_handle):
    """Check if the goal was achieved successfully."""
    if goal_handle is None:
        return GoalStatus.STATUS_FAILED
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(auto_chaos, get_result_future)
    return get_result_future.result().status

def generatePosition(point):
    """Convert point dictionary to a geometry_msgs/Point."""
    position = Point()
    position.x = point["x"]
    position.y = point["y"]
    position.z = point["z"]
    return position

def generateOrientation():
    """Generate a simple orientation for the robot."""
    quat = Quaternion()
    quat.w = 1.0
    quat.x = 0.0
    quat.y = 0.0
    quat.z = 0.0
    return quat

if __name__ == '__main__':
    main()
