import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from math import sin, cos
import numpy as np
import json
import threading
from geometry_msgs.msg import PoseWithCovarianceStamped

class ToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_action_client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.initialpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.move_status = False
        self.feedback = {"state": "stay", 
        "number_of_recoveries": 0,
        "distance_remaining": 0,
        "navigation_time": 0}
        self.target = [0,0,0]
                    #states: moving to goal, goal reached, goal cant be reached
    def euler_to_quaternion(self, yaw):
        pitch = 0
        roll = 0
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
    def set_pose(self,x,y,theta):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = float(x)
        pose_msg.pose.pose.position.y = float(y)
        pose_msg.pose.pose.position.z = float(0)
        qx, qy, qz, qw = self.euler_to_quaternion(theta)
        pose_msg.pose.pose.orientation.w = qw
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        # print(pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.w)
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = 0.0685
        self.initialpose.publish(pose_msg)

    def send_goal(self, x,y,theta):
        self.target = [x,y,theta]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        qx, qy, qz, qw = self.euler_to_quaternion(theta)
        goal_msg.pose.pose.orientation.w = qw
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        # print(goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.x, goal_msg.pose.pose.orientation.w, goal_msg.pose.pose.orientation.y)
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.feedback["state"] = "goal cant be reached"
            return

        self.feedback["state"]="moving to goal"

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        if self.feedback["number_of_recoveries"] < 6:
            if self.feedback["distance_remaining"] < 0.4:
                self.feedback["state"] = "goal reached"
            else:
                self.send_goal(self.target[0], self.target[1], self.target[2])
        else:
            self.feedback["state"] = "goal cant be reached"
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.feedback["number_of_recoveries"] = feedback.number_of_recoveries
        self.feedback["distance_remaining"] = feedback.distance_remaining
        self.feedback["navigation_time"] = feedback.navigation_time.sec
        # self.get_logger().info('Received feedback: {0}'.format(feedback))
    def get_feedback(self):
        return self.feedback

def main(args=None):
    rclpy.init(args=args)

    action_client = ToPoseClient()
    thread = threading.Thread(target=rclpy.spin, args=(action_client, ), daemon=True)
    thread.start()
    rate = action_client.create_rate(2)
    # future = action_client.send_goal(-1.85,-1.12,5.1)
    action_client.set_pose(1.0,0.0,3.0)
    while rclpy.ok():
        try:
            print(action_client.get_feedback())
            rate.sleep()
        except KeyboardInterrupt:
            rclpy.shutdown()

if __name__ == '__main__':
    main()