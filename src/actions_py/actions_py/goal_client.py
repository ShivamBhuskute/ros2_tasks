import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_demo_pkg.action import TrackGoal
import random
import sys

class GoalClient(Node):
    def __init__(self):
        super().__init__("goal_client")
        self.client = ActionClient(self, TrackGoal, "track_goal")
        self.timer = self.create_timer(10.0, self.send_goal)

    def send_goal(self):
        goal = TrackGoal.Goal()
        goal.x = random.uniform(-5.0, 5.0)
        goal.y = random.uniform(-5.0, 5.0)

        self.client.wait_for_server()
        self.get_logger().info(f"SENDING GOAL: x={goal.x:.2f}, y={goal.y:.2f}")
        send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        send_goal_future.add_done_callback(self.goal_response)

    def feedback(self, feedback_msg):
        self.get_logger().info(f"Distance left: {feedback_msg.feedback.distance_left:.2f}")

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result)

    def get_result(self, result_future):
        result = result_future.result().result
        if(result.result == "Goal Completed"):
            self.get_logger().info(f"Result: {result.result}")
            sys.exit()

def main(args=None):
    rclpy.init(args=args)
    node = GoalClient()
    rclpy.spin(node)
    rclpy.shutdown()
