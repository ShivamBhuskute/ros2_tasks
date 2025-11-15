import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_demo_pkg.action import TrackGoal
import math
from rclpy.duration import Duration

class TrackGoalServer(Node):
    def __init__(self):
        super().__init__("track_goal_server")
        self.server = ActionServer(
            self,
            TrackGoal,
            "track_goal",
            execute_callback=self.execute_callback,
        )
        self.get_logger().info("SERVER STARTED")

    def execute_callback(self, goal_handle):
        target = goal_handle.request
        x2, y2 = target.x, target.y

        self.get_logger().info("Executing the goal")

        feedback = TrackGoal.Feedback()

        x1, y1 = 0.0, 0.0
        rate = 2.0
        dt = 1.0 / rate

        while True:
            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            feedback.distance_left = distance
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Distance left: {distance:.2f}")

            if distance < 0.5:
                break

            x1 += 0.1 * (x2 - x1)
            y1 += 0.1 * (y2 - y1)

            self.get_clock().sleep_for(Duration(seconds=dt))

        goal_handle.succeed()
        result = TrackGoal.Result()
        result.result = "Goal Completed"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrackGoalServer()
    rclpy.spin(node)
    rclpy.shutdown()
