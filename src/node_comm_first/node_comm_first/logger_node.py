import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import sys

class Logger(Node):
    def __init__(self):
        super().__init__("logger_node")
        self.subscription = self.create_subscription(
            Vector3, "/processor_node", self.log_callback, 10
        )
        self.latest_length = 0.0
        self.data = []

    def log_callback(self, msg: Vector3):
        self.latest_length = msg.x
        self.data.append(self.latest_length)

        if len(self.data) > 200:
            # self.data = self.data[-200:]
            sys.exit()

        with open("logger_data.txt", "w") as f:
            for val in self.data:
                f.write(f"{val}\n")

        self.get_logger().info(f"Logged vector length: {self.latest_length}")

def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    rclpy.spin(node)
    rclpy.shutdown()