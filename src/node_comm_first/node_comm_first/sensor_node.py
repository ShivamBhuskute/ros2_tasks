import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import random

class sensor_node(Node):
    def __init__(self):
        super().__init__("sensor_node")

        self.declare_parameter("publish_rate", 5)
        rate = self.get_parameter("publish_rate").value
        timer_period = 1.0 / float(rate)

        self.publisher_ = self.create_publisher(Vector3, "/sensor_node", 10)

        self.timer = self.create_timer(timer_period, self.publish_data)

    def publish_data(self):
        msg = Vector3()
        msg.x = random.uniform(-5.0, 5.0)
        msg.y = random.uniform(-5.0, 5.0)
        msg.z = random.uniform(-5.0, 5.0)
        self.publisher_.publish(msg)
        self.get_logger().info("Published: " + str(msg.x) + str(msg.y) + str(msg.z))


def main(args=None):
    rclpy.init(args=args)
    node = sensor_node()
    rclpy.spin(node)
    rclpy.shutdown()