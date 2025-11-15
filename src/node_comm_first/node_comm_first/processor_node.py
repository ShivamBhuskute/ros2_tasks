import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math

class Processor_Node(Node):
    def __init__(self):
        super().__init__("processor_node")
        self.processor_node = self.create_subscription(Vector3, "/sensor_node", self.pose_callback, 10)

        self.publisher = self.create_publisher(Vector3, "/processor_node", 10)

    def pose_callback(self, msg: Vector3):
        x2 = msg.x**2
        y2 = msg.y**2
        z2 = msg.z**2

        length = math.sqrt(x2 + y2 + z2)

        vec_msg = Vector3()
        vec_msg.x = length
        vec_msg.y = 0.0
        vec_msg.z = 0.0

        self.publisher.publish(vec_msg)

        self.get_logger().info("Legth of the vector is: " + str(length))




def main(args=None):
    rclpy.init(args=args)
    node = Processor_Node()
    rclpy.spin(node)
    rclpy.shutdown()
