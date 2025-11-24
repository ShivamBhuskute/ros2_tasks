import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time # Added for initialization delay

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.declare_parameter('publish_rate', 10.0)
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Image, '/arena_camera/image_raw', 10)
        
        self.br = CvBridge()
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open video stream or file.")
            rclpy.shutdown()
            return

        time.sleep(1) 
        
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f'Camera Publisher started at {self.publish_rate} Hz.')

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            ros_image = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            
            self.publisher_.publish(ros_image)
        else:
            self.get_logger().warn('Failed to read frame from camera.')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()