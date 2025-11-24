import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ZoneDetector(Node):
    def __init__(self):
        super().__init__('zone_detector')
        
        self.subscription = self.create_subscription(
            Image,
            '/arena_camera/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()

        self.publisher_ = self.create_publisher(String, '/detected_zone', 10)
        
        self.get_logger().info('Zone Detector node initialized.')

    def get_color_ranges(self, color_name):

        if color_name == 'red':
            lower1 = np.array([0, 100, 100])
            upper1 = np.array([10, 255, 255])
            lower2 = np.array([160, 100, 100])
            upper2 = np.array([179, 255, 255])
            return [(lower1, upper1), (lower2, upper2)]
        
        elif color_name == 'green':
            lower = np.array([35, 100, 100])
            upper = np.array([85, 255, 255])
            return [(lower, upper)]
        
        elif color_name == 'blue':
            lower = np.array([90, 100, 100])
            upper = np.array([130, 255, 255])
            return [(lower, upper)]
        
        elif color_name == "white":
            lower = np.array([0, 0, 180])
            upper = np.array([179, 30, 255])
            return [(lower, upper)]
        
        return []

    def process_detection(self, hsv_image, frame, color_name, draw_color):
        color_ranges = self.get_color_ranges(color_name)
        mask_combined = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        for lower, upper in color_ranges:
            mask = cv2.inRange(hsv_image, lower, upper)
            mask_combined = cv2.bitwise_or(mask_combined, mask)

        contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        
        for cnt in contours:
            if cv2.contourArea(cnt) > 500: 
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
                
                center_x = x + w // 2
                center_y = y + h // 2
                
                label = f"{color_name.upper()} ZONE"
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)
                
                # Prepare detection result for publishing/logging
                detections.append(f"Detected Zone: {color_name.capitalize()} | Position: ({center_x}, {center_y})")

        return detections

    def listener_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        all_detections = []
        
        red_detections = self.process_detection(hsv_image, frame, 'red', (0, 0, 255))
        all_detections.extend(red_detections)

        green_detections = self.process_detection(hsv_image, frame, 'green', (0, 255, 0))
        all_detections.extend(green_detections)
        
        blue_detections = self.process_detection(hsv_image, frame, 'blue', (255, 0, 0))
        all_detections.extend(blue_detections)

        
        if all_detections:
            output_string = "; ".join(all_detections)
            
            self.get_logger().info(f"Published: {output_string}")
            
            msg = String()
            msg.data = output_string
            self.publisher_.publish(msg)
        else:
            self.get_logger().info("No zones detected.")
            msg = String()
            msg.data = "No Zone"
            self.publisher_.publish(msg)

        cv2.imshow("Robocon Zone Detector", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    zone_detector = ZoneDetector()
    rclpy.spin(zone_detector)
    zone_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()