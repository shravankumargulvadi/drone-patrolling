import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        img = cv2.imread('/home/shravan/Projects/men_with_gun.jpg')  
        if img is None:
            self.get_logger().error("Failed to read image!")
            return
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published test image.')

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

