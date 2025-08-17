import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class Coffee(Node):
    def __init__(self):
        super().__init__('coffee_node')
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            10
        )
        self.img_publisher = self.create_publisher(Image, 'coffee_detection', 10)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #self.get_logger().info('Received an image!')
            self.publish_processed_image()
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def publish_processed_image(self):
        try:
            templates = [
                cv2.imread("/home/opencv/vision-ws/src/opencv_ros/asset/example.png"),
                cv2.imread("/home/opencv/vision-ws/src/opencv_ros/asset/example2.png")
            ]
            best_match_val = -1
            best_match_idx = -1
            best_match_loc = None
            method = cv2.TM_CCOEFF_NORMED

            for i, template in enumerate(templates):
                result = cv2.matchTemplate(self.cv_image, template, method)
                _, max_val, _, max_loc = cv2.minMaxLoc(result)

                if max_val > best_match_val:
                    best_match_val = max_val
                    best_match_idx = i
                    best_match_loc = max_loc
                    best_template_shape = template.shape[:2]

            # 繪製結果
            h, w = best_template_shape
            top_left = best_match_loc
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(self.cv_image, top_left, bottom_right, (0, 255, 0), 2)
            cv2.putText(self.cv_image, f"Best Match: Template {best_match_idx + 1}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
            self.img_publisher.publish(msg)
            #self.get_logger().info('Processed image published!')
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Coffee()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()