import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

cm_per_pixel = 0.115625

class Algorithm:
    def __init__(self):
        self.red = (0, 0, 255)  # BGR format for red color
        self.green = (0, 255, 0)  # BGR format for green color
        self.blue = (255, 0, 0)  # BGR format for blue color
        
    def detect_red_dots(self, input_img):
        """Find red dots in the image"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
        
        # Define more restrictive range for red color in HSV
        # Red color has two ranges in HSV - making them tighter
        lower_red1 = np.array([0, 120, 100])      # Higher saturation and value
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 100])    # Higher saturation and value
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # Apply more aggressive morphological operations to clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        return red_mask
        
    def detect_stickers(self, input_img):
        """Convert image through various morphological operations for sticker detection"""
        gray = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)
        can = cv2.Canny(gray, 80, 150)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        dil = cv2.dilate(can, kernel)
        mor = cv2.morphologyEx(dil, cv2.MORPH_OPEN, kernel)
        ero = cv2.erode(mor, kernel)
        
        thre = cv2.adaptiveThreshold(ero, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                   cv2.THRESH_BINARY_INV, 15, 10)
        blur = cv2.medianBlur(thre, 5)
        
        return blur
    
    def find_targets(self, input_img):
        """Find both red dots and stickers in the image"""
        
        # Detect red dots
        red_mask = self.detect_red_dots(input_img)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Detect stickers
        sticker_mask = self.detect_stickers(input_img)
        sticker_contours, _ = cv2.findContours(sticker_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        red_dots = []  # points of interest for red dots
        stickers = []  # points of interest for stickers
        
        # Process red dot contours
        for contour in red_contours:
            area = cv2.contourArea(contour)
            
            # Filter for small circular objects (red dots)
            if 50 < area < 2000:  # Adjust size range for small dots
                # Get bounding rectangle
                rect = cv2.boundingRect(contour)
                x, y, w, h = rect
                
                # Check if it's roughly circular (width/height ratio close to 1)
                ratio = w / h if h > 0 else 0
                if 0.7 <= ratio <= 1.3:  # Allow some tolerance for circular shape
                    # Calculate center point
                    center_x = x + w // 2
                    center_y = y + h // 2
                    red_dots.append((center_x, center_y))
        
        # Process sticker contours
        for contour in sticker_contours:
            area = cv2.contourArea(contour)
            rect = cv2.boundingRect(contour)
            
            ratio = rect[2] / rect[3]  # width / height
            fillratio = area / (rect[2] * rect[3])  # area / (width * height)
            
            # Check if it's roughly square and well-filled
            if (0.6 <= ratio <= 1.4) and (fillratio >= 0.6):
                if 7000 < area < 30000:
                    # Calculate center point
                    center_x = rect[0] + rect[2] // 2
                    center_y = rect[1] + rect[3] // 2
                    stickers.append((center_x, center_y))
        
        # Return the first detected targets
        red_dot_center = red_dots[0] if red_dots else None
        sticker_center = stickers[0] if stickers else None
        
        return red_dot_center, sticker_center

def process_image(image):
    """Main function to process an image file"""
    # Create algorithm instance and process
    algo = Algorithm()
    red_dot_center, sticker_center = algo.find_targets(image)
    
    return red_dot_center, sticker_center

class Coffee(Node):
    def __init__(self):
        super().__init__('coffee_sticker_detection')
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(
            Image,
            "/kesler/cam_downward/color/image_raw",
            self.image_callback,
            10
        )
        # Create integer publisher for x-coordinate difference
        self.x_diff_publisher = self.create_publisher(Int32, 'x_trans', 10)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_and_publish()
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            
    def process_and_publish(self):
        try:
            # image processing logic
            red_dot_center, sticker_center = process_image(self.cv_image)
            
            # Calculate and publish x-coordinate difference
            if red_dot_center and sticker_center:
                x_diff = sticker_center[0] - red_dot_center[0]

                x_diff *= -1
                x_diff *= cm_per_pixel
                
                # Create and publish the x-coordinate difference message
                diff_msg = Int32()
                diff_msg.data = int(x_diff)  # Convert to integer
                self.x_diff_publisher.publish(diff_msg)
                
                # Log the results
                self.get_logger().info(f'Red dot at: {red_dot_center}, Sticker at: {sticker_center}, X-diff: {x_diff}')
            else:
                # If either target is not found, publish 0 or a special value
                diff_msg = Int32()
                diff_msg.data = 0  # You can change this to a special value like -999 if preferred
                self.x_diff_publisher.publish(diff_msg)
                
                if not red_dot_center:
                    self.get_logger().warn('Red dot not detected')
                if not sticker_center:
                    self.get_logger().warn('Sticker not detected')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Coffee()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

