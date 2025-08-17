import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from interfaces.srv import ProcessCoffee

class PhotoCaptureService(Node):
    def __init__(self):
        super().__init__('photo_capture_service_node')
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Create subscription to the camera topic to keep latest image
        self.img_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10
        )
        
        # Create the service
        self.srv = self.create_service(
            ProcessCoffee,
            'process_coffee',
            self.process_coffee_callback
        )
        
        # Create directory for saved images if it doesn't exist
        self.save_dir = "/home/tdkkesler/cam_process/Camera_tutorial_tdk/packages/opencv-pkg/captured_images"
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.get_logger().info('Photo capture service started. Listening to camera and waiting for service calls...')

    def image_callback(self, msg):
        # Keep the latest image available for service calls
        self.latest_image = msg
        
    def process_coffee_callback(self, request, response):
        try:
            # Check if status is 2 (take photo command)
            if request.status != 2:
                self.get_logger().info(f'Received status {request.status}, but only status 2 triggers photo capture')
                response.photo_taken = False
                return response
                
            if self.latest_image is None:
                self.get_logger().warn('No image available from camera topic')
                response.photo_taken = False
                return response
                
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"coffee_photo_{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)
            
            # Save the image
            success = cv2.imwrite(filepath, cv_image)
            
            if success:
                self.get_logger().info(f'Photo saved successfully: {filepath}')
                self.get_logger().info(f'Image dimensions: {cv_image.shape[1]}x{cv_image.shape[0]}')
                response.photo_taken = True
            else:
                self.get_logger().error('Failed to save photo')
                response.photo_taken = False
                
        except Exception as e:
            self.get_logger().error(f'Error processing service request: {e}')
            response.photo_taken = False
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PhotoCaptureService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

