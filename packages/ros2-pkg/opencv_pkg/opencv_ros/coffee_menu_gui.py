import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

def is_near_border(contour, img_shape, border_margin=20):
    """
    Check if any point of the contour is too close to the image border
    """
    height, width = img_shape[:2]
    
    for point in contour:
        x, y = point[0]
        if (x < border_margin or x > width - border_margin or 
            y < border_margin or y > height - border_margin):
            return True
    return False

def are_contours_similar(contour1, contour2, threshold=30):
    """
    Check if two contours are similar (likely duplicates) by comparing their centroids and areas
    """
    # Calculate centroids
    M1 = cv2.moments(contour1)
    M2 = cv2.moments(contour2)
    
    if M1["m00"] == 0 or M2["m00"] == 0:
        return False
    
    cx1, cy1 = int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"])
    cx2, cy2 = int(M2["m10"] / M2["m00"]), int(M2["m01"] / M2["m00"])
    
    # Check if centroids are close
    distance = math.sqrt((cx1 - cx2)**2 + (cy1 - cy2)**2)
    
    # Check if areas are similar
    area1 = cv2.contourArea(contour1)
    area2 = cv2.contourArea(contour2)
    area_ratio = min(area1, area2) / max(area1, area2) if max(area1, area2) > 0 else 0
    
    return distance < threshold and area_ratio > 0.7

def remove_duplicate_contours(contours):
    """
    Remove duplicate/similar contours from the list, keeping the inner (smaller area) one
    """
    if len(contours) <= 1:
        return contours
    
    unique_contours = []
    
    for contour in contours:
        is_duplicate = False
        for i, unique_contour in enumerate(unique_contours):
            if are_contours_similar(contour, unique_contour):
                is_duplicate = True
                # Keep the contour with smaller area (inner one)
                current_area = cv2.contourArea(contour)
                existing_area = cv2.contourArea(unique_contour)
                if current_area < existing_area:
                    unique_contours[i] = contour  # Replace with the smaller (inner) one
                break
        
        if not is_duplicate:
            unique_contours.append(contour)
    
    return unique_contours

def calculate_average_color(img, contour):
    """
    Calculate the average color of the region inside the contour
    Returns the average grayscale value
    """
    # Create a mask for the contour region
    mask = np.zeros(img.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [contour], 255)
    
    # Convert to grayscale if needed
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    
    # Calculate mean color within the masked region
    mean_color = cv2.mean(gray, mask=mask)[0]
    
    return mean_color

def find_most_distinct_parallelogram(img, parallelograms):
    """
    Find the parallelogram with the most distinct average color
    Then analyze the shell/mask content of the selected parallelogram
    Returns the index of the most distinct parallelogram and comparison info
    """
    if len(parallelograms) <= 1:
        return (0, "only one", 0) if parallelograms else (-1, "none", 0)
    
    # Step 1: Calculate average color for each parallelogram
    avg_colors = []
    for parallelogram in parallelograms:
        avg_color = calculate_average_color(img, parallelogram)
        avg_colors.append(avg_color)
    
    # ...existing code...
    # Only return index and color value, no print or shell analysis
    max_min_distance = -1
    most_distinct_idx = 0
    for i, color1 in enumerate(avg_colors):
        min_distance = float('inf')
        for j, color2 in enumerate(avg_colors):
            if i != j:
                distance = abs(color1 - color2)
                min_distance = min(min_distance, distance)
        if min_distance > max_min_distance:
            max_min_distance = min_distance
            most_distinct_idx = i
    return most_distinct_idx, None, avg_colors[most_distinct_idx]

def analyze_region_color(img, contour):
    """
    Analyze the color composition of both the shell (border) and inner center of the region inside the contour
    Shell width is 7.5% of the parallelogram's dimensions
    Inner center is a small parallelogram with 5% margin from the border
    Returns color information and dominant colors
    """
    # Create a mask for the contour region
    mask = np.zeros(img.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [contour], 255)
    
    # Get bounding rectangle
    x, y, w, h = cv2.boundingRect(contour)
    
    # Extract the region of interest
    roi_mask = mask[y:y+h, x:x+w]
    
    if roi_mask.size == 0:
        return "No valid region"
    
    # Calculate shell thickness - make it more inside (increased from 7.5% to 12%)
    shell_thickness = int(min(w, h) * 0.12)
    if shell_thickness < 2:
        shell_thickness = 2
    
    # Create shell mask with NO upper side - only analyze sides and bottom
    # First, create the outer boundary (erode slightly from edge to avoid edge artifacts)
    edge_erosion = max(1, shell_thickness // 4)  # Small erosion from edge
    kernel_edge = np.ones((edge_erosion*2+1, edge_erosion*2+1), np.uint8)
    outer_shell = cv2.erode(roi_mask, kernel_edge, iterations=1)
    
    # Create inner boundary (normal erosion for sides and bottom)
    kernel_inner = np.ones((shell_thickness*2+1, shell_thickness*2+1), np.uint8)
    inner_boundary = cv2.erode(roi_mask, kernel_inner, iterations=1)
    
    # Shell = outer_shell - inner_boundary
    shell_mask = cv2.subtract(outer_shell, inner_boundary)
    
    # Remove the upper portion of the shell mask (no analysis on top)
    upper_removal_height = int(h * 0.25)  # Remove top 25% of the parallelogram
    shell_mask[:upper_removal_height, :] = 0  # Zero out the upper portion
    
    # Create center mask (small parallelogram in the center with ±5% of side length)
    # Calculate 5% of the average side length
    center_size = int((min(w, h) * 0.05))
    if center_size < 1:
        center_size = 1
    
    # Create center mask by finding the center and creating a small region around it
    center_y, center_x = h // 2, w // 2
    
    # Create a small rectangular mask around the center (±5% of side length)
    center_mask = np.zeros_like(roi_mask)
    y1 = max(0, center_y - center_size)
    y2 = min(h, center_y + center_size)
    x1 = max(0, center_x - center_size)
    x2 = min(w, center_x + center_size)
    
    center_mask[y1:y2, x1:x2] = 255
    
    # Make sure the center mask only covers areas within the parallelogram
    center_mask = cv2.bitwise_and(center_mask, roi_mask)
    
    # Convert to grayscale for analysis
    if len(img.shape) == 3:
        roi_gray = cv2.cvtColor(img[y:y+h, x:x+w], cv2.COLOR_BGR2GRAY)
    else:
        roi_gray = img[y:y+h, x:x+w].copy()
    
    # Apply the masks to analyze both regions
    shell_masked = cv2.bitwise_and(roi_gray, shell_mask)
    center_masked = cv2.bitwise_and(roi_gray, center_mask)
    
    center_pixels = np.sum(center_mask > 0)
    shell_avg_color = cv2.mean(shell_masked, mask=shell_mask)[0]
    center_avg_color = cv2.mean(center_masked, mask=center_mask)[0]

    # Only return center brightness info
    return {
        'is_mostly_light': center_avg_color >= shell_avg_color if center_pixels > 0 else False
    }


def is_parallelogram(points):
    """
    Check if 4 points form a parallelogram by verifying that opposite sides are parallel
    """
    if len(points) != 4:
        return False
    
    # Calculate vectors for each side
    sides = []
    for i in range(4):
        p1 = points[i][0]
        p2 = points[(i + 1) % 4][0]
        vector = (p2[0] - p1[0], p2[1] - p1[1])
        sides.append(vector)
    
    # Check if opposite sides are parallel (cross product should be close to 0)
    def are_parallel(v1, v2, tolerance=0.15):  # Increased tolerance slightly
        # Cross product in 2D: v1.x * v2.y - v1.y * v2.x
        cross_product = abs(v1[0] * v2[1] - v1[1] * v2[0])
        # Normalize by magnitudes
        mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
        mag2 = math.sqrt(v2[0]**2 + v2[1]**2)
        if mag1 == 0 or mag2 == 0:
            return False
        normalized_cross = cross_product / (mag1 * mag2)
        return normalized_cross < tolerance
    
    # Check if opposite sides are parallel
    parallel1 = are_parallel(sides[0], sides[2])  # side 0 and side 2
    parallel2 = are_parallel(sides[1], sides[3])  # side 1 and side 3
    
    return parallel1 and parallel2

def detect_parallelograms(img):
    # Read the image


    # Resize image to make it smaller for better display and processing
    height, width = img.shape[:2]
    max_dimension = 800  # Maximum width or height
    
    if width > max_dimension or height > max_dimension:
        if width > height:
            new_width = max_dimension
            new_height = int(height * (max_dimension / width))
        else:
            new_height = max_dimension
            new_width = int(width * (max_dimension / height))
        
        img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_AREA)
        # ...existing code...
    
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 30, 100)
    kernel = np.ones((3,3), np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    parallelograms = []
    all_quadrilaterals = []
    for contour in contours:
        epsilon = 0.015 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 4:
            area = cv2.contourArea(approx)
            if area > 500:
                if not is_near_border(approx, img.shape, border_margin=30):
                    all_quadrilaterals.append(approx)
                    if is_parallelogram(approx):
                        parallelograms.append(approx)
    all_quadrilaterals = remove_duplicate_contours(all_quadrilaterals)
    parallelograms = remove_duplicate_contours(parallelograms)
    # Draw directly on the input image
    for i, parallelogram in enumerate(parallelograms):
        cv2.drawContours(img, [parallelogram], -1, (0, 255, 0), 3)
    if parallelograms:
        all_vertices = []
        for parallelogram in parallelograms:
            vertices = parallelogram.reshape(-1, 2)
            all_vertices.extend(vertices)
        all_vertices = np.array(all_vertices)
        all_para_center_x = int(np.mean(all_vertices[:, 0]))
        all_para_center_y = int(np.mean(all_vertices[:, 1]))
        cv2.circle(img, (all_para_center_x, all_para_center_y), 8, (0, 255, 255), -1)
    center_brightness = 0  # Initialize to 0
    if parallelograms:
        most_distinct_idx, _, _ = find_most_distinct_parallelogram(img, parallelograms)
        color_analysis = analyze_region_color(img, parallelograms[most_distinct_idx])
        center_brightness = 1 if color_analysis.get('is_mostly_light', False) else 2  # 1 for white, 2 for black
        M = cv2.moments(parallelograms[most_distinct_idx])
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            vertices = parallelograms[most_distinct_idx].reshape(-1, 2)
            para_center_x = int(np.mean(vertices[:, 0]))
            para_center_y = int(np.mean(vertices[:, 1]))
            cv2.circle(img, (para_center_x, para_center_y), 5, (255, 0, 0), -1)
            if cx < all_para_center_x and cy < all_para_center_y:
                pos_label = 1
            elif cx >= all_para_center_x and cy < all_para_center_y:
                pos_label = 2
            elif cx < all_para_center_x and cy >= all_para_center_y:
                pos_label = 3
            else:
                pos_label = 4
            cv2.putText(img, f"Center result: {center_brightness}", (cx-60, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.putText(img, f"{pos_label}", (cx-60, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,128,255), 2)
    return img

class Coffee(Node):
    def __init__(self):
        super().__init__('coffee_gui')
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(
            Image,
            "/kesler/cam_forward/color/image_raw",
            self.image_callback,
            10
        )
        self.img_publisher = self.create_publisher(Image, 'coffee_detection', 10)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.publish_processed_image()
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    def publish_processed_image(self):
        try:
            # image processing logic
            self.cv_image = detect_parallelograms(self.cv_image)
            msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
            self.img_publisher.publish(msg)
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

