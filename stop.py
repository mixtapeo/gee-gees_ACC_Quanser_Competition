import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class StopDetector(Node):
    def __init__(self):
        super().__init__('stop_sign_detector')
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.subscription_stop = self.create_subscription(Image, '/camera/color_image', self.stop_callback, qos_profile)
        self.subscriber_depth = self.create_subscription(Image, 'camera/depth_image', self.calldepth, qos_profile)
        self.depth_stop = self.create_publisher(Float32, '/stop_depth', qos_profile)
        self.depths = None
        self.stop_depth = None
        
    def calldepth(self, depth_message):
        self.depths = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='16UC1')
     
    def stop_callback(self, stopmsg):
        frameforstop = self.bridge.imgmsg_to_cv2(stopmsg, desired_encoding='bgr8')
        if frameforstop is None:
            return
        #Look for stop sign.
        image_stop, stop_flag, stop_x, stop_y = self.detect_stop_sign(frameforstop.copy())

        #cv2.circle(image_stop, (450, 470), 2, (255, 0, 0), -1)  # Green Circle
        
        if self.depths is not None:
            cv2.imshow("stop", image_stop)
            cv2.waitKey(1)
            depth_from_stop_sign = self.depths[stop_y, stop_x] / 1000.0
            #depth_msg_stop = Float32()
            #depth_msg_stop.data = float(depth_from_stop_sign)
            if stop_x is not None:
                depth_msg = Float32()
                depth_msg.data = depth_from_stop_sign
                self.depth_stop.publish(depth_msg)           

            #self.get_logger().info(f"Depth from Stop Sign: {depth_from_stop_sign:.2f} centimeters")
        
        
    def detect_stop_sign(self, image):
        gray_stop = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur_stop = cv2.GaussianBlur(gray_stop, (5, 5), 0)
        edges_stop = cv2.Canny(blur_stop, 50, 150)
        stop_sign_detected = False
        centroid_x_stop = None
        centroid_y_stop = None
        
        # Find contours in the edge-detected image for stop sign
        contours, _ = cv2.findContours(edges_stop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Convert the frame to HSV color space for color detection
        hsv_stop = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range of red color in HSV space (stop sign is typically red)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Create a mask for red areas in the image
        mask1_stop = cv2.inRange(hsv_stop, lower_red1, upper_red1)
        mask2_stop = cv2.inRange(hsv_stop, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1_stop, mask2_stop)
        
        # Process each contour
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 40:  # Ignore small contours
                continue

            # Approximate the contour to a polygon
            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)

            # Check if the shape is an octagon (8 sides)
            if len(approx) >= 7 and len(approx) <= 8:
                # Check if the area of the detected octagon is within a region that has red color
                x, y, w, h = cv2.boundingRect(approx)
                roistop = red_mask[y:y+h, x:x+w]  # Region of interest from red mask
                red_area = cv2.countNonZero(roistop)  # Count non-zero pixels in the red mask

                # If the red area is sufficiently large, it's a stop sign
                if red_area > (w * h * 0.60):  # Threshold for the amount of red in the region
                    stop_sign_detected = True
                    # Draw the bounding box around the detected stop sign
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 3)
                    centroid_x_stop = x + w // 2
                    centroid_y_stop = y + h // 2
        return image, stop_sign_detected, centroid_x_stop, centroid_y_stop
               

def main(args=None):
    rclpy.init(args=args)
    stop_sign_detector = StopDetector()
    rclpy.spin(stop_sign_detector)
    stop_sign_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
