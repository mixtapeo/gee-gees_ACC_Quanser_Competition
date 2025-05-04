#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from qcar2_interfaces.msg import MotorCommands
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage 
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
import subprocess

class LaneDetectionPIDController(Node):
    def __init__(self):
        super().__init__('lane_detection_pid')

        self.bridge = CvBridge()
        
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,  # Volatile (sensor data) history
            depth=10,  # Buffer depth for messages
            reliability=QoSReliabilityPolicy.RELIABLE  # Best Effort Reliability
        )
        
        self.initial_speed = True
        
        self.subscriber_depth = self.create_subscription(Image, 'camera/depth_image', self.calldepth, qos)
        self.subscriber_depth_yield= self.create_subscription(Image, 'camera/depth_image', self.calldepth_yield, qos)
        self.subscription = self.create_subscription(Image, '/camera/color_image', self.image_callback, qos)
        self.subscription_light = self.create_subscription(Image, '/camera/color_image', self.light_callback, qos)
        self.subscription_stop = self.create_subscription(Image, '/camera/color_image', self.stop_callback, qos)
        self.subscription_yield = self.create_subscription(Image, '/camera/color_image', self.yield_callback, qos)
        self.motor_pub = self.create_publisher(MotorCommands, '/qcar2_motor_speed_cmd', qos)
        self.stop_sign_publisher = self.create_publisher(Image, '/detected_stop_sign', qos)
        
        self.return_to_hub = False
        self.hub_eta = 0
        #self.car_pose = Pose()

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

        # Control parameters
        self.throttle = 0.23
        self.image_center_buffer = deque(maxlen=5)
        self.start_time = time.time()
        self.get_logger().info("Lane Detection with PID Initialized")
        
        self.stop_start_time = None
        self.stop_duration = 3.5
        self.stop_cooldown = 10
        self.last_stop_time = 0
        self.is_stopping = False
        self.stop_depth_margin = 0
        
        self.yield_start_time = None
        self.yield_duration = 3.5
        self.yield_cooldown = 15.0
        self.last_yield_time = 0.0 
        self.is_yielding = False
        self.done_yielding = False
      
        self.passed_first_light = False
        
        self.activate = False
        self.green_counter = 0
        self.red_counter = 0
        
        self.green_flag = False
        self.red_flag = False
        self.yellow_flag = False
        self.second_green_flag_checker = False
        
        self.yield_flagged = False
        self.stop_flagged = False
        self.yield_countdown = 0.0
        
        #dynamic pid values
        self.declare_parameter('kp', 0.30)
        self.declare_parameter('kd', 0.13)  #pd before stop
        self.declare_parameter('ki', 0.0)
        
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.ki = self.get_parameter('ki').value
        
        self.light_timer = False
        self.turn_right_once = False
        self.countdown = 0
        self.countdown_turn_right = 0
        self.active_time = 0
        self.light_delay = 0
        
        self.depths_yield = None
        self.depths = None
        
        self.passenger_picked = False
        self.passenger_dropped = False
        self.activate_passenger_pickup = False
        self.activate_passenger_dropoff = False
        self.countdown_for_pickup = 0
        self.countdown_for_dropoff = 0
        
        self.roundabout_deviation = False
        self.roundabout_timer = 0
        self.round_duration = 0
        
    def calldepth(self, depth_message):
        self.depths = Image()
        self.depths = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='16UC1')
    
    def calldepth_yield(self, depthmessage):
        self.depths_yield = Image()
        self.depths_yield = self.bridge.imgmsg_to_cv2(depthmessage, desired_encoding='16UC1')
    
    def set_state(self, state):
        """Set the current state of the traffic light if the change is valid."""
        if self.is_valid_transition(state):
            self.current_state = state
        else:
            raise ValueError(f"Invalid state transition from {self.current_state} to {state}.")
        
    def light_callback(self, lightmsg):
        frame_received = self.bridge.imgmsg_to_cv2(lightmsg, desired_encoding='bgr8')
        if frame_received is None:
            return
            
        #Look for traffic light.
        if self.passed_first_light == False:
            frameforlight = frame_received[0:400, 170:380]
        else:
            frameforlight = frame_received[0:400, 185:430]

        #Convert to HSV
        hsv_frame = cv2.cvtColor(frameforlight, cv2.COLOR_BGR2HSV)
        
        #Create masks to avoid random or noisy detections in the sky and ground.   
        red_mask_filter = cv2.inRange(hsv_frame, (0, 100, 100), (10, 255, 255)) + cv2.inRange(hsv_frame, (160, 100, 100), (180, 255, 255))
        yellow_mask_filter = cv2.inRange(hsv_frame, (20, 100, 100), (30, 255, 255))
        green_mask_filter = cv2.inRange(hsv_frame, (40, 100, 100), (70, 255, 255))

        #Combine all masks and mask original image.
        mask = red_mask_filter | yellow_mask_filter | green_mask_filter
        masked_frame = cv2.bitwise_and(frameforlight, frameforlight, mask=mask)
        heightl, widthl = masked_frame.shape[:2]
        
        lower_red1 = np.array([1, 180, 230])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 180, 230])
        upper_red2 = np.array([180, 255, 255])
        
        lower_yellow = np.array([21, 120, 245])
        upper_yellow = np.array([35, 190, 255])

        # --- GREEN ---
        lower_green = np.array([50, 150, 200])
        upper_green = np.array([70, 255, 255])
        
        mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask_red1, mask_red2)

        # YELLOW mask
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)

        # GREEN mask
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        
        #cv2.imshow("Red Mask", red_mask) 
        #cv2.imshow("Green Mask", green_mask) 
        #cv2.waitKey(1)
        
        if self.light_timer == True:
            if (time.time() - self.light_delay) >= 3:
                self.activate = True
            now = time.time()
            if not self.red_flag:
                self.active_time += now - self.countdown
            self.countdown = now
            
            #self.get_logger().info(f"Active time: {self.active_time:.2f} seconds")
                #self.light_timer = False
            
            if self.stop_depth_margin <= 7:
                required_delay = 11
            
            else:
                required_delay = 12
                
            if (self.active_time) >= 10.5: #10 or 10.5
                self.countdown_right = time.time()
                self.turn_right_once = True
                self.light_timer = False

        if self.activate == True:
            red_pixels = cv2.countNonZero(red_mask)
            yellow_pixels = cv2.countNonZero(yellow_mask)
            green_pixels = cv2.countNonZero(green_mask)
            
            if red_pixels >= 10:
                self.get_logger().info("Red Light is ON! Stopping car!")
                self.red_flag = True
                self.green_flag = False
                self.yellow_flag = False
                self.green_counter = 0

            elif yellow_pixels > 1000:
                self.get_logger().info("Yellow Light is ON! Slowing down!")
                self.yellow_flag = True
                self.red_flag = False
                self.green_flag = False
                self.green_counter = 0

            elif green_pixels >= 2:
                self.get_logger().info("Green Light is ON! Accelerating car!")
                if self.passed_first_light == False:
                    self.green_flag = True
                    self.red_flag = False
                    self.yellow_flag = False
                elif self.passed_first_light == True:
                    self.green_flag = True
                    self.red_flag = False
                    self.second_green_flag_checker = True
            
            if self.passed_first_light == True and self.activate == True:
                if self.second_green_flag_checker == True: 
                    self.activate = False
                
            if self.turn_right_once == True:
                self.get_logger().info(f"Turning right!")
                while (time.time() - self.countdown_right) <= 7: #5.5 or 6.5 or 6 7.5 for 5.83 cm stop depth
                    self.publish_motor_command(-0.35, 0.2) #-0.4
                self.kp = 0.37
                self.turn_right_once = False
                self.activate = False
                self.red_flag = False
                self.passed_first_light = True
                self.activate_passenger_pickup = True
                self.countdown_for_pickup = time.time()
                self.roundabout_timer = time.time()
                self.roundabout_deviation = True
                self.round_duration = time.time()                              
        
    def yield_callback(self, yieldmsg):
        frameforyield = self.bridge.imgmsg_to_cv2(yieldmsg, desired_encoding='bgr8')
        if frameforyield is None:
            return
        #Look for yield sign.
        if self.stop_flagged == True and (time.time() - self.yield_countdown >= 7):
            image_yield, yield_flag, yield_x, yield_y = self.detect_yield_sign(frameforyield.copy())
            if yield_flag == True:
                if self.time_to_yield(yield_x, yield_y):
                    self.yield_flagged = True
                    return
                        
            if self.is_yielding and (time.time() - self.yield_start_time) >= self.yield_duration:
                self.is_yielding = False
                self.last_yield_time = time.time()
                self.yield_flagged = False
        
    def stop_callback(self, stopmsg):
        frameforstop = self.bridge.imgmsg_to_cv2(stopmsg, desired_encoding='bgr8')
        if frameforstop is None:
            return
        #Look for stop sign.
        image_stop, stop_flag, stop_x, stop_y = self.detect_stop_sign(frameforstop.copy())
        cv2.circle(image_stop, (450, 470), 2, (255, 0, 0), -1)  # Green Circle\
        if stop_flag == True:
            self.stop_flagged = True
            self.yield_countdown = time.time()
            if self.time_to_stop(stop_x, stop_y):
                return
            
        if self.is_stopping:
            if time.time() - self.stop_start_time < self.stop_duration:
                self.publish_motor_command(0.0, 0.0)
                return
            else:
                self.is_stopping = False
                self.last_stop_time = time.time()
                turn_time = time.time() #left turn time
                self.is_turning_left=True
                #self.stop_flagged = False
                self.countdown = time.time()
                self.light_delay = time.time()
                self.kp = 0.54 #0.52 
                self.kd = 0.1 #pd after stop
                self.light_timer = True
                return            
    
    def detect_yellow_lane_and_curvature(self, image):
        #image = self.bridge.imgmsg_to_cv2(ima, desired_encoding='bgr8')
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Yellow color mask in HSV
        lower_yellow = np.array([18, 94, 140])
        upper_yellow = np.array([48, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        
        coords = np.column_stack(np.where(mask > 0))
        if coords.shape[0] < 100:
            return image, None, False, None

        ys = coords[:, 0]
        xs = coords[:, 1]
        
        roi_bottom = int(image.shape[0] * 0.7)
        maskpoints = ys > roi_bottom
        
        ys_filtered = ys[maskpoints]
        xs_filtered = xs[maskpoints]

        poly_coeffs = np.polyfit(ys, xs, 2)

        y_vals = np.linspace(np.min(ys), np.max(ys), num=100)
        x_vals = np.polyval(poly_coeffs, y_vals)
        
        x_center = int(np.mean(x_vals))

        for x, y in zip(x_vals.astype(int), y_vals.astype(int)):
            if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                cv2.circle(image, (x, y), 2, (0, 255, 255), -1)

        A = poly_coeffs[0]
        B = poly_coeffs[1]
        y_eval = np.max(ys)
        curvature = ((1 + (2 * A * y_eval + B) ** 2) ** 1.5) / np.abs(2 * A)
        
        aa, bb, cc = poly_coeffs
        y_near = image.shape[0] - 20  # Pick a point near the bottom of the image
        x_near = (aa * (y_near ** 2) + bb * y_near + cc) #+ 120 # was 100 before
        
        image_center_x = image.shape[1] // 2  # center of the image
        yellow_on_left = x_near < image_center_x
        
        cv2.circle(image, (int(x_near), int(y_near)), 5, (0, 0, 255), -1)
        cv2.line(image, (image_center_x, 0), (image_center_x, image.shape[0]), (255, 255, 0), 1)

        #cv2.putText(image, f"Yellow Lane Radius: {int(curvature_radius)} px", (50, 50),
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


        return image, curvature, yellow_on_left, x_center
    
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
        
    def detect_yield_sign(self, imageforyield):
        #gray_yield = cv2.cvtColor(imageforyield, cv2.COLOR_BGR2GRAY)
        #blur_yield = cv2.GaussianBlur(gray_yield, (5, 5), 0)
        #edges_yield = cv2.Canny(blur_yield, 100, 250)
        yield_sign_detected = False
        centroid_x_yield = None
        centroid_y_yield = None
        
        # Convert the frame to HSV color space for color detection
        hsv_yield = cv2.cvtColor(imageforyield, cv2.COLOR_BGR2HSV)
        
        lower_red1_y = np.array([0, 120, 70])  # Lower bound of red (light red)
        upper_red1_y = np.array([10, 255, 255])  # Upper bound of red (light red)

        lower_red2_y = np.array([170, 120, 70])  # Lower bound of red (dark red)
        upper_red2_y = np.array([180, 255, 255])  # Upper bound of red (dark red)

        # Define the HSV range for white (inner triangle) of the yield sign
        lower_white_y = np.array([0, 0, 200])  # Lower bound of white (high brightness)
        upper_white_y = np.array([180, 25, 255])  # Upper bound of white (low saturation)
        
        # -----------
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Create a mask for red areas in the image
        #mask1_stop = cv2.inRange(hsv_stop, lower_red1, upper_red1)
        #mask2_stop = cv2.inRange(hsv_stop, lower_red2, upper_red2)
        #red_mask = cv2.bitwise_or(mask1_stop, mask2_stop)

        # Create masks for red and white areas
        mask1_y = cv2.inRange(hsv_yield, lower_red1, upper_red1)
        mask2_y = cv2.inRange(hsv_yield, lower_red2, upper_red2)
        red_mask_y = cv2.bitwise_or(mask1_y, mask2_y)

        # Create mask for white areas (inner triangle of yield sign)
        white_mask_y = cv2.inRange(hsv_yield, lower_white_y, upper_white_y)

        # Combine the red and white masks
        combined_mask_y = cv2.bitwise_or(red_mask_y, white_mask_y)
        # Apply morphological closing to bridge gaps between red and white regions
        kernelw = np.ones((2, 2), np.uint8)
        combined_mask_y = cv2.morphologyEx(combined_mask_y, cv2.MORPH_CLOSE, kernelw)

        # Find contours in the edge-detected image for stop sign
        contours_yield, _ = cv2.findContours(red_mask_y, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #cv2.imshow("Combined Mask", combined_mask_y)
        
        # Process each contour
        for cnt_y in contours_yield:
            area_y = cv2.contourArea(cnt_y)
            if area_y < 10:  # Ignore small contours
                continue

            # Approximate the contour to a polygon
            approx_y = cv2.approxPolyDP(cnt_y, 0.04 * cv2.arcLength(cnt_y, True), True)

            # Check if the shape is an octagon (8 sides)
            if len(approx_y) >= 2 and len(approx_y) <= 3:
                # Check if the area of the detected octagon is within a region that has red color
                xy, yy, wy, hy = cv2.boundingRect(approx_y)
                roiy = red_mask_y[yy:yy+hy, xy:xy+wy]  # Region of interest from red mask
                red_area_y = cv2.countNonZero(roiy)  # Count non-zero pixels in the red mask

                # If the red area is moderate, it's a stop sign
                if red_area_y > (wy * hy * 0.4):  # Threshold for the amount of red in the region
                    yield_sign_detected = True
                    # Draw the bounding box around the detected stop sign
                    cv2.rectangle(imageforyield, (xy, yy), (xy + wy, yy + hy), (0, 255, 0), 3)
                    centroid_x_yield = xy + wy // 2
                    centroid_y_yield = yy + hy // 2
        return imageforyield, yield_sign_detected, centroid_x_yield, centroid_y_yield
        
    def detect_round_sign(self, imageround):
        hsv_round = cv2.cvtColor(imageround, cv2.COLOR_BGR2HSV)

        # Define yellow color range in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        # Create mask for yellow color
        yellow_mask_round= cv2.inRange(hsv_round, lower_yellow, upper_yellow)

        # Apply some blur to smooth out edges
        blurred_round = cv2.GaussianBlur(yellow_mask_round, (5, 5), 0)

        # Find edges using Canny
        edges_round = cv2.Canny(blurred_round, 50, 150)

        # Find contours
        contours_round, _ = cv2.findContours(edges_round, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
    def time_to_stop(self, stop_x, stop_y):
        if self.depths is None:
            return False

        depth_from_stop_sign = self.depths[stop_y, stop_x] / 1000.0   # convert mm to centimeters
        self.get_logger().info(f"Depth from Stop Sign: {depth_from_stop_sign:.2f} centimeters")

        if depth_from_stop_sign <= 7.7 and depth_from_stop_sign >= 2 and not self.is_stopping and (time.time() - self.last_stop_time) > self.stop_cooldown: #2 < x < 7.7
            self.stop_depth_margin = depth_from_stop_sign
            self.get_logger().info("Stop sign detected within range. Stopping.")
            self.publish_motor_command(0.0, 0.0)
            #subprocess.run(["ros2", "param", "set", "/qcar2_hardware", "led_color_id", "1"])
            self.stop_start_time = time.time()
            self.is_stopping = True
            return True
            
        return False
        
    def time_to_yield(self, yield_x, yield_y):
        if self.depths_yield is None:
            return False

        depth_from_yield_sign = self.depths_yield[yield_y, yield_x] / 1000.0  # convert mm to cm
        if depth_from_yield_sign <= 3.8 and depth_from_yield_sign >= 0 and not self.is_yielding and (time.time() - self.last_yield_time) > self.yield_cooldown:
            if self.done_yielding == False:
                self.get_logger().info("Yield Sign Detected! No cars approaching!")
            self.yield_start_time = time.time()
            self.done_yielding = True
            self.is_yielding = True
            return True
            
        return False

    def image_callback(self, msg):
        try:
            steering_angle = 0.28
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame is None or frame.size == 0:
                #self.get_logger().warn("Received an empty frame!")
                return
            
            height, width, _ = frame.shape
            roi = frame[int(height / 2):, :]
            
            yellow_viz, radius, left_flag, xnear = self.detect_yellow_lane_and_curvature(roi.copy())
            if xnear is not None:
                cv2.circle(yellow_viz, (xnear, yellow_viz.shape[0] // 2), 5, (255, 0, 0), -1)
            #cv2.imshow("Yellow Lane Detection", yellow_viz)
            #cv2.waitKey(1)

            # Preprocessing
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 50, 150)
        
            _, asphalt_mask = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
            hsv_white = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower_asphalt = np.array([0, 0, 0])
            upper_asphalt = np.array([180, 80, 100])
            asphalt_mask = cv2.inRange(hsv_white, lower_asphalt, upper_asphalt)
            
            lower_white = np.array([0, 0, 180])      # Allow slightly darker whites
            upper_white = np.array([180, 50, 255])   # Allow more saturation

            white_mask = cv2.inRange(hsv_white, lower_white, upper_white)
            
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            dilated_asphalt = cv2.dilate(asphalt_mask, kernel, iterations=2)
            
            white_on_asphalt = cv2.bitwise_and(white_mask, dilated_asphalt)
            
            hsvy = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower_yelloww = np.array([18, 94, 140])
            upper_yelloww = np.array([50, 255, 255])
            masky = cv2.inRange(hsvy, lower_yelloww, upper_yelloww)
            
            yellow_edges = cv2.Canny(masky, 50, 150)

            # Hough Line Detection
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=20, maxLineGap=120)
           
            #yellowlines = cv2.HoughLinesP(yellow_viz, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=100)
            white_lines = cv2.HoughLinesP(white_on_asphalt, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=100) #100 before
            
            frame_width = edges.shape[1]
            min_x_frame = int(frame_width * 0.30)  # 30% from left
            max_x_frame = int(frame_width * 0.91)  # 100% from left

            left_lines = []
            right_lines = []
            
            def average_x(lines):
                if not lines:
                    return None
                xs = [x1 for x1, _, x2, _ in lines] + [x2 for _, _, x2, _ in lines]
                return int(np.mean(xs))
            
            if radius is None: #If detection of yellow lanes by curvature method doesn't exist.
                if lines is not None:
                    for line in lines:
                        x1, y1, x2, y2 = line[0]
                        if None in [x1, y1, x2, y2]:
                            continue
                        if not (min_x_frame < x1 < max_x_frame and min_x_frame < x2 < max_x_frame):
                            continue
                        if x2 == x1:
                            continue
                        slope = (y2 - y1) / (x2 - x1)
                        if abs(slope) < 0.20:  # filter almost horizontal lines
                            continue
                        if slope < 0 and x1 < int(frame_width * 0.5):
                            left_lines.append((x1, y1, x2, y2))
                
                        #else:
                        #    right_lines.append((x1, y1, x2, y2)) #right lanes can't be yellow
            
            if white_lines is not None: #find white lanes
                for line in white_lines:
                    x1w, y1w, x2w, y2w = line[0]
                    if None in [x1w, y1w, x2w, y2w]:
                        continue
                    line_length = np.sqrt((x2w - x1w)**2 + (y2w - y1w)**2)
                    angle = np.degrees(np.arctan2(y2w - y1w, x2w - x1w))
                        #self.get_logger().info(f"angle ={angle}")
                    if not (min_x_frame < x1w < max_x_frame and min_x_frame < x2w < max_x_frame):
                        continue
                    if x2w == x1w:
                        continue
                    if line_length < 100:
                        continue
                    slope = (y2w - y1w) / (x2w - x1w)
                    if abs(slope) < 0.2:
                        continue
                        #if x1w < int(frame_width * 0.5) and x2w < int(frame_width * 0.5):
                    #if slope < 0:
                        #left_lines.append((x1w, y1w, x2w, y2w))
                        #if x1w > int(frame_width * 0.5) and x2w > int(frame_width * 0.5):
                    if slope > 0:
                        right_lines.append((x1w, y1w, x2w, y2w))
            
            if radius is None:
                left_x = average_x(left_lines)
                right_x = average_x(right_lines)

                if left_x is not None and right_x is not None:
                    lane_center = (left_x + right_x) // 2
                elif left_x is not None:
                    lane_center = left_x + 150
                elif right_x is not None:
                    lane_center = right_x - 150
                else:
                    #self.get_logger().warn("No lane lines detected.")
                    lane_center = None
                        
            elif radius is not None:
                right_x = average_x(right_lines)
                left_x = xnear if xnear is not None else None #get the x pixel coordinate from curvature function.
                
                if self.roundabout_deviation == True:
                    if (time.time() - self.roundabout_timer) >= 6: #5 or 5.5
                        left_x = left_x + 300
                        self.get_logger().info("Adjusting for Roundabout.")
                    if (time.time() - self.round_duration) >= 6: #8
                        self.roundabout_deviation = False
                        self.kp = 0.35
                        self.get_logger().info("Adjusting Finished.")
                
                if left_x is not None and right_x is not None:
                    lane_center = (left_x + right_x) // 2
                elif left_x is not None:
                    lane_center = left_x + 150
                elif right_x is not None and self.is_stopping == True:
                    lane_center = right_x - 150
                else:
                    #self.get_logger().warn("No lane lines detected.")
                    lane_center = None
               
            vis_image = roi.copy()
            
            if radius is None:
                if left_lines:
                    for x1, y1, x2, y2 in left_lines:
                        cv2.line(vis_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

            if right_lines:
                for x1, y1, x2, y2 in right_lines:
                    cv2.line(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if lane_center is not None:
                    #cv2.line(vis_image, (lane_center, 0), (lane_center, vis_image.shape[0]), (255, 0, 0), 2)
                cv2.circle(vis_image, (lane_center, vis_image.shape[0] // 2), 5, (255, 0, 0), -1)
                #cv2.waitKey(1)
                
                #cv2.imshow("Lane Detection", vis_image)
                #cv2.waitKey(1)
            # Smooth lane center using a buffer
                self.image_center_buffer.append(lane_center)
                if len(self.image_center_buffer) < self.image_center_buffer.maxlen:
                    self.get_logger().info("Warming up lane center buffer...")
                    return
                smoothed_center = int(np.mean(self.image_center_buffer))

                # PID control
                image_center = width // 2
                error = smoothed_center - image_center
                abs_error = abs(error)

                #PID Parameters
                # Live PID parameters
                kp = self.get_parameter('kp').get_parameter_value().double_value
                kd = self.get_parameter('kd').get_parameter_value().double_value
                ki = self.get_parameter('ki').get_parameter_value().double_value

                #self.integral += error * dt

                #steering_angle = -(kp * error + kd * derivative + ki * self.integral) / (width / 2)
                #steering_angle = max(min(steering_angle, 1.0), -1.0)
            

                current_time = time.time()
                dt = current_time - self.prev_time
                self.prev_time = current_time
                #self.get_logger().info(f"error={error}")

                self.integral += error * dt
                derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
                    
                self.integral = max(min(self.integral, 100), -100)
                derivative = max(min(derivative, 100), -100)
                    
                self.prev_error = error
                
                #self.get_logger().info(f"Using special PD: kp={self.kp}, kd={self.kd}")
                steering_angle = -(self.kp * error + self.ki * self.integral + self.kd * derivative) / (width / 2)
                steering_angle = max(min(steering_angle, 1.0), -1.0)
                #self.get_logger().info(f"Throttle: {self.throttle:.2f}, Error: {error:.2f}, Steering: {steering_angle:.2f}, kp={self.kp:.2f}, kd={self.kd:.2f}")
                if time.time() - self.start_time < 2.0:
                    steering_angle = 0.0
                    
            if self.activate_passenger_pickup == True:
                if ( time.time() - self.countdown_for_pickup ) >= 15: #15
                    subprocess.run(["ros2", "param", "set", "/qcar2_hardware", "led_color_id", "2"])
                    self.get_logger().info("Picking up passenger!")
                    self.publish_motor_command(0.0, 0.0)
                    self.kp = 0.3
                    time.sleep(3.3)
                    self.activate_passenger_pickup = False
                    self.countdown_for_dropoff = time.time()
                    self.activate_passenger_dropoff = True
                    
            if self.activate_passenger_dropoff == True:
                if ( time.time() - self.countdown_for_dropoff ) >= 28: #27
                    subprocess.run(["ros2", "param", "set", "/qcar2_hardware", "led_color_id", "1"])
                    self.get_logger().info("Dropping off passenger!")
                    self.publish_motor_command(0.0, 0.0)
                    time.sleep(3.3)
                    self.kp = 0.5
                    self.activate_passenger_dropoff = False
                    self.activate = True
                    
            if self.return_to_hub == True:
                self.get_logger().info("Returning to Hub! ETA: 47 seconds")
                #self.get_logger().info(f"ETA Counter: {time.time() - self.hub_eta}")
                if (time.time() - self.hub_eta) >= 47:
                    acc = time.time()
                    j=1
                    while (time.time() - acc) <= 2.7:
                        self.throttle = 0.85
                        steering_angle=-0.05
                        self.publish_motor_command(steering_angle, self.throttle)
                    while j==1:
                        self.throttle = 0.0
                        steering_angle = 0.0
                        self.publish_motor_command(steering_angle, self.throttle) 
                        self.get_logger().info("Trip Over!")
                        self.return_to_hub = False
                    
            if self.red_flag == True: #and self.stop_flagged == False: 
                self.throttle = 0.0
            else:
                if self.second_green_flag_checker == True:
                    straight_time = time.time()
                    self.kp = 0.35
                    while (time.time() - straight_time) <= 3.5:
                        self.throttle = 0.85
                        steering_angle= -0.04
                        self.publish_motor_command(steering_angle, self.throttle)
                    left_time = time.time()
                    while (time.time() - left_time) <= 3:
                        self.throttle = 0.23
                        steering_angle= 0.15
                        self.publish_motor_command(steering_angle, self.throttle)
                    self.return_to_hub = True
                    self.hub_eta = time.time()
                    self.second_green_flag_checker = False
                else:
                    self.throttle = 0.23
            self.publish_motor_command(steering_angle, self.throttle)
                #self.get_logger().info(f"Throttle: {self.throttle:.2f}, Steering: {steering_angle:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error in image processing: {e}")

    def publish_motor_command(self, throttle, steering_angle):
        msg = MotorCommands()
        msg.motor_names = ['steering_angle', 'motor_throttle']
        msg.values = [throttle, steering_angle]
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
