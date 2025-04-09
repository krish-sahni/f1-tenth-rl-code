#!/usr/bin/env python3

import time
import math
import numpy as np
import cv2
import rospy
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path



class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()

        # Uncomment this line for lane detection of videos in rosbag and irl car
        self.sub_image = rospy.Subscriber('/D435I/color/image_raw', Image, self.img_callback, queue_size=1)

        # Debugging rostopic
        # To view cropped image
        self.pub_image_cropped = rospy.Publisher("lane_detection/raw_cropped", Image, queue_size=1)
        # To view image in hsv
        self.pub_image_hsv = rospy.Publisher("lane_detection/hsv", Image, queue_size=1)
        # To view img in colour mask + edge detection
        self.pub_debug_combined = rospy.Publisher("lane_detection/combined_image", Image, queue_size=1)
        # To view img in edge detection
        self.pub_debug_gray = rospy.Publisher("lane_detection/gray", Image, queue_size=1)
        # To view img in color mask
        self.pub_debug_color = rospy.Publisher("lane_detection/color", Image, queue_size=1)
        # To view img in perspective transform
        self.pub_debug_warped = rospy.Publisher("lane_detection/warped", Image, queue_size=1)
        # To view waypoints 
        self.pub_debug_waypoints = rospy.Publisher("lane_detection/waypoints", Image, queue_size=1)

        # Publishing waypoints
        self.pub_waypoints_birdseye = rospy.Publisher("waypoints/birdseye", Path, queue_size=1)
        # Publishing errors
        self.error_pub = rospy.Publisher('waypoints/error', Float32, queue_size=10)

        # To store previous waypoints
        self.previouswaypoints = None
        self.previouserror = None
        self.error = None

        # parameters for error generation
        self.lookaheaddist = 1
        self.offset = 25
        self.minpix = 20
        self.maxpix = 55
        self.midpoint = 160
        self.publisherror = True # use to publish error
        self.publishwaypoints = False # use to publish waypoints
        self.displaywaypoints = False # use to display waypoints in rqt

        # To store previous waypoints
        self.previouswaypoints = None
        self.previouserror = None
        self.error = None
        # Parameters for Colour masking
        self.saturation_high = 255
        self.saturation_low = 80
        self.hue_high = 100
        self.hue_low = 50
        self.lightness_high = 255
        self.lightness_low = 0

    
    # Call back funtion to get data and trigger lane detection
    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        raw_img = cv_image.copy()


        # Resizing
        cropped_image = cv2.resize(raw_img, (320, 240))  # x,y
        # Cropping
        # cropped_image = cropped_image[160:, :320,:]
        cropped_image = cropped_image[150:, :320,:]

        crop_img_msg = self.bridge.cv2_to_imgmsg(cropped_image, 'bgr8')
        self.pub_image_cropped.publish(crop_img_msg)


        # To use uncropped image ( uncomment for auto drive)
        # cropped_image = cv2.resize(raw_img, (640, 480))
        # self.detection(raw_img)

        # To use cropped image ( uncomment for irl )
        self.detection(cropped_image)



################################################### F1tenth irl car Settings ##################################################333

    # Edge detection 
    def gradient_thresh(self, img, thresh_min, thresh_max):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        #2. Gaussian blur the image
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        #4. Use cv2.addWeighted() to combine the results
        #5. Convert each pixel to unint8, then apply threshold to get binary image

        ## TODO
        # Converting img to gray scale
        grayscale_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Gaussian Bluring
        sigmaX = 5
        shapeOfTheKernel = (5,5)
        gaussianblured_img = cv2.GaussianBlur(grayscale_img, shapeOfTheKernel, sigmaX )
        
        # Taking derievatives
        scale = 1 # To scale the gradiant after derivating, default = 1
        delta = 0 # To offset the gradiant after scaling, default - 0
        ddepth = cv2.CV_16S # 16 bit signed int
        kernel_size = 3   # Default is 3?

        grad_x = cv2.Sobel(gaussianblured_img, ddepth, 1, 0, ksize=kernel_size, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)
        # Gradient-Y
        # grad_y = cv.Scharr(gray,ddepth,0,1)
        grad_y = cv2.Sobel(gaussianblured_img, ddepth, 0, 1, ksize=kernel_size, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)

        # Combining Results
        alpha = 1 # Weight of derivative against x, edge for y axis
        beta = 0 # Weight of derivative against y, edge for x axis
        gamma = 0 # Scaling factor of the result image 
        result_img = cv2.addWeighted(grad_x, alpha, grad_y, beta, gamma)

        # Converting each pixel to unsigned int and applying threshold
        # ddepth = cv2.CV_8U # 16 bit signed int
        img_8bit_unsigned = cv2.convertScaleAbs(result_img)

        _, binary_output = cv2.threshold(img_8bit_unsigned, thresh_min, thresh_max, cv2.THRESH_BINARY)
        
        return binary_output


    # Color masking
    def color_thresh(self, img, thresh):
        """
        Convert RGB to HSL and apply thresholding on the S channel to create a binary image.
        
        Parameters:
        - img: Input RGB image
        - thresh: Tuple of min and max threshold values for the S channel
        
        Returns:
        - binary_output: Binary image after applying threshold on the S channel
        """
        # 1. Convert the image from RGB to HSL
        hsl_img = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        hsl_img_msg = self.bridge.cv2_to_imgmsg(hsl_img, 'bgr8')
        self.pub_image_hsv.publish(hsl_img_msg)
        
        # 2. Extract the H, L, and S channels
        H = hsl_img[:, :, 0]  # Hue channel
        L = hsl_img[:, :, 1]  # Lightness channel
        S = hsl_img[:, :, 2]  # Saturation channel

        # 4. Apply the threshold on the S channel
        binary_output = np.zeros_like(S)
        binary_output[(S >= thresh[0]) & (S <= thresh[1]) & (H >= self.hue_low) & (H <= self.hue_high) & (L >= self.lightness_low) & (L <= self.lightness_high)] = 1

        return binary_output
 
    # combining edge and color masking
    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        ## TODO
        SobelOutput = self.gradient_thresh(img,25,100)
        # print(SobelOutput)
        self.pub_debug_gray.publish(self.bridge.cv2_to_imgmsg(SobelOutput, 'mono8'))

        ColorOutput = self.color_thresh(img,(80,255))
        color_output_converted = (ColorOutput.astype(np.uint8)) * 255
        self.pub_debug_color.publish(self.bridge.cv2_to_imgmsg(color_output_converted, 'mono8'))
        binaryImage = np.zeros_like(SobelOutput)

        # use this to switch between using edge detection or not
        binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1 # use for only color
        # binaryImage[(ColorOutput==1)|(SobelOutput>0.5)] = 1 # use for edge + color
        # Remove noise from binary image
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)
        binary_img_uint8 = (binaryImage.astype(np.uint8)) * 255
        self.pub_debug_combined.publish(self.bridge.cv2_to_imgmsg(binary_img_uint8, 'mono8'))

        return binaryImage

    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        ## TODO
        img = (img.astype(np.uint8)) * 255
        
        # Source points
        # used in rosbags and irl car
        roi_points = np.array([[10,3],[310,3],[1,79],[319,79]]) # new points after cropping
        src_pts = np.float32([roi_points[0], roi_points[2], roi_points[1], roi_points[3]])

        # Destination points
        # dst_pts = np.float32([[0,0], [0, 720], [1280, 0], [1280, 720]]) # 1280 x 720 res
        # dst_pts = np.float32([[0,0], [0, 480], [640, 0], [640, 480]]) # 640 x 480 res
        dst_pts = np.float32([[0,0], [0, 240], [320, 0], [320, 240]]) # 320 x 240 res

        M_inv = cv2.getPerspectiveTransform(dst_pts, src_pts)
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        # warped = cv2.warpPerspective(img, M, (1280,720))
        # warped = cv2.warpPerspective(img, M, (640,480))
        warped = cv2.warpPerspective(img, M, (320,240))    
        ####

        return warped, M, M_inv
    


####################################### Common #################################3333


    # To find waypoints
    def line_fit(self,binary_warped):
        """
        Find and fit lane lines
        """
        # histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        
        # Finding histogram, that is the lane 
        histogram = np.sum(binary_warped, axis=0)
        # Highest pixel values along axis
        centerx_base = np.argmax(histogram)
        # print(centerx_base)

        # Getting all non zero values of the x,y coordinates
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Getting waypoints
        waypoints = []
        error_line = []
        minpix = self.minpix
        maxpix = self.maxpix

        # Iterate through all the unique row indices (y-values)
        for row in np.unique(nonzeroy):
            # Count how many x values are associated with the current row (y-value)
            count = np.sum(nonzeroy == row)
            # Store the count in the dictionary with the row as the key
            if minpix < count < maxpix:
                x_coords_in_row = nonzerox[nonzeroy == row]
                closest_x = x_coords_in_row[np.argmin(np.abs(x_coords_in_row - centerx_base))]
                if(np.abs(centerx_base - closest_x) <= 100 ): # and row > 20:
                    # print(x_coords_in_row[len(x_coords_in_row) // 2])
                    # waypoints.append((closest_x,row))
                    waypoints.append((x_coords_in_row[len(x_coords_in_row) // 2],row))
                    error_line.append((self.midpoint,row))
       

        out_img = (np.dstack((binary_warped, binary_warped, binary_warped))*255).astype('uint8')
        
        if(len(waypoints) > 25) : #and waypoints[-1][1] > 150:
            return waypoints, error_line ,out_img
        
        return None, error_line, out_img


    def detection(self, img):

        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)
        self.pub_debug_warped.publish(self.bridge.cv2_to_imgmsg(img_birdeye, 'mono8'))
        waypoints, error_line, waypoints_img = self.line_fit(img_birdeye)
        

        if waypoints != None:
            
            length = len(waypoints)
            point = length - round( length * self.lookaheaddist)
            detected_point_x = waypoints[point][0]
            self.error = self.midpoint - detected_point_x + self.offset
            if self.publisherror:
                self.error_pub.publish(Float32(self.error))
            print(f"lane detected, error = {self.error}")
            self.previouswaypoints = waypoints
            if self.displaywaypoints:
                detected_point = waypoints[point]
                offset_point = (self.midpoint + self.offset, waypoints[point][1])
                cv2.circle(waypoints_img, detected_point, radius=2, color=(0, 0, 255), thickness=-1)
                cv2.circle(waypoints_img, offset_point, radius=2, color=(0, 255, 0), thickness=-1)

                # for point in waypoints:
                #     cv2.circle(waypoints_img, point, radius=2, color=(0, 0, 255), thickness=-1)  # Red color
                # for point in error_line:
                #     cv2.circle(waypoints_img, point, radius=2, color=(0, 255, 0), thickness=-1)  # Green color

            if self.publishwaypoints:
                path_msg_birdseye = Path()
                for wp in waypoints:
                    # print(wp[0])
                    pose = PoseStamped()
                    pose.pose.position.x = wp[0]
                    pose.pose.position.y = wp[1]
                    pose.pose.position.z = 0
                    path_msg_birdseye.poses.append(pose)

                self.pub_waypoints_birdseye.publish(path_msg_birdseye)
        
        elif self.previouswaypoints != None:
            
            length = len(self.previouswaypoints)
            point = length - round( length * self.lookaheaddist)
            detected_point_x = self.previouswaypoints[point][0]
            self.error = self.midpoint - detected_point_x + self.offset
            if self.publisherror:
                self.error_pub.publish(Float32(self.error))
            print(f" Unable to detect lanes, prev error = {self.error}")

            if self.displaywaypoints:
                detected_point = self.previouswaypoints[point]
                offset_point = (self.midpoint + self.offset, waypoints[point][1])
                cv2.circle(waypoints_img, detected_point, radius=2, color=(0, 0, 255), thickness=-1)
                cv2.circle(waypoints_img, offset_point, radius=2, color=(0, 255, 0), thickness=-1)
                # for point in self.previouswaypoints:
                #     cv2.circle(waypoints_img, point, radius=2, color=(0, 0, 255), thickness=-1)  # Red color
                # for point in error_line:
                #     cv2.circle(waypoints_img, point, radius=2, color=(0, 255, 0), thickness=-1)  # Green color

            if self.publishwaypoints:
                path_msg_birdseye = Path()
                for wp in self.previouswaypoints:
                    # print(wp[0])
                    pose = PoseStamped()
                    pose.pose.position.x = wp[0]
                    pose.pose.position.y = wp[1]
                    pose.pose.position.z = 0
                    path_msg_birdseye.poses.append(pose)
                self.pub_waypoints_birdseye.publish(path_msg_birdseye)

        
        else:
            if self.publisherror:
                self.error_pub.publish(Float32(float('inf')))
            print(" Unable to detect lanes")

        if self.displaywaypoints:
            self.pub_debug_waypoints.publish(self.bridge.cv2_to_imgmsg(waypoints_img, 'bgr8'))

        return None


if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
