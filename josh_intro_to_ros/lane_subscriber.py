#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import numpy as np
import time

class LaneSubscriber(Node):

    image = None
    heading = None

    def __init__(self):
        '''
        Initiallizes a Lane Subscriber Node
        Subscribes to heading and camera
        publishes to desired heading
        '''
        super().__init__("LaneSubscriber")

        self.cvb = CvBridge()

        self.heading_subscriber = self.create_subscription(
            Int16,
            "bluerov2/heading",
            self.headingSubscriberCallback,
            10
        )

        self.image_subscriber = self.create_subscription(
            Image,
            "bluerov2/camera",
            self.imageCallback,
            10
        )

        self.desired_heading_publisher = self.create_publisher(
            Int16,
            "bluerov2/desired_heading",
            10
        )


        # ardusub testing
        self.image = cv2.imread("lanes.png")
        self.create_timer(1.0, self.rotation)

    def headingSubscriberCallback(self,msg):
        '''
        Heading Subscriber Callback
        Assigns message data to heading attribute

        msg: the message from the callback
        '''
        self.heading = msg.data
    
    def imageCallback(self,msg):
        '''
        Image subscriber callback
        converts image message to cv2 image
        calls rotation/recommendation function

        msg: the message from the callback
        '''
        image = self.cvb.imgmsg_to_cv2(msg, desired_encoding = "bgr8")
        self.get_logger().info("IMAGE RECEIVED")
        # image = image[320:640,0:480]
        # blur = cv2.GaussianBlur(image,(33,33),0)
        # kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        # sharpened = cv2.filter2D(image, -1, kernel)

        self.image = image
        self.rotation()
        # time.sleep(1)

    def rotation(self):
        '''
        calls functions to determine which way to rotate
        to follow the lane
        '''
        self.get_logger().info("IMAGING")
        if (self.heading == None):
            return
        img = self.image
        lines = self.detect_lines(img)
        if (lines is None):
            return
        self.get_logger().info(f"LINES: {len(lines)}")
        if (len(lines) == 0):
            return
        lanes = self.detect_lanes(lines)
        self.get_logger().info(f"LANES: {len(lanes)}")
        if (len(lanes) == 0):
            recommendation = self.recommendation(lines)
        else:
            middleLanes = self.get_lane_center(lanes)
            recommendation = self.recommendation(middleLanes)
        self.get_logger().info("STUFF")
        msg = Int16()
        msg.data = self.heading + int(recommendation)

        self.draw_lines(self.image,lines)
        self.draw_lanes(self.image,lanes)
        cv2.imwrite("testfile.png",self.image)

        self.desired_heading_publisher.publish(msg)

    def detect_lines(self,img, threshold1 = 150, threshold2 = 250, apertureSize = 3, minLineLength = 100, maxLineGap = 50):
        '''
        Detects lines in the image

        img: the Image to analyze
        threshold1: First threshold of cv2.Canny
        threshold2: Second threshold of cv2.Canny
        aperture: the aperture size for the computer to use
        minLineLength: the min line length allowed and added to lines array
        maxLineGap: the maximum line gap between lines in the picture

        returns lines: an array of coordinates of lines
        '''
        cv2.imwrite("afile.png",img)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, threshold1 = int(threshold1), threshold2 = int(threshold2), apertureSize = apertureSize) # detect edges
        cv2.imwrite("edges.png", edges)
        lines = None
        lines = cv2.HoughLinesP(
                        edges,
                        1,
                        np.pi/180,
                        50,
                        minLineLength = int(minLineLength),
                        maxLineGap = int(maxLineGap),
                ) # detect lines
        self.get_logger().info(f"LANES: {type(lines)}")
        return lines

    def draw_lines(self,img,lines, color = (255,0,0)):
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), color, 2)

    def get_slopes_intercepts(self,lines):
        '''
        Gets the slopes and intercepts from an array of lines

        lines: an array of lines with two coordinates to define each line

        returns:
        slopes: an array of the slopes of the respective lines
        intercepts: an array of the x-intercepts of the respective lines
        '''
        slopes = []
        intercepts = []
        for line in lines:
            x1,y1,x2,y2 = line[0]
            slope = (y2-y1)/(x2-x1)
            inverse_slope = (x2-x1)/(y2-y1)
            intercept = x1 - y1*inverse_slope
            slopes.append(slope)
            intercepts.append(intercept)
        return slopes, intercepts
        
    def detect_lanes(self,lines):
        '''
        detects the lanes of the pool
        Steps:
        1. Group lines based on the slope
        2. Create one line for each group by finding the min coordinate and the max coordinate
        3. Sort lines based on slope so that it goes from barely neg to abs neg to abs pos to barely pos
        4. Find lanes by looking at how close two lines are at the y-value of 2000 and determining which lines form which lanes

        lines: a list of the lines defined by two coordinates

        returns:
        lanes: a list of the lanes defined by two lines
        '''
        # group lines based on slope
        slopes,intercepts = self.get_slopes_intercepts(lines)
        linesGroup = []
        for i in range(len(lines)):
            if (i == 0):
                linesGroup.append([lines[i]])
            else:
                slope = slopes[i]
                added = False 
                for group in linesGroup:
                    x1,y1,x2,y2 = group[0][0]
                    slope2 = (y2-y1)/(x2-x1)
                    
                    if (np.abs(1.0-slope/slope2) < 0.05): # error within 5%
                        group.append(lines[i])
                        added = True
                if (not added):
                    linesGroup.append([lines[i]])
        
        # create singular line from a group of lines
        lessLines = []
        for group in linesGroup:
            x1,y1,x2,y2 = float('inf'),float('inf'),float('-inf'),float('-inf')
            for line in group:
                l = line[0]
                if (l[0] < x1 and l[1] < y1):
                    x1,y1 = l[0],l[1]
                if (l[2] > x2 and l[3] > y2):
                    x2,y2 = l[2],l[3]
            lessLines.append(np.array([[x1,y1,x2,y2]]))
        
        # sort based on slope
        lanes = []
        slopes, intercepts = self.get_slopes_intercepts(lessLines)
        slopesArgs = np.argsort(slopes)

        # need to change so sorted range goes from barely neg to max neg to max pos to barely pos
        i = 0
        while (i < len(slopesArgs)):
            if (slopes[slopesArgs[i]] > 0):
                slice = slopesArgs[0:i]
                slice = slice[::-1]
                slopesArgs[0:i] = slice
                break
            i += 1
        
        i = len(slopesArgs)-1
        while (i >= 0):
            if (slopes[slopesArgs[i]] < 0):
                slice = slopesArgs[i+1:len(slopesArgs)-1]
                slice = slice[::-1]
                slopesArgs[i+1:len(slopesArgs)-1] = slice
                break
            i -= 1
            
        sortedSlopes = []
        sortedLines = []
        for i in range(len(lessLines)):
            sortedLines.append(lessLines[slopesArgs[i]])
            sortedSlopes.append(slopes[slopesArgs[i]])

        # finding lanes
        slopes = sortedSlopes
        i = 1
        while i < len(sortedLines)-1:
            line = sortedLines[i]
            slope = slopes[i]
            x1,y1,x2,y2 = line[0]
            intercept = (2000-y1)/slope + x1

            x1,y1,x2,y2 = sortedLines[i-1][0]
            prevIntercept = (2000-y1)/slope + x1

            x1,y1,x2,y2 = sortedLines[i+1][0]
            nextIntercept = (2000-y1)/slope + x1
            if (np.abs(1.0-slope/slopes[i-1]) < np.abs(1.0-slope/slopes[i+1]) and np.abs(intercept-prevIntercept)<np.abs(intercept-nextIntercept) and np.abs(intercept-prevIntercept) < 2000):
                lanes.append([sortedLines[i-1],sortedLines[i]])
            elif (np.abs(1.0-slope/slopes[i-1]) >= np.abs(1.0-slope/slopes[i+1]) and np.abs(intercept-prevIntercept)>=np.abs(intercept-nextIntercept) and np.abs(intercept-nextIntercept) < 2000):
                lanes.append([sortedLines[i],sortedLines[i+1]])
            i+=1

        return lanes

    def get_lane_center(self,lanes):
        '''
        Finds the center of a lane by averaging the coordinates of the two lines

        lanes: the list of lanes defined by two lines

        returns:
        middleLanes: a list of lines that are the average of the two lines that define the lane
        '''
        middleLanes = []
        for lane in lanes:
            line1,line2 = lane[0], lane[1]
            x1, y1, x2, y2 = line1[0]
            line1x1,line1y1,line1x2,line1y2 = line1[0]
            line2x1,line2y1,line2x2,line2y2 = line2[0]
            # average points
            x1 = (line1x1 + line2x1)/2
            y1 = (line1y1 + line2y1)/2
            x2 = (line1x2 + line2x2)/2
            y2 = (line1y2 + line2y2)/2
            middleLanes.append(np.array([[x1,y1,x2,y2]]))
        return middleLanes
    
    def recommendation(self,middleLanes):
        '''
        creates a recommendation based on the closest lane

        middleLanes: the list of lines that form the middle of the lanes

        returns:
        desired heading change in degrees
        '''
        intercept = float('inf')
        slope = float('inf')
        for line in middleLanes:
            x1,y1,x2,y2 = line[0]
            laneSlope = (y2-y1)/(x2-x1)
            laneIntercept = (2000-y1)/slope + x1
            if (np.abs(laneIntercept-3824/2) < np.abs(intercept-3824/2)):
                intercept = laneIntercept
                slope = laneSlope
        self.get_logger().info(f"DEGREES CHANGE: {np.degrees(np.arctan(1.0/slope))}")
        return np.degrees(np.arctan(1.0/slope))
    
    def draw_lanes(self,img, lanes):
        middleLanes = []
        for lane in lanes:
            line1,line2 = lane[0], lane[1]
            x1, y1, x2, y2 = line1[0]
            line1x1,line1y1,line1x2,line1y2 = line1[0]
            line2x1,line2y1,line2x2,line2y2 = line2[0]
            # average points
            x1 = (line1x1 + line2x1)/2
            y1 = (line1y1 + line2y1)/2
            x2 = (line1x2 + line2x2)/2
            y2 = (line1y2 + line2y2)/2
            cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
            middleLanes.append(np.array([[x1,y1,x2,y2]]))
    
def main(args = None):
    rclpy.init(args = args)
    lane_subscriber = LaneSubscriber()

    try:
        rclpy.spin(lane_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        lane_subscriber.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()