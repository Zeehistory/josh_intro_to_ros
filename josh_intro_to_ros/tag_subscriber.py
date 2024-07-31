#!/usr/bin/env python3

import cv2
import matplotlib.pyplot as plt
from dt_apriltags import Detector
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from cv_bridge import CvBridge
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import numpy as np
import time

class TagSubscriber(Node):

    heading = None
    image = None
    at_detector = None

    def __init__(self):
        super().__init__("TagSubscriber")
        
        self.cvb = CvBridge()

        self.heading_subscriber = self.create_subscription(
            Int16,
            "bluerov2/heading",
            self.headingCallback,
            10
        )

        self.apriltag_subscriber = self.create_subscription(
            Image, 
            "bluerov2/camera", 
            self.apriltagCallback, 
            10
        )

        self.desired_heading_publisher = self.create_publisher(
            Int16,
            "bluerov2/desired_heading",
            10
        )

        self.at_detector = Detector(families='tag36h11',
                            nthreads=1,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)

    def headingCallback(self,msg):
        self.heading = msg.data
        

    def detect_april_tags(self,frame):
        """
        Detects AprilTags in the given frame and draws lines and IDs on the detected tags.

        Args:
            frame (numpy.ndarray): The input frame in grayscale format.

        Returns:
            numpy.ndarray: The frame with detected AprilTags drawn.
        """
        # Initialize the AprilTag detector
        tags = self.at_detector.detect(frame, estimate_tag_pose=True, camera_params=[1000,1000,frame.shape[1]/2,frame.shape[0]/2], tag_size=0.1)
        return tags

    def outline_tags(self,frame, tags): 
        color_frame = frame
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(color_frame, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

            cv2.putText(color_frame, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.8,
                        color=(0, 0, 255))
        return color_frame                         
        
    def calc_horiz_angle(self,img, tag): 
        "Calculating the Horizontal Angle - 80 is the Field of View (Horizontal)"
        x = tag.center[0]
        return 80 * (x-img.shape[1]/2)/img.shape[1]

    def calc_rel_angle(self,img, tag): 
        "Calculating the relative Angle - 64 is the Field of View (Vert)"
        y = tag.center[1]
        return 64*(y-img.shape[0]/2)/img.shape[0]

    def calc_dist(self,img, tag): 
        return np.linalg.norm(tag.pose_t)
        
    def apriltagCallback(self, msg):
        "generating image and identifying the april tag while also creating moving the robot to match heading with actual location of april tag"
        img = self.cvb.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gblur, gray = self.process_frame(img)
        tags = self.detect_april_tags(gray)
        color_frame = self.outline_tags(img,tags)
        
        for tag in tags:
            x_angle = self.calc_horiz_angle(img,tag)
            y_angle = self.calc_rel_angle(img,tag)
            z_distance = self.calc_dist(img,tag)
            self.get_logger().info(f"x Angle: {x_angle}, y Angle: {y_angle}, Distance: {z_distance}")
            desired_heading = x_angle + self.heading #or would this be + instead of - (x_angle +) 
            desired_heading = desired_heading % 360 

            self.desired_heading_publisher.publish(Int16(data=int(desired_heading)))
            self.get_logger().info(f"current heading: {self.heading}, desired heading: {desired_heading}")
        
        color_frame = self.outline_tags(img,tags)
        cv2.imwrite("tagframe.png",color_frame)
        time.sleep(1)

        if self.heading is None:
            self.get_logger().warning("Current heading is not available.")
            return
        

    def process_frame(self,frame):
        """
        Applies Gaussian blur and converts the frame to grayscale.

        Args:
            frame (numpy.ndarray): The input frame in BGR format.

        Returns:
            tuple: A tuple containing the Gaussian blurred frame and the grayscale frame.
        """
        gblur = cv2.GaussianBlur(frame, (5, 5), 0)
        gray = cv2.cvtColor(gblur, cv2.COLOR_BGR2GRAY)
        return gblur, gray

    def display_frame(self,frame, title='Frame'):
        """
        Displays the given frame using matplotlib.

        Args:
            frame (numpy.ndarray): The frame to be displayed.
            title (str): The title of the frame window.
        """
        plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        plt.title(title)
        plt.axis('off')
        plt.show()
        plt.pause(0.001)  # Pause to allow the image to be displayed

    def process_video_iterative(self,video_path):
        """
        Processes a video, displaying and detecting AprilTags every 1000 frames.
        """
        cap = cv2.VideoCapture(video_path)
        frame_count = 0

        while cap.isOpened():
            isTrue, frame = cap.read()
            if not isTrue:
                break
                
            frame_count += 1
                
            if frame_count % 400 == 0:
                gblur, gray = self.process_frame(frame)
                at_detection = self.detect_april_tags(gray)
                self.display_frame(at_detection)
                
            cap.release()
            plt.close()

def main(args=None):
    rclpy.init(args=args)

    node = TagSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()        