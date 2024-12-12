#!/usr/bin/env python3

import cv2
import matplotlib.pyplot as plt
from dt_apriltags import Detector
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from cv_bridge import CvBridge
from std_msgs.msg import Int16, Bool
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import numpy as np
import time

class TagSubscriber(Node):
    """
    Attributes:
        heading (int): Current heading of the robot.
        image (numpy.ndarray): Latest received image frame.
        at_detector (Detector): Instance of the AprilTag detector.

    """
    heading = None
    image = None
    at_detector = None

    def __init__(self):
        """
        Initializes the TagSubscriber node, setting up required subscriptions, publishers, 
        and the AprilTag detector.

        Subscriptions:
            - "bluerov2/heading": Current robot heading (Int16).
            - "bluerov2/camera": Image feed from the robot's camera (Image).

        Publishers:
            - "bluerov2/tag_desired_heading": Desired heading to align with detected tag (Int16).
            - "bluerov2/x": Forward movement control signal (Int16).
            - "bluerov2/lights_control": Control signal for activating lights near a tag (Bool).
        """
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
            "bluerov2/tag_desired_heading",
            10
        )

        self.forward_publisher = self.create_publisher(
            Int16,
            "bluerov2/x",
            10
        )

        self.lights_publisher = self.create_publisher(
            Bool,
            "bluerov2/lights_control",
            10
        )

        self.at_detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

    def headingCallback(self, msg):
        """
        Callback for the "bluerov2/heading" topic.
        Updates the current heading value.

        Args:
            msg (Int16): Message containing the current heading in degrees.
        """
        self.heading = msg.data

    def detect_april_tags(self, frame):
        """
        Detects AprilTags in the given frame and retrieves tag details.

        Args:
            frame (numpy.ndarray): Grayscale frame for tag detection.

        Returns:
            list: Detected tags with their properties, including corners and ID.
        """
        tags = self.at_detector.detect(
            frame, 
            estimate_tag_pose=True, 
            camera_params=[1000, 1000, frame.shape[1] / 2, frame.shape[0] / 2], 
            tag_size=0.1
        )
        return tags

    def outline_tags(self, frame, tags):
        """
        Draws outlines and IDs for detected AprilTags in the frame.

        Args:
            frame (numpy.ndarray): Original color frame.
            tags (list): List of detected tags.

        Returns:
            numpy.ndarray: Frame with outlined tags and IDs.
        """
        color_frame = frame
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(color_frame, 
                         tuple(tag.corners[idx - 1, :].astype(int)), 
                         tuple(tag.corners[idx, :].astype(int)), 
                         (0, 255, 0))
            cv2.putText(
                color_frame,
                str(tag.tag_id),
                org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255)
            )
        return color_frame

    def calc_horiz_angle(self, img, tag):
        """
        Calculates the horizontal angle of a detected tag.

        Args:
            img (numpy.ndarray): Original image.
            tag: Detected tag properties.

        Returns:
            float: Horizontal angle offset in degrees.
        """
        x = tag.center[0]
        return 80 * (x - img.shape[1] / 2) / img.shape[1]

    def calc_rel_angle(self, img, tag):
        """
        Calculates the vertical angle of a detected tag.

        Args:
            img (numpy.ndarray): Original image.
            tag: Detected tag properties.

        Returns:
            float: Vertical angle offset in degrees.
        """
        y = tag.center[1]
        return 64 * (y - img.shape[0] / 2) / img.shape[0]

    def calc_dist(self, img, tag):
        """
        Calculates the distance to a detected tag.

        Args:
            img (numpy.ndarray): Original image.
            tag: Detected tag properties.

        Returns:
            float: Euclidean distance to the tag.
        """
        return np.linalg.norm(tag.pose_t)

    def apriltagCallback(self, msg):
        """
        Callback for the "bluerov2/camera" topic. Processes an image, detects tags, calculates 
        control signals, and publishes them.

        Args:
            msg (Image): Image message containing the camera feed.
        """
        img = self.cvb.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gblur, gray = self.process_frame(img)
        tags = self.detect_april_tags(gray)
        color_frame = self.outline_tags(img, tags)

        if not tags:
            self.desired_heading_publisher.publish(Int16())
            return

        for tag in tags:
            x_angle = self.calc_horiz_angle(img, tag)
            y_angle = self.calc_rel_angle(img, tag)
            z_distance = self.calc_dist(img, tag)
            desired_heading = (x_angle + self.heading) % 360
            self.desired_heading_publisher.publish(Int16(data=int(desired_heading)))

            if z_distance < 3.0:
                self.lights_publisher.publish(Bool(data=True))

        cv2.imwrite("tagframe.png", color_frame)

    def process_frame(self, frame):
        """
        Applies Gaussian blur and converts a frame to grayscale.

        Args:
            frame (numpy.ndarray): Input frame.

        Returns:
            tuple: Gaussian blurred and grayscale frames.
        """
        gblur = cv2.GaussianBlur(frame, (5, 5), 0)
        gray = cv2.cvtColor(gblur, cv2.COLOR_BGR2GRAY)
        return gblur, gray

    def display_frame(self, frame, title='Frame'):
        """
        Displays a frame using matplotlib.

        Args:
            frame (numpy.ndarray): Frame to be displayed.
            title (str): Title of the display window.
        """
        plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        plt.title(title)
        plt.axis('off')
        plt.show()
        plt.pause(0.001)

    def process_video_iterative(self, video_path):
        """
        Processes a video file, detecting AprilTags at regular intervals.

        Args:
            video_path (str): Path to the video file.
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
