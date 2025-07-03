#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def preprocess_image(frame, mode=0):
    """
    Pre-processes the image using the same modes as the original script.
    The mode is now controlled by a ROS parameter.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if mode == 0:  # Adaptive Threshold (Default)
        rospy.logdebug("Using Preprocessing Mode 0: Adaptive Threshold")
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        processed = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                          cv2.THRESH_BINARY_INV, 11, 4)
    elif mode == 1:  # Simple Binary Threshold
        rospy.logdebug("Using Preprocessing Mode 1: Simple Binary")
        _, processed = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY_INV)
    elif mode == 2:  # Otsu's Binarization
        rospy.logdebug("Using Preprocessing Mode 2: Otsu's Binarization")
        _, processed = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    elif mode == 3:  # Dilation
        rospy.logdebug("Using Preprocessing Mode 3: Dilation")
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        dilated = cv2.dilate(gray, kernel, iterations=1)
        _, processed = cv2.threshold(dilated, 130, 255, cv2.THRESH_BINARY_INV)
    else:  # Just Grayscale
        rospy.logdebug("Using Preprocessing Mode 4: Grayscale")
        processed = gray

    return processed

class ImageCaptureNode:
    def __init__(self):
        rospy.init_node('image_capture_node', anonymous=True)
        rospy.loginfo("Image Capture Node with Full Preprocessing started.")

        # --- Get the preprocessing mode from ROS Param Server ---
        # The '~' makes it a private parameter.
        # It defaults to 0 if not set elsewhere.
        self.mode = rospy.get_param('~preprocessing_mode', 0)
        rospy.loginfo(f"Using preprocessing mode: {self.mode}")

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Error: Could not open webcam.")
            rospy.signal_shutdown("Could not open webcam.")
            return

        self.raw_image_pub = rospy.Publisher('/juno/camera/raw', Image, queue_size=10)
        self.processed_image_pub = rospy.Publisher('/juno/camera/preprocessed', Image, queue_size=10)

        self.rate = rospy.Rate(30)

    def run(self):
        """Captures, processes, and publishes frames."""
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame.")
                continue

            # 1. Preprocess the frame using the selected mode
            processed_frame = preprocess_image(frame, self.mode)

            try:
                # 2. Publish the original raw image
                raw_ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.raw_image_pub.publish(raw_ros_image)

                # 3. Publish the preprocessed image
                processed_ros_image = self.bridge.cv2_to_imgmsg(processed_frame, "mono8")
                self.processed_image_pub.publish(processed_ros_image)
            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")

            self.rate.sleep()

        self.cap.release()
        rospy.loginfo("Image Capture Node shutting down.")

if __name__ == '__main__':
    try:
        capture_node = ImageCaptureNode()
        capture_node.run()
    except rospy.ROSInterruptException:
        pass
