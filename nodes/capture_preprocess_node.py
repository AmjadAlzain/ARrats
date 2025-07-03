#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def preprocess_image(frame):
    """
    Pre-processes the image to improve OCR accuracy.
    This function is now part of the capture node.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # Using adaptive thresholding as the primary method
    processed = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv2.THRESH_BINARY_INV, 11, 4)
    return processed

class ImageCaptureNode:
    def __init__(self):
        rospy.init_node('image_capture_node', anonymous=True)
        rospy.loginfo("Image Capture Node with Preprocessing started.")

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Error: Could not open webcam.")
            rospy.signal_shutdown("Could not open webcam.")
            return

        # Publisher for the original, raw camera feed
        self.raw_image_pub = rospy.Publisher('/juno/camera/raw', Image, queue_size=10)
        
        # Publisher for the preprocessed (black and white) camera feed
        self.processed_image_pub = rospy.Publisher('/juno/camera/preprocessed', Image, queue_size=10)

        self.rate = rospy.Rate(30) # 30 Hz

    def run(self):
        """Captures, processes, and publishes frames."""
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame.")
                continue

            # 1. Preprocess the captured frame
            processed_frame = preprocess_image(frame)

            try:
                # 2. Publish the original raw image
                raw_ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.raw_image_pub.publish(raw_ros_image)

                # 3. Publish the preprocessed image
                # Note: The processed image is grayscale, so encoding is "mono8"
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
