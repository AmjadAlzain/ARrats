#!/usr/bin/env python

import rospy
import cv2
import pytesseract
from PIL import Image
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class OcrNode:
    def __init__(self):
        rospy.init_node('ocr_node', anonymous=True)
        rospy.loginfo("OCR Node started.")

        self.bridge = CvBridge()
        self.scan_requested = False
        self.latest_frame = None

        # --- Publisher and Subscribers ---
        self.text_pub = rospy.Publisher('/juno/ocr/raw_text', String, queue_size=10)
        rospy.Subscriber('/juno/scan_command', String, self.scan_command_callback)
        
        # --- MODIFICATION ---
        # Subscribe to the PREPROCESSED image topic, not the raw one.
        rospy.Subscriber('/juno/camera/preprocessed', Image, self.image_callback)

    def scan_command_callback(self, msg):
        rospy.loginfo("Scan command received!")
        self.scan_requested = True
        self.process_image()

    def image_callback(self, data):
        """Stores the latest preprocessed frame."""
        try:
            # The incoming image is grayscale ("mono8")
            self.latest_frame = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def process_image(self):
        """Performs OCR on the latest frame."""
        if not self.scan_requested or self.latest_frame is None:
            return

        rospy.loginfo("Processing image for OCR...")
        frame_to_process = self.latest_frame.copy()
        self.scan_requested = False # Reset flag

        try:
            # The image is already preprocessed, so we can directly perform OCR
            custom_config = r'--oem 3 --psm 6'
            text = pytesseract.image_to_string(Image.fromarray(frame_to_process), config=custom_config)

            if text.strip():
                rospy.loginfo(f"OCR Detected Text: {text.strip()}")
                self.text_pub.publish(text)
            else:
                rospy.logwarn("No text detected in the image.")
                self.text_pub.publish("")
        except Exception as e:
            rospy.logerr(f"An error occurred during OCR: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        ocr_node = OcrNode()
        ocr_node.run()
    except rospy.ROSInterruptException:
        pass
