#!/usr/bin/env python

import rospy
import os
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from google.cloud import vision

class GoogleOCRNode:
    def __init__(self):
        """
        Initializes the ROS node, publishers, subscribers, and Google Vision client.
        """
        rospy.init_node('google_ocr_node', anonymous=True)
        rospy.loginfo("Google OCR Node started.")

        # --- Path Setup for Credentials ---
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        key_path = os.path.join(project_root, 'Keys', 'arats-ocr-1b7025967ee1.json')

        if not os.path.exists(key_path):
            rospy.logerr(f"Google Cloud credentials not found at: {key_path}")
            rospy.signal_shutdown("Missing credentials file.")
            return
            
        os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = key_path
        
        # Initialize Google Vision client
        try:
            self.vision_client = vision.ImageAnnotatorClient()
            rospy.loginfo("Google Vision API client initialized successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Google Vision client: {e}")
            rospy.signal_shutdown("Google Vision client initialization failed.")
            return

        self.bridge = CvBridge()

        # Publisher for the OCR text output
        self.ocr_text_pub = rospy.Publisher('/juno/ocr/text', String, queue_size=10)

        # Subscriber to the preprocessed image topic
        self.image_sub = rospy.Subscriber('/juno/camera/preprocessed', Image, self.image_callback, queue_size=1)

    def image_callback(self, ros_image):
        """
        Callback function for the image subscriber.
        Performs OCR on the received image and publishes the result.
        """
        rospy.logdebug("Received an image for OCR.")
        try:
            # Convert the ROS Image message to an OpenCV image (mono8)
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "mono8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        try:
            # Convert the OpenCV image to bytes for the Vision API
            _, encoded_image = cv2.imencode('.png', cv_image)
            content = encoded_image.tobytes()
            
            # Perform text detection
            image = vision.Image(content=content)
            response = self.vision_client.text_detection(image=image)
            
            if response.error.message:
                rospy.logwarn(f"Google Vision API Error: {response.error.message}")
                return

            texts = response.text_annotations
            if texts:
                detected_text = texts[0].description
                rospy.loginfo(f"Detected Text: {detected_text[:100]}...") # Log a snippet
                # Publish the full text
                self.ocr_text_pub.publish(detected_text)
            else:
                rospy.loginfo("No text detected in the image.")

        except Exception as e:
            rospy.logerr(f"An error occurred during OCR processing: {e}")

    def run(self):
        """
        Keeps the node running.
        """
        rospy.loginfo("Google OCR Node is running and waiting for images.")
        rospy.spin()

if __name__ == '__main__':
    try:
        ocr_node = GoogleOCRNode()
        ocr_node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal(f"An unhandled exception occurred: {e}")
