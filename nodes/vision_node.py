#!/usr/bin/env python
# --- IMPORTS ---
import rospy
import cv2
import pytesseract
from juno_core.srv import TriggerScan, TriggerScanResponse

# --- CLASS DEFINITION ---
class VisionNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('vision_node')

        # Create the ROS Service server
        self.service = rospy.Service('/juno/perform_ocr_scan', TriggerScan, self.handle_ocr_scan)
        
        rospy.loginfo("Vision node is ready to perform OCR scans.")

    def handle_ocr_scan(self, req):
        """Service handler to capture an image, process it, and return OCR text."""
        rospy.loginfo("OCR scan requested. Capturing image...")
        
        # Use camera index 0 (default webcam). Change if you have multiple cameras.
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            rospy.logerr("Cannot open camera.")
            return TriggerScanResponse("")

        # Capture a single frame
        ret, frame = cap.read()
        cap.release() # Release camera immediately

        if not ret:
            rospy.logerr("Failed to capture frame from camera.")
            return TriggerScanResponse("")

        rospy.loginfo("Image captured. Performing OCR...")

        # --- Image Pre-processing for better OCR ---
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)
        processed_image = cv2.adaptiveThreshold(blurred_frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        
        # --- Perform OCR using Tesseract ---
        custom_config = r'--oem 3 --psm 6'
        try:
            extracted_text = pytesseract.image_to_string(processed_image, config=custom_config)
            rospy.loginfo("OCR Extracted Text: \n--- \n%s\n---", extracted_text)
            
            # Return the extracted text in the service response
            return TriggerScanResponse(extracted_text)
        except Exception as e:
            rospy.logerr("An error occurred during OCR processing: %s", str(e))
            return TriggerScanResponse("")

    def run(self):
        # Keeps the node alive, waiting for service calls
        rospy.spin()

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    try:
        vision_node = VisionNode()
        vision_node.run()
    except rospy.ROSInterruptException:
        pass