#!/usr/bin/env python
import rospy
import cv2
import pytesseract
from allergen_scanner.srv import TriggerScan, TriggerScanResponse # Use your package name

class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node')
        self.service = rospy.Service('/juno/perform_ocr_scan', TriggerScan, self.handle_ocr_scan)
        rospy.loginfo("Vision node is ready to perform OCR scans.")

    def handle_ocr_scan(self, req):
        rospy.loginfo("OCR scan requested. Capturing image...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            rospy.logerr("Cannot open camera.")
            return TriggerScanResponse("")
        ret, frame = cap.read()
        cap.release()
        if not ret:
            rospy.logerr("Failed to capture frame from camera.")
            return TriggerScanResponse("")
        rospy.loginfo("Image captured. Performing OCR...")
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        processed_image = cv2.adaptiveThreshold(gray_frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        try:
            extracted_text = pytesseract.image_to_string(processed_image)
            rospy.loginfo("OCR Extracted Text: \n%s", extracted_text)
            return TriggerScanResponse(extracted_text)
        except Exception as e:
            rospy.logerr("An error occurred during OCR processing: %s", str(e))
            return TriggerScanResponse("")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        VisionNode().run()
    except rospy.ROSInterruptException:
        pass
