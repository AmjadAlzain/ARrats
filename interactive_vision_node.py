#!/usr/bin/env python
import rospy
import os
import cv2
import numpy as np
import threading
import queue  # Correct Python 3 import

from google.cloud import vision
from std_msgs.msg import String
from allergen_scanner.srv import TriggerScan, TriggerScanResponse
from rospkg import RosPack

def calculate_focus_score(image):
    if len(image.shape) > 2:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    return cv2.Laplacian(gray, cv2.CV_64F).var()

class InteractiveVisionNode:
    def __init__(self):
        rospy.init_node('interactive_vision_node')

        # --- NEW: State variable to hold the voice command ---
        self.camera_action = None
        
        # --- THREADING: Tools to communicate between the service thread and the main thread ---
        self.scan_request_event = threading.Event()
        self.result_queue = queue.Queue()

        # --- SUBSCRIBER: To receive voice commands for capture/quit ---
        rospy.Subscriber('/juno/camera_control_command', String, self.camera_command_callback)

        # --- Google Vision Client Initialization ---
        try:
            rp = RosPack()
            package_path = rp.get_path('allergen_scanner')
            key_path = os.path.join(package_path, 'Keys', 'arats-ocr-1b7025967ee1.json')
            os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = key_path
            self.vision_client = vision.ImageAnnotatorClient()
            rospy.loginfo("Google Vision client initialized successfully.")
        except Exception as e:
            rospy.logfatal(f"Failed to initialize Google Vision client: {e}")
            rospy.signal_shutdown("Init failed.")
            return

        # --- SERVICE: The service handler that runs in a background thread ---
        self.service = rospy.Service('/juno/interactive_scan', TriggerScan, self.handle_scan_request)
        rospy.loginfo("Interactive Vision Node is ready and waiting for scan requests.")

    def camera_command_callback(self, msg):
        """This callback runs in a ROS thread and safely sets the action for the main thread."""
        rospy.loginfo(f"Camera command '{msg.data}' received from speech node.")
        self.camera_action = msg.data

    def handle_scan_request(self, req):
        """
        This function runs in a background ROS thread.
        Its ONLY job is to signal the main loop and then wait for the result.
        """
        rospy.loginfo("Service request received. Signaling main thread to start camera.")
        self.scan_request_event.set()  # Trigger the main loop
        
        # Block and wait for the main loop to put a result in the queue
        ocr_text = self.result_queue.get()
        rospy.loginfo("Result received from main thread. Sending response.")
        
        return TriggerScanResponse(ocr_text)

    def run_camera_and_ocr(self):
        """
        This function contains all OpenCV and OCR logic.
        It is ONLY ever called from the main thread.
        """
        rospy.loginfo("Main thread: Opening camera view...")
        self.camera_action = None # Reset state at the beginning of a run
        
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened():
            rospy.logerr("Cannot open camera.")
            return ""

        window_name = "Juno Camera View"
        final_frame = None
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        
        # Loop until a voice command is received (camera_action is set)
        while self.camera_action is None and not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret: break
            
            display_frame = frame.copy()
            focus_score = calculate_focus_score(display_frame)
            cv2.putText(display_frame, f"Sharpness: {int(focus_score)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, "Say 'capture' or 'quit'", (20, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow(window_name, display_frame)

            cv2.waitKey(1)
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                self.camera_action = 'quit'
                
        # Check the action received from the voice command
        if self.camera_action == 'capture':
            ret, final_frame = cap.read()
            if not ret: final_frame = None
            rospy.loginfo("Frame captured by voice command!")
        else:
            final_frame = None
            rospy.loginfo("Exiting camera view due to voice command or window close.")
        
        cap.release()
        cv2.destroyAllWindows()
        cv2.waitKey(1)

        if final_frame is None:
            return ""

        # Perform OCR
        rospy.loginfo("Performing Google Vision OCR...")
        try:
            gray_frame = cv2.cvtColor(final_frame, cv2.COLOR_BGR2GRAY)
            sharpen_kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
            processed_image = cv2.filter2D(gray_frame, -1, sharpen_kernel)
            _, encoded_image = cv2.imencode('.png', processed_image)
            image = vision.Image(content=encoded_image.tobytes())
            response = self.vision_client.text_detection(image=image, image_context=vision.ImageContext(language_hints=["en"]))
            if response.text_annotations:
                detected_text = response.text_annotations[0].description.strip()
                rospy.loginfo(f"DEBUG OCR Raw Text: ----\n{detected_text}\n----")
                return detected_text
            return ""
        except Exception as e:
            rospy.logerr(f"An error occurred during Google OCR processing: {e}")
            return ""

    def run(self):
        """
        This is the node's main loop. It waits for the service handler to signal
        that a scan is requested, then runs the camera logic in this main thread.
        """
        while not rospy.is_shutdown():
            # Wait for the service handler to set the event.
            if self.scan_request_event.wait(timeout=0.5):
                # Once signaled, run the camera logic
                detected_text = self.run_camera_and_ocr()
                
                # Put the result in the queue for the service handler to retrieve
                self.result_queue.put(detected_text)
                
                # Reset the event to wait for the next scan request
                self.scan_request_event.clear()

if __name__ == "__main__":
    try:
        node = InteractiveVisionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass