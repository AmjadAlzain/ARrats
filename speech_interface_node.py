#!/usr/bin/env python
import rospy
import speech_recognition as sr
import pyttsx3
from std_msgs.msg import String

class SpeechInterface:
    def __init__(self):
        rospy.init_node('speech_interface_node', anonymous=True)
        self.command_pub = rospy.Publisher('/juno/user_command', String, queue_size=10)
        self.camera_cmd_pub = rospy.Publisher('/juno/camera_control_command', String, queue_size=1)

        rospy.Subscriber('/juno/robot_response', String, self.speak_response)
        
        rospy.Subscriber('/juno/speech_control', String, self.handle_control_command)

        self.tts_engine = pyttsx3.init()
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        self.mode = "idle" # Can be "idle" or "camera_command"

        with self.microphone as source:
            rospy.loginfo("Calibrating microphone...")
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Calibration complete. Ready to listen.")

    def speak_response(self, msg):
        rospy.loginfo("Juno says: %s", msg.data)
        self.tts_engine.say(msg.data)
        self.tts_engine.runAndWait()

    def handle_control_command(self, msg):
        if msg.data == 'camera_mode':
            rospy.loginfo("Entering camera command mode.")
            self.mode = "camera_command"
        elif msg.data == 'idle_mode':
            rospy.loginfo("Returning to idle mode.")
            self.mode = "idle"
            
    def listen_for_commands(self):
        while not rospy.is_shutdown():
            if self.mode == "idle":
                self.listen_for_activation()
            elif self.mode == "camera_command":
                self.listen_for_camera_command()
            else:
                rospy.sleep(0.1)

    def listen_for_activation(self):
        rospy.loginfo("Listening for activation word 'hello'...")
        with self.microphone as source:
            try:
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=3)
                recognized_text = self.recognizer.recognize_google(audio).lower()
                rospy.loginfo(f"DEBUG: Juno heard: '{recognized_text}'")
                if "hello" in recognized_text:
                    rospy.loginfo("Activation word detected!")
                    self.tts_engine.say("Yes?")
                    self.tts_engine.runAndWait()
                    rospy.loginfo("Listening for command...")
                    command_audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                    command_text = self.recognizer.recognize_google(command_audio)
                    rospy.loginfo("User command heard: %s", command_text)
                    self.command_pub.publish(command_text)
            except (sr.WaitTimeoutError, sr.UnknownValueError):
                pass 
            except sr.RequestError as e: 
                rospy.logerr("Could not request results from Google; {0}".format(e))
    
    def listen_for_camera_command(self):
        rospy.loginfo("Listening for 'capture' or 'quit'...")
        with self.microphone as source:
            try:
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=3)
                recognized_text = self.recognizer.recognize_google(audio).lower()
                rospy.loginfo(f"DEBUG (Camera Mode): Heard '{recognized_text}'")
                if 'capture' in recognized_text:
                    rospy.loginfo("'capture' command heard.")
                    self.camera_cmd_pub.publish('capture')
                    self.mode = "paused" # Temporarily pause while vision node processes
                elif 'quit' in recognized_text or 'cancel' in recognized_text:
                    rospy.loginfo("'quit' command heard.")
                    self.camera_cmd_pub.publish('quit')
                    self.mode = "paused" # Temporarily pause
            except (sr.WaitTimeoutError, sr.UnknownValueError):
                pass
            except sr.RequestError as e:
                rospy.logerr("Could not request results from Google; {0}".format(e))                

if __name__ == '__main__':
    try:
        SpeechInterface().listen_for_commands()
    except rospy.ROSInterruptException:
        pass