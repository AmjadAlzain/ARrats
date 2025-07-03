#!/usr/bin/env python
import rospy
import speech_recognition as sr
import pyttsx3
from std_msgs.msg import String

class SpeechInterface:
    def __init__(self):
        rospy.init_node('speech_interface_node', anonymous=True)
        self.command_pub = rospy.Publisher('/juno/user_command', String, queue_size=10)
        rospy.Subscriber('/juno/robot_response', String, self.speak_response)
        self.tts_engine = pyttsx3.init()
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        with self.microphone as source:
            rospy.loginfo("Calibrating microphone...")
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Calibration complete. Ready to listen.")

    def speak_response(self, msg):
        rospy.loginfo("Juno says: %s", msg.data)
        self.tts_engine.say(msg.data)
        self.tts_engine.runAndWait()

# This is inside the SpeechInterface class in speech_interface_node.py

    def listen_for_commands(self):
        while not rospy.is_shutdown():
            with self.microphone as source:
                rospy.loginfo("Listening for activation word 'Hello'...")
                try:
                    # 1. Listen for audio
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=3)

                    # 2. Convert audio to text
                    recognized_text = self.recognizer.recognize_google(audio).lower()
                    
                    # ==================================================================== #
                    # == NEW DEBUG LINE: This prints whatever the system heard.           == #
                    # == This is the key to seeing what Juno is detecting.                == #
                    rospy.loginfo(f"DEBUG: Juno heard: '{recognized_text}'")
                    # ==================================================================== #

                    # 3. Check if the wake word is in the recognized text
                    if "hello" in recognized_text:
                        rospy.loginfo("Activation word detected!")
                        self.tts_engine.say("Yes?")
                        self.tts_engine.runAndWait()

                        # 4. Listen for the actual command that follows
                        rospy.loginfo("Listening for command...")
                        command_audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                        command_text = self.recognizer.recognize_google(command_audio)
                        
                        # This log info was already here and is also very useful
                        rospy.loginfo("User command heard: %s", command_text)
                        
                        # 5. Publish the command for the manager node
                        self.command_pub.publish(command_text)

                except sr.WaitTimeoutError:
                    # This happens when you don't say anything, it's normal.
                    pass 
                except sr.UnknownValueError:
                    # This happens when speech is detected, but it's not clear enough to recognize.
                    rospy.logwarn("--> Could not understand audio, please speak more clearly.")
                except sr.RequestError as e: 
                    rospy.logerr("Could not request results from Google; {0}".format(e))

if __name__ == '__main__':
    try:
        SpeechInterface().listen_for_commands()
    except rospy.ROSInterruptException:
        pass
