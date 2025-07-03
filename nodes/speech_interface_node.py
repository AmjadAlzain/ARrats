#!/usr/bin/env python
# --- IMPORTS ---
import rospy
import speech_recognition as sr
import pyttsx3
from std_msgs.msg import String

# --- CLASS DEFINITION ---
class SpeechInterface:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('speech_interface_node', anonymous=True)

        # Publisher for user commands (text)
        self.command_pub = rospy.Publisher('/juno/user_command', String, queue_size=10)

        # Subscriber for robot responses (text)
        rospy.Subscriber('/juno/robot_response', String, self.speak_response)

        # Initialize Text-to-Speech (TTS) engine
        self.tts_engine = pyttsx3.init()

        # Initialize Speech-to-Text (STT) recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Calibrate microphone for ambient noise
        with self.microphone as source:
            rospy.loginfo("Calibrating microphone for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Calibration complete. Ready to listen.")

    def speak_response(self, msg):
        """Callback function to speak out the robot's response."""
        rospy.loginfo("Juno says: %s", msg.data)
        self.tts_engine.say(msg.data)
        self.tts_engine.runAndWait()

    def listen_for_commands(self):
        """Main loop to listen for the activation word and subsequent commands."""
        while not rospy.is_shutdown():
            with self.microphone as source:
                rospy.loginfo("Listening for activation word 'Juno'...")
                try:
                    # Listen for audio with a timeout
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=2)
                    # Recognize the activation word using Google's free API
                    activation_text = self.recognizer.recognize_google(audio).lower()

                    if "juno" in activation_text:
                        rospy.loginfo("Activation word detected!")
                        self.tts_engine.say("Yes?")
                        self.tts_engine.runAndWait()

                        # Listen for the actual command
                        rospy.loginfo("Listening for command...")
                        command_audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                        command_text = self.recognizer.recognize_google(command_audio)
                        rospy.loginfo("User command: %s", command_text)

                        # Publish the command as a ROS message
                        self.command_pub.publish(command_text)

                except sr.WaitTimeoutError:
                    pass # It's normal to have silence
                except sr.UnknownValueError:
                    rospy.logwarn("Google Speech Recognition could not understand audio")
                except sr.RequestError as e:
                    rospy.logerr("Could not request results from Google; {0}".format(e))

# --- MAIN EXECUTION ---
if __name__ == '__main__':
    try:
        interface = SpeechInterface()
        interface.listen_for_commands()
    except rospy.ROSInterruptException:
        pass