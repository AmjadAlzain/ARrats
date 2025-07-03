#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

class VerbalReporterNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('verbal_reporter_node', anonymous=True)
        rospy.loginfo("Verbal Reporter Node activated.")

        # Initialize the SoundClient for text-to-speech
        # 'blocking=True' means it will finish speaking before the next command is processed
        self.soundhandle = SoundClient(blocking=True)
        rospy.sleep(1) # Wait for the sound client to initialize

        # Subscribe to the topic where Juno's text responses are published
        rospy.Subscriber('/juno/robot_response', String, self.speak_callback)

    def speak_callback(self, msg):
        """
        This function is called every time a message is received on the topic.
        It takes the text from the message and speaks it.
        """
        response_text = msg.data
        rospy.loginfo(f"Verbal Reporter received: '{response_text}'")
        self.soundhandle.say(response_text)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        reporter_node = VerbalReporterNode()
        reporter_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Verbal Reporter Node terminated.")
