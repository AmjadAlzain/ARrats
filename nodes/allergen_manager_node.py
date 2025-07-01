import rospy
from std_msgs.msg import String

class AllergenManagerNode:
    def __init__(self):
        rospy.init_node('allergen_manager_node', anonymous=True)
        rospy.loginfo("Initializing Allergen Manager Node")
        
        # Initialize a set to keep track of allergens
        self.allergens = set()

        # Publisher for allergen information
        self.allergen_pub = rospy.Publisher('/juno/robot_response', String, queue_size=10)
        rospy.Subscriber('/juno/user_command', String, self.command_callback)
        
        # Initial greeting
        rospy.sleep(1) 
        self.publish_response("Hello, I am Juno. To begin, please set up your allergy profile by saying 'Juno, add an allergy'.")

    def publish_response(self, text):
        rospy.loginfo("Publishing response: %s", text)
        self.response_pub.publish(String(text))

    def command_callback(self, msg):
        """Processes commands received from the speech interface."""
        command = msg.data.lower()
        rospy.loginfo("Received command: '%s'", command)

        if "add" in command and "allergy" in command:
            words = command.split()
            try:
                allergy_to_add = words[words.index("allergy") + 1]
                self.allergens.add(allergy_to_add)
                self.publish_response(f"Allergy '{allergy_to_add}' added successfully.")
                rospy.loginfo("Current allergens: %s", self.allergens)
            except (IndexError, ValueError):
                self.publish_response("Sorry, I didn't catch the allergy you want to add.")

        elif "remove" in command and "allergy" in command:
            words = command.split()
            try:
                allergy_to_remove = words[words.index("allergy") + 1]
                if allergy_to_remove in self.allergens:
                    self.allergens.remove(allergy_to_remove)
                    self.publish_response(f"Allergy '{allergy_to_remove}' removed successfully.")
                else:
                    self.publish_response(f"Allergy '{allergy_to_remove}' not found in your profile.")
                rospy.loginfo("Current allergens: %s", self.allergens)
            except (IndexError, ValueError):
                self.publish_response("Sorry, I didn't catch that. Please say, for example, 'remove dairy'.")

        elif 'what are my allergies' in command or 'list my allergies' in command or 'show my profile' in command:
            if not self.allergens:
                self.publish_response("You have no allergies set up.")
            else:   
                allergies_list = ', '.join(self.allergens)
                self.publish_response(f"Your current allergies are: {allergies_list}.")

        elif 'scan' in command:
            if not self.allergens:
                self.publish_response("Please set up your allergy profile first by adding allergies.")
                return

            self.publish_response("Scanning for allergens... Please wait.")
        
        else:
            self.publish_response("I'm sorry, I didn't understand that. You can say 'add allergy', 'remove', or 'list allergies'.")
    
    def run(self):
        rospy.loginfo("Allergen Manager Node is running.")
        rospy.spin()
    
if __name__ == '__main__':
    try:
        allergen_manager_node = AllergenManagerNode()
        allergen_manager_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Allergen Manager Node terminated.")