#!/usr/bin/env python

# --- NEW IMPORTS ---
import re
import nltk
from nltk.stem import WordNetLemmatizer

# --- ROS IMPORTS ---
import rospy
from std_msgs.msg import String
from juno_core.srv import TriggerScan

# --- NEW: ALLERGEN KEYWORD DICTIONARY ---
# This dictionary makes the detection much more powerful.
ALLERGEN_KEYWORDS = {
    'eggs': ['egg', 'albumin', 'ovalbumin'],
    'peanuts': ['peanut', 'groundnut'],
    'shellfish': ['shrimp', 'prawn', 'crab', 'lobster', 'shellfish'],
    'strawberries': ['strawberry', 'strawberries'],
    'tomatoes': ['tomato', 'tomatoes'],
    'chocolate': ['chocolate', 'cocoa'],
    'pollen': ['pollen'],
    'cats': ['cat', 'cats'], # Example non-food allergen
    'soy': ['soy', 'soya', 'soy lecithin'],
    'milk': ['milk', 'casein', 'whey', 'lactose', 'cream'],
    'tree nuts': ['almond', 'walnut', 'cashew', 'pecan', 'hazelnut', 'pistachio', 'macadamia', 'brazil nut', 'tree nut'],
    'wheat': ['wheat', 'gluten', 'farina', 'spelt', 'semolina'],
    'fish': ['fish', 'anchovy', 'bass', 'catfish', 'cod', 'flounder', 'grouper', 'haddock', 'hake', 'halibut', 'herring', 'mahi mahi', 'perch', 'pike', 'pollock', 'salmon', 'scrod', 'sole', 'snapper', 'swordfish', 'tilapia', 'trout', 'tuna'],
    'sesame': ['sesame', 'tahini'],
}

# --- CLASS DEFINITION ---
class AllergenManager:
    def __init__(self):
        rospy.init_node('allergen_manager_node')

        # --- NEW: Initialize the lemmatizer once for efficiency ---
        self.lemmatizer = WordNetLemmatizer()
        
        # User's allergy profile (the simple names, e.g., 'milk', 'eggs')
        self.allergens = set()

        # ROS Publishers and Subscribers
        self.response_pub = rospy.Publisher('/juno/robot_response', String, queue_size=10)
        rospy.Subscriber('/juno/user_command', String, self.command_callback)

        # ROS Service Client
        rospy.loginfo("Waiting for the OCR service...")
        rospy.wait_for_service('/juno/perform_ocr_scan')
        self.ocr_service_client = rospy.ServiceProxy('/juno/perform_ocr_scan', TriggerScan)
        rospy.loginfo("OCR service is available.")
        
        # Initial greeting
        rospy.sleep(1) 
        self.publish_response("Hello, I am Juno. My allergen detection is now upgraded. Please set up your allergy profile.")

    def publish_response(self, text):
        rospy.loginfo("Publishing response: %s", text)
        self.response_pub.publish(String(text))
    
    # --- NEW: ADVANCED ALLERGEN DETECTION LOGIC ---
    # This method incorporates your new, advanced logic.
    def _analyze_ingredients(self, ingredient_text):
        """
        Analyzes OCR text for allergens using a keyword dictionary and lemmatization.
        Returns a list of tuples: (allergen, triggering_ingredient)
        """
        detected_allergens = []
        already_reported = set()
        
        # 1. Pre-process the ingredient text
        ingredient_text_lower = ingredient_text.lower()
        
        # 2. Iterate through the user's profile
        for allergen in self.allergens:
            # Get the list of keywords for this allergen, default to the allergen name itself
            keywords = ALLERGEN_KEYWORDS.get(allergen.lower(), [allergen.lower()])
            lemmatized_keywords = [self.lemmatizer.lemmatize(k) for k in keywords]
            
            # Use regex to find potential ingredient words in the text
            # This is simpler and more robust than splitting by comma/semicolon
            text_tokens = re.findall(r'\b\w+\b', ingredient_text_lower)
            lemmatized_text_tokens = [self.lemmatizer.lemmatize(t) for t in text_tokens]

            # 3. Check for matches
            for i, lemmatized_token in enumerate(lemmatized_text_tokens):
                if lemmatized_token in lemmatized_keywords:
                    # Found a match!
                    original_ingredient_word = text_tokens[i]
                    key = (allergen, original_ingredient_word)

                    if key not in already_reported:
                        detected_allergens.append(key)
                        already_reported.add(key)
        
        return detected_allergens

    # --- UPDATED: This method now calls the new logic ---
    def perform_scan(self):
        """Calls the OCR service and uses the advanced logic to check for allergens."""
        try:
            response = self.ocr_service_client()
            ocr_text = response.ocr_text
            
            if not ocr_text.strip():
                self.publish_response("I couldn't read any text. Please try again with better lighting.")
                return

            # Call our new, powerful analysis function
            detected = self._analyze_ingredients(ocr_text)

            # Formulate and publish the final response
            if not detected:
                self.publish_response("Good news! I did not find any of your specified allergens.")
            else:
                # Create a more descriptive response
                response_parts = []
                for allergen, trigger in detected:
                    response_parts.append(f"{allergen} from the ingredient '{trigger}'")
                
                detected_str = ", and ".join(response_parts)
                final_response = f"Warning! This product may contain {detected_str}."
                self.publish_response(final_response)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            self.publish_response("Sorry, there was an error with the vision system.")

    # --- The command callback remains mostly the same ---
    def command_callback(self, msg):
        command = msg.data.lower()
        rospy.loginfo("Received command: '%s'", command)

        if 'add' in command and 'allergy' in command:
            words = command.split()
            try:
                allergen_to_add = words[words.index('allergy') + 1]
                if allergen_to_add in ALLERGEN_KEYWORDS:
                    self.allergens.add(allergen_to_add)
                    self.publish_response(f"{allergen_to_add.capitalize()} added to your profile.")
                    rospy.loginfo(f"Current allergens: {self.allergens}")
                else:
                    self.publish_response(f"Sorry, I don't have a profile for {allergen_to_add}. I know about eggs, milk, soy, and others.")
            except (ValueError, IndexError):
                self.publish_response("I didn't catch that. Please say, for example, 'add allergy milk'.")
        elif 'remove' in command:
            words = command.split()
            try:
                allergen_to_remove = words[words.index('remove') + 1]
                if allergen_to_remove in self.allergens:
                    self.allergens.remove(allergen_to_remove)
                    self.publish_response(f"{allergen_to_remove.capitalize()} removed.")
                else:
                    self.publish_response(f"I don't have {allergen_to_remove} in your profile.")
            except (ValueError, IndexError):
                self.publish_response("I didn't catch that. Please say, for example, 'remove dairy'.")
        elif 'what are my allergies' in command or 'show my profile' in command:
            if not self.allergens:
                self.publish_response("Your allergy profile is currently empty.")
            else:
                profile_str = ", ".join(sorted(list(self.allergens)))
                self.publish_response(f"I am currently watching for: {profile_str}.")
        elif 'scan' in command:
            if not self.allergens:
                self.publish_response("Your allergy profile is empty. Please add an allergy before scanning.")
                return
            self.publish_response("Okay, analyzing with advanced detection. Please hold the ingredient list steady.")
            self.perform_scan()
        else:
            self.publish_response("I'm sorry, I didn't understand. You can say 'add allergy', 'remove', or 'scan'.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = AllergenManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass