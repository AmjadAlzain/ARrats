#!/usr/bin/env python
import re
import nltk
from nltk.stem import WordNetLemmatizer
import rospy
from std_msgs.msg import String
from allergen_scanner.srv import TriggerScan # Use your package name here

ALLERGEN_KEYWORDS = {
    'eggs': ['egg', 'albumin', 'ovalbumin'],
    'peanuts': ['peanut', 'groundnut'],
    'shellfish': ['shrimp', 'prawn', 'crab', 'lobster', 'shellfish'],
    'soy': ['soy', 'soya', 'soy lecithin'],
    'milk': ['milk', 'casein', 'whey', 'lactose', 'cream'],
    'tree nuts': ['almond', 'walnut', 'cashew', 'pecan', 'hazelnut', 'pistachio'],
    'wheat': ['wheat', 'gluten', 'farina', 'spelt', 'semolina'],
    'fish': ['fish', 'anchovy', 'bass', 'cod', 'salmon', 'tuna'],
    'sesame': ['sesame', 'tahini'],
}

class AllergenManager:
    def __init__(self):
        rospy.init_node('allergen_manager_node')
        self.lemmatizer = WordNetLemmatizer()
        self.allergens = set()
        self.response_pub = rospy.Publisher('/juno/robot_response', String, queue_size=10)
        rospy.Subscriber('/juno/user_command', String, self.command_callback)
        rospy.loginfo("Waiting for the OCR service...")
        rospy.wait_for_service('/juno/perform_ocr_scan')
        self.ocr_service_client = rospy.ServiceProxy('/juno/perform_ocr_scan', TriggerScan)
        rospy.loginfo("OCR service is available.")
        rospy.sleep(1) 
        self.publish_response("Hello, I am Juno. Please set up your allergy profile.")

    def publish_response(self, text):
        rospy.loginfo("Publishing response: %s", text)
        self.response_pub.publish(String(text))
    
    def _analyze_ingredients(self, ingredient_text):
        detected_allergens = []
        already_reported = set()
        ingredient_text_lower = ingredient_text.lower()
        for allergen in self.allergens:
            keywords = ALLERGEN_KEYWORDS.get(allergen.lower(), [allergen.lower()])
            lemmatized_keywords = [self.lemmatizer.lemmatize(k) for k in keywords]
            text_tokens = re.findall(r'\b\w+\b', ingredient_text_lower)
            lemmatized_text_tokens = [self.lemmatizer.lemmatize(t) for t in text_tokens]
            for i, lemmatized_token in enumerate(lemmatized_text_tokens):
                if lemmatized_token in lemmatized_keywords:
                    original_ingredient_word = text_tokens[i]
                    key = (allergen, original_ingredient_word)
                    if key not in already_reported:
                        detected_allergens.append(key)
                        already_reported.add(key)
        return detected_allergens

    def perform_scan(self):
        try:
            response = self.ocr_service_client()
            ocr_text = response.ocr_text
            if not ocr_text.strip():
                self.publish_response("I couldn't read any text. Please try again.")
                return
            detected = self._analyze_ingredients(ocr_text)
            if not detected:
                self.publish_response("Good news! I did not find any of your specified allergens.")
            else:
                response_parts = [f"{allergen} from the ingredient '{trigger}'" for allergen, trigger in detected]
                detected_str = ", and ".join(response_parts)
                final_response = f"Warning! This product may contain {detected_str}."
                self.publish_response(final_response)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            self.publish_response("Sorry, there was an error with the vision system.")

    def command_callback(self, msg):
        command = msg.data.lower()
        if 'add' in command and 'allergy' in command:
            words = command.split()
            try:
                allergen_to_add = words[words.index('allergy') + 1]
                if allergen_to_add in ALLERGEN_KEYWORDS:
                    self.allergens.add(allergen_to_add)
                    self.publish_response(f"{allergen_to_add.capitalize()} added to your profile.")
                else:
                    self.publish_response(f"Sorry, I only know about common food allergens like milk or eggs.")
            except (ValueError, IndexError):
                self.publish_response("I didn't catch that. Please say, 'add allergy milk'.")
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
                self.publish_response("I didn't catch that. Please say, 'remove dairy'.")
        elif 'what are my allergies' in command or 'show my profile' in command:
            if not self.allergens:
                self.publish_response("Your allergy profile is currently empty.")
            else:
                profile_str = ", ".join(sorted(list(self.allergens)))
                self.publish_response(f"I am currently watching for: {profile_str}.")
        elif 'scan' in command:
            if not self.allergens:
                self.publish_response("Your profile is empty. Please add an allergy first.")
                return
            self.publish_response("Okay, analyzing. Please hold the ingredients steady.")
            self.perform_scan()
        else:
            self.publish_response("I'm sorry, I didn't understand. You can say 'add allergy', 'remove', or 'scan'.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        AllergenManager().run()
    except rospy.ROSInterruptException:
        pass