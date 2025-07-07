#!/usr/bin/env python
import rospy
import re
import nltk
from nltk.stem import WordNetLemmatizer
from std_msgs.msg import String
from allergen_scanner.srv import TriggerScan

# Ensure that the WordNet lemmatizer is downloaded
nltk.download('wordnet', quiet=True)

# Define a dictionary of allergens and their keywords
# This dictionary maps allergen names to lists of keywords that can be used to identify them in ingredient lists.
ALLERGEN_KEYWORDS = {
    'eggs': ['egg', 'eggs', 'albumin', 'ovalbumin', 'ovoglobulin', 'lysozyme', 'ovomucin', 'vitellin', 'ovovitellin', 'livetin', 'mayonnaise'],
    'peanuts': ['peanut', 'peanuts', 'groundnut', 'arachis', 'beer nut', 'goober', 'monkey nut', 'peanut oil'],
    'shellfish': ['shellfish', 'shrimp', 'prawn', 'crab', 'lobster', 'crayfish', 'krill', 'scampi', 'crevette', 'langoustine'],
    'soy': ['soy', 'soya', 'soybean', 'edamame', 'miso', 'natto', 'shoyu', 'tamari', 'tempeh', 'tofu', 'soy lecithin', 'hydrolyzed soy protein'],
    'milk': ['milk', 'dairy', 'casein', 'caseinates', 'cream', 'curds', 'custard', 'ghee', 'half-and-half', 'hydrolysates', 'lactalbumin', 'lactose', 'pudding', 'whey', 'yogurt', 'nougat', 'cheese'],
    'nuts': ['nuts','nut','tree nut', 'almond', 'brazil nut', 'cashew', 'chestnut', 'hazelnut', 'filbert', 'gianduja', 'hickory nut', 'macadamia nut', 'marzipan', 'pecan', 'pesto', 'pistachio', 'praline', 'shea nut', 'walnut'],
    'wheat': ['wheat', 'bran', 'bread crumb', 'bulgur', 'cereal extract', 'couscous', 'durum', 'einkorn', 'emmer', 'farina', 'flour', 'gluten', 'matzoh', 'pasta', 'seitan', 'semolina', 'spelt', 'triticale', 'wheat germ'],
    'fish': ['fish', 'anchovy', 'anchovies', 'bass', 'catfish', 'cod', 'flounder', 'grouper', 'haddock', 'hake', 'halibut', 'herring', 'mahi-mahi', 'perch', 'pike', 'pollock', 'salmon', 'scrod', 'sole', 'snapper', 'swordfish', 'tilapia', 'trout', 'tuna', 'surimi'],
    'sesame': ['sesame', 'sesame seed', 'tahini', 'benne', 'gingelly'],
    'mustard': ['mustard', 'mustard flour', 'mustard seed'],
    'celery': ['celery', 'celeriac'],
    'lupin': ['lupin', 'lupine'],
    'molluscs': ['mollusc', 'mollusk', 'oyster', 'mussel', 'scallop', 'clam', 'cockle', 'squid', 'octopus', 'snail', 'escargot', 'calamari'],
    'sulphites': ['sulphite', 'sulfite', 'sulphur dioxide', 'sulfur dioxide', 'potassium bisulphite', 'sodium bisulphite'],
    'chocolate': ['chocolate', 'cacao', 'cocoa'], 
}
class AllergenManager:
    def __init__(self):
        rospy.init_node('allergen_manager_node')
        self.lemmatizer = WordNetLemmatizer()
        self.allergens = set()
        
        self.response_pub = rospy.Publisher('/juno/robot_response', String, queue_size=10)
        self.speech_control_pub = rospy.Publisher('/juno/speech_control', String, queue_size=1)        
        
        rospy.Subscriber('/juno/user_command', String, self.command_callback)
            
        rospy.loginfo("Allergen Manager is ready.")
        self.publish_response("Hello, I am Juno. You can ask me to add allergies or scan a product.")

    def publish_response(self, text):
        self.response_pub.publish(String(text))

    def _analyze_ingredients(self, ingredient_text):
        """Analyze the ingredient text for known allergens."""
        detected = []
        already_reported = set()
        text_lower = ingredient_text.lower()
        
        ingredients = re.split(r'[;,()]', text_lower)
        ingredients = [i.strip() for i in ingredients if i.strip()]
        
        for allergen in self.allergens:
            keywords = ALLERGEN_KEYWORDS.get(allergen, [allergen])
            for keyword in keywords:
                lemmatized_keyword = self.lemmatizer.lemmatize(keyword)
                for ingredient in ingredients:
                    if lemmatized_keyword in ingredient:
                        key = (allergen, ingredient)
                        if key not in already_reported:
                            detected.append(key)
                            already_reported.add(key)
        return detected

    def perform_scan(self):
            self.publish_response("Okay, a camera window will open. Please position the ingredient list and press say 'capture'.")
            self.speech_control_pub.publish('camera_mode')            
            try:                
                rospy.loginfo("Calling the interactive scan service...")
                rospy.wait_for_service('/juno/interactive_scan')
                scan_client = rospy.ServiceProxy('/juno/interactive_scan', TriggerScan)
                
                rospy.loginfo("Calling the interactive scan service...")
                response = scan_client() 
                ocr_text = response.ocr_text

                if not ocr_text.strip():
                    self.publish_response("Scan was cancelled or no text was found.")
                else:
                    detected_allergens = self._analyze_ingredients(ocr_text)
                    if not detected_allergens:
                        self.publish_response("Good news! I did not find any of your specified allergens.")
                    else:
                        response_parts = [f"{allergen} from the ingredient '{trigger}'" for allergen, trigger in detected_allergens]
                        detected_str = ", and ".join(response_parts)
                        final_response = f"Warning! This product may contain {detected_str}."
                        self.publish_response(final_response)

            except rospy.ROSException:
                rospy.logerr("Interactive scan service not available.")
                self.publish_response("Sorry, the camera system is not available right now.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                self.publish_response("Sorry, there was an error during the scan.")
            finally:
                rospy.loginfo("Resuming normal speech recognition.")
                self.speech_control_pub.publish('idle_mode')
    
    def command_callback(self, msg):
        cmd = msg.data.lower()
        if 'add' in cmd and 'allergy' in cmd:
            allergen = cmd.split('allergy')[-1].strip()
            if allergen in ALLERGEN_KEYWORDS:
                self.allergens.add(allergen)
                self.publish_response(f"{allergen.capitalize()} added to your profile.")
            else:
                self.publish_response(f"Sorry, I don't know about '{allergen}'.")
        elif 'remove' in cmd:
            allergen = cmd.split('remove')[-1].strip()
            if allergen in self.allergens:
                self.allergens.remove(allergen)
                self.publish_response(f"{allergen.capitalize()} removed.")
            else:
                self.publish_response(f"You don't have {allergen} in your profile.")
        elif 'allergies' in cmd or 'profile' in cmd:
            if not self.allergens:
                self.publish_response("Your allergy profile is empty.")
            else:
                profile_str = ", ".join(sorted([a.capitalize() for a in self.allergens]))
                self.publish_response(f"I am watching for: {profile_str}.")
        elif 'scan' in cmd:
            if not self.allergens:
                self.publish_response("Please add an allergy to your profile first.")
            else:
                self.perform_scan()
        else:
            self.publish_response("Sorry, I didn't understand. You can say 'add allergy', 'remove', or 'scan'.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        AllergenManager().run()
    except rospy.ROSInterruptException:
        pass
