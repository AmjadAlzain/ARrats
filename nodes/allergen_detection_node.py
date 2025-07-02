import re
import nltk
from nltk.stem import WordNetLemmatizer
nltk.download('wordnet') #running for first time --> download wordnet data

# Mapping of allergens to associated keywords/ingredients
ALLERGEN_KEYWORDS = {
    'eggs': ['egg', 'albumin', 'ovalbumin'],
    'peanuts': ['peanut', 'groundnut'],
    'shellfish': ['shrimp', 'prawn', 'crab', 'lobster', 'shellfish'],
    'strawberries': ['strawberry', 'strawberries'],
    'tomatoes': ['tomato', 'tomatoes'],
    'chocolate': ['chocolate', 'cocoa'],
    'pollen': ['pollen'],
    'cats': ['cat', 'cats'],
    'soy': ['soy', 'soya', 'soy lecithin'],
    'milk': ['milk', 'casein', 'whey', 'lactose'],
    'tree nuts': ['almond', 'walnut', 'cashew', 'pecan', 'hazelnut', 'pistachio', 'macadamia', 'brazil nut', 'tree nut'],
    'wheat': ['wheat', 'gluten', 'farina', 'spelt', 'semolina'],
    'fish': ['fish', 'anchovy', 'bass', 'catfish', 'cod', 'flounder', 'grouper', 'haddock', 'hake', 'halibut', 'herring', 'mahi mahi', 'perch', 'pike', 'pollock', 'salmon', 'scrod', 'sole', 'snapper', 'swordfish', 'tilapia', 'trout', 'tuna'],
    'sesame': ['sesame', 'tahini'],
}

def lemmatize_word(word, lemmatizer):
    return lemmatizer.lemmatize(word)

def detect_allergens(user_allergens, ingredient_text):
    lemmatizer = WordNetLemmatizer()
    detected = []
    ingredient_text_lower = ingredient_text.lower()

    # 1. Look for 'contains:' and 'may contain:' sections
    contains_matches = re.findall(r'(contains:|may contain:)([^\n\r]*)', ingredient_text_lower)
    sections_to_check = []
    if contains_matches:
        for label, section in contains_matches:
            sections_to_check.append(section)
    # Always also check the full ingredient list (before any 'contains:' or 'may contain:')
    # Extract up to the first 'contains:' or 'may contain:'
    split_sections = re.split(r'(contains:|may contain:)', ingredient_text_lower)
    if split_sections:
        main_ingredients = split_sections[0]
        sections_to_check.insert(0, main_ingredients)
    else:
        sections_to_check.insert(0, ingredient_text_lower)

    # 2. For each section, split into ingredients and lemmatize
    already_reported = set()
    for section in sections_to_check:
        # Split by comma, semicolon, or parentheses
        ingredients = re.split(r'[;,()]', section)
        ingredients = [i.strip() for i in ingredients if i.strip()]
        for allergen in user_allergens:
            keywords = ALLERGEN_KEYWORDS.get(allergen.lower(), [allergen.lower()])
            # Lemmatize keywords
            lemmatized_keywords = [lemmatize_word(k, lemmatizer) for k in keywords]
            for ingredient in ingredients:
                # Tokenize and lemmatize each word in the ingredient
                tokens = re.findall(r'\b\w+\b', ingredient)
                lemmatized_tokens = [lemmatize_word(t, lemmatizer) for t in tokens]
                # Check for any lemmatized keyword in lemmatized tokens or ingredient
                for keyword, lemmatized_keyword in zip(keywords, lemmatized_keywords):
                    key = (allergen, ingredient.strip())
                    if key in already_reported:
                        continue
                    if (lemmatized_keyword in lemmatized_tokens) or (lemmatized_keyword in ingredient):
                        detected.append((allergen, ingredient.strip()))
                        already_reported.add(key)
                        break
    if detected:
        print("Alert! The following allergens were detected:")
        for allergen, ingredient in detected:
            print(f"- {allergen} (triggered by: '{ingredient}')")
    else:
        print("No allergens from your profile were detected in the ingredient list.")

if __name__ == "__main__":
    user_allergy_profile = ['eggs', 'peanuts', 'shellfish', 'strawberries', 'tomatoes', 'chocolate', 'pollen', 'cats', 'soy']
    # Example ingredient list (simulating OCR output)
    ingredient_text = "Ingredients: sugar, cocoa butter, chocolates, peanuts, milk, soy lecithins, vanilla. Contains: eggs, tomatoes. May contain: shellfish, strawberries."
    print("User allergy profile:", user_allergy_profile)
    print("Ingredient list:", ingredient_text)
    detect_allergens(user_allergy_profile, ingredient_text)
