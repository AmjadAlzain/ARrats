import cv2
import pytesseract
from PIL import Image
import re
from thefuzz import fuzz

# --- Configuration ---
# If tesseract is not in your PATH, include the following line
pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'

def preprocess_image(image, mode=0):
    """
    Pre-processes the image to improve OCR accuracy using different modes.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    if mode == 0:  # Adaptive Threshold
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        processed = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                           cv2.THRESH_BINARY_INV, 11, 4)
    elif mode == 1:  # Simple Binary Threshold
        _, processed = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY_INV)
    elif mode == 2: # Otsu's Binarization
        _, processed = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    elif mode == 3: # Dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
        dilated = cv2.dilate(gray, kernel, iterations=1)
        _, processed = cv2.threshold(dilated, 130, 255, cv2.THRESH_BINARY_INV)
    else:  # Just Grayscale
        processed = gray
        
    return processed

def load_master_ingredients(filepath="ingredients_list.txt"):
    """Loads the master ingredient list from a file."""
    try:
        with open(filepath, 'r') as f:
            # Read lines, strip whitespace, and filter out empty lines
            ingredients = [line.strip().lower() for line in f if line.strip()]
        print(f"Loaded {len(ingredients)} ingredients from {filepath}")
        return set(ingredients)
    except FileNotFoundError:
        print(f"Warning: Master ingredient list '{filepath}' not found. No ingredients will be detected.")
        return set()

def extract_ingredients(text, master_list, similarity_threshold=70):
    """
    Finds ingredients from a master list within the OCR text using fuzzy matching.
    """
    if not master_list:
        return "Master ingredient list is empty."

    # Clean and normalize the OCR text, then split into individual words
    ocr_text_lower = text.lower()
    ocr_words = re.sub(r'[^\w\s]', ' ', ocr_text_lower).split()
    
    found_ingredients = set() # Use a set to avoid duplicate matches
    
    # For multi-word ingredients in the master list
    master_multi_word = {ing for ing in master_list if ' ' in ing}
    # For single-word ingredients
    master_single_word = master_list - master_multi_word

    # Check for multi-word ingredients first
    for master_ing in master_multi_word:
        # Use partial ratio for phrases which might be found within the text block
        if fuzz.partial_ratio(master_ing, ocr_text_lower) > similarity_threshold + 10: # Higher threshold for partials
            found_ingredients.add(master_ing.capitalize())

    # Check for single-word ingredients
    for ocr_word in ocr_words:
        if len(ocr_word) < 3: # Ignore very short, likely noisy words
            continue
        for master_ing in master_single_word:
            # Compare the OCR word against the master list
            similarity = fuzz.ratio(ocr_word, master_ing)
            if similarity >= similarity_threshold:
                found_ingredients.add(master_ing.capitalize())
                break # Move to the next OCR word once a match is found
            
    if found_ingredients:
        return ", ".join(sorted(list(found_ingredients)))
    
    return "No known ingredients detected."

def main():
    """
    Main function to capture video, perform OCR, and display results.
    """
    master_ingredients = load_master_ingredients()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    roi = None
    paused_frame = None
    preprocessing_mode = 0
    modes = ["Adaptive", "Simple", "Otsu", "Dilated", "Grayscale"]
    
    print("\n--- Instructions ---")
    print("Press 'r' to select a Region of Interest (ROI).")
    print("Press 's' to scan the current frame (or ROI).")
    print("Press 'p' to cycle preprocessing modes.")
    print("Press 'c' to clear the ROI.")
    print("Press 'q' to quit.")
    print("--------------------\n")

    while True:
        if paused_frame is None:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame.")
                break
        else:
            frame = paused_frame.copy()

        display_frame = frame.copy()

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('p'):
            preprocessing_mode = (preprocessing_mode + 1) % len(modes)
            print(f"Switched to preprocessing mode: {modes[preprocessing_mode]}")
            paused_frame = None # Unpause to see the new mode effect
        elif key == ord('r'):
            # Select ROI
            cv2.destroyAllWindows() # Close the current window to select ROI on a new one
            roi = cv2.selectROI("Select ROI, then press ENTER or SPACE", frame, fromCenter=False, showCrosshair=True)
            cv2.destroyWindow("Select ROI, then press ENTER or SPACE")
            print(f"ROI selected at {roi}")
            paused_frame = None # Unpause after selecting ROI
        elif key == ord('s'):
            # Scan the current frame
            paused_frame = frame.copy()
            print("\n--- Scanning Frame ---")
            
            scan_area = paused_frame
            if roi and roi[2] > 0 and roi[3] > 0:
                # If ROI is defined, use it
                x, y, w, h = roi
                scan_area = paused_frame[y:y+h, x:x+w]

            # Pre-process and perform OCR
            processed_image = preprocess_image(scan_area, mode=preprocessing_mode)
            try:
                # To see the preprocessed image for debugging, uncomment the next line
                cv2.imshow("Processed", processed_image)
                custom_config = r'--oem 3 --psm 6'
                text = pytesseract.image_to_string(Image.fromarray(processed_image), config=custom_config)

                print("--- Raw OCR Output (All text found in image) ---")
                if text.strip():
                    print(text)
                else:
                    print("(No text detected)")
                print("-------------------------------------------------")

                ingredients = extract_ingredients(text, master_ingredients)
                print("--- Detected Ingredients (from master list) ---")
                print(ingredients)
                print("-------------------------------------------------\n")

            except pytesseract.TesseractNotFoundError:
                print("Tesseract Error: Tesseract is not installed or not in your PATH.")
                break
            except Exception as e:
                print(f"An error occurred: {e}")
            
            # Unpause after scanning
            paused_frame = None

        elif key == ord('c'):
            # Clear ROI
            roi = None
            print("ROI cleared.")

        # Draw ROI on the display frame
        if roi and roi[2] > 0 and roi[3] > 0:
            x, y, w, h = roi
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(display_frame, "ROI", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display current mode
        mode_text = f"Mode: {modes[preprocessing_mode]}"
        cv2.putText(display_frame, mode_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('Webcam Feed', display_frame)

    cap.release()
    cv2.destroyAllWindows()
    print("Script finished.")

if __name__ == "__main__":
    main()
