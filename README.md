# Allergen Scanner

Juno is a voice-activated assistant built with ROS (Robot Operating System) that helps users identify potential allergens in food product ingredient lists. Users can manage a personal allergy profile and use a live camera feed to scan products. All interactions, including starting the scan and capturing the image, are controlled by voice commands.

---
## Features

* **Voice-Controlled Interface**: All functions are initiated through voice commands.
* **Personalized Allergy Profiles**: Add or remove specific allergens to track.
* **Live Camera Scanning**: A real-time camera view helps you position the product's ingredient list.
* **Voice-Activated Capture**: Say "capture" to take a picture of the ingredients, no keyboard required.
* **Google Cloud Vision OCR**: Utilizes powerful Optical Character Recognition to accurately read text from images.
* **Intelligent Allergen Detection**: Scans the recognized text for hundreds of keywords related to 14 major allergen groups.
* **Real-time Audio Feedback**: Juno speaks back to the user to confirm actions and report findings.

---
## System Architecture

The system consists of three main ROS nodes that communicate using topics and services:

1.  **`speech_interface_node` (The Ears and Mouth)**
    * Listens for an activation word ("hello") and subsequent commands.
    * Enters a special "camera mode" to listen for "capture" or "quit" during a scan.
    * Publishes recognized commands to other nodes.
    * Subscribes to text responses and converts them to speech using `pyttsx3`.

2.  **`allergen_manager_node` (The Brain)**
    * Manages the user's allergy profile (`add`, `remove`, `show`).
    * Orchestrates the scanning process by calling the vision service.
    * Controls the state of the `speech_interface_node` (e.g., switching to camera mode).
    * Analyzes OCR text to find potential allergens based on its keyword dictionary.
    * Publishes text responses for Juno to speak.

3.  **`interactive_vision_node` (The Eyes)**
    * Provides a ROS service (`/juno/interactive_scan`) that, when called, opens a camera view.
    * Listens for voice commands from the `speech_interface_node` to either capture an image or quit.
    * Performs OCR on the captured image using the Google Cloud Vision API.
    * Returns the detected text to the `allergen_manager_node`.

---
## Dependencies

### System Packages
* **ROS Noetic Ninjemys**: (Or another version, but code is tested on a Python 3 environment).
* **Python 3**: The scripts are written in Python 3.
* **PortAudio**: Required by the `PyAudio` library.
    ```bash
    sudo apt-get install portaudio19-dev
    ```

### Python Libraries
You can install these using `pip`:
```bash
pip install --upgrade google-cloud-vision SpeechRecognition pyttsx3 Pyaudio nltk

After installing `nltk`, you need to download the 'wordnet' corpus. Run this command in a terminal:

```bash
python3 -c "import nltk; nltk.download('wordnet')"

## Setup and Installation

### 1. Google Cloud Vision Credentials

1.  Follow the Google Cloud documentation to create a service account and download your JSON credentials key.
2.  Create a folder named `Keys` inside your `allergen_scanner` package directory (`catkin_ws/src/allergen_scanner/Keys`).
3.  Place your downloaded JSON key file inside this `Keys` folder.
4.  **Important**: Ensure the name of your key file matches the one specified in `interactive_vision_node.py` (e.g., `arats-ocr-1b7025967ee1.json`), or update the path in the script to match your filename.

### 2. ROS Workspace

1.  Place the `allergen_scanner` package inside the `src` folder of your Catkin workspace.
2.  Make sure all Python scripts in the `nodes` directory are executable:
    ```bash
    cd ~/catkin_ws/src/allergen_scanner/nodes/
    chmod +x *.py
    ```
3.  Build the workspace from its root directory:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

---
## How to Run

### 1. Launch the System

Open a terminal, source your workspace, and use the provided launch file:
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch allergen_scanner juno.launch

This will start all three nodes. You will see log messages in your terminal as the nodes initialize, calibrate the microphone, and connect to the Google Vision API.

### 2. Interact with Juno

Once you see the "Listening for activation word 'hello'..." message, you can start talking to Juno.

    Activate Juno: Say "Hello". Juno will respond with "Yes?".

    Add an Allergy: Say "Add allergy nuts" or "Add allergy milk". Juno will confirm what was added.

    Check Your Profile: Say "What are my allergies?".

    Start a Scan: Say "Scan product".

        Juno will say, "Okay, a camera window will open..."

        A window will appear showing your camera feed. Position the ingredient list in front of the camera.

        Say "Capture" to take the picture.

        Say "Quit" or "Cancel" to exit the camera view without scanning.

    Get Results: After capturing, Juno will analyze the text and report if any of your specified allergens were found. The system will then return to listening for the "hello" activation word.
