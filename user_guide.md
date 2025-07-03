# Juno Robot Allergen Scanner User Guide

This guide provides step-by-step instructions for setting up and running the allergen scanner ROS package on your Juno robot.

## Prerequisites

Before you begin, ensure your Juno robot has the following installed:

*   ROS (Robot Operating System)
*   Git
*   Python 3 and pip
*   The following Python libraries:
    *   `speech_recognition`
    *   `pyttsx3`
    *   `google-cloud-vision`
    *   `opencv-python`
    *   `thefuzz`
    *   `nltk`

You can install the Python libraries using pip:

```bash
pip install speechrecognition pyttsx3 google-cloud-vision opencv-python thefuzz nltk
```

You will also need to download the NLTK data for the lemmatizer:
```python
import nltk
nltk.download('wordnet')
```

## Setup and Installation

Follow these steps to set up the allergen scanner package on your robot:

### 1. Create a Catkin Workspace

If you don't already have a catkin workspace, create one:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

### 2. Clone the Repository

Clone the repository into the `src` directory of your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone <your-repository-url> .
```
*Replace `<your-repository-url>` with the actual URL of your GitHub repository.*

### 3. Build the Package

Navigate to the root of your catkin workspace and build the package:

```bash
cd ~/catkin_ws
catkin_make
```

### 4. Source the Workspace

After the build is complete, source the `setup.bash` file to add the workspace to your ROS environment:

```bash
source ~/catkin_ws/devel/setup.bash
```

**Note:** You should add this command to your `~/.bashrc` file to avoid having to source it in every new terminal session.

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Running the Nodes

To launch all the nodes in the allergen scanner package, use the provided launch file:

```bash
roslaunch allergen_scanner juno.launch
```

This command will start all the necessary nodes, including the speech interface, vision system, and allergen manager. The robot should now be ready to accept voice commands and perform allergen scans.

## How to Use

1.  **Activate the Robot**: Say "Hello" to activate the robot's listening mode.
2.  **Add Allergies**: Use commands like "add allergy peanuts" to add allergens to your profile.
3.  **Scan Products**: Say "scan" to have the robot scan a product for allergens.
4.  **Check Profile**: Ask "what are my allergies" to check your current allergy profile.
