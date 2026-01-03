# Complete Setup Guide - Step by Step

This guide will take you through everything from installing ROS2 to running your hand gesture controlled robot.

## Prerequisites Check

Before starting, make sure you have:
- [ ] Ubuntu 24.04 (or macOS for development, but ROS2 runs on Ubuntu)
- [ ] Internet connection
- [ ] At least 10GB free disk space
- [ ] Webcam (for testing hand gestures)

---

## Part 1: Install ROS2 Jazzy (30-45 minutes)

### Step 1.1: Update System

```bash
sudo apt update
sudo apt upgrade -y
```

### Step 1.2: Set Locale

```bash
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 1.3: Add ROS2 Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 1.4: Install ROS2 Jazzy Desktop

```bash
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

This will take 10-20 minutes depending on your internet speed.

### Step 1.5: Source ROS2

```bash
source /opt/ros/jazzy/setup.bash
```

**Important:** Add this to your `~/.bashrc` so it runs automatically:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### Step 1.6: Install Additional Tools

```bash
sudo apt install python3-argcomplete python3-colcon-common-extensions -y
```

### Step 1.7: Verify Installation

```bash
# Open a new terminal (or source bashrc)
source ~/.bashrc

# Check ROS2 version
ros2 --version

# List available packages
ros2 pkg list | head -20
```

You should see output showing ROS2 is installed. If you see errors, check the installation steps again.

---

## Part 2: Install Python Dependencies (5 minutes)

### Step 2.1: Install pip if not already installed

```bash
sudo apt install python3-pip -y
```

### Step 2.2: Install Required Python Packages

```bash
pip3 install numpy opencv-python mediapipe
```

**Note:** If you get permission errors, use:
```bash
pip3 install --user numpy opencv-python mediapipe
```

### Step 2.3: Verify Installation

```bash
python3 -c "import cv2; import numpy; import mediapipe; print('All packages installed!')"
```

You should see "All packages installed!" without errors.

---

## Part 3: Set Up Your Workspace (5 minutes)

### Step 3.1: Create ROS2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Step 3.2: Add Your Packages

```bash
cd ~/ros2_ws/src

# Symlink your repositories (adjust paths as needed)
ln -s /Users/hasancoban/Desktop/repos/hand-gesture-robot hand_gesture_robot
ln -s /Users/hasancoban/Desktop/repos/HandRecognition hand_recognition
```

**Note:** If you're on macOS, you'll need to copy the files or use a different method. On Ubuntu, adjust the paths accordingly.

### Step 3.3: Build the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

This will compile your packages. You should see:
```
Summary: X packages finished [X.Xs]
```

### Step 3.4: Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

Add this to your `~/.bashrc`:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Part 4: Test Hand Gesture System (10 minutes)

### Step 4.1: Check Camera Access

```bash
# List video devices
ls /dev/video*

# On macOS (if testing there first)
# System Preferences → Security & Privacy → Camera → Allow terminal access
```

### Step 4.2: Test Hand Recognition Node

```bash
# Make sure ROS2 and workspace are sourced
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Run the hand gesture node
ros2 run hand_recognition hand_drive_node
```

**What you should see:**
- A camera window opens showing your webcam feed
- Hand landmarks appear when you show your hands
- Status text showing "Drive DISABLED" or "Drive ENABLED"
- Speed and direction values

**If you see errors:**
- Camera not found: Check camera permissions and device ID
- Import errors: Make sure you installed Python packages
- ROS2 errors: Verify ROS2 is sourced correctly

### Step 4.3: Test ROS2 Topics (in another terminal)

```bash
# Terminal 2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# List topics
ros2 topic list

# You should see:
# /cmd_vel
# /drive_enabled

# Monitor cmd_vel
ros2 topic echo /cmd_vel
```

**What you should see:**
- When you make gestures, you'll see Twist messages with linear.x and angular.z values
- Values change as you move your hands

### Step 4.4: Test with Launch File

```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch hand_gesture_robot full_system.launch.py
```

This should work the same as running the node directly.

---

## Part 5: Troubleshooting Common Issues

### Issue: "ros2: command not found"

**Solution:**
```bash
source /opt/ros/jazzy/setup.bash
# Or add to ~/.bashrc and restart terminal
```

### Issue: "Package not found" or "No executable found"

**Solution:**
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Issue: Camera not opening

**Solution:**
```bash
# Check camera device
ls /dev/video*

# Try different camera ID
ros2 launch hand_gesture_robot hand_control.launch.py camera_id:=1
```

### Issue: Python import errors

**Solution:**
```bash
# Reinstall packages
pip3 install --user --upgrade numpy opencv-python mediapipe

# Check Python version (should be 3.10+)
python3 --version
```

### Issue: Permission denied for /dev/ttyUSB0 (for future ESP32)

**Solution:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

---

## Part 6: Verify Everything Works

### Checklist:

- [ ] ROS2 Jazzy installed and sourced
- [ ] Python packages installed (numpy, opencv, mediapipe)
- [ ] Workspace built successfully
- [ ] Hand gesture node runs without errors
- [ ] Camera window opens and shows video
- [ ] Hand detection works (landmarks appear)
- [ ] `/cmd_vel` topic publishes messages
- [ ] Values change when making gestures

### Test Commands:

```bash
# 1. Check ROS2
ros2 --version

# 2. Check packages
ros2 pkg list | grep hand

# 3. Check nodes
ros2 node list

# 4. Check topics
ros2 topic list
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
```

---

## Next Steps

Once Part 1-4 are working:

1. **Test thoroughly** - Make sure hand gestures work correctly
2. **Calibrate** - Adjust speed/direction sensitivity if needed
3. **Then move to hardware** - See HARDWARE_SETUP.md for ESP32/Arduino setup

**Don't move to hardware until the software is working perfectly!**

---

## Getting Help

If you're stuck:

1. Check error messages carefully
2. Verify each step was completed
3. Check ROS2 is sourced in every terminal
4. Make sure workspace is built and sourced
5. Test with simple ROS2 commands first

Common mistakes:
- Forgetting to source ROS2 in new terminals
- Not building the workspace after changes
- Camera permissions not granted
- Python packages not installed

