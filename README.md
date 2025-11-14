# Blueye BTS 2025 Tutorial – Setup and Run Guide

This README walks you through:

- Installing **ROS 2 Humble** and **Gazebo Garden ROS interface**
- Cloning and building the **blueye_bts_2025_tutorial** workspace
- Running all required nodes for the tutorial

Assumptions:

- You are on **Ubuntu 22.04** (ROS 2 Humble target).
- You will define or are given an environment script called `SOURCE_ENV` (for example, it may source `/opt/ros/humble/setup.bash` and set extra variables).

---

## 1. Install ROS 2 Humble (Ubuntu 22.04)

### 1.1 Set up locale

```bash
sudo apt update
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 1.2 Add required repositories

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
```bash
sudo apt update
sudo apt install curl gnupg lsb-release
```

```bash
sudo mkdir -p /usr/share/keyrings
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.3 Install ROS 2 Humble (desktop)

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### 1.4 Install Gazebo Garden ROS 2 integration

```bash
sudo apt update
sudo apt install ros-humble-ros-gzgarden
```

### 1.5 Install some packages

```bash
pip install pyquaternion
```

```bash
pip install pynput
```

```bash
sudo apt install ros-humble-ament-cmake
```

```bash
sudo apt install python3-colcon-common-extensions
```

## 2. Clone and Build blueye_bts_2025_tutorial

### 2.1 Clone the repository

```bash
git clone https://github.com/Urlaxle/blueye_bts_2025_tutorial.git
```

### 2.1 Source your environment and build

**SOURCE_ENV** should be a script or alias that sets up your ROS environment (for example, it may contain source _/opt/ros/humble/setup.bash_ plus any extra environment variables you need).

From inside _blueye_bts_2025_tutorial_:

```bash
source SOURCE_ENV
colcon build
```

After the build finishes:

```bash
source install/setup.bash
```

## 3. Running the Tutorial

You will run multiple ROS 2 commands, each in a separate terminal.

The pattern is:
1. Open a new terminal.
1. _cd_ into the workspace.
1. Run:
```bash
      source SOURCE_ENV
      source install/setup.bash
```
4.	Then run the specified ros2 command.


### 3.1 Terminal 1 – Gazebo + Blueye simulation


```bash
cd blueye_bts_2025_tutorial
source SOURCE_ENV
source install/setup.bash
ros2 launch gz_bringup blueye_bts.launch.py
```


### 3.2 Terminal 2 – Blueye simulator handler


```bash
cd blueye_bts_2025_tutorial
source SOURCE_ENV
source install/setup.bash
ros2 run bts_blueye_handler bts_blueye_simulator_handler
```

### 3.3 Terminal 3 – ArUco detector (simulator)

```bash
cd blueye_bts_2025_tutorial
source SOURCE_ENV
source install/setup.bash
ros2 run bts_aruco_detector bts_aruco_detector_simulator
```


### 3.4 Terminal 4 – Docking guidance node


```bash
cd blueye_bts_2025_tutorial

source SOURCE_ENV
source install/setup.bash
ros2 run bts_docking_guidance bts_docking_guidance
```

### 3.5 Terminal 2 – Blueye simulator handler


```bash
cd blueye_bts_2025_tutorial
source SOURCE_ENV
source install/setup.bash
ros2 run bts_docking_controller bts_docking_controller
```

### 3.6 Terminal 6 – Keyboard / keypoint controller

```bash
cd blueye_bts_2025_tutorial
source SOURCE_ENV
source install/setup.bash
ros2 run bts_keypoint_controller bts_keyboard_controller
```
Press X in the docking controller to enable the docking! 

