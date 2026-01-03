# Complete ROS Setup and Implementation Guide

This guide provides step-by-step instructions for setting up and using ROS (Robot Operating System) with the AI Pet Robot for autonomous navigation and control.

## Table of Contents
1. [What is ROS?](#what-is-ros)
2. [Prerequisites](#prerequisites)
3. [Installing ROS](#installing-ros)
4. [Building the Workspace](#building-the-workspace)
5. [Running ROS Nodes](#running-ros-nodes)
6. [ROS Control Integration](#ros-control-integration)
7. [Testing and Verification](#testing-and-verification)
8. [Troubleshooting](#troubleshooting)

## What is ROS?

**ROS (Robot Operating System)** is a flexible framework for writing robot software. It provides:
- **Hardware abstraction**: Control motors, sensors without worrying about hardware details
- **Device drivers**: Pre-built drivers for common robotics hardware
- **Communication**: Message passing between different robot components
- **Navigation**: Autonomous movement, obstacle avoidance, path planning
- **Visualization**: Tools to see what your robot is doing (RViz)

### How ROS Works with This Robot

```
┌─────────────────────────────────────────────────────────────┐
│                      ROS SYSTEM                              │
│                                                               │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────┐  │
│  │ Navigation   │ ───▶ │ ROS Bridge   │ ───▶ │ WebSocket│  │
│  │ Stack        │      │ (Port 8000)  │      │ Server   │  │
│  └──────────────┘      └──────────────┘      └──────────┘  │
│         │                                            │        │
│         │ /cmd_vel                                   │        │
│         │ (velocity commands)                        │        │
│         ▼                                            ▼        │
│  ┌──────────────┐                          ┌──────────────┐ │
│  │ Motor        │                          │ Raspberry Pi │ │
│  │ Controller   │ ◀────────────────────────│ Hardware     │ │
│  └──────────────┘                          └──────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

ROS can **take full control** of the robot for autonomous operation while still allowing manual control when needed.

## Prerequisites

### Hardware Requirements
- Ubuntu 20.04 (for ROS Noetic) or Ubuntu 18.04 (for ROS Melodic)
- At least 4GB RAM (8GB recommended)
- 10GB free disk space

### Software Requirements
- Python 3.8+ (for ROS Noetic)
- Git
- CMake and build tools

## Installing ROS

### Option 1: ROS Noetic (Ubuntu 20.04) - RECOMMENDED

```bash
# 1. Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2. Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 3. Update package index
sudo apt update

# 4. Install ROS Noetic Desktop-Full (includes RViz, rqt, robot navigation, etc.)
sudo apt install ros-noetic-desktop-full

# 5. Install additional dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# 6. Initialize rosdep
sudo rosdep init
rosdep update

# 7. Setup environment (add to ~/.bashrc for persistence)
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Option 2: ROS Melodic (Ubuntu 18.04)

```bash
# 1. Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2. Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 3. Update and install
sudo apt update
sudo apt install ros-melodic-desktop-full

# 4. Install dependencies
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# 5. Initialize rosdep
sudo rosdep init
rosdep update

# 6. Setup environment
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify Installation

```bash
# Check ROS installation
rosversion -d
# Should output: noetic (or melodic)

# Check if roscore can start
roscore &
# Wait a few seconds, then press Ctrl+C to stop it
```

## Building the Workspace

### 1. Navigate to ROS Workspace

```bash
cd /path/to/AI-Pet-robot-for-mental-health-and-personal-assistance/ros_workspace
```

### 2. Install Dependencies

```bash
# Install Python dependencies for ROS nodes
pip3 install -r src/pet_robot_ros/requirements.txt

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
# Clean previous builds (optional, recommended for first time)
rm -rf build/ devel/

# Build with catkin_make
catkin_make

# If you get errors, try:
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### 4. Source the Workspace

```bash
# Source the workspace setup
source devel/setup.bash

# Add to ~/.bashrc for automatic sourcing (optional)
echo "source /path/to/ros_workspace/devel/setup.bash" >> ~/.bashrc
```

### 5. Verify Build

```bash
# List available ROS packages
rospack list | grep pet_robot

# Should show:
# pet_robot_ros /path/to/ros_workspace/src/pet_robot_ros
```

## Running ROS Nodes

### Starting the ROS Core

ROS requires a **roscore** to be running for all nodes to communicate:

```bash
# Terminal 1: Start roscore
roscore
```

Keep this terminal open. All ROS nodes need roscore running.

### Launching the Pet Robot ROS Package

#### Option A: Launch All Nodes (Recommended)

```bash
# Terminal 2: Launch all pet robot nodes
roslaunch pet_robot_ros pet_robot.launch
```

This starts:
- Mental health node (affirmations, mood logging, breathing exercises)
- WebSocket bridge node (connects ROS to port 8000)
- Motor controller node (translates ROS commands to motor controls)

#### Option B: Launch Only Mental Health Features

```bash
# Terminal 2: Launch only mental health nodes
roslaunch pet_robot_ros mental_health.launch
```

#### Option C: Launch Individual Nodes

```bash
# Terminal 2: Start mental health node
rosrun pet_robot_ros mental_health_node.py

# Terminal 3: Start websocket bridge
rosrun pet_robot_ros websocket_bridge_node.py

# Terminal 4: Start motor controller
rosrun pet_robot_ros motor_controller_node.py
```

### Starting the Main Server

```bash
# Terminal 3 (or new terminal): Start the main control server
cd server
python3 server.py
```

The server will automatically connect to ROS if it detects ROS nodes running.

## ROS Control Integration

### Understanding Control Modes

The robot supports two control modes:

1. **Manual Mode** (default)
   - Direct control via web interface or mobile app
   - Commands sent directly to Raspberry Pi
   - No ROS involvement

2. **Autonomous Mode** (ROS)
   - ROS navigation stack controls the robot
   - Autonomous path planning and obstacle avoidance
   - Can still be overridden for safety

### Switching Control Modes

#### Via Web Interface

1. Open `http://localhost:8000` in your browser
2. Look for "Control Mode" toggle
3. Switch between "Manual" and "Autonomous"

#### Via ROS Command

```bash
# Switch to autonomous mode
rostopic pub /control_mode std_msgs/String "data: 'autonomous'" -1

# Switch to manual mode
rostopic pub /control_mode std_msgs/String "data: 'manual'" -1
```

#### Via WebSocket (programmatically)

```python
import json
import websocket

ws = websocket.create_connection("ws://localhost:8000/ws/control")
ws.send(json.dumps({
    "type": "set_control_mode",
    "mode": "autonomous"
}))
```

### Sending Movement Commands via ROS

When in autonomous mode, ROS controls the robot:

```bash
# Move forward at 0.5 m/s
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

# Rotate (turn left)
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -r 10

# Stop (press Ctrl+C on the rostopic pub command)
```

### Using Mental Health Features via ROS

```bash
# Get a positive affirmation
rosservice call /mental_health/get_affirmation "{}"

# Log mood
rosservice call /mental_health/log_mood "mood: 'happy'
intensity: 8
notes: 'Feeling great today!'"

# Start breathing exercise
rosservice call /mental_health/start_breathing_exercise "duration: 300"

# Get mood history
rosservice call /mental_health/get_mood_history "{}"
```

## Testing and Verification

### 1. Check Active ROS Nodes

```bash
# List all running ROS nodes
rosnode list

# Should show something like:
# /mental_health_node
# /websocket_bridge_node
# /motor_controller_node
```

### 2. Check ROS Topics

```bash
# List all topics
rostopic list

# Should include:
# /cmd_vel (movement commands)
# /robot_state (current robot state)
# /control_mode (manual/autonomous)
```

### 3. Monitor Topics

```bash
# Monitor velocity commands
rostopic echo /cmd_vel

# Monitor robot state
rostopic echo /robot_state
```

### 4. Check ROS Services

```bash
# List available services
rosservice list

# Should include:
# /mental_health/get_affirmation
# /mental_health/log_mood
# /mental_health/start_breathing_exercise
```

### 5. Test Motor Control

```bash
# Test forward movement
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.3}" -1

# Wait 2 seconds, then stop
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}" -1
```

### 6. Test WebSocket Bridge

Check server logs for:
```
[INFO] ROS bridge connected
[INFO] ROS control mode: autonomous
```

### 7. Visualize with RViz (Optional)

```bash
# Launch RViz for visualization
rosrun rviz rviz

# Or if you have a launch file:
roslaunch pet_robot_ros visualization.launch
```

## Advanced Features

### Setting Up Navigation Stack (Optional)

For full autonomous navigation with mapping:

```bash
# Install navigation stack
sudo apt install ros-noetic-navigation

# Install mapping tools
sudo apt install ros-noetic-slam-gmapping
sudo apt install ros-noetic-amcl
```

### Create a Map of Your Environment

```bash
# Start mapping
roslaunch pet_robot_ros mapping.launch

# Drive the robot around (use manual control)
# The map will be built automatically

# Save the map when done
rosrun map_server map_saver -f my_home_map
```

### Navigate Autonomously with Map

```bash
# Load map and start navigation
roslaunch pet_robot_ros navigation.launch map_file:=my_home_map.yaml

# Set navigation goal via RViz:
# 1. Open RViz
# 2. Click "2D Nav Goal" button
# 3. Click and drag on map to set goal
# Robot will autonomously navigate there!
```

## Troubleshooting

### Problem: "roscore: command not found"

**Solution:**
```bash
# Source ROS setup
source /opt/ros/noetic/setup.bash

# Or add to ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

### Problem: "No module named 'rospy'"

**Solution:**
```bash
# Make sure you're using Python 3
python3 --version

# Rebuild workspace with Python 3
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### Problem: "Unable to contact ROS master"

**Solution:**
```bash
# Make sure roscore is running
roscore

# Check ROS_MASTER_URI
echo $ROS_MASTER_URI
# Should be: http://localhost:11311
```

### Problem: "Package 'pet_robot_ros' not found"

**Solution:**
```bash
# Make sure workspace is built
cd ros_workspace
catkin_make

# Source workspace
source devel/setup.bash

# Verify
rospack find pet_robot_ros
```

### Problem: ROS Bridge Not Connecting to Server

**Solution:**
```bash
# Check server is running
curl http://localhost:8000/health

# Check WebSocket endpoint
curl -i -N -H "Connection: Upgrade" \
     -H "Upgrade: websocket" \
     http://localhost:8000/ws/ros

# Check server logs for connection errors
```

### Problem: Robot Not Responding to ROS Commands

**Solution:**
1. Check control mode:
   ```bash
   rostopic echo /control_mode
   ```
2. Make sure mode is "autonomous"
3. Check Raspberry Pi is connected to server
4. Verify motor controller node is running:
   ```bash
   rosnode list | grep motor
   ```

### Problem: Build Errors

**Solution:**
```bash
# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
cd ros_workspace
rm -rf build/ devel/
catkin_make

# If Python errors, specify Python 3
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Quick Reference Commands

```bash
# Start ROS
roscore

# Launch robot
roslaunch pet_robot_ros pet_robot.launch

# List nodes
rosnode list

# List topics
rostopic list

# Monitor topic
rostopic echo /cmd_vel

# Publish to topic
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}"

# Call service
rosservice call /mental_health/get_affirmation "{}"

# Check package
rospack find pet_robot_ros

# Build workspace
catkin_make

# Clean build
rm -rf build/ devel/ && catkin_make
```

## Understanding the ROS Architecture

### Nodes
- **mental_health_node**: Provides mental health support services
- **websocket_bridge_node**: Bridges ROS and web server
- **motor_controller_node**: Translates ROS commands to motor movements

### Topics
- **/cmd_vel**: Velocity commands (Twist messages)
- **/robot_state**: Current robot state
- **/control_mode**: Current control mode (manual/autonomous)
- **/camera/image_raw**: Camera feed (if available)

### Services
- **/mental_health/get_affirmation**: Get positive affirmation
- **/mental_health/log_mood**: Log current mood
- **/mental_health/get_mood_history**: Get mood history
- **/mental_health/start_breathing_exercise**: Start breathing exercise

### Messages
- **geometry_msgs/Twist**: Velocity commands
- **std_msgs/String**: Simple string messages
- **sensor_msgs/Image**: Camera images

## Next Steps

1. **Test basic ROS functionality** with the commands above
2. **Integrate with your web interface** - control mode switching
3. **Add sensors** - ultrasonic, IR, etc. as ROS nodes
4. **Implement navigation** - use navigation stack for autonomous movement
5. **Customize mental health features** - add more services and actions

## Additional Resources

- **ROS Wiki**: http://wiki.ros.org/
- **ROS Tutorials**: http://wiki.ros.org/ROS/Tutorials
- **ROS Answers**: https://answers.ros.org/
- **Pet Robot ROS Package README**: `ros_workspace/src/pet_robot_ros/README.md`

## Support

If you encounter issues:
1. Check the troubleshooting section above
2. Review ROS logs: `rosnode list` and `rosnode info <node_name>`
3. Check server logs for connection issues
4. Consult ROS documentation
5. Open an issue on GitHub with:
   - ROS version (`rosversion -d`)
   - Ubuntu version (`lsb_release -a`)
   - Error messages and logs
   - Steps to reproduce

---

**Remember**: ROS gives your robot **autonomous capabilities** while maintaining manual control options. Start simple, test thoroughly, and gradually add more complex features!
