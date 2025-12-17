# ROS Integration for AI Pet Robot

## Overview

This package provides full ROS (Robot Operating System) integration for the AI Pet Robot mental health companion. It enables standardized robot control, mental health support features, and integration with the broader ROS ecosystem.

## Features

### Core ROS Capabilities
- **Robot Control**: Standard `cmd_vel` interface for movement
- **State Publishing**: Real-time robot state on ROS topics
- **Service-based API**: Mental health services (affirmations, mood logging)
- **Action Server**: Guided breathing exercises
- **Custom Messages**: Emotion, MoodLog, RobotState, SensorData

### Mental Health Integration
- Mood tracking with supportive responses
- Positive affirmation service
- Breathing exercise action server
- Crisis resource awareness
- Privacy-focused logging

## Prerequisites

### Software Requirements
- ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
- Python 3.6+
- catkin build tools

### Hardware Requirements (Optional)
- Raspberry Pi 4
- ESP12E microcontroller
- L298N motor driver
- Touch sensors, ultrasonic distance sensor

## Installation

### 1. Install ROS

For Ubuntu 20.04 (ROS Noetic):
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
```

### 2. Setup ROS Environment

```bash
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

### 3. Create and Build Workspace

```bash
# Navigate to the ros_workspace directory
cd ros_workspace

# Initialize workspace (if not already done)
catkin_make

# Source the workspace
source devel/setup.bash
echo "source ~/path/to/ros_workspace/devel/setup.bash" >> ~/.bashrc
```

### 4. Install Dependencies

```bash
# Install Python dependencies
sudo apt install python3-pip
pip3 install -r ../server/requirements.txt

# For Raspberry Pi hardware (on the Pi)
pip3 install -r ../hardware/raspberry_pi/requirements.txt
```

## Package Structure

```
pet_robot_ros/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── msg/                        # Custom message definitions
│   ├── Emotion.msg
│   ├── MoodLog.msg
│   ├── RobotState.msg
│   └── SensorData.msg
├── srv/                        # Service definitions
│   ├── GetAffirmation.srv
│   ├── LogMood.srv
│   └── SetEmotion.srv
├── action/                     # Action definitions
│   └── BreathingExercise.action
├── nodes/                      # ROS node implementations
│   ├── robot_controller_node.py
│   ├── mental_health_node.py
│   └── emotion_node.py
├── scripts/                    # Utility scripts
│   └── test_robot.py
├── launch/                     # Launch files
│   ├── pet_robot.launch
│   └── mental_health.launch
└── config/                     # Configuration files
    ├── robot_params.yaml
    └── mental_health_params.yaml
```

## Quick Start

### Launch All Nodes

```bash
# Source your workspace
source devel/setup.bash

# Launch the full robot system
roslaunch pet_robot_ros pet_robot.launch

# Or launch only mental health features
roslaunch pet_robot_ros mental_health.launch
```

### Basic Usage Examples

#### 1. Control Robot Movement

```bash
# Publish velocity commands
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# Stop the robot
rostopic pub /cmd_vel geometry_msgs/Twist "{}"
```

#### 2. Get a Positive Affirmation

```bash
# Call the affirmation service
rosservice call /mental_health/get_affirmation "{}"
```

#### 3. Log Your Mood

```bash
# Log mood with the service
rosservice call /mental_health/log_mood "mood: 'anxious'
intensity: 7
notes: 'Feeling worried about tomorrow'"
```

#### 4. Start Breathing Exercise

```bash
# Send action goal (requires actionlib)
rostopic pub /mental_health/breathing_exercise/goal pet_robot_ros/BreathingExerciseActionGoal "goal:
  exercise_type: 'box_breathing'
  duration_seconds: 64"
```

#### 5. Monitor Robot State

```bash
# Echo robot state topic
rostopic echo /robot/state
```

## ROS API Reference

### Topics

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/robot/state` | `RobotState` | Complete robot state (10 Hz) |
| `/robot/emotion` | `Emotion` | Current emotion state (10 Hz) |
| `/robot/emotion_state` | `Emotion` | Processed emotion (1 Hz) |
| `/mental_health/mood_log` | `MoodLog` | Mood entries when logged |
| `/mental_health/affirmation` | `String` | Affirmations when requested |

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Velocity commands for movement |
| `/robot/set_emotion` | `String` | Emotion change commands |
| `/robot/emotion_command` | `String` | Emotion processing commands |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/robot/set_emotion_srv` | `SetEmotion` | Set robot's emotion and intensity |
| `/mental_health/get_affirmation` | `GetAffirmation` | Get a positive affirmation |
| `/mental_health/log_mood` | `LogMood` | Log mood with support response |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/mental_health/breathing_exercise` | `BreathingExercise` | Guided breathing exercise |

## Testing

### Run Test Scripts

```bash
# Test movement
rosrun pet_robot_ros test_robot.py movement

# Test affirmation service
rosrun pet_robot_ros test_robot.py affirmation

# Test mood logging
rosrun pet_robot_ros test_robot.py mood

# Monitor robot state
rosrun pet_robot_ros test_robot.py monitor
```

### Verify Installation

```bash
# Check if package is found
rospack find pet_robot_ros

# List all nodes
rospack list-names | grep pet_robot

# Check message definitions
rosmsg show pet_robot_ros/Emotion
rosmsg show pet_robot_ros/RobotState

# Check service definitions
rossrv show pet_robot_ros/LogMood
```

## Integration with Existing System

The ROS nodes integrate seamlessly with the existing Python/FastAPI server:

1. **Hardware Control**: Uses the existing `raspberry_pi_controller.py`
2. **Mental Health Features**: Mirrors the HTTP API as ROS services
3. **State Synchronization**: Can publish state to both ROS topics and WebSocket

### Running Both Systems Together

```bash
# Terminal 1: Start ROS nodes
roslaunch pet_robot_ros pet_robot.launch

# Terminal 2: Start FastAPI server
cd server
python server.py

# Both systems can coexist and share the hardware controller
```

## Configuration

### Robot Parameters (`config/robot_params.yaml`)

```yaml
robot:
  name: "AI_Pet_Robot"
  motor:
    max_speed: 255
    default_speed: 200
  sensors:
    touch:
      enabled: true
    distance:
      enabled: true
      max_range: 400  # cm
```

### Mental Health Parameters (`config/mental_health_params.yaml`)

```yaml
mood_tracking:
  enabled: true
  log_to_file: true

crisis_detection:
  enabled: true
  keywords:
    - "want to kill myself"
    - "want to end it all"
```

## Troubleshooting

### Package Not Found

```bash
# Make sure workspace is built
cd ~/ros_workspace
catkin_make

# Source the workspace
source devel/setup.bash
```

### Python Import Errors

```bash
# Install missing dependencies
pip3 install -r ../server/requirements.txt
pip3 install -r ../hardware/raspberry_pi/requirements.txt
```

### Hardware Not Available

The nodes will run in simulation mode if hardware is not available. Check logs:

```bash
# View node logs
rosnode list
rosnode info /robot_controller

# Check for warnings
rostopic echo /rosout | grep WARN
```

## Advanced Usage

### Custom Launch Configuration

Create your own launch file:

```xml
<launch>
  <arg name="use_hardware" default="false"/>
  
  <node name="robot_controller" pkg="pet_robot_ros" 
        type="robot_controller_node.py" output="screen">
    <param name="use_hardware" value="$(arg use_hardware)"/>
  </node>
</launch>
```

### Integration with RViz

Visualize robot state in RViz (requires additional setup for 3D model).

### Integration with MoveIt

For advanced motion planning (future enhancement).

## Development

### Adding New Messages

1. Create `.msg` file in `msg/` directory
2. Add to `CMakeLists.txt` under `add_message_files()`
3. Rebuild: `catkin_make`
4. Source: `source devel/setup.bash`

### Adding New Services

1. Create `.srv` file in `srv/` directory
2. Add to `CMakeLists.txt` under `add_service_files()`
3. Rebuild and source

### Creating New Nodes

1. Add Python script to `nodes/` directory
2. Make executable: `chmod +x nodes/your_node.py`
3. Add to `CMakeLists.txt` under `catkin_install_python()`
4. Rebuild

## Contributing

Contributions are welcome! Please ensure:
- Code follows ROS Python style guide
- New nodes include proper logging
- Services and actions are documented
- Launch files are provided for new features

## Resources

- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [ROS Best Practices](http://wiki.ros.org/BestPractices)
- [actionlib Documentation](http://wiki.ros.org/actionlib)
- [Original Project README](../README.md)

## License

MIT License - See LICENSE file for details

## Support

For issues or questions:
- GitHub Issues: [Repository Issues](https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance/issues)
- ROS Answers: Tag with `pet_robot_ros`

---

**Note**: This ROS package complements the existing FastAPI/WebSocket server and can be used independently or together for maximum flexibility.
