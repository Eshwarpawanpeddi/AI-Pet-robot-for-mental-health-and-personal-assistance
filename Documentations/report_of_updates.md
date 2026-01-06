AI Pet Robot: Repository Update & Challenge Report
1. Core Project Configuration (The Source of Truth)
Android Package Name: com.petrobot.ai_pet_robot

JVM Target Version: 21 (Java 21 / Kotlin 21)

Android Gradle Plugin (AGP): 8.11.1

Kotlin Version: 2.2.20

ROS Distribution: Melodic (Ubuntu 18.04 / Bionic)

Backend Server: FastAPI (Python 3.10+) utilizing Google Gemini Pro.

2. Flutter Mobile App (Android)
Updates Performed:
Package Standardization: Aligned all references to the namespace com.petrobot.ai_pet_robot.

File System Realignment: Moved MainActivity.kt to the strictly required path: android/app/src/main/kotlin/com/petrobot/ai_pet_robot/.

Kotlin DSL Conversion: Fully converted build.gradle.kts and settings.gradle.kts from legacy Groovy syntax to proper Kotlin DSL (e.g., using id("...") instead of id "...").

Plugin Upgrade: Updated speech_to_text to ^7.4.0-beta.8 to ensure compatibility with modern Flutter "v2" embedding.

Manifest & Permissions: Added RECORD_AUDIO, INTERNET, and the RecognitionService <queries> tag to AndroidManifest.xml.

Challenges Faced:
Legacy "v1" Embedding: The speech_to_text plugin was throwing Unresolved reference 'Registrar' errors because it was using deprecated Flutter methods.

Path/Namespace Mismatch: The Kotlin compiler failed to resolve embedding and FlutterActivity because the folder structure (com/example/...) did not match the declared package name.

Groovy vs. Kotlin DSL Confusion: The build scripts used .kts extensions but contained Groovy code, causing over 20 script compilation errors.

JVM Target Conflicts: The Java compiler defaulted to 1.8 while Kotlin defaulted to 21, resulting in a build-stopping "Inconsistent JVM Target Compatibility" error.

3. ROS (Robot Operating System) Workspace
Updates Performed:
Distribution Alignment: Installed ROS Melodic to match the Ubuntu 18.04 environment in WSL2.

CMake Fixes: Removed references to a missing motor_control_node.py in CMakeLists.txt to allow catkin_make to complete.

Environment Cleanup: Sanitized ~/.bashrc to remove conflicting ROS Noetic paths.

Module Linking: Sourced the local devel/setup.bash to register custom Python action modules (BreathingExercise).

Challenges Faced:
OS/ROS Mismatch: Initial attempts to use ROS Noetic (Ubuntu 20.04) on an 18.04 system caused "Setup file not found" errors.

Missing Python Modules: Even after a 100% successful build, nodes threw ModuleNotFoundError: No module named 'pet_robot_ros.action'. This was solved by correctly sourcing the local workspace rather than just the global ROS environment.

Dependency Conflicts: Standalone catkin installations conflicted with the official ROS build tools, requiring a purge and re-installation from the ROS repositories.

4. System Architecture & Linking (For Copilot)
Connectivity Logic:
App → Server: The Flutter app connects to the Python server via WebSockets for real-time motor control and REST for mental health logs.

Server → ROS: The FastAPI server acts as a bridge, sending velocity commands to the ROS /cmd_vel topic and triggering Mental Health ActionServers.

WSL2 Networking: Since the robot logic runs in WSL2, the Flutter app (on Windows/Physical Device) must use the WSL IP (hostname -I) configured in app_config.dart.

Node Responsibilities:
robot_controller_node.py: Subscribes to movement topics and translates them to I2C commands for the ESP12E.

mental_health_node.py: Manages the BreathingExercise action server and LogMood services.

emotion_node.py: Monitors sensor data to update the robot's emotional state.

5. Instructions for Future Development
Android: Any new Activity must remain in the com.petrobot.ai_pet_robot package. Do not change the JVM target from 21.

Gradle: Always use parentheses for plugins in .kts files: id("com.android.application").

ROS Nodes: Python nodes must use #!/usr/bin/env python for Melodic compatibility (Python 2.7) unless a specific Python 3 environment is activated.

Hardware: The ESP12E motor controller is at I2C address 8. All ROS velocity commands must scale to the 0-255 PWM range used in motor_controller.ino.