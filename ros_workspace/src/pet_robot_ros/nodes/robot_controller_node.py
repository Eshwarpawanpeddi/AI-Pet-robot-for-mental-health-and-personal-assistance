#!/usr/bin/env python3
"""
ROS Node for Robot Controller
Integrates with existing Raspberry Pi controller
"""

import rospy
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../hardware/raspberry_pi'))

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pet_robot_ros.msg import RobotState, SensorData, Emotion
from pet_robot_ros.srv import SetEmotion, SetEmotionResponse

try:
    from raspberry_pi_controller import RaspberryPiController, Direction
    HAS_HARDWARE = True
except ImportError:
    rospy.logwarn("Raspberry Pi hardware not available, running in simulation mode")
    HAS_HARDWARE = False


class RobotControllerNode:
    """Main robot controller ROS node"""
    
    def __init__(self):
        rospy.init_node('robot_controller_node', anonymous=False)
        
        # Initialize hardware controller if available
        self.controller = None
        if HAS_HARDWARE:
            try:
                self.controller = RaspberryPiController()
                rospy.loginfo("Hardware controller initialized")
            except Exception as e:
                rospy.logerr(f"Failed to initialize hardware: {e}")
        
        # Robot state
        self.current_emotion = Emotion()
        self.current_emotion.emotion = "neutral"
        self.current_emotion.intensity = 0.5
        self.current_emotion.timestamp = rospy.Time.now()
        
        # Publishers
        self.state_pub = rospy.Publisher('robot/state', RobotState, queue_size=10)
        self.emotion_pub = rospy.Publisher('robot/emotion', Emotion, queue_size=10)
        
        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        self.emotion_sub = rospy.Subscriber('robot/set_emotion', String, self.emotion_callback)
        
        # Services
        self.set_emotion_srv = rospy.Service('robot/set_emotion_srv', SetEmotion, self.handle_set_emotion)
        
        # Timer for publishing state
        self.state_timer = rospy.Timer(rospy.Duration(0.1), self.publish_state)
        
        rospy.loginfo("Robot Controller Node started")
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        if not self.controller:
            rospy.logdebug(f"Received cmd_vel (simulation): linear={msg.linear.x}, angular={msg.angular.z}")
            return
        
        try:
            linear = msg.linear.x
            angular = msg.angular.z
            
            # Convert to motor commands
            if abs(linear) < 0.01 and abs(angular) < 0.01:
                self.controller.stop()
            elif linear > 0 and abs(angular) < 0.1:
                speed = int(min(abs(linear) * 255, 255))
                self.controller.move_forward(speed)
            elif linear < 0 and abs(angular) < 0.1:
                speed = int(min(abs(linear) * 255, 255))
                self.controller.move_backward(speed)
            elif angular > 0:
                speed = int(min(abs(angular) * 200, 255))
                self.controller.turn_left(speed)
            elif angular < 0:
                speed = int(min(abs(angular) * 200, 255))
                self.controller.turn_right(speed)
        except Exception as e:
            rospy.logerr(f"Error executing motor command: {e}")
    
    def emotion_callback(self, msg):
        """Handle emotion updates"""
        self.current_emotion.emotion = msg.data
        self.current_emotion.timestamp = rospy.Time.now()
        rospy.loginfo(f"Emotion changed to: {msg.data}")
    
    def handle_set_emotion(self, req):
        """Service handler for setting emotion"""
        try:
            self.current_emotion.emotion = req.emotion
            self.current_emotion.intensity = req.intensity
            self.current_emotion.timestamp = rospy.Time.now()
            
            return SetEmotionResponse(
                success=True,
                message=f"Emotion set to {req.emotion}"
            )
        except Exception as e:
            return SetEmotionResponse(
                success=False,
                message=f"Error: {str(e)}"
            )
    
    def publish_state(self, event):
        """Publish robot state periodically"""
        state = RobotState()
        state.emotion = self.current_emotion
        state.is_speaking = False
        state.is_listening = True
        state.battery_level = 100
        
        # Get sensor data if hardware available
        if self.controller:
            try:
                sensor_dict = self.controller.get_sensor_data()
                state.sensor_data.touch_detected = sensor_dict.get('touch_detected', False)
                state.sensor_data.distance = sensor_dict.get('distance', 0.0)
                state.sensor_data.temperature = sensor_dict.get('temperature', 25.0)
                state.sensor_data.timestamp = rospy.Time.now()
                state.battery_level = sensor_dict.get('battery_level', 100)
            except Exception as e:
                rospy.logwarn_throttle(10, f"Error reading sensors: {e}")
        
        self.state_pub.publish(state)
        self.emotion_pub.publish(self.current_emotion)
    
    def shutdown(self):
        """Cleanup on shutdown"""
        rospy.loginfo("Shutting down robot controller...")
        if self.controller:
            try:
                self.controller.stop()
                self.controller.cleanup()
            except Exception as e:
                rospy.logerr(f"Error during cleanup: {e}")


if __name__ == '__main__':
    try:
        node = RobotControllerNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
