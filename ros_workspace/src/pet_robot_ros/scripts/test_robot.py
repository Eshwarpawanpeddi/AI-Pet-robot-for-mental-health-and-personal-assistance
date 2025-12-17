#!/usr/bin/env python3
"""
Test script for ROS Pet Robot
Demonstrates basic ROS functionality
"""

import rospy
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pet_robot_ros.srv import GetAffirmation, LogMood
from pet_robot_ros.msg import RobotState


def test_movement():
    """Test robot movement commands"""
    rospy.init_node('test_robot', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    rospy.sleep(1)  # Wait for publisher to connect
    
    print("Testing movement commands...")
    
    # Move forward
    cmd = Twist()
    cmd.linear.x = 0.5
    print("Moving forward...")
    pub.publish(cmd)
    rospy.sleep(2)
    
    # Stop
    cmd = Twist()
    print("Stopping...")
    pub.publish(cmd)
    rospy.sleep(1)
    
    # Turn left
    cmd = Twist()
    cmd.angular.z = 0.5
    print("Turning left...")
    pub.publish(cmd)
    rospy.sleep(2)
    
    # Stop
    cmd = Twist()
    pub.publish(cmd)
    print("Movement test complete!")


def test_affirmation():
    """Test affirmation service"""
    rospy.init_node('test_affirmation', anonymous=True)
    
    print("Waiting for affirmation service...")
    rospy.wait_for_service('mental_health/get_affirmation')
    
    try:
        get_affirmation = rospy.ServiceProxy('mental_health/get_affirmation', GetAffirmation)
        response = get_affirmation()
        print(f"\nAffirmation: {response.affirmation}")
        print(f"Timestamp: {response.timestamp}\n")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def test_mood_logging():
    """Test mood logging service"""
    rospy.init_node('test_mood', anonymous=True)
    
    print("Waiting for mood logging service...")
    rospy.wait_for_service('mental_health/log_mood')
    
    try:
        log_mood = rospy.ServiceProxy('mental_health/log_mood', LogMood)
        response = log_mood(
            mood="anxious",
            intensity=7,
            notes="Feeling a bit worried today"
        )
        
        print(f"\nMood logged successfully: {response.success}")
        print(f"Response: {response.response}")
        print(f"Suggestions: {', '.join(response.suggestions)}\n")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def monitor_state():
    """Monitor robot state"""
    rospy.init_node('monitor_state', anonymous=True)
    
    def callback(msg):
        print(f"\n--- Robot State ---")
        print(f"Emotion: {msg.emotion.emotion} (intensity: {msg.emotion.intensity})")
        print(f"Speaking: {msg.is_speaking}, Listening: {msg.is_listening}")
        print(f"Battery: {msg.battery_level}%")
        print(f"Touch detected: {msg.sensor_data.touch_detected}")
        print(f"Distance: {msg.sensor_data.distance} cm")
    
    rospy.Subscriber('robot/state', RobotState, callback)
    
    print("Monitoring robot state (Ctrl+C to exit)...")
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: test_robot.py [movement|affirmation|mood|monitor]")
        sys.exit(1)
    
    test_type = sys.argv[1]
    
    try:
        if test_type == 'movement':
            test_movement()
        elif test_type == 'affirmation':
            test_affirmation()
        elif test_type == 'mood':
            test_mood_logging()
        elif test_type == 'monitor':
            monitor_state()
        else:
            print(f"Unknown test type: {test_type}")
            print("Available tests: movement, affirmation, mood, monitor")
    except rospy.ROSInterruptException:
        pass
