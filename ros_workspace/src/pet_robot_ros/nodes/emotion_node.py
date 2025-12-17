#!/usr/bin/env python3
"""
ROS Node for Emotion Processing
Monitors robot state and manages emotion transitions
"""

import rospy
from pet_robot_ros.msg import Emotion, RobotState
from std_msgs.msg import String


class EmotionNode:
    """Emotion processing ROS node"""
    
    def __init__(self):
        rospy.init_node('emotion_node', anonymous=False)
        
        # Current emotion state
        self.current_emotion = "neutral"
        self.emotion_intensity = 0.5
        
        # Publishers
        self.emotion_pub = rospy.Publisher('robot/emotion_state', Emotion, queue_size=10)
        
        # Subscribers
        self.state_sub = rospy.Subscriber('robot/state', RobotState, self.state_callback)
        self.command_sub = rospy.Subscriber('robot/emotion_command', String, self.command_callback)
        
        # Timer for emotion updates
        self.update_timer = rospy.Timer(rospy.Duration(1.0), self.update_emotion)
        
        rospy.loginfo("Emotion Node started")
    
    def state_callback(self, msg):
        """Handle robot state updates"""
        # Could implement emotion changes based on state
        pass
    
    def command_callback(self, msg):
        """Handle emotion command updates"""
        self.current_emotion = msg.data
        rospy.loginfo(f"Emotion command received: {msg.data}")
    
    def update_emotion(self, event):
        """Publish current emotion periodically"""
        emotion_msg = Emotion()
        emotion_msg.emotion = self.current_emotion
        emotion_msg.intensity = self.emotion_intensity
        emotion_msg.timestamp = rospy.Time.now()
        
        self.emotion_pub.publish(emotion_msg)


if __name__ == '__main__':
    try:
        node = EmotionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
