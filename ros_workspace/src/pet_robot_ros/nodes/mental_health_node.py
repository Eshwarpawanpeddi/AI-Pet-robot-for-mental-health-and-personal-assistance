#!/usr/bin/env python3
"""
ROS Node for Mental Health Support Features
Provides mood tracking, affirmations, and crisis resources
"""

import rospy
import random
from std_msgs.msg import String
from pet_robot_ros.msg import MoodLog
from pet_robot_ros.srv import GetAffirmation, GetAffirmationResponse
from pet_robot_ros.srv import LogMood, LogMoodResponse
from pet_robot_ros.action import BreathingExercise
import actionlib


class MentalHealthNode:
    """Mental health support ROS node"""
    
    def __init__(self):
        rospy.init_node('mental_health_node', anonymous=False)
        
        # Affirmations list
        self.affirmations = [
            "You are stronger than you think.",
            "Every small step forward is progress.",
            "You deserve kindness and compassion, especially from yourself.",
            "It's okay to take things one day at a time.",
            "You are doing the best you can, and that's enough.",
            "Your feelings are valid.",
            "You have overcome challenges before, and you can do it again.",
            "Taking care of yourself is not selfish, it's necessary.",
            "You are worthy of love and support.",
            "Progress, not perfection."
        ]
        
        # Publishers
        self.mood_pub = rospy.Publisher('mental_health/mood_log', MoodLog, queue_size=10)
        self.affirmation_pub = rospy.Publisher('mental_health/affirmation', String, queue_size=10)
        
        # Services
        self.affirmation_srv = rospy.Service(
            'mental_health/get_affirmation',
            GetAffirmation,
            self.handle_get_affirmation
        )
        self.mood_srv = rospy.Service(
            'mental_health/log_mood',
            LogMood,
            self.handle_log_mood
        )
        
        # Action server for breathing exercises
        self.breathing_server = actionlib.SimpleActionServer(
            'mental_health/breathing_exercise',
            BreathingExercise,
            execute_cb=self.execute_breathing_exercise,
            auto_start=False
        )
        self.breathing_server.start()
        
        rospy.loginfo("Mental Health Node started")
    
    def handle_get_affirmation(self, req):
        """Service handler for getting affirmations"""
        affirmation = random.choice(self.affirmations)
        
        # Publish for subscribers
        self.affirmation_pub.publish(affirmation)
        
        return GetAffirmationResponse(
            affirmation=affirmation,
            timestamp=rospy.Time.now()
        )
    
    def handle_log_mood(self, req):
        """Service handler for logging mood"""
        # Create mood log message
        mood_log = MoodLog()
        mood_log.mood = req.mood
        mood_log.intensity = req.intensity
        mood_log.notes = req.notes
        mood_log.timestamp = rospy.Time.now()
        
        # Publish mood log
        self.mood_pub.publish(mood_log)
        
        # Generate supportive response
        responses = {
            "anxious": "I understand you're feeling anxious. Would you like to try some breathing exercises together?",
            "sad": "I'm here for you. Sometimes it helps to talk about what's on your mind. I'm listening.",
            "happy": "That's wonderful! I love seeing you happy. What's making you feel good today?",
            "stressed": "Stress can be overwhelming. Let's take a moment to breathe and focus on one thing at a time.",
            "tired": "It sounds like you need some rest. Have you been getting enough sleep?",
            "angry": "I hear that you're upset. It's okay to feel angry. Want to talk about what's bothering you?",
            "neutral": "Thanks for checking in. How can I support you today?"
        }
        
        suggestions_map = {
            "anxious": ["breathing_exercise", "grounding_technique", "talk_it_out"],
            "sad": ["talk_it_out", "positive_affirmation", "gentle_activity"],
            "stressed": ["breathing_exercise", "break_reminder", "prioritize_tasks"],
            "tired": ["rest_reminder", "hydration_check", "gentle_movement"]
        }
        
        response = responses.get(req.mood, responses["neutral"])
        suggestions = suggestions_map.get(req.mood, ["talk_it_out"])
        
        rospy.loginfo(f"Mood logged: {req.mood} (intensity: {req.intensity})")
        
        return LogMoodResponse(
            success=True,
            response=response,
            suggestions=suggestions
        )
    
    def execute_breathing_exercise(self, goal):
        """Execute breathing exercise action"""
        rospy.loginfo(f"Starting breathing exercise: {goal.exercise_type}")
        
        # Box breathing: 4 seconds in, 4 hold, 4 out, 4 hold
        steps = [
            "Let's do box breathing together.",
            "Breathe in through your nose for 4 seconds...",
            "Hold your breath for 4 seconds...",
            "Breathe out through your mouth for 4 seconds...",
            "Hold for 4 seconds...",
        ]
        
        feedback = BreathingExercise.Feedback()
        
        total_steps = len(steps) * 4  # Repeat 4 times
        current_step = 0
        
        rate = rospy.Rate(0.25)  # One step every 4 seconds
        
        for cycle in range(4):
            for i, instruction in enumerate(steps):
                if self.breathing_server.is_preempt_requested():
                    rospy.loginfo("Breathing exercise preempted")
                    self.breathing_server.set_preempted()
                    return
                
                feedback.current_step = current_step
                feedback.instruction = instruction
                feedback.progress = current_step / float(total_steps)
                self.breathing_server.publish_feedback(feedback)
                
                rospy.loginfo(f"Step {current_step + 1}/{total_steps}: {instruction}")
                
                current_step += 1
                rate.sleep()
        
        # Success
        result = BreathingExercise.Result()
        result.completed = True
        result.message = "Breathing exercise completed successfully!"
        self.breathing_server.set_succeeded(result)
        rospy.loginfo("Breathing exercise completed")


if __name__ == '__main__':
    try:
        node = MentalHealthNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
