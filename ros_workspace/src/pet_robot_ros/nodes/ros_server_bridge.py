#!/usr/bin/env python3
"""
ROS-Server Bridge
Connects ROS navigation/control with the FastAPI server for autonomous mode
"""

import rospy
import asyncio
import websockets
import json
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class ROSServerBridge:
    """Bridge between ROS and FastAPI server"""
    
    def __init__(self, server_url="ws://localhost:8000/ws/ros"):
        rospy.init_node('ros_server_bridge', anonymous=False)
        
        self.server_url = server_url
        self.websocket = None
        self.running = False
        self.autonomous_mode = False
        
        # ROS Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.emotion_pub = rospy.Publisher('robot/set_emotion', String, queue_size=10)
        
        # ROS Subscribers
        self.state_sub = rospy.Subscriber('robot/state', String, self.state_callback)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        
        rospy.loginfo("ROS-Server Bridge initialized")
    
    def state_callback(self, msg):
        """Forward robot state to server"""
        if self.websocket and self.running:
            asyncio.run(self.send_to_server({
                'type': 'ros_state',
                'data': msg.data
            }))
    
    def scan_callback(self, msg):
        """Process laser scan for obstacle avoidance"""
        if not self.autonomous_mode:
            return
        
        # Simple obstacle avoidance
        min_distance = min(msg.ranges[len(msg.ranges)//3:2*len(msg.ranges)//3])
        
        if min_distance < 0.5:  # Less than 50cm
            rospy.loginfo("Obstacle detected, stopping")
            self.stop_robot()
    
    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    async def send_to_server(self, data):
        """Send data to server"""
        try:
            if self.websocket:
                await self.websocket.send(json.dumps(data))
        except Exception as e:
            rospy.logerr(f"Error sending to server: {e}")
    
    async def handle_server_message(self, message):
        """Handle messages from server"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'set_mode':
                mode = data.get('mode', 'manual')
                self.autonomous_mode = (mode == 'autonomous')
                rospy.loginfo(f"Mode changed to: {mode}")
                
            elif msg_type == 'move' and not self.autonomous_mode:
                # Only accept manual commands when not in autonomous mode
                direction = data.get('direction', 'stop')
                speed = data.get('speed', 0) / 100.0  # Convert to 0-1 range
                
                twist = Twist()
                if direction == 'forward':
                    twist.linear.x = speed
                elif direction == 'backward':
                    twist.linear.x = -speed
                elif direction == 'left':
                    twist.angular.z = speed
                elif direction == 'right':
                    twist.angular.z = -speed
                
                self.cmd_vel_pub.publish(twist)
                
            elif msg_type == 'emotion':
                emotion = data.get('emotion', 'neutral')
                self.emotion_pub.publish(String(data=emotion))
                
        except Exception as e:
            rospy.logerr(f"Error handling server message: {e}")
    
    async def connect_to_server(self):
        """Connect to FastAPI server via WebSocket"""
        while self.running:
            try:
                async with websockets.connect(self.server_url) as websocket:
                    self.websocket = websocket
                    rospy.loginfo("Connected to server")
                    
                    # Send connection message
                    await websocket.send(json.dumps({
                        'type': 'ros_connected',
                        'device': 'ROS'
                    }))
                    
                    # Listen for messages
                    async for message in websocket:
                        await self.handle_server_message(message)
                        
            except Exception as e:
                rospy.logerr(f"WebSocket connection error: {e}")
                await asyncio.sleep(5)  # Reconnect delay
    
    def run(self):
        """Start the bridge"""
        self.running = True
        
        # Run WebSocket in separate thread
        def websocket_thread():
            asyncio.run(self.connect_to_server())
        
        thread = threading.Thread(target=websocket_thread, daemon=True)
        thread.start()
        
        rospy.loginfo("ROS-Server Bridge running")
        rospy.spin()
    
    def shutdown(self):
        """Shutdown the bridge"""
        self.running = False
        self.stop_robot()
        rospy.loginfo("ROS-Server Bridge shutting down")

if __name__ == '__main__':
    try:
        bridge = ROSServerBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down ROS-Server Bridge")
