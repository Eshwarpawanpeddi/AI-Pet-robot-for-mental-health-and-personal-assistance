"""
Shared state module for multi-port robot control system.
This module maintains global state accessible across all server instances.
"""
import asyncio
from typing import List, Dict, Optional
from datetime import datetime
import logging

logger = logging.getLogger(__name__)

class RobotState:
    """Centralized robot state shared across all servers"""
    
    def __init__(self):
        # Emotion state
        self.emotion = "neutral"        # Robot face emotion
        self.user_emotion = "neutral"   # Tracked sentiment of the user (deprecated, use current_emotion)
        self.current_emotion = "unknown"  # Current detected user emotion from port 9999
        
        # Audio state
        self.is_listening = False
        self.is_speaking = False
        self.last_transcript = ""
        
        # Hardware state
        self.battery_level = 100
        self.camera_enabled = False
        self.latest_camera_frame = None
        
        # Connection state
        self.connected_clients = []
        self.camera_clients = []
        self.raspberry_pi_client = None
        self.ros_client = None
        self.gemini_session = None
        
        # Control state
        self.control_mode = "manual"  # manual or autonomous
        
        # Mental health monitoring
        self.user_emotion_history = []  # Track emotion over time
        self.mental_health_insights = []
        self.concern_level = 0  # 0-10 scale
        
        # Emotion display subscribers (for port 10000)
        self.emotion_display_clients = []
        
        # Port-specific Gemini control
        self.gemini_enabled_port_8000 = True  # Enable/disable Gemini on port 8000
        self.gemini_enabled_port_3000 = True  # Enable/disable Gemini on port 3000
        
        # Task scheduling and reminders
        self.scheduled_tasks = []  # List of scheduled tasks
        self.reminders = []  # List of active reminders
        
        # Lock for thread-safe access (will be initialized in async context)
        self._lock = None
    
    async def _ensure_lock(self):
        """Lazily initialize the lock in async context"""
        if self._lock is None:
            self._lock = asyncio.Lock()
    
    async def set_emotion(self, emotion: str):
        """Set robot emotion and notify all subscribers"""
        await self._ensure_lock()
        async with self._lock:
            self.emotion = emotion
            logger.info(f"Emotion changed to: {emotion}")
            await self._notify_emotion_subscribers()
    
    async def _notify_emotion_subscribers(self):
        """Notify all emotion display clients of emotion change"""
        for client in self.emotion_display_clients[:]:
            try:
                await client.send_json({
                    'type': 'emotion_update',
                    'emotion': self.emotion
                })
            except Exception as e:
                logger.error(f"Failed to notify emotion subscriber: {e}")
                self.emotion_display_clients.remove(client)
    
    async def add_emotion_subscriber(self, client):
        """Add a client to receive emotion updates"""
        await self._ensure_lock()
        async with self._lock:
            if client not in self.emotion_display_clients:
                self.emotion_display_clients.append(client)
                logger.info(f"Added emotion subscriber. Total: {len(self.emotion_display_clients)}")
    
    async def remove_emotion_subscriber(self, client):
        """Remove a client from emotion updates"""
        await self._ensure_lock()
        async with self._lock:
            if client in self.emotion_display_clients:
                self.emotion_display_clients.remove(client)
                logger.info(f"Removed emotion subscriber. Total: {len(self.emotion_display_clients)}")
    
    async def set_current_emotion(self, emotion: str):
        """Update current user emotion from detection"""
        await self._ensure_lock()
        async with self._lock:
            self.current_emotion = emotion
            logger.info(f"User emotion detected: {emotion}")
            # Also update legacy user_emotion field for backward compatibility
            self.user_emotion = emotion
    
    async def add_scheduled_task(self, task: Dict):
        """Add a scheduled task or reminder"""
        await self._ensure_lock()
        async with self._lock:
            if task.get('type') == 'reminder':
                self.reminders.append(task)
            else:
                self.scheduled_tasks.append(task)
            logger.info(f"Added task: {task.get('description', 'Unknown')}")
    
    async def remove_scheduled_task(self, task_id: str):
        """Remove a scheduled task or reminder by ID"""
        await self._ensure_lock()
        async with self._lock:
            self.scheduled_tasks = [t for t in self.scheduled_tasks if t.get('id') != task_id]
            self.reminders = [r for r in self.reminders if r.get('id') != task_id]
            logger.info(f"Removed task: {task_id}")
    
    def get_state_dict(self) -> Dict:
        """Get current state as dictionary"""
        return {
            'emotion': self.emotion,
            'user_emotion': self.user_emotion,
            'current_emotion': self.current_emotion,
            'battery_level': self.battery_level,
            'is_listening': self.is_listening,
            'is_speaking': self.is_speaking,
            'last_transcript': self.last_transcript,
            'camera_enabled': self.camera_enabled,
            'control_mode': self.control_mode,
            'raspberry_pi_connected': self.raspberry_pi_client is not None,
            'ros_connected': self.ros_client is not None,
            'concern_level': self.concern_level,
            'gemini_enabled_port_8000': self.gemini_enabled_port_8000,
            'gemini_enabled_port_3000': self.gemini_enabled_port_3000,
            'scheduled_tasks_count': len(self.scheduled_tasks),
            'reminders_count': len(self.reminders)
        }

# Global shared state instance
robot_state = RobotState()
