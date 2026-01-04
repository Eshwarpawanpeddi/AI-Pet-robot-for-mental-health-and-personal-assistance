"""
Autonomous Navigation Module
Uses YOLO object detection for obstacle avoidance and autonomous movement
"""
import asyncio
import logging
import numpy as np
import cv2
import base64
from typing import Dict, List, Optional, Tuple
from datetime import datetime
from enum import Enum
import os

logger = logging.getLogger(__name__)


class NavigationMode(Enum):
    """Navigation modes"""
    IDLE = "idle"
    EXPLORING = "exploring"
    AVOIDING = "avoiding"
    STOPPED = "stopped"


class MotorCommand(Enum):
    """Motor control commands"""
    STOP = "stop"
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"


class AutonomousNavigator:
    """
    Autonomous navigation system using YOLOv8 for obstacle detection
    """
    
    def __init__(self, model_name: str = "yolov8n", confidence_threshold: float = 0.5):
        """
        Initialize the autonomous navigator
        
        Args:
            model_name: YOLO model to use (yolov8n for nano, yolov8s for small, etc.)
            confidence_threshold: Minimum confidence for object detection
        """
        self.model = None
        self.model_name = model_name
        self.confidence_threshold = confidence_threshold
        self.is_enabled = False
        self.is_running = False
        self.current_mode = NavigationMode.IDLE
        self.navigation_task = None
        
        # Navigation parameters
        self.obstacle_distance_threshold = 0.3  # Fraction of image height
        self.safe_speed = 50  # Base speed when clear
        self.caution_speed = 30  # Speed when obstacles detected
        self.min_safe_width = 0.2  # Minimum clear width fraction
        
        # State tracking
        self.last_command = MotorCommand.STOP
        self.last_detection_time = None
        self.detection_history = []
        self.max_history = 10
        
        # Callbacks
        self.motor_command_callback = None
        
        logger.info(f"AutonomousNavigator initialized with {model_name}")
    
    async def initialize_model(self):
        """Load the YOLO model"""
        try:
            from ultralytics import YOLO
            
            # Try to load the model
            model_path = f"{self.model_name}.pt"
            
            # Check if model exists locally, if not it will be downloaded
            self.model = YOLO(model_path)
            logger.info(f"YOLO model {self.model_name} loaded successfully")
            return True
            
        except ImportError:
            logger.error("ultralytics package not installed. Install with: pip install ultralytics")
            return False
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            return False
    
    def set_motor_command_callback(self, callback):
        """
        Set the callback function for sending motor commands
        
        Args:
            callback: Async function with signature: async def callback(command: str, speed: int)
        """
        self.motor_command_callback = callback
        logger.info("Motor command callback set")
    
    async def start_navigation(self):
        """Start autonomous navigation"""
        if not self.model:
            logger.error("Cannot start navigation: model not initialized")
            return False
        
        if self.is_running:
            logger.warning("Navigation already running")
            return False
        
        self.is_enabled = True
        self.is_running = True
        self.current_mode = NavigationMode.EXPLORING
        logger.info("Autonomous navigation started")
        return True
    
    async def stop_navigation(self):
        """Stop autonomous navigation"""
        self.is_enabled = False
        self.is_running = False
        self.current_mode = NavigationMode.IDLE
        
        # Send stop command
        await self._send_motor_command(MotorCommand.STOP, 0)
        logger.info("Autonomous navigation stopped")
    
    async def process_frame(self, frame_data: dict):
        """
        Process a camera frame for navigation
        
        Args:
            frame_data: Dictionary containing 'frame' (base64 encoded image)
        """
        if not self.is_enabled or not self.model:
            return
        
        try:
            # Decode frame
            frame_base64 = frame_data.get('frame', '')
            if not frame_base64:
                return
            
            # Convert base64 to image
            img_bytes = base64.b64decode(frame_base64)
            nparr = np.frombuffer(img_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is None:
                logger.warning("Failed to decode frame")
                return
            
            # Run object detection
            detections = await self._detect_objects(frame)
            
            # Analyze scene and determine action
            navigation_decision = self._analyze_scene(detections, frame.shape)
            
            # Execute navigation decision
            await self._execute_navigation(navigation_decision)
            
            # Update detection history
            self._update_history(detections)
            self.last_detection_time = datetime.now()
            
        except Exception as e:
            logger.error(f"Error processing frame for navigation: {e}")
    
    async def _detect_objects(self, frame: np.ndarray) -> List[Dict]:
        """
        Run YOLO detection on frame
        
        Args:
            frame: OpenCV image array
            
        Returns:
            List of detection dictionaries
        """
        try:
            # Run inference
            results = self.model(frame, conf=self.confidence_threshold, verbose=False)
            
            detections = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    
                    detections.append({
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'confidence': confidence,
                        'class_id': class_id,
                        'class_name': class_name
                    })
            
            return detections
            
        except Exception as e:
            logger.error(f"Error in object detection: {e}")
            return []
    
    def _analyze_scene(self, detections: List[Dict], frame_shape: Tuple) -> Dict:
        """
        Analyze detected objects and determine navigation action
        
        Args:
            detections: List of detected objects
            frame_shape: Shape of the frame (height, width, channels)
            
        Returns:
            Navigation decision dictionary
        """
        height, width = frame_shape[:2]
        
        # Initialize decision
        decision = {
            'command': MotorCommand.FORWARD,
            'speed': self.safe_speed,
            'mode': NavigationMode.EXPLORING,
            'reason': 'Path clear'
        }
        
        if not detections:
            # No obstacles detected - move forward
            return decision
        
        # Analyze obstacle positions
        obstacles_ahead = []
        obstacles_left = []
        obstacles_right = []
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            
            # Calculate center and size
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            obj_width = x2 - x1
            obj_height = y2 - y1
            
            # Normalize coordinates
            norm_center_x = center_x / width
            norm_center_y = center_y / height
            norm_width = obj_width / width
            norm_height = obj_height / height
            
            # Check if obstacle is in lower portion of frame (closer)
            if norm_center_y > (1 - self.obstacle_distance_threshold):
                # Classify by horizontal position
                if norm_center_x < 0.33:
                    obstacles_left.append(det)
                elif norm_center_x > 0.67:
                    obstacles_right.append(det)
                else:
                    obstacles_ahead.append(det)
        
        # Determine action based on obstacle positions
        if obstacles_ahead:
            # Obstacle directly ahead
            if len(obstacles_left) < len(obstacles_right):
                # Turn left (fewer obstacles)
                decision['command'] = MotorCommand.LEFT
                decision['speed'] = self.caution_speed
                decision['mode'] = NavigationMode.AVOIDING
                decision['reason'] = f'Avoiding obstacle ahead - turning left'
            else:
                # Turn right (fewer obstacles)
                decision['command'] = MotorCommand.RIGHT
                decision['speed'] = self.caution_speed
                decision['mode'] = NavigationMode.AVOIDING
                decision['reason'] = f'Avoiding obstacle ahead - turning right'
        
        elif obstacles_left and not obstacles_right:
            # Obstacles on left - steer right
            decision['command'] = MotorCommand.RIGHT
            decision['speed'] = self.caution_speed
            decision['mode'] = NavigationMode.AVOIDING
            decision['reason'] = 'Obstacles on left - steering right'
        
        elif obstacles_right and not obstacles_left:
            # Obstacles on right - steer left
            decision['command'] = MotorCommand.LEFT
            decision['speed'] = self.caution_speed
            decision['mode'] = NavigationMode.AVOIDING
            decision['reason'] = 'Obstacles on right - steering left'
        
        elif obstacles_left and obstacles_right:
            # Obstacles on both sides - move slowly forward or stop
            if len(obstacles_ahead) > 0:
                decision['command'] = MotorCommand.STOP
                decision['speed'] = 0
                decision['mode'] = NavigationMode.STOPPED
                decision['reason'] = 'Obstacles surrounding - stopping'
            else:
                decision['command'] = MotorCommand.FORWARD
                decision['speed'] = self.caution_speed // 2
                decision['mode'] = NavigationMode.AVOIDING
                decision['reason'] = 'Narrow passage - proceeding slowly'
        
        return decision
    
    async def _execute_navigation(self, decision: Dict):
        """
        Execute the navigation decision
        
        Args:
            decision: Navigation decision dictionary
        """
        command = decision['command']
        speed = decision['speed']
        
        # Only send command if it differs from last command (avoid spam)
        if command != self.last_command:
            await self._send_motor_command(command, speed)
            self.last_command = command
            self.current_mode = decision['mode']
            logger.info(f"Navigation: {decision['reason']} - {command.value} at speed {speed}")
    
    async def _send_motor_command(self, command: MotorCommand, speed: int):
        """
        Send motor command via callback
        
        Args:
            command: Motor command enum
            speed: Speed value (0-100)
        """
        if self.motor_command_callback:
            try:
                await self.motor_command_callback(command.value, speed)
            except Exception as e:
                logger.error(f"Error sending motor command: {e}")
    
    def _update_history(self, detections: List[Dict]):
        """
        Update detection history
        
        Args:
            detections: List of current detections
        """
        self.detection_history.append({
            'timestamp': datetime.now(),
            'count': len(detections),
            'detections': detections
        })
        
        # Trim history
        if len(self.detection_history) > self.max_history:
            self.detection_history.pop(0)
    
    def get_status(self) -> Dict:
        """
        Get current navigation status
        
        Returns:
            Status dictionary
        """
        return {
            'enabled': self.is_enabled,
            'running': self.is_running,
            'mode': self.current_mode.value,
            'last_command': self.last_command.value,
            'last_detection_time': self.last_detection_time.isoformat() if self.last_detection_time else None,
            'detection_history_size': len(self.detection_history),
            'model_loaded': self.model is not None
        }
    
    def get_parameters(self) -> Dict:
        """
        Get current navigation parameters
        
        Returns:
            Parameters dictionary
        """
        return {
            'model_name': self.model_name,
            'confidence_threshold': self.confidence_threshold,
            'obstacle_distance_threshold': self.obstacle_distance_threshold,
            'safe_speed': self.safe_speed,
            'caution_speed': self.caution_speed,
            'min_safe_width': self.min_safe_width
        }
    
    def update_parameters(self, params: Dict):
        """
        Update navigation parameters
        
        Args:
            params: Dictionary of parameters to update
        """
        if 'confidence_threshold' in params:
            self.confidence_threshold = params['confidence_threshold']
        if 'obstacle_distance_threshold' in params:
            self.obstacle_distance_threshold = params['obstacle_distance_threshold']
        if 'safe_speed' in params:
            self.safe_speed = params['safe_speed']
        if 'caution_speed' in params:
            self.caution_speed = params['caution_speed']
        if 'min_safe_width' in params:
            self.min_safe_width = params['min_safe_width']
        
        logger.info(f"Navigation parameters updated: {params}")


# Global instance
autonomous_navigator = AutonomousNavigator()
