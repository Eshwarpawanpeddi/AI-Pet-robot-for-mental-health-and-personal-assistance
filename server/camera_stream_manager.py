"""
Shared Camera Stream Manager
Provides thread-safe camera frame access for multiple consumers (emotion detection, navigation, etc.)
"""
import asyncio
import logging
import threading
from typing import List, Callable, Optional
from datetime import datetime
import numpy as np

logger = logging.getLogger(__name__)


class CameraStreamManager:
    """
    Manages a shared camera stream that can be consumed by multiple services.
    Uses a subscriber pattern to broadcast frames to all registered consumers.
    """
    
    def __init__(self, max_buffer_size: int = 5):
        """
        Initialize the camera stream manager
        
        Args:
            max_buffer_size: Maximum number of frames to buffer
        """
        self.subscribers = []
        self.latest_frame = None
        self.frame_buffer = []
        self.max_buffer_size = max_buffer_size
        self.lock = threading.Lock()
        self.async_lock = None  # Will be initialized in async context
        self.frame_count = 0
        self.last_frame_time = None
        self.is_streaming = False
        
        logger.info(f"CameraStreamManager initialized with buffer size {max_buffer_size}")
    
    async def _ensure_async_lock(self):
        """Lazily initialize async lock in async context"""
        if self.async_lock is None:
            self.async_lock = asyncio.Lock()
    
    def subscribe(self, callback: Callable):
        """
        Subscribe to camera frame updates
        
        Args:
            callback: Async function to call when new frame arrives
                     Signature: async def callback(frame_data: dict)
                     Note: Async callbacks are strongly preferred for best performance.
                           Synchronous callbacks will be run in an executor which may
                           cause performance degradation with many subscribers.
        """
        with self.lock:
            if callback not in self.subscribers:
                self.subscribers.append(callback)
                logger.info(f"New subscriber added. Total subscribers: {len(self.subscribers)}")
    
    def unsubscribe(self, callback: Callable):
        """
        Unsubscribe from camera frame updates
        
        Args:
            callback: The callback function to remove
        """
        with self.lock:
            if callback in self.subscribers:
                self.subscribers.remove(callback)
                logger.info(f"Subscriber removed. Total subscribers: {len(self.subscribers)}")
    
    async def publish_frame(self, frame_data: dict):
        """
        Publish a new camera frame to all subscribers
        
        Args:
            frame_data: Dictionary containing frame information
                       Expected keys: 'frame' (base64 encoded image), 'timestamp', etc.
        """
        await self._ensure_async_lock()
        async with self.async_lock:
            # Update latest frame
            self.latest_frame = frame_data
            self.frame_count += 1
            self.last_frame_time = datetime.now()
            self.is_streaming = True
            
            # Add to buffer (FIFO)
            self.frame_buffer.append(frame_data)
            if len(self.frame_buffer) > self.max_buffer_size:
                self.frame_buffer.pop(0)
            
            # Notify all subscribers
            await self._notify_subscribers(frame_data)
    
    async def _notify_subscribers(self, frame_data: dict):
        """
        Notify all subscribers of new frame
        
        Args:
            frame_data: The frame data to send to subscribers
        """
        # Create a copy of subscribers list to avoid modification during iteration
        with self.lock:
            subscribers_copy = self.subscribers.copy()
        
        # Notify each subscriber
        for callback in subscribers_copy:
            try:
                # Call the callback (which should be async)
                if asyncio.iscoroutinefunction(callback):
                    await callback(frame_data)
                else:
                    # If not async, run in executor
                    await asyncio.get_running_loop().run_in_executor(None, callback, frame_data)
            except Exception as e:
                logger.error(f"Error notifying subscriber: {e}")
                # Optionally remove failed subscriber
                with self.lock:
                    if callback in self.subscribers:
                        self.subscribers.remove(callback)
    
    def get_latest_frame(self) -> Optional[dict]:
        """
        Get the most recent frame (synchronous)
        
        Returns:
            The latest frame data or None if no frames available
        """
        with self.lock:
            return self.latest_frame
    
    async def get_latest_frame_async(self) -> Optional[dict]:
        """
        Get the most recent frame (async)
        
        Returns:
            The latest frame data or None if no frames available
        """
        await self._ensure_async_lock()
        async with self.async_lock:
            return self.latest_frame
    
    def get_frame_buffer(self) -> List[dict]:
        """
        Get all buffered frames
        
        Returns:
            List of frame data dictionaries
        """
        with self.lock:
            return self.frame_buffer.copy()
    
    def get_stats(self) -> dict:
        """
        Get stream statistics
        
        Returns:
            Dictionary with stream stats
        """
        with self.lock:
            return {
                'frame_count': self.frame_count,
                'subscribers': len(self.subscribers),
                'buffer_size': len(self.frame_buffer),
                'last_frame_time': self.last_frame_time.isoformat() if self.last_frame_time else None,
                'is_streaming': self.is_streaming
            }
    
    def clear_buffer(self):
        """Clear the frame buffer"""
        with self.lock:
            self.frame_buffer.clear()
            logger.info("Frame buffer cleared")
    
    def reset(self):
        """Reset the stream manager state"""
        with self.lock:
            self.latest_frame = None
            self.frame_buffer.clear()
            self.frame_count = 0
            self.last_frame_time = None
            self.is_streaming = False
            logger.info("Stream manager reset")


# Global instance for shared use across all services
camera_stream_manager = CameraStreamManager()
