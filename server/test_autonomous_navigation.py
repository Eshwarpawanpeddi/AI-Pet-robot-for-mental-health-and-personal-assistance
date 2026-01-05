#!/usr/bin/env python3
"""
Test script for autonomous navigation feature
Tests camera stream manager, navigation logic, and integration
"""
import asyncio
import base64
import numpy as np
import cv2
from camera_stream_manager import CameraStreamManager
from autonomous_navigation import AutonomousNavigator, MotorCommand

async def test_camera_stream_manager():
    """Test the camera stream manager"""
    print("\n=== Testing Camera Stream Manager ===")
    
    manager = CameraStreamManager(max_buffer_size=3)
    
    # Test initial state
    stats = manager.get_stats()
    assert stats['frame_count'] == 0, "Initial frame count should be 0"
    assert stats['subscribers'] == 0, "Initial subscriber count should be 0"
    print("✓ Initial state is correct")
    
    # Test subscription
    received_frames = []
    
    async def test_callback(frame_data):
        received_frames.append(frame_data)
    
    manager.subscribe(test_callback)
    stats = manager.get_stats()
    assert stats['subscribers'] == 1, "Should have 1 subscriber"
    print("✓ Subscription works")
    
    # Test frame publishing
    test_frame = {
        'frame': 'test_base64_data',
        'timestamp': '2026-01-01T00:00:00'
    }
    await manager.publish_frame(test_frame)
    
    # Give async callback time to execute
    await asyncio.sleep(0.1)
    
    assert len(received_frames) == 1, "Should have received 1 frame"
    assert received_frames[0]['frame'] == 'test_base64_data'
    print("✓ Frame publishing and notification works")
    
    # Test multiple frames
    for i in range(5):
        await manager.publish_frame({'frame': f'frame_{i}', 'timestamp': f'time_{i}'})
        await asyncio.sleep(0.05)
    
    stats = manager.get_stats()
    assert stats['frame_count'] == 6, f"Should have 6 frames total, got {stats['frame_count']}"
    assert stats['buffer_size'] <= 3, "Buffer should not exceed max size"
    print(f"✓ Frame buffering works (buffer size: {stats['buffer_size']})")
    
    # Test unsubscription
    manager.unsubscribe(test_callback)
    stats = manager.get_stats()
    assert stats['subscribers'] == 0, "Should have 0 subscribers after unsubscribe"
    print("✓ Unsubscription works")
    
    print("✅ Camera Stream Manager tests passed!")
    return True

async def test_autonomous_navigator():
    """Test the autonomous navigator"""
    print("\n=== Testing Autonomous Navigator ===")
    
    navigator = AutonomousNavigator()
    
    # Test initial state
    status = navigator.get_status()
    assert not status['enabled'], "Should not be enabled initially"
    assert not status['running'], "Should not be running initially"
    assert status['mode'] == 'idle', "Should be in idle mode"
    print("✓ Initial state is correct")
    
    # Test parameters
    params = navigator.get_parameters()
    assert params['model_name'] == 'yolov8n', "Should use yolov8n model"
    assert params['confidence_threshold'] == 0.5, "Default confidence should be 0.5"
    assert params['safe_speed'] == 50, "Default safe speed should be 50"
    print("✓ Default parameters are correct")
    
    # Test parameter updates
    new_params = {
        'confidence_threshold': 0.6,
        'safe_speed': 60,
        'caution_speed': 35
    }
    navigator.update_parameters(new_params)
    updated_params = navigator.get_parameters()
    assert updated_params['confidence_threshold'] == 0.6, "Confidence should be updated"
    assert updated_params['safe_speed'] == 60, "Safe speed should be updated"
    assert updated_params['caution_speed'] == 35, "Caution speed should be updated"
    print("✓ Parameter updates work")
    
    # Test motor command callback
    received_commands = []
    
    async def mock_motor_callback(command: str, speed: int):
        received_commands.append((command, speed))
    
    navigator.set_motor_command_callback(mock_motor_callback)
    
    # Test scene analysis with mock detections
    # Simulate obstacle ahead (center of frame)
    frame_shape = (480, 640, 3)  # height, width, channels
    detections = [
        {
            'bbox': (280, 350, 360, 430),  # Center, low in frame (close)
            'confidence': 0.8,
            'class_id': 0,
            'class_name': 'person'
        }
    ]
    
    decision = navigator._analyze_scene(detections, frame_shape)
    assert decision['command'] in [MotorCommand.LEFT, MotorCommand.RIGHT], \
        "Should turn when obstacle ahead"
    assert decision['speed'] == navigator.caution_speed, \
        "Should use caution speed when avoiding"
    print(f"✓ Scene analysis works (decision: {decision['command'].value})")
    
    # Test with obstacle on left
    detections_left = [
        {
            'bbox': (50, 350, 150, 430),  # Left side, low in frame
            'confidence': 0.7,
            'class_id': 0,
            'class_name': 'chair'
        }
    ]
    
    decision = navigator._analyze_scene(detections_left, frame_shape)
    assert decision['command'] == MotorCommand.RIGHT, \
        "Should turn right when obstacle on left"
    print("✓ Left obstacle avoidance works")
    
    # Test with clear path
    detections_clear = []
    decision = navigator._analyze_scene(detections_clear, frame_shape)
    assert decision['command'] == MotorCommand.FORWARD, \
        "Should move forward when path is clear"
    assert decision['speed'] == navigator.safe_speed, \
        "Should use safe speed when clear"
    print("✓ Clear path detection works")
    
    print("✅ Autonomous Navigator tests passed!")
    return True

async def test_integration():
    """Test integration between camera stream and navigation"""
    print("\n=== Testing Integration ===")
    
    manager = CameraStreamManager()
    navigator = AutonomousNavigator()
    
    # Create a simple test frame (black image with white rectangle as obstacle)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    # Draw white rectangle in center (simulating obstacle)
    cv2.rectangle(frame, (280, 350), (360, 430), (255, 255, 255), -1)
    
    # Encode to base64
    _, buffer = cv2.imencode('.jpg', frame)
    frame_base64 = base64.b64encode(buffer).decode('utf-8')
    
    frame_data = {
        'frame': frame_base64,
        'timestamp': '2026-01-01T00:00:00'
    }
    
    # Subscribe navigator to camera stream
    manager.subscribe(navigator.process_frame)
    
    # Note: We can't fully test YOLO detection without the model
    # but we can test the pipeline
    print("✓ Navigator subscribed to camera stream")
    
    # Publish frame (won't process without YOLO model, but tests pipeline)
    await manager.publish_frame(frame_data)
    await asyncio.sleep(0.1)
    
    print("✓ Frame published to navigator (YOLO detection would occur here)")
    
    # Verify stats
    stats = manager.get_stats()
    assert stats['subscribers'] == 1, "Should have 1 subscriber"
    assert stats['frame_count'] == 1, "Should have processed 1 frame"
    print(f"✓ Integration pipeline working (subscribers: {stats['subscribers']}, frames: {stats['frame_count']})")
    
    print("✅ Integration tests passed!")
    return True

async def main():
    """Run all tests"""
    print("=" * 60)
    print("AUTONOMOUS NAVIGATION FEATURE TEST SUITE")
    print("=" * 60)
    
    try:
        # Run tests
        await test_camera_stream_manager()
        await test_autonomous_navigator()
        await test_integration()
        
        print("\n" + "=" * 60)
        print("✅ ALL TESTS PASSED!")
        print("=" * 60)
        print("\nNote: Full YOLO detection requires 'ultralytics' package.")
        print("Install with: pip install ultralytics")
        
        return 0
        
    except AssertionError as e:
        print(f"\n❌ TEST FAILED: {e}")
        return 1
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit_code = asyncio.run(main())
    exit(exit_code)
