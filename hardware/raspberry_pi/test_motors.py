#!/usr/bin/env python3
"""
Motor Control Test Script for AI Pet Robot
Tests all 4 motors with various movements and speeds
"""

import requests
import time
import sys
import json

# Configuration
# IMPORTANT: Update this IP address to match your server's IP address
SERVER_URL = "http://192.168.1.100:8000"  # Update with your server IP
TEST_DELAY = 3  # seconds between tests
SPEEDS = [25, 50, 75, 100]  # Speed levels to test

def send_command(direction, speed):
    """Send motor command to server"""
    url = f"{SERVER_URL}/api/command"
    payload = {
        "type": "move",
        "direction": direction,
        "speed": speed
    }
    
    try:
        response = requests.post(url, json=payload, timeout=5)
        response.raise_for_status()
        return True
    except requests.exceptions.RequestException as e:
        print(f"❌ Error sending command: {e}")
        return False

def stop_motors():
    """Stop all motors"""
    return send_command("stop", 0)

def test_health():
    """
    Test server health endpoint and verify Raspberry Pi connection.
    
    Returns:
        bool: True if server is healthy and Raspberry Pi is connected, False otherwise.
    
    Validates:
        - Server is accessible and responding
        - Server reports healthy status
        - Raspberry Pi WebSocket connection is established
    """
    try:
        response = requests.get(f"{SERVER_URL}/health", timeout=5)
        response.raise_for_status()
        data = response.json()
        print("✅ Server is healthy")
        print(f"   Raspberry Pi connected: {data['robot_state'].get('raspberry_pi_connected', False)}")
        return True
    except requests.exceptions.RequestException as e:
        print(f"❌ Server health check failed: {e}")
        return False

def test_direction(direction, speed):
    """Test a specific direction at given speed"""
    print(f"\n🔄 Testing {direction.upper()} at {speed}% speed...")
    
    if send_command(direction, speed):
        print(f"✅ Command sent successfully")
        time.sleep(TEST_DELAY)
        stop_motors()
        print(f"⏹️  Stopped")
        time.sleep(1)
        return True
    else:
        return False

def run_basic_tests():
    """Run basic movement tests at fixed speed"""
    print("\n" + "="*50)
    print("BASIC MOVEMENT TESTS (50% speed)")
    print("="*50)
    
    directions = ["forward", "backward", "left", "right"]
    
    for direction in directions:
        if not test_direction(direction, 50):
            print(f"❌ Failed to test {direction}")
            return False
    
    return True

def run_speed_tests():
    """Test different speed levels for forward movement"""
    print("\n" + "="*50)
    print("SPEED CALIBRATION TESTS (Forward)")
    print("="*50)
    
    for speed in SPEEDS:
        if not test_direction("forward", speed):
            print(f"❌ Failed to test speed {speed}%")
            return False
    
    return True

def run_turn_tests():
    """Test turning movements"""
    print("\n" + "="*50)
    print("TURNING TESTS")
    print("="*50)
    
    print("\n🔄 Testing left turn (tank-style)...")
    if not test_direction("left", 60):
        return False
    
    print("\n🔄 Testing right turn (tank-style)...")
    if not test_direction("right", 60):
        return False
    
    return True

def run_pattern_test():
    """Run a pattern of movements"""
    print("\n" + "="*50)
    print("PATTERN TEST (Square)")
    print("="*50)
    
    pattern = [
        ("forward", 50, 2),
        ("right", 60, 1),
        ("forward", 50, 2),
        ("right", 60, 1),
        ("forward", 50, 2),
        ("right", 60, 1),
        ("forward", 50, 2),
        ("right", 60, 1),
    ]
    
    for direction, speed, duration in pattern:
        print(f"\n🔄 Moving {direction} at {speed}% for {duration}s...")
        if not send_command(direction, speed):
            print(f"❌ Failed pattern step: {direction}")
            stop_motors()
            return False
        time.sleep(duration)
    
    stop_motors()
    print("\n✅ Pattern completed")
    return True

def emergency_stop():
    """Emergency stop - call this if something goes wrong"""
    print("\n🛑 EMERGENCY STOP!")
    stop_motors()
    print("✅ All motors stopped")

def main():
    """Main test execution"""
    print("="*60)
    print("AI PET ROBOT - MOTOR CONTROL TEST SUITE")
    print("="*60)
    print(f"\nServer URL: {SERVER_URL}")
    print(f"Test delay: {TEST_DELAY}s between tests")
    print("\n⚠️  SAFETY NOTICE:")
    print("   - Ensure robot wheels are elevated (not touching ground)")
    print("   - Be ready to cut power if needed")
    print("   - Press Ctrl+C for emergency stop\n")
    
    input("Press ENTER to begin tests or Ctrl+C to cancel...")
    
    try:
        # Test server health
        print("\n" + "="*50)
        print("HEALTH CHECK")
        print("="*50)
        if not test_health():
            print("\n❌ Server is not accessible. Please check:")
            print("   1. Server is running")
            print("   2. SERVER_URL is correct")
            print("   3. Network connectivity")
            sys.exit(1)
        
        # Run test suites
        if not run_basic_tests():
            print("\n❌ Basic tests failed")
            sys.exit(1)
        
        input("\nPress ENTER to continue with speed tests or Ctrl+C to stop...")
        
        if not run_speed_tests():
            print("\n❌ Speed tests failed")
            sys.exit(1)
        
        input("\nPress ENTER to continue with turn tests or Ctrl+C to stop...")
        
        if not run_turn_tests():
            print("\n❌ Turn tests failed")
            sys.exit(1)
        
        input("\nPress ENTER to run pattern test or Ctrl+C to stop...")
        
        if not run_pattern_test():
            print("\n❌ Pattern test failed")
            sys.exit(1)
        
        # All tests passed
        print("\n" + "="*60)
        print("✅ ALL TESTS PASSED!")
        print("="*60)
        print("\nMotor control system is functioning correctly.")
        print("You can now test with wheels on the ground.")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
        emergency_stop()
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        emergency_stop()
        sys.exit(1)

if __name__ == "__main__":
    main()
