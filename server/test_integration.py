#!/usr/bin/env python3
"""
Integration test script for AI Pet Robot multimodal features.
Tests server functionality, WebSocket connections, and API endpoints.
"""

import asyncio
import json
import sys
import base64
from typing import Optional
import aiohttp

# Test configuration
SERVER_URL = "http://localhost:8000"
WS_URL = "ws://localhost:8000"

class Colors:
    """ANSI color codes for terminal output"""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    END = '\033[0m'

def print_success(message: str):
    print(f"{Colors.GREEN}✓ {message}{Colors.END}")

def print_error(message: str):
    print(f"{Colors.RED}✗ {message}{Colors.END}")

def print_info(message: str):
    print(f"{Colors.BLUE}ℹ {message}{Colors.END}")

def print_warning(message: str):
    print(f"{Colors.YELLOW}⚠ {message}{Colors.END}")

async def test_health_endpoint():
    """Test server health endpoint"""
    print_info("Testing health endpoint...")
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(f"{SERVER_URL}/health") as response:
                if response.status == 200:
                    data = await response.json()
                    print_success(f"Health check passed - Status: {data['status']}")
                    print_info(f"  Gemini initialized: {data.get('gemini_initialized', False)}")
                    print_info(f"  ESP connected: {data['robot_state'].get('esp_connected', False)}")
                    print_info(f"  Raspberry Pi connected: {data['robot_state'].get('raspberry_pi_connected', False)}")
                    return True
                else:
                    print_error(f"Health check failed with status {response.status}")
                    return False
    except Exception as e:
        print_error(f"Health check error: {e}")
        return False

async def test_auth_token_generation():
    """Test authentication token generation"""
    print_info("Testing authentication token generation...")
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(f"{SERVER_URL}/api/auth/token") as response:
                if response.status == 200:
                    data = await response.json()
                    token = data.get('token')
                    if token:
                        print_success(f"Token generated successfully: {token[:20]}...")
                        return token
                    else:
                        print_error("Token not in response")
                        return None
                else:
                    print_error(f"Token generation failed with status {response.status}")
                    return None
    except Exception as e:
        print_error(f"Token generation error: {e}")
        return None

async def test_get_state():
    """Test robot state retrieval"""
    print_info("Testing state retrieval...")
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(f"{SERVER_URL}/api/state") as response:
                if response.status == 200:
                    state = await response.json()
                    print_success("State retrieved successfully")
                    print_info(f"  Emotion: {state.get('emotion', 'unknown')}")
                    print_info(f"  Control mode: {state.get('control_mode', 'unknown')}")
                    print_info(f"  Battery: {state.get('battery_level', 0)}%")
                    return True
                else:
                    print_error(f"State retrieval failed with status {response.status}")
                    return False
    except Exception as e:
        print_error(f"State retrieval error: {e}")
        return False

async def test_mode_change():
    """Test mode change functionality"""
    print_info("Testing mode change...")
    try:
        async with aiohttp.ClientSession() as session:
            # Change to autonomous
            async with session.post(
                f"{SERVER_URL}/api/mode",
                json={"mode": "autonomous"}
            ) as response:
                if response.status == 200:
                    print_success("Mode changed to autonomous")
                else:
                    print_error(f"Mode change to autonomous failed: {response.status}")
                    return False
            
            # Change back to manual
            async with session.post(
                f"{SERVER_URL}/api/mode",
                json={"mode": "manual"}
            ) as response:
                if response.status == 200:
                    print_success("Mode changed back to manual")
                    return True
                else:
                    print_error(f"Mode change to manual failed: {response.status}")
                    return False
    except Exception as e:
        print_error(f"Mode change error: {e}")
        return False

async def test_mood_logging():
    """Test mood logging feature"""
    print_info("Testing mood logging...")
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"{SERVER_URL}/api/mood",
                json={
                    "mood": "happy",
                    "intensity": 8,
                    "notes": "Test mood log"
                }
            ) as response:
                if response.status == 200:
                    data = await response.json()
                    print_success(f"Mood logged: {data.get('mood', 'unknown')}")
                    print_info(f"  Response: {data.get('response', '')[:60]}...")
                    return True
                else:
                    print_error(f"Mood logging failed with status {response.status}")
                    return False
    except Exception as e:
        print_error(f"Mood logging error: {e}")
        return False

async def test_affirmation():
    """Test affirmation endpoint"""
    print_info("Testing affirmation generation...")
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(f"{SERVER_URL}/api/affirmation") as response:
                if response.status == 200:
                    data = await response.json()
                    affirmation = data.get('affirmation', '')
                    print_success(f"Affirmation: {affirmation}")
                    return True
                else:
                    print_error(f"Affirmation failed with status {response.status}")
                    return False
    except Exception as e:
        print_error(f"Affirmation error: {e}")
        return False

async def test_websocket_connection(token: Optional[str] = None):
    """Test WebSocket connection"""
    print_info("Testing WebSocket connection...")
    try:
        import websockets
        
        ws_url = f"{WS_URL}/ws/control"
        if token:
            ws_url += f"?token={token}"
        
        async with websockets.connect(ws_url) as websocket:
            print_success("WebSocket connected")
            
            # Test get_state command
            await websocket.send(json.dumps({"type": "get_state"}))
            response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
            data = json.loads(response)
            
            if 'emotion' in data:
                print_success("Received state update via WebSocket")
                return True
            else:
                print_error("Invalid state response")
                return False
                
    except asyncio.TimeoutError:
        print_error("WebSocket timeout")
        return False
    except Exception as e:
        print_error(f"WebSocket error: {e}")
        return False

async def run_all_tests():
    """Run all integration tests"""
    print("=" * 60)
    print("AI Pet Robot - Integration Tests")
    print("=" * 60)
    print()
    
    results = []
    
    # Test 1: Health check
    results.append(("Health Check", await test_health_endpoint()))
    print()
    
    # Test 2: Authentication token
    token = await test_auth_token_generation()
    results.append(("Auth Token", token is not None))
    print()
    
    # Test 3: State retrieval
    results.append(("State Retrieval", await test_get_state()))
    print()
    
    # Test 4: Mode change
    results.append(("Mode Change", await test_mode_change()))
    print()
    
    # Test 5: Mood logging
    results.append(("Mood Logging", await test_mood_logging()))
    print()
    
    # Test 6: Affirmation
    results.append(("Affirmation", await test_affirmation()))
    print()
    
    # Test 7: WebSocket (optional - may fail if websockets not installed)
    try:
        results.append(("WebSocket", await test_websocket_connection(token)))
    except ImportError:
        print_warning("WebSocket test skipped (websockets package not installed)")
        print_info("Install with: pip install websockets")
    print()
    
    # Summary
    print("=" * 60)
    print("Test Summary")
    print("=" * 60)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "PASSED" if result else "FAILED"
        color = Colors.GREEN if result else Colors.RED
        print(f"{color}{test_name}: {status}{Colors.END}")
    
    print()
    print(f"Results: {passed}/{total} tests passed")
    
    if passed == total:
        print_success("All tests passed! 🎉")
        return 0
    else:
        print_error(f"{total - passed} test(s) failed")
        return 1

if __name__ == "__main__":
    try:
        exit_code = asyncio.run(run_all_tests())
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\nTests interrupted by user")
        sys.exit(1)
    except Exception as e:
        print_error(f"Test suite error: {e}")
        sys.exit(1)
