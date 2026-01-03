#!/usr/bin/env python3
"""
Test script to verify new features implementation
"""
import os
import sys

def test_files_exist():
    """Test that all required files exist"""
    print("Testing file existence...")
    
    required_files = [
        "server/mobile_web_server.py",
        "server/emotion_detection_server.py",
        "server/launch_all.py",
        "server/gemini_integration.py",
        "ROS_SETUP_GUIDE.md",
        "SPEAKER_SETUP_GUIDE.md",
        "FEATURES_GUIDE.md",
        "emotion_model.h5"
    ]
    
    all_exist = True
    for file in required_files:
        exists = os.path.exists(file)
        status = "✓" if exists else "✗"
        print(f"  {status} {file}")
        if not exists and file != "emotion_model.h5":  # emotion_model.h5 may not exist in test env
            all_exist = False
    
    return all_exist

def test_mobile_web_features():
    """Test mobile web server has new features"""
    print("\nTesting mobile web server features...")
    
    with open("server/mobile_web_server.py", "r") as f:
        content = f.read()
    
    features = {
        "Larger camera view (400px)": "height: 400px" in content,
        "Speed control": "speedSlider" in content or "speed-slider" in content,
        "Joystick mode": "joystick" in content.lower(),
        "WASD controls": "wasd" in content.lower() or "'w'" in content.lower(),
        "Mode toggle": "switchControlMode" in content or "mode-tab" in content,
    }
    
    all_present = True
    for feature, present in features.items():
        status = "✓" if present else "✗"
        print(f"  {status} {feature}")
        if not present:
            all_present = False
    
    return all_present

def test_emotion_detection_server():
    """Test emotion detection server structure"""
    print("\nTesting emotion detection server...")
    
    with open("server/emotion_detection_server.py", "r") as f:
        content = f.read()
    
    features = {
        "Port 9999 configuration": "9999" in content,
        "OpenCV import": "import cv2" in content,
        "TensorFlow import": "import tensorflow" in content or "tf.keras" in content,
        "Emotion model loading": "emotion_model.h5" in content,
        "Face cascade": "haarcascade" in content,
        "Gemini integration": "gemini" in content.lower(),
        "WebSocket endpoint": "/ws/emotion" in content,
        "Health endpoint": "/health" in content,
    }
    
    all_present = True
    for feature, present in features.items():
        status = "✓" if present else "✗"
        print(f"  {status} {feature}")
        if not present:
            all_present = False
    
    return all_present

def test_gemini_configuration():
    """Test Gemini API is using gemini-1.5-flash"""
    print("\nTesting Gemini configuration...")
    
    with open("server/gemini_integration.py", "r") as f:
        content = f.read()
    
    uses_flash = "gemini-1.5-flash" in content
    status = "✓" if uses_flash else "✗"
    print(f"  {status} Using gemini-1.5-flash model")
    
    return uses_flash

def test_launch_all_updated():
    """Test launch_all.py includes emotion detection"""
    print("\nTesting launch_all.py updates...")
    
    with open("server/launch_all.py", "r") as f:
        content = f.read()
    
    features = {
        "Emotion detection server method": "start_emotion_detection_server" in content,
        "Port 9999 mentioned": "9999" in content,
        "Emotion detection in startup": "emotion_detection_server.py" in content,
    }
    
    all_present = True
    for feature, present in features.items():
        status = "✓" if present else "✗"
        print(f"  {status} {feature}")
        if not present:
            all_present = False
    
    return all_present

def test_documentation():
    """Test documentation is comprehensive"""
    print("\nTesting documentation...")
    
    docs = {
        "ROS_SETUP_GUIDE.md": [
            "ROS Noetic",
            "catkin_make",
            "roslaunch",
            "Troubleshooting"
        ],
        "SPEAKER_SETUP_GUIDE.md": [
            "40mm",
            "8 Ohms",
            "PAM8403",
            "espeak"
        ],
        "FEATURES_GUIDE.md": [
            "Port 3000",
            "Port 9999",
            "WASD",
            "Joystick"
        ]
    }
    
    all_complete = True
    for doc, keywords in docs.items():
        if os.path.exists(doc):
            with open(doc, "r") as f:
                content = f.read()
            
            print(f"\n  Checking {doc}:")
            for keyword in keywords:
                present = keyword.lower() in content.lower()
                status = "✓" if present else "✗"
                print(f"    {status} Contains '{keyword}'")
                if not present:
                    all_complete = False
        else:
            print(f"  ✗ {doc} not found")
            all_complete = False
    
    return all_complete

def test_requirements():
    """Test requirements.txt has new dependencies"""
    print("\nTesting requirements.txt...")
    
    with open("server/requirements.txt", "r") as f:
        content = f.read()
    
    dependencies = {
        "opencv-python": "opencv-python" in content,
        "tensorflow": "tensorflow" in content,
        "numpy": "numpy" in content,
    }
    
    all_present = True
    for dep, present in dependencies.items():
        status = "✓" if present else "✗"
        print(f"  {status} {dep}")
        if not present:
            all_present = False
    
    return all_present

def main():
    """Run all tests"""
    print("=" * 60)
    print("AI Pet Robot - Features Implementation Test")
    print("=" * 60)
    
    os.chdir("/home/runner/work/AI-Pet-robot-for-mental-health-and-personal-assistance/AI-Pet-robot-for-mental-health-and-personal-assistance")
    
    tests = [
        ("File Existence", test_files_exist),
        ("Mobile Web Features", test_mobile_web_features),
        ("Emotion Detection Server", test_emotion_detection_server),
        ("Gemini Configuration", test_gemini_configuration),
        ("Launch Script Updates", test_launch_all_updated),
        ("Documentation", test_documentation),
        ("Requirements", test_requirements),
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"\n  Error in {test_name}: {e}")
            results.append((test_name, False))
    
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    
    for test_name, passed in results:
        status = "✓ PASSED" if passed else "✗ FAILED"
        print(f"{status}: {test_name}")
    
    all_passed = all(result for _, result in results)
    
    print("\n" + "=" * 60)
    if all_passed:
        print("✓ All tests passed!")
        print("=" * 60)
        return 0
    else:
        print("✗ Some tests failed")
        print("=" * 60)
        return 1

if __name__ == "__main__":
    sys.exit(main())
