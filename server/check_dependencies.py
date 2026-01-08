#!/usr/bin/env python3
"""
Dependency Checker for AI Pet Robot Servers
Validates that all required Python packages are installed before starting servers.
"""

import sys
import importlib.util
from typing import List, Tuple

# Required dependencies for each server
CORE_DEPENDENCIES = [
    ('fastapi', 'FastAPI web framework'),
    ('uvicorn', 'ASGI server'),
    ('aiohttp', 'Async HTTP client'),
    ('dotenv', 'Environment variable management (python-dotenv)'),
]

SERVER_8000_DEPENDENCIES = CORE_DEPENDENCIES + [
    ('google.generativeai', 'Google Generative AI (google-generativeai)'),
    ('numpy', 'NumPy for numerical operations'),
    ('cv2', 'OpenCV (opencv-python)'),
    ('websockets', 'WebSocket support'),
]

SERVER_10000_DEPENDENCIES = CORE_DEPENDENCIES + [
    ('websockets', 'WebSocket support'),
]

SERVER_3000_DEPENDENCIES = CORE_DEPENDENCIES + [
    ('websockets', 'WebSocket support'),
]

SERVER_9999_DEPENDENCIES = CORE_DEPENDENCIES + [
    ('cv2', 'OpenCV (opencv-python)'),
    ('numpy', 'NumPy for numerical operations'),
    ('tensorflow', 'TensorFlow for emotion detection'),
    ('google.generativeai', 'Google Generative AI (google-generativeai)'),
]

OPTIONAL_DEPENDENCIES = [
    ('ultralytics', 'YOLO for autonomous navigation (optional)'),
]


def check_dependency(module_name: str) -> bool:
    """Check if a Python module is installed"""
    spec = importlib.util.find_spec(module_name)
    return spec is not None


def check_dependencies(dependencies: List[Tuple[str, str]], server_name: str = "Server") -> Tuple[bool, List[str]]:
    """
    Check if all required dependencies are installed
    Returns: (all_installed: bool, missing_packages: List[str])
    """
    missing = []
    
    print(f"\n{'='*60}")
    print(f"Checking dependencies for {server_name}")
    print(f"{'='*60}\n")
    
    for module_name, description in dependencies:
        if check_dependency(module_name):
            print(f"✓ {description}")
        else:
            print(f"✗ {description} - MISSING")
            missing.append(module_name)
    
    return len(missing) == 0, missing


def check_optional_dependencies():
    """Check optional dependencies"""
    print(f"\n{'='*60}")
    print("Optional Dependencies")
    print(f"{'='*60}\n")
    
    for module_name, description in OPTIONAL_DEPENDENCIES:
        if check_dependency(module_name):
            print(f"✓ {description}")
        else:
            print(f"○ {description} - Not installed (optional)")


def get_install_command(missing_packages: List[str]) -> str:
    """Generate pip install command for missing packages"""
    # Map Python module names to pip package names
    package_map = {
        'cv2': 'opencv-python',
        'dotenv': 'python-dotenv',
        'google.generativeai': 'google-generativeai',
    }
    
    pip_packages = []
    for module in missing_packages:
        pip_packages.append(package_map.get(module, module))
    
    return f"pip3 install {' '.join(pip_packages)}"


def main():
    """Main dependency checker"""
    server_map = {
        '8000': (SERVER_8000_DEPENDENCIES, 'Primary Control Server (Port 8000)'),
        '10000': (SERVER_10000_DEPENDENCIES, 'Emotion Display Server (Port 10000)'),
        '3000': (SERVER_3000_DEPENDENCIES, 'Mobile Web Server (Port 3000)'),
        '9999': (SERVER_9999_DEPENDENCIES, 'Emotion Detection Server (Port 9999)'),
        'all': (None, 'All Servers'),
    }
    
    if len(sys.argv) > 1 and sys.argv[1] in server_map:
        port = sys.argv[1]
    else:
        port = 'all'
    
    print("\n" + "="*60)
    print("AI Pet Robot - Dependency Checker")
    print("="*60)
    
    all_good = True
    all_missing = []
    
    if port == 'all':
        # Check all servers
        for server_port, (deps, name) in server_map.items():
            if server_port == 'all':
                continue
            success, missing = check_dependencies(deps, name)
            if not success:
                all_good = False
                all_missing.extend(missing)
        
        # Remove duplicates
        all_missing = list(set(all_missing))
    else:
        deps, name = server_map[port]
        all_good, all_missing = check_dependencies(deps, name)
    
    # Check optional dependencies
    check_optional_dependencies()
    
    # Print summary
    print(f"\n{'='*60}")
    print("Summary")
    print(f"{'='*60}\n")
    
    if all_good:
        print("✓ All required dependencies are installed!")
        print("\nYou can now start the servers:")
        print("  - Port 8000:  python3 server.py")
        print("  - Port 10000: python3 emotion_display_server.py")
        print("  - Port 3000:  python3 mobile_web_server.py")
        print("  - Port 9999:  python3 emotion_detection_server.py")
        print("\nOr use the launcher:")
        print("  python3 launch_all.py")
        return 0
    else:
        print("✗ Missing dependencies detected!")
        print("\nTo install missing dependencies, run:")
        print(f"\n  {get_install_command(all_missing)}")
        print("\nOr install all dependencies from requirements.txt:")
        print("\n  pip3 install -r requirements.txt")
        print("\nNote: Some packages may require system dependencies.")
        print("      See README.md for detailed installation instructions.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
