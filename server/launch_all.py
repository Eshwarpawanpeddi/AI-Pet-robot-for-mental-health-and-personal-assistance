#!/usr/bin/env python3
"""
AI Pet Robot - Integrated Launcher
Starts all components in the correct order with proper monitoring.
Supports multi-port setup:
- Port 8000: Primary control server
- Port 10000: Emotion display server
- Port 3000: Mobile web interface
- Port 9999: Emotion detection server
"""

import subprocess
import sys
import time
import os
import signal
from pathlib import Path

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    print("Warning: python-dotenv not installed. Environment variables must be set manually.")
    def load_dotenv():
        pass
    load_dotenv()

SERVER_HOST = os.getenv("SERVER_HOST", "localhost")

class RobotLauncher:
    def __init__(self):
        self.processes = []
        self.base_dir = Path(__file__).parent.parent

    def check_dependencies(self):
        """Check if required dependencies are installed"""
        print("\n" + "="*70)
        print("Checking Dependencies...")
        print("="*70 + "\n")
        try:
            sys.path.insert(0, str(self.base_dir / "server"))
            from check_dependencies import check_dependency
            missing = []
            required_packages = [
                ('fastapi', 'FastAPI'),
                ('uvicorn', 'Uvicorn'),
                ('aiohttp', 'aiohttp'),
                ('dotenv', 'python-dotenv'),
            ]
            for module_name, package_name in required_packages:
                if not check_dependency(module_name):
                    missing.append(package_name)
            if missing:
                print("✗ Missing required dependencies:")
                for pkg in missing:
                    print(f"  - {pkg}")
                print("\nTo install missing dependencies, run:")
                print("  pip3 install " + " ".join(missing))
                print("\nOr install all dependencies:")
                print("  pip3 install -r server/requirements.txt")
                print("\nFor a detailed dependency check, run:")
                print("  python3 server/check_dependencies.py")
                print()
                return False
            else:
                print("✓ All core dependencies are installed")
                return True
        except Exception as e:
            print(f"⚠️  Could not check dependencies: {e}")
            print("Proceeding with server startup...")
            return True

    def cleanup(self, signum=None, frame=None):
        """Clean up all processes"""
        print("\n\n🛑 Shutting down all components...")
        for name, process in self.processes:
            print(f"   Stopping {name}...")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        print("✓ All components stopped")
        sys.exit(0)

    def start_component(self, script_name, friendly_name, port=None, sleep_time=2):
        """Start a component python script and add to process list."""
        print(f"🚀 Starting {friendly_name}...")
        server_dir = self.base_dir / "server"
        env = os.environ.copy()
        process = subprocess.Popen(
            [sys.executable, script_name],
            cwd=server_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1,
            env=env
        )
        self.processes.append((f"{friendly_name} ({port})" if port else friendly_name, process))
        time.sleep(sleep_time)
        if port:
            print(f"✓ {friendly_name} started on http://{SERVER_HOST}:{port}\n")
        else:
            print(f"✓ {friendly_name} started\n")
        return process

    def start_server(self):
        return self.start_component("server.py", "Primary Server", port=8000, sleep_time=3)

    def start_emotion_display_server(self):
        return self.start_component("emotion_display_server.py", "Emotion Display", port=10000, sleep_time=2)

    def start_mobile_web_server(self):
        return self.start_component("mobile_web_server.py", "Mobile Web", port=3000, sleep_time=2)

    def start_emotion_detection_server(self):
        return self.start_component("emotion_detection_server.py", "Emotion Detection", port=9999, sleep_time=2)

    def start_raspberry_pi_sim(self):
        print("🤖 Starting Raspberry Pi Controller (Simulation Mode)...")
        print("   Note: For real hardware, run this on the Raspberry Pi")
        pi_dir = self.base_dir / "hardware" / "raspberry_pi"
        if not (pi_dir / "raspberry_pi_controller.py").exists():
            print("   ⚠️  Raspberry Pi controller not found, skipping...")
            return None
        env = os.environ.copy()
        process = subprocess.Popen(
            [sys.executable, "raspberry_pi_controller.py"],
            cwd=pi_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1,
            env=env
        )
        self.processes.append(("Raspberry Pi", process))
        time.sleep(2)
        print("✓ Raspberry Pi controller started\n")
        return process

    def start_ros_bridge(self):
        print("🔗 Checking for ROS...")
        try:
            subprocess.run(["rosversion", "-d"], capture_output=True, check=True, timeout=1)
            print("   ROS detected! Starting ROS bridge...")
            ros_dir = self.base_dir / "ros_workspace" / "src" / "pet_robot_ros"
            if not (ros_dir / "nodes" / "ros_server_bridge.py").exists():
                print("   ⚠️  ROS bridge not found, skipping...")
                return None
            process = subprocess.Popen(
                ["rosrun", "pet_robot_ros", "ros_server_bridge.py"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            self.processes.append(("ROS Bridge", process))
            time.sleep(2)
            print("✓ ROS bridge started\n")
            return process
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
            print("   ℹ️  ROS not available, skipping autonomous mode")
            print("   To enable: Install ROS and run 'roslaunch pet_robot_ros ros_bridge.launch'\n")
            return None

    def monitor_processes(self):
        print("=" * 70)
        print("🎮 AI Pet Robot System Running - Multi-Port Setup")
        print("=" * 70)
        print("\n📊 Active Components:")
        for name, _ in self.processes:
            print(f"   ✓ {name}")
        print("\n🌐 Access Points (Each port has unique features):")
        print(f"   - Port 8000  → http://{SERVER_HOST}:8000")
        print(f"                  Full Control Panel: Camera, Movement, Audio, AI Chat")
        print(f"   - Port 10000 → http://{SERVER_HOST}:10000")
        print(f"                  Emotion Display + Analytics Dashboard (/dashboard)")
        print(f"   - Port 3000  → http://{SERVER_HOST}:3000")
        print(f"                  Mobile Web Interface (touch-friendly)")
        print(f"   - Port 9999  → http://{SERVER_HOST}:9999")
        print(f"                  Emotion Detection (facial recognition)")
        print("\n📱 Mobile app: Configure server IP to this machine's IP address")
        print("\n⌨️  Press Ctrl+C to stop all components\n")
        print("=" * 70)
        print("\n📋 System Logs:\n")
        try:
            while True:
                for name, process in self.processes:
                    if process.poll() is not None:
                        print(f"\n⚠️  {name} stopped unexpectedly!")
                        return
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    def run(self, mode="full"):
        signal.signal(signal.SIGINT, self.cleanup)
        signal.signal(signal.SIGTERM, self.cleanup)
        print("\n" + "=" * 70)
        print("AI Pet Robot - Integrated Launcher (Multi-Port)")
        print("=" * 70)
        print(f"\nMode: {mode.upper()}\n")
        if not self.check_dependencies():
            print("\n❌ Cannot start servers due to missing dependencies.")
            print("Please install the required packages and try again.\n")
            sys.exit(1)
        print()
        if mode == "servers":
            self.start_server()
            self.start_emotion_display_server()
            self.start_mobile_web_server()
            self.start_emotion_detection_server()
        elif mode == "full":
            self.start_server()
            self.start_emotion_display_server()
            self.start_mobile_web_server()
            self.start_emotion_detection_server()
            self.start_raspberry_pi_sim()
            self.start_ros_bridge()
        elif mode == "server+pi":
            self.start_server()
            self.start_emotion_display_server()
            self.start_mobile_web_server()
            self.start_emotion_detection_server()
            self.start_raspberry_pi_sim()
        elif mode == "server-only":
            self.start_server()
        try:
            self.monitor_processes()
        finally:
            self.cleanup()

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="AI Pet Robot Integrated Launcher - Multi-Port Setup",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python launch_all.py                    # Start all three servers (default)
  python launch_all.py --full             # Start servers + hardware components
  python launch_all.py --with-pi          # Start servers + Pi simulation
  python launch_all.py --server-only      # Start only primary server (port 8000)

Port Setup:
  - Port 8000: Primary control (movement, camera, AI)
  - Port 10000: Emotion display (animated face)
  - Port 3000: Mobile web interface
  - Port 9999: Emotion detection (facial expression analysis)

For production:
  - Run this launcher on your main computer
  - Run raspberry_pi_controller.py on the Raspberry Pi
  - Run roslaunch pet_robot_ros ros_bridge.launch if using ROS
        """
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="Start all components (servers, Pi sim, ROS if available)"
    )
    parser.add_argument(
        "--with-pi",
        action="store_true",
        help="Start servers with Pi simulation"
    )
    parser.add_argument(
        "--server-only",
        action="store_true",
        help="Start only the primary control server (port 8000)"
    )
    args = parser.parse_args()
    launcher = RobotLauncher()
    if args.full:
        launcher.run(mode="full")
    elif args.with_pi:
        launcher.run(mode="server+pi")
    elif args.server_only:
        launcher.run(mode="server-only")
    else:
        launcher.run(mode="servers")

if __name__ == "__main__":
    load_dotenv()
    main()