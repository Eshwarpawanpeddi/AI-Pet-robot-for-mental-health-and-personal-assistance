#!/usr/bin/env python3
"""
AI Pet Robot - Integrated Launcher
Starts all components in the correct order with proper monitoring.
Now supports multi-port setup:
- Port 8000: Primary control server
- Port 1000: Emotion display server
- Port 3000: Mobile web interface
"""

import subprocess
import sys
import time
import os
import signal
from pathlib import Path

class RobotLauncher:
    def __init__(self):
        self.processes = []
        self.base_dir = Path(__file__).parent.parent
        
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
    
    def start_server(self):
        """Start the FastAPI primary control server (port 8000)"""
        print("🚀 Starting Primary Control Server (Port 8000)...")
        server_dir = self.base_dir / "server"
        process = subprocess.Popen(
            [sys.executable, "server.py"],
            cwd=server_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        self.processes.append(("Primary Server (8000)", process))
        
        # Wait for server to start
        time.sleep(3)
        print("✓ Primary server started on http://localhost:8000\n")
        return process
    
    def start_emotion_display_server(self):
        """Start the emotion display server (port 1000)"""
        print("🎨 Starting Emotion Display Server (Port 1000)...")
        server_dir = self.base_dir / "server"
        process = subprocess.Popen(
            [sys.executable, "emotion_display_server.py"],
            cwd=server_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        self.processes.append(("Emotion Display (1000)", process))
        
        time.sleep(2)
        print("✓ Emotion display server started on http://localhost:1000\n")
        return process
    
    def start_mobile_web_server(self):
        """Start the mobile web control server (port 3000)"""
        print("📱 Starting Mobile Web Control Server (Port 3000)...")
        server_dir = self.base_dir / "server"
        process = subprocess.Popen(
            [sys.executable, "mobile_web_server.py"],
            cwd=server_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        self.processes.append(("Mobile Web (3000)", process))
        
        time.sleep(2)
        print("✓ Mobile web server started on http://localhost:3000\n")
        return process
    
    def start_raspberry_pi_sim(self):
        """Start Raspberry Pi controller in simulation mode"""
        print("🤖 Starting Raspberry Pi Controller (Simulation Mode)...")
        print("   Note: For real hardware, run this on the Raspberry Pi")
        pi_dir = self.base_dir / "hardware" / "raspberry_pi"
        
        if not (pi_dir / "raspberry_pi_controller.py").exists():
            print("   ⚠️  Raspberry Pi controller not found, skipping...")
            return None
        
        process = subprocess.Popen(
            [sys.executable, "raspberry_pi_controller.py"],
            cwd=pi_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        self.processes.append(("Raspberry Pi", process))
        time.sleep(2)
        print("✓ Raspberry Pi controller started\n")
        return process
    
    def start_ros_bridge(self):
        """Start ROS bridge if ROS is available"""
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
        """Monitor all processes and show their output"""
        print("=" * 70)
        print("🎮 AI Pet Robot System Running - Multi-Port Setup")
        print("=" * 70)
        print("\n📊 Active Components:")
        for name, _ in self.processes:
            print(f"   ✓ {name}")
        print("\n🌐 Access points:")
        print("   - Primary Control:    http://localhost:8000")
        print("   - Emotion Display:    http://localhost:1000")
        print("   - Mobile Interface:   http://localhost:3000")
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
        """Run the launcher in specified mode"""
        # Set up signal handlers
        signal.signal(signal.SIGINT, self.cleanup)
        signal.signal(signal.SIGTERM, self.cleanup)
        
        print("\n" + "=" * 70)
        print("AI Pet Robot - Integrated Launcher (Multi-Port)")
        print("=" * 70)
        print(f"\nMode: {mode.upper()}\n")
        
        if mode == "servers":
            # Start all three web servers
            self.start_server()
            self.start_emotion_display_server()
            self.start_mobile_web_server()
        elif mode == "full":
            # All components including hardware
            self.start_server()
            self.start_emotion_display_server()
            self.start_mobile_web_server()
            self.start_raspberry_pi_sim()
            self.start_ros_bridge()
        elif mode == "server+pi":
            # Servers and Pi simulation
            self.start_server()
            self.start_emotion_display_server()
            self.start_mobile_web_server()
            self.start_raspberry_pi_sim()
        elif mode == "server-only":
            # Just primary server (legacy mode)
            self.start_server()
        
        # Monitor
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
  - Port 1000: Emotion display (animated face)
  - Port 3000: Mobile web interface
  
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
        launcher.run(mode="servers")  # Default: all three servers

if __name__ == "__main__":
    main()
