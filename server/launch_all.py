#!/usr/bin/env python3
"""
AI Pet Robot - Integrated Launcher
Starts all components in the correct order with proper monitoring
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
        """Start the FastAPI server"""
        print("🚀 Starting FastAPI Server...")
        server_dir = self.base_dir / "server"
        process = subprocess.Popen(
            [sys.executable, "server.py"],
            cwd=server_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        self.processes.append(("Server", process))
        
        # Wait for server to start
        time.sleep(3)
        print("✓ Server started on http://localhost:8000\n")
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
            subprocess.run(["rosversion", "-d"], capture_output=True, check=True, timeout=2)
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
        print("🎮 AI Pet Robot System Running")
        print("=" * 70)
        print("\n📊 Active Components:")
        for name, _ in self.processes:
            print(f"   ✓ {name}")
        print("\n🌐 Access the web interface at: http://localhost:8000")
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
        print("AI Pet Robot - Integrated Launcher")
        print("=" * 70)
        print(f"\nMode: {mode.upper()}\n")
        
        if mode == "server":
            # Just server
            self.start_server()
        elif mode == "full":
            # All components
            self.start_server()
            self.start_raspberry_pi_sim()
            self.start_ros_bridge()
        elif mode == "server+pi":
            # Server and Pi simulation
            self.start_server()
            self.start_raspberry_pi_sim()
        
        # Monitor
        try:
            self.monitor_processes()
        finally:
            self.cleanup()

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="AI Pet Robot Integrated Launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python launch_all.py                 # Start server only
  python launch_all.py --full          # Start all components (if available)
  python launch_all.py --with-pi       # Start server + Pi simulation
  
For production:
  - Run this script on your main computer
  - Run raspberry_pi_controller.py on the Raspberry Pi
  - Run roslaunch pet_robot_ros ros_bridge.launch if using ROS
        """
    )
    
    parser.add_argument(
        "--full",
        action="store_true",
        help="Start all components (server, Pi sim, ROS if available)"
    )
    parser.add_argument(
        "--with-pi",
        action="store_true",
        help="Start server with Pi simulation"
    )
    
    args = parser.parse_args()
    
    launcher = RobotLauncher()
    
    if args.full:
        launcher.run(mode="full")
    elif args.with_pi:
        launcher.run(mode="server+pi")
    else:
        launcher.run(mode="server")

if __name__ == "__main__":
    main()
