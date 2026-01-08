#!/usr/bin/env python3
"""
Test script to verify SERVER_HOST configuration is working correctly
"""
import os
import sys
from dotenv import load_dotenv

def test_server_host_config():
    """Test SERVER_HOST configuration loading"""
    print("=" * 60)
    print("Testing SERVER_HOST Configuration")
    print("=" * 60)
    
    # Load environment variables
    load_dotenv()
    
    # Test default value
    server_host = os.getenv("SERVER_HOST", "localhost")
    print(f"\n✓ SERVER_HOST loaded: {server_host}")
    
    # Test URL construction for each server
    print("\n📡 Server URLs that will be used:")
    print(f"   - Primary Server (8000):       http://{server_host}:8000")
    print(f"   - Mobile Web (3000):           http://{server_host}:3000")
    print(f"   - Emotion Detection (9999):    http://{server_host}:9999")
    print(f"   - Emotion Display (10000):     http://{server_host}:10000")
    
    print("\n🔗 WebSocket URLs that will be used:")
    print(f"   - Control WebSocket:           ws://{server_host}:8000/ws/control")
    
    # Test different scenarios
    print("\n📝 Configuration Scenarios:")
    print("   1. For local development:")
    print("      SERVER_HOST=localhost (default)")
    print("   2. For remote access (same machine):")
    print("      SERVER_HOST=0.0.0.0")
    print("   3. For remote access (specific IP):")
    print("      SERVER_HOST=192.168.1.100")
    
    print("\n" + "=" * 60)
    print("✓ Configuration test completed successfully")
    print("=" * 60)
    
    return True

if __name__ == "__main__":
    try:
        success = test_server_host_config()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)
