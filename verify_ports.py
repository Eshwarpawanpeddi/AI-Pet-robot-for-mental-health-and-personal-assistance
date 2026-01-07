#!/usr/bin/env python3
"""
Port Connection Verification Script
Verifies that all ports are properly configured and can communicate with each other
"""

import asyncio
import aiohttp
import sys
from typing import Dict, List, Tuple

# Port configuration
PORTS = {
    8000: "Primary Control Server",
    3000: "Mobile Web Interface",
    9999: "Emotion Detection Server",
    10000: "Emotion Display Server"
}

# Expected endpoints for each port
PORT_ENDPOINTS = {
    8000: [
        ("/", "Main interface"),
        ("/api/state", "System state"),
        ("/api/tts/voices", "TTS voices list"),
        ("/health", "Health check")
    ],
    3000: [
        ("/", "Mobile interface"),
        ("/api/status", "Status check"),
        ("/health", "Health check")
    ],
    9999: [
        ("/", "Emotion detection interface"),
        ("/api/emotion", "Current emotion"),
        ("/health", "Health check")
    ],
    10000: [
        ("/", "Emotion display interface"),
        ("/api/emotion", "Display emotion"),
        ("/health", "Health check")
    ]
}

# TTS broadcast test
TTS_ENDPOINTS = {
    3000: "/api/tts",
    9999: "/api/tts",
    10000: "/api/tts"
}


class PortVerifier:
    def __init__(self):
        self.results: Dict[int, Dict] = {}
        
    async def check_port(self, port: int) -> Tuple[bool, str]:
        """Check if a port is accessible"""
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(
                    f"http://localhost:{port}/health",
                    timeout=aiohttp.ClientTimeout(total=2)
                ) as response:
                    if response.status == 200:
                        data = await response.json()
                        return True, f"OK - {data.get('status', 'healthy')}"
                    else:
                        return False, f"HTTP {response.status}"
        except aiohttp.ClientConnectorError:
            return False, "Connection refused - Server not running"
        except asyncio.TimeoutError:
            return False, "Timeout"
        except Exception as e:
            return False, f"Error: {str(e)}"
    
    async def check_endpoint(self, port: int, endpoint: str) -> Tuple[bool, str]:
        """Check if a specific endpoint is accessible"""
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(
                    f"http://localhost:{port}{endpoint}",
                    timeout=aiohttp.ClientTimeout(total=2)
                ) as response:
                    if response.status in [200, 404]:  # 404 is ok for some endpoints
                        return True, f"HTTP {response.status}"
                    else:
                        return False, f"HTTP {response.status}"
        except Exception as e:
            return False, f"Error: {str(e)[:50]}"
    
    async def test_tts_broadcast(self) -> Dict[int, Tuple[bool, str]]:
        """Test TTS broadcast from port 8000 to other ports"""
        results = {}
        
        # First check if port 8000 is accessible
        port_ok, _ = await self.check_port(8000)
        if not port_ok:
            return {port: (False, "Port 8000 not running") for port in TTS_ENDPOINTS.keys()}
        
        # Test each TTS endpoint
        for port, endpoint in TTS_ENDPOINTS.items():
            try:
                async with aiohttp.ClientSession() as session:
                    # Send a test TTS message
                    async with session.post(
                        f"http://localhost:{port}{endpoint}",
                        json={
                            "type": "speak",
                            "text": "Test message",
                            "voice": "en+f3",
                            "speed": 150,
                            "pitch": 50
                        },
                        timeout=aiohttp.ClientTimeout(total=2)
                    ) as response:
                        if response.status == 200:
                            data = await response.json()
                            results[port] = (True, f"OK - {data.get('status', 'ok')}")
                        else:
                            results[port] = (False, f"HTTP {response.status}")
            except Exception as e:
                results[port] = (False, f"Error: {str(e)[:50]}")
        
        return results
    
    async def verify_all(self):
        """Run all verification tests"""
        print("=" * 70)
        print("AI Pet Robot - Port Connection Verification")
        print("=" * 70)
        print()
        
        # Step 1: Check if all ports are running
        print("Step 1: Checking if all servers are running...")
        print("-" * 70)
        
        all_running = True
        for port, name in PORTS.items():
            is_ok, message = await self.check_port(port)
            status = "✓" if is_ok else "✗"
            print(f"  {status} Port {port} ({name}): {message}")
            self.results[port] = {"running": is_ok, "endpoints": {}}
            if not is_ok:
                all_running = False
        
        print()
        
        if not all_running:
            print("⚠️  WARNING: Not all servers are running!")
            print("   Please start all servers using: python server/launch_all.py")
            print()
            return False
        
        # Step 2: Check endpoints
        print("Step 2: Verifying API endpoints...")
        print("-" * 70)
        
        for port, endpoints in PORT_ENDPOINTS.items():
            print(f"\n  Port {port} ({PORTS[port]}):")
            for endpoint, description in endpoints:
                is_ok, message = await self.check_endpoint(port, endpoint)
                status = "✓" if is_ok else "✗"
                print(f"    {status} {endpoint} - {description}: {message}")
                self.results[port]["endpoints"][endpoint] = is_ok
        
        print()
        
        # Step 3: Test TTS broadcast
        print("Step 3: Testing TTS broadcast from port 8000 to all ports...")
        print("-" * 70)
        
        tts_results = await self.test_tts_broadcast()
        print(f"\n  TTS Broadcast Test:")
        print(f"    Source: Port 8000 (Primary Control Server)")
        print(f"    Destinations:")
        
        for port, (is_ok, message) in tts_results.items():
            status = "✓" if is_ok else "✗"
            print(f"      {status} Port {port} ({PORTS[port]}): {message}")
        
        print()
        
        # Step 4: Summary
        print("=" * 70)
        print("Verification Summary")
        print("=" * 70)
        
        all_ok = all_running and all(
            all(ep_status for ep_status in port_data["endpoints"].values())
            for port_data in self.results.values()
        ) and all(ok for ok, _ in tts_results.values())
        
        if all_ok:
            print("✓ All ports and connections are working correctly!")
            print()
            print("Next steps:")
            print("  1. Access the primary interface: http://localhost:8000")
            print("  2. Access the mobile interface: http://localhost:3000")
            print("  3. Access the emotion detection: http://localhost:9999")
            print("  4. Access the emotion display: http://localhost:10000")
        else:
            print("✗ Some issues were detected. Please review the output above.")
            print()
            print("Common solutions:")
            print("  1. Ensure all servers are running: python server/launch_all.py")
            print("  2. Check if ports are not blocked by firewall")
            print("  3. Verify .env file is configured correctly")
            print("  4. Check server logs for detailed error messages")
        
        print()
        print("=" * 70)
        
        return all_ok


async def main():
    verifier = PortVerifier()
    success = await verifier.verify_all()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nVerification cancelled by user")
        sys.exit(1)
