import asyncio
import websockets
import json
from raspberry_pi_motor_control import move_forward, move_backward, turn_left, turn_right, stop_motors

async def handle_movement(websocket):
    async for message in websocket:
        try:
            data = json.loads(message)
            command = data.get("command")
            speed = int(data.get("speed", 75))
            if command == "move_forward":
                move_forward(speed)
            elif command == "move_backward":
                move_backward(speed)
            elif command == "turn_left":
                turn_left(speed)
            elif command == "turn_right":
                turn_right(speed)
            elif command == "stop":
                stop_motors()
        except Exception as e:
            print(f"Error handling movement command: {e}")

async def main():
    # Listen on all interfaces, port 8000
    async with websockets.serve(handle_movement, "0.0.0.0", 8000):
        print("WebSocket server started on ws://0.0.0.0:8000/")
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("WebSocket server stopped.")
