#!/usr/bin/env python3
import asyncio
import websockets
import json
from rplidar import RPLidar
import signal
import sys

class LidarServer:
    def __init__(self, host="0.0.0.0", port=8765, lidar_port="/dev/ttyUSB0", baudrate=115200):
        self.host = host
        self.port = port
        self.lidar_port = lidar_port
        self.baudrate = baudrate
        self.server = None
        self.lidar = None

        # Setup graceful shutdown
        signal.signal(signal.SIGINT, self.handle_shutdown)
        signal.signal(signal.SIGTERM, self.handle_shutdown)

    def handle_shutdown(self, signum, frame):
        print("\nShutting down gracefully...")
        if self.lidar:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        if self.server:
            self.server.close()
        sys.exit(0)

    async def handle_client(self, websocket, path):
        """Handle individual client connections"""
        client_id = id(websocket)
        print(f"New client connected! ID: {client_id}")

        try:
            self.lidar = RPLidar(self.lidar_port, baudrate=self.baudrate)
            print(f"LiDAR connected on {self.lidar_port}")

            # Main data streaming loop
            for scan in self.lidar.iter_scans():
                data = [
                    {
                        "angle": round(angle, 2),
                        "distance": round(distance, 2),
                        "quality": quality
                    }
                    for quality, angle, distance in scan
                ]

                try:
                    await websocket.send(json.dumps(data))
                    await asyncio.sleep(0.1)  # Adjust rate if needed
                except websockets.exceptions.ConnectionClosed:
                    print(f"Client {client_id} disconnected")
                    break

        except Exception as e:
            print(f"Error with client {client_id}: {e}")

        finally:
            if self.lidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            print(f"Cleaned up resources for client {client_id}")

    async def start(self):
        """Start the WebSocket server"""
        self.server = await websockets.serve(
            self.handle_client,
            self.host,
            self.port
        )
        print(f"LiDAR WebSocket server running on ws://{self.host}:{self.port}")
        await self.server.wait_closed()

def main():
    server = LidarServer()
    try:
        asyncio.run(server.start())
    except KeyboardInterrupt:
        print("\nServer shutdown requested")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()
