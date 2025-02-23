# test_server.py (Run this on Raspberry Pi)
import asyncio
import websockets

async def echo(websocket):
    async for message in websocket:
        print(f"Received: {message}")
        await websocket.send(f"Echo: {message}")

async def main():
    async with websockets.serve(echo, "0.0.0.0", 8765):
        print("Test server running on ws://0.0.0.0:8765")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())

# test_client.py (Run this on your computer)
import asyncio
import websockets

async def test():
    uri = "ws://192.168.220.83:8765"
    try:
        async with websockets.connect(uri, timeout=20) as websocket:
            print("Connected!")
            await websocket.send("Hello!")
            response = await websocket.recv()
            print(f"Received: {response}")
    except Exception as e:
        print(f"Connection failed: {e}")

if __name__ == "__main__":
    asyncio.run(test())

