import asyncio
import websockets
import signal
from aiohttp import web

# Global flag for indicating if Ctrl+C was pressed
ctrl_c_pressed = False


# Signal handler function
def signal_handler(signal, frame):
    global ctrl_c_pressed
    ctrl_c_pressed = True
    print("\nCtrl+C pressed. Shutting down...")


signal.signal(signal.SIGINT, signal_handler)

# WebSocket handler for forwarding data to clients
clients = set()


async def websocket_handler(websocket, path):
    clients.add(websocket)
    try:
        async for message in websocket:
            pass
    finally:
        clients.remove(websocket)


# Function to broadcast data to all connected clients
async def broadcast_data(data):
    if clients:
        await asyncio.wait([client.send(data) for client in clients])


# Function to receive data from the C++ WebSocket server and forward it
async def forward_data():
    uri = "ws://localhost:9002"
    while not ctrl_c_pressed:
        try:
            async with websockets.connect(uri) as websocket:
                async for message in websocket:
                    print("Received from C++ server:", message)  # Debug print
                    await broadcast_data(message)
        except (websockets.ConnectionClosed, OSError) as e:
            print(f"Connection lost: {e}. Reconnecting in 2 seconds...")
            await asyncio.sleep(2)


# HTTP handler to serve the HTML file
async def index(request):
    return web.FileResponse("index.html")


# Setup the HTTP server
app = web.Application()
app.router.add_get("/", index)


async def main():
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "localhost", 8080)
    await site.start()

    websocket_server = websockets.serve(
        websocket_handler, "localhost", 9003
    )  # Clients connect to this server

    await asyncio.gather(websocket_server, forward_data())


asyncio.run(main())
