import asyncio
import json
from aiohttp import web


async def handle_websocket(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    async for msg in ws:
        if msg.type == web.WSMsgType.text:
            await ws.send_str(msg.data)

    return ws


async def send_data(app):
    while True:
        # Simulate data from the C++ application
        data = {
            "timestamp": asyncio.get_event_loop().time(),
            "torque1": 0.0,  # Replace with actual data
            "torque2": 0.0,  # Replace with actual data
            "position1": 0.0,  # Replace with actual data
            "position2": 0.0,  # Replace with actual data
            "speed1": 0.0,  # Replace with actual data
            "speed2": 0.0,  # Replace with actual data
            "acceleration1": 0.0,  # Replace with actual data
            "acceleration2": 0.0,  # Replace with actual data
        }
        if app["websocket"]:
            await app["websocket"].send_str(json.dumps(data))
        await asyncio.sleep(0.1)  # Adjust the frequency as needed


async def init_app():
    app = web.Application()
    app["websocket"] = None
    app.router.add_get("/ws", handle_websocket)
    app.on_startup.append(start_background_tasks)
    return app


async def start_background_tasks(app):
    app["send_data"] = asyncio.create_task(send_data(app))


web.run_app(init_app(), port=3000)
