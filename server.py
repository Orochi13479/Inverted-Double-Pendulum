import asyncio
import json
from aiohttp import web


async def handle_websocket(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    request.app["websocket"] = ws

    async for msg in ws:
        if msg.type == web.WSMsgType.text:
            await ws.send_str(msg.data)

    return ws


async def send_data(app):
    while True:
        # Simulate data from the C++ application
        data = {
            "timestamp": asyncio.get_event_loop().time(),
            "motor1_torque": 0.0,  # Replace with actual data
            "motor1_position": 0.0,  # Replace with actual data
            "motor1_velocity": 0.0,  # Replace with actual data
            "motor1_temperature": 0.0,  # Replace with actual data
            "motor1_fault": False,  # Replace with actual data
            "motor2_torque": 0.0,  # Replace with actual data
            "motor2_position": 0.0,  # Replace with actual data
            "motor2_velocity": 0.0,  # Replace with actual data
            "motor2_temperature": 0.0,  # Replace with actual data
            "motor2_fault": False,  # Replace with actual data
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
