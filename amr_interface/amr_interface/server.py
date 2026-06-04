#!/usr/bin/env python3

from __future__ import annotations
import asyncio
import json
from pathlib import Path
from typing import TYPE_CHECKING
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory

if TYPE_CHECKING:
    from amr_interface.main import InterfaceNode


def resolve_web_dir() -> Path:
    try:
        candidate = Path(get_package_share_directory("amr_interface")) / "webpage"
        if (candidate / "index.html").exists():
            return candidate
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / "webpage"


def build_app(node: InterfaceNode, web_dir: Path, loop: asyncio.AbstractEventLoop) -> web.Application:
    app = web.Application()
    websockets: set[web.WebSocketResponse] = set()

    def broadcast(payload: dict):
        message = json.dumps(payload)
        for ws in list(websockets):
            if ws.closed:
                websockets.discard(ws)
                continue
            asyncio.run_coroutine_threadsafe(ws.send_str(message), loop)

    node.set_topic_callback(broadcast)

    async def serve_index(_request):
        return web.FileResponse(web_dir / "index.html")

    async def serve_nipplejs(_request):
        return web.FileResponse(web_dir / "nipplejs.min.js")

    async def websocket_handler(request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        websockets.add(ws)

        async for incoming in ws:
            if incoming.type == WSMsgType.TEXT:
                try:
                    payload = json.loads(incoming.data)
                except json.JSONDecodeError:
                    continue
                node.handle_event(payload)
            elif incoming.type == WSMsgType.ERROR:
                break

        websockets.discard(ws)
        return ws

    app.router.add_get("/", serve_index)
    app.router.add_get("/nipplejs.js", serve_nipplejs)
    app.router.add_get("/ws", websocket_handler)
    return app


def run_web_server(node: InterfaceNode, web_dir: Path):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    app = build_app(node, web_dir, loop)
    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())

    http = node.settings.http
    site = web.TCPSite(runner, http.host, http.port)
    loop.run_until_complete(site.start())
    node.get_logger().info(f"Available at http://{http.host}:{http.port}/")
    loop.run_forever()
