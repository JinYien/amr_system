from __future__ import annotations
import asyncio
import json
from pathlib import Path
from typing import TYPE_CHECKING
from aiohttp import WSMsgType, web

if TYPE_CHECKING:
    from web_interface.node import WebInterfaceNode

SNAPSHOT_PERIOD = 0.2


def create_web_app(node: "WebInterfaceNode", pages_directory: Path, loop: asyncio.AbstractEventLoop) -> web.Application:
    app = web.Application()
    open_sockets: set[web.WebSocketResponse] = set()

    def broadcast_to_browsers(payload: dict):
        message = json.dumps(payload)
        for socket in list(open_sockets):
            if socket.closed:
                open_sockets.discard(socket)
                continue
            asyncio.run_coroutine_threadsafe(socket.send_str(message), loop)

    node.set_browser_sender(broadcast_to_browsers)

    async def serve_index(request):
        return web.FileResponse(pages_directory / "index.html")

    async def serve_nipplejs(request):
        return web.FileResponse(pages_directory / "nipplejs.min.js")

    async def redirect_to_log(request):
        host = request.host.rsplit(":", 1)[0]
        raise web.HTTPFound(f"{request.scheme}://{host}:{node.settings.logging.port}/")

    async def handle_websocket(request):
        socket = web.WebSocketResponse()
        await socket.prepare(request)
        open_sockets.add(socket)

        async for message in socket:
            if message.type == WSMsgType.TEXT:
                try:
                    event = json.loads(message.data)
                except json.JSONDecodeError:
                    continue
                node.handle_browser_event(event)
            elif message.type == WSMsgType.ERROR:
                break

        open_sockets.discard(socket)
        return socket

    app.router.add_get("/", serve_index)
    app.router.add_get("/nipplejs.js", serve_nipplejs)
    app.router.add_get("/log", redirect_to_log)
    app.router.add_get("/ws", handle_websocket)
    return app


def create_log_app(node: "WebInterfaceNode", pages_directory: Path) -> web.Application:
    app = web.Application()
    logger = node.topic_logger

    async def serve_page(request):
        return web.FileResponse(pages_directory / "log.html")

    async def redirect_to_control(request):
        host = request.host.rsplit(":", 1)[0]
        raise web.HTTPFound(f"{request.scheme}://{host}:{node.settings.server.port}/")

    async def serve_download(request):
        path = logger.file_path(request.match_info["name"])
        if path is None:
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers={"Content-Disposition": f'attachment; filename="{path.name}"'})

    async def handle_websocket(request):
        socket = web.WebSocketResponse()
        await socket.prepare(request)

        async def push_snapshots():
            while not socket.closed:
                await socket.send_str(json.dumps({"type": "snapshot", **logger.snapshot()}))
                await asyncio.sleep(SNAPSHOT_PERIOD)

        pusher = asyncio.ensure_future(push_snapshots())
        try:
            async for message in socket:
                if message.type != WSMsgType.TEXT:
                    continue
                try:
                    event = json.loads(message.data)
                except json.JSONDecodeError:
                    continue
                if event.get("type") == "log_start":
                    logger.start()
                elif event.get("type") == "log_stop":
                    name = logger.stop()
                    if name:
                        await socket.send_str(json.dumps({"type": "log_saved", "name": name}))
        finally:
            pusher.cancel()
        return socket

    app.router.add_get("/", serve_page)
    app.router.add_get("/control", redirect_to_control)
    app.router.add_get("/ws", handle_websocket)
    app.router.add_get("/download/{name}", serve_download)
    return app


def start_web_server(node: "WebInterfaceNode", pages_directory: Path):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    server = node.settings.server

    app = create_web_app(node, pages_directory, loop)
    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    loop.run_until_complete(web.TCPSite(runner, server.host, server.port).start())

    log_app = create_log_app(node, pages_directory)
    log_runner = web.AppRunner(log_app)
    loop.run_until_complete(log_runner.setup())
    loop.run_until_complete(web.TCPSite(log_runner, server.host, node.settings.logging.port).start())

    loop.run_forever()
