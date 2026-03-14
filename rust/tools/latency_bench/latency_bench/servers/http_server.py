# SPDX-License-Identifier: MulanPSL-2.0
"""HTTP echo server for latency benchmark. POST body = request, response body = echo."""

import asyncio
from aiohttp import web


async def echo_handler(request: web.Request) -> web.Response:
    data = await request.read()
    return web.Response(body=data, content_type="application/octet-stream")


def main():
    app = web.Application()
    app.router.add_post("/echo", echo_handler)
    print("HTTP echo server listening on http://0.0.0.0:18080", flush=True)
    web.run_app(app, host="0.0.0.0", port=18080)


if __name__ == "__main__":
    main()
