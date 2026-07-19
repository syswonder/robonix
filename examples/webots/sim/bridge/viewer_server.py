#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Serve WebotsView and proxy/cache resources needed by remote browsers."""

import hashlib
import mimetypes
import os
import urllib.error
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


VIEWER_ROOT = os.environ.get(
    "WEBOTS_VIEWER_ROOT", "/usr/local/webots/resources/web/streaming_viewer"
)
VIEWER_HOST = os.environ.get("WEBOTS_VIEWER_HOST", "0.0.0.0")
VIEWER_PORT = int(os.environ.get("WEBOTS_VIEWER_PORT", "8080"))
PUBLIC_STREAM_PORT = int(os.environ.get("WEBOTS_PUBLIC_STREAM_PORT", "1235"))
RAW_STREAM_HTTP = os.environ.get(
    "WEBOTS_RAW_STREAM_HTTP", "http://127.0.0.1:1234"
).rstrip("/")
WWI_UPSTREAM = "https://cyberbotics.com/wwi/R2025a/"
GITHUB_RAW = "https://raw.githubusercontent.com/"
GITHUB_CDN = "https://fastly.jsdelivr.net/gh/"
CACHE_ROOT = "/tmp/webots-viewer-cache"
WEBOTS_RESOURCES_ROOTS = (
    "/usr/local/webots/resources",
    "/usr/local/webots/resources/wren",
    "/usr/local/webots/resources/projects",
)


class ViewerHandler(SimpleHTTPRequestHandler):
    def translate_path(self, path: str) -> str:
        original = self.directory
        self.directory = VIEWER_ROOT
        try:
            return super().translate_path(path)
        finally:
            self.directory = original

    def do_GET(self) -> None:
        """Route local viewer assets and proxied Webots resources."""
        if self.path.split("?", 1)[0] in ("/", "/index.html"):
            self.serve_index()
        elif self.path == "/healthz":
            self.send_response(204)
            self.end_headers()
        elif self.path.startswith("/stream/"):
            self.proxy_stream()
        elif self.path.startswith("/github/"):
            self.proxy_github()
        elif self.path.startswith(("/wwi/", "/wwi-full/")):
            self.proxy_wwi()
        else:
            super().do_GET()

    def serve_index(self) -> None:
        """Serve the viewer page with its runtime WebSocket port injected."""
        try:
            with open(os.path.join(VIEWER_ROOT, "index.html"), "rb") as source:
                content = source.read().replace(
                    b"__WEBOTS_STREAM_PORT__", str(PUBLIC_STREAM_PORT).encode()
                )
        except OSError as error:
            self.send_error(500, f"Viewer index unavailable: {error}")
            return
        self.send_content(content, "index.html", 0)

    @staticmethod
    def safe_relative(path: str, prefix: str) -> str | None:
        relative = path.removeprefix(prefix).split("?", 1)[0]
        return None if ".." in relative.split("/") else relative

    @staticmethod
    def cache_path(namespace: str, relative: str) -> str:
        digest = hashlib.sha256(f"{namespace}/{relative}".encode()).hexdigest()
        os.makedirs(CACHE_ROOT, exist_ok=True)
        return os.path.join(CACHE_ROOT, digest)

    @staticmethod
    def fetch(url: str, timeout: int = 60) -> bytes:
        request = urllib.request.Request(
            url, headers={"User-Agent": "Robonix-Webots-Viewer/1.0"}
        )
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.read()

    def send_content(self, content: bytes, relative: str, max_age: int) -> None:
        content_type = mimetypes.guess_type(relative)[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", f"public, max-age={max_age}")
        self.end_headers()
        self.wfile.write(content)

    def proxy_wwi(self) -> None:
        prefix = "/wwi-full/" if self.path.startswith("/wwi-full/") else "/wwi/"
        relative = self.safe_relative(self.path, prefix)
        if relative is None:
            self.send_error(400)
            return

        cache_path = self.cache_path("wwi", relative)
        try:
            if not os.path.exists(cache_path):
                content = self.local_webots_resource(relative)
                if content is None:
                    content = self.fetch(WWI_UPSTREAM + relative)
                content = self.patch_wwi(relative, content, prefix)
                with open(cache_path, "wb") as output:
                    output.write(content)
            with open(cache_path, "rb") as source:
                content = source.read()
        except (OSError, urllib.error.URLError) as error:
            self.send_error(502, f"Webots asset proxy failed: {error}")
            return
        self.send_content(content, relative, 86400)

    @staticmethod
    def local_webots_resource(relative: str) -> bytes | None:
        for root in WEBOTS_RESOURCES_ROOTS:
            real_root = os.path.realpath(root)
            candidate = os.path.realpath(os.path.join(root, relative))
            if candidate.startswith(real_root + os.sep) and os.path.isfile(candidate):
                with open(candidate, "rb") as source:
                    return source.read()
        return None

    @staticmethod
    def patch_wwi(relative: str, content: bytes, prefix: str) -> bytes:
        if relative == "WebotsView.js":
            return content.replace(WWI_UPSTREAM.encode(), prefix.encode())
        if relative == "webots.js":
            return content.replace(
                b"const httpServerUrl = 'http' + this.url.slice(2);",
                b"const httpServerUrl = window.location.origin + '/stream';",
            )
        if relative == "ImageLoader.js":
            return content.replace(
                b"img.setAttribute('crossOrigin', '');",
                b"if (new URL(src, location.href).origin !== location.origin) img.setAttribute('crossOrigin', '');",
            ).replace(
                b"img.src = 'https://cyberbotics.com/wwi/images/missing_texture.png';",
                b"img.src = 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+/p9sAAAAASUVORK5CYII=';",
            ).replace(
                b"url = url.replace('webots://', 'https://raw.githubusercontent.com/' + ImageLoader.repository + '/webots/' + ImageLoader.branch + '/');",
                b"url = url.replace('webots://', window.location.origin + '/github/' + ImageLoader.repository + '/webots/' + ImageLoader.branch + '/');",
            ).replace(
                b"    if (typeof prefix !== 'undefined' && !url.startsWith('http')) {",
                b"    if (url.startsWith('https://raw.githubusercontent.com/'))\n      url = window.location.origin + '/github/' + url.substring('https://raw.githubusercontent.com/'.length);\n    if (typeof prefix !== 'undefined' && !url.startsWith('http')) {",
            )
        if relative == "MeshLoader.js":
            return content.replace(
                b"worldsPath = MeshLoader.currentWorld;\n      worldsPath = worldsPath.substring(0, worldsPath.lastIndexOf('/')) + '/';",
                b"worldsPath = MeshLoader.currentWorld;\n      worldsPath = typeof worldsPath !== 'undefined' ? worldsPath.substring(0, worldsPath.lastIndexOf('/')) + '/' : '';",
            )
        if relative == "Toolbar.js":
            return content.replace(
                b"    this.createWorldSelectionPane();\n    this.worldSelectionButton.addEventListener('mouseup', _ => this.#changeWorldSelectionPaneVisibility(_));\n    window.addEventListener('click', _ => this.#closeWorldSelectionPaneOnClick(_));\n\n    if (!(typeof this.parentNode.showWorldSelection === 'undefined' || this.parentNode.showWorldSelection) ||\n      this.#view.worlds.length <= 1)\n      this.worldSelectionButton.style.display = 'none';\n    else\n      this.minWidth += 44;",
                b"    if (!(typeof this.parentNode.showWorldSelection === 'undefined' || this.parentNode.showWorldSelection) ||\n      typeof this.#view.worlds === 'undefined' || this.#view.worlds.length <= 1) {\n      this.worldSelectionButton.style.display = 'none';\n      return;\n    }\n\n    this.createWorldSelectionPane();\n    this.worldSelectionButton.addEventListener('mouseup', _ => this.#changeWorldSelectionPaneVisibility(_));\n    window.addEventListener('click', _ => this.#closeWorldSelectionPaneOnClick(_));\n    this.minWidth += 44;",
            )
        return content

    def proxy_github(self) -> None:
        relative = self.safe_relative(self.path, "/github/")
        if relative is None:
            self.send_error(400)
            return
        cache_path = self.cache_path("github", relative)
        try:
            if not os.path.exists(cache_path):
                parts = relative.split("/", 3)
                urls = [GITHUB_RAW + relative]
                if len(parts) == 4:
                    owner, repository, reference, path = parts
                    urls = [
                        f"{GITHUB_CDN}{owner}/{repository}@{reference}/{path}",
                        f"https://cdn.jsdelivr.net/gh/{owner}/{repository}@{reference}/{path}",
                        f"https://gh-proxy.com/{GITHUB_RAW}{relative}",
                    ]
                last_error: Exception | None = None
                for url in urls:
                    try:
                        content = self.fetch(url, timeout=90)
                        break
                    except (OSError, urllib.error.URLError) as error:
                        last_error = error
                else:
                    raise OSError(f"all asset mirrors failed: {last_error}")
                with open(cache_path, "wb") as output:
                    output.write(content)
            with open(cache_path, "rb") as source:
                content = source.read()
        except (OSError, urllib.error.URLError) as error:
            self.send_error(502, f"GitHub asset proxy failed: {error}")
            return
        self.send_content(content, relative, 86400)

    def proxy_stream(self) -> None:
        path = self.path.removeprefix("/stream")
        try:
            request = urllib.request.Request(
                RAW_STREAM_HTTP + path,
                headers={"User-Agent": "Robonix-Webots-Viewer/1.0"},
            )
            with urllib.request.urlopen(request, timeout=60) as response:
                content = response.read()
                content_type = response.headers.get_content_type()
            if path.endswith(".js"):
                content = content.replace(
                    b"https://cyberbotics.com/wwi/R2025a/", b"/wwi-full/"
                )
        except (OSError, urllib.error.URLError) as error:
            self.send_error(502, f"Webots stream asset proxy failed: {error}")
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "public, max-age=3600")
        self.end_headers()
        self.wfile.write(content)


if __name__ == "__main__":
    server = ThreadingHTTPServer((VIEWER_HOST, VIEWER_PORT), ViewerHandler)
    server.daemon_threads = True
    print(
        f"viewer listening on http://{VIEWER_HOST}:{VIEWER_PORT} "
        f"(WebSocket port {PUBLIC_STREAM_PORT})",
        flush=True,
    )
    server.serve_forever()
