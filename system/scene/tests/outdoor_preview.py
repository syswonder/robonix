"""Serve the real Scene page for browser tests; Playwright supplies API fixtures."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys

root=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(root/'robonix/system/scene'),str(root/'robonix/pylib/robonix-api')]
from scene_service.web import _shell_page, _asset


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path!='/outdoor':
            self.send_response(404);self.end_headers();return
        body=_asset('outdoor.html').replace('__OUTDOOR_COSTMAPS_JS__',_asset('outdoor_costmaps.js'))
        data=_shell_page('/outdoor',body,'Scene outdoor preview').encode()
        self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.end_headers();self.wfile.write(data)

    def log_message(self,*_):pass


if __name__=='__main__':
    ThreadingHTTPServer(('127.0.0.1',18763),Handler).serve_forever()
