# SPDX-License-Identifier: MulanPSL-2.0
import tempfile
import asyncio
import io
import json
import os
import unittest
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

from verify_rerun_ui import (api_body, bound_evidence, expected_outage_error, recovered,
                             read_api, recording_proof, retain_evidence, run_lifecycle,
                             run, scene_up, wait_proof, write_event)


class RerunUITests(unittest.TestCase):
    def test_repeated_expected_outage_retains_counts_and_full_log(self):
        from verify_rerun_ui import retain_expected_outage
        audit, expected, stream = {}, [], io.StringIO()
        text = 'Failed to load resource: net::ERR_CONNECTION_REFUSED'
        for timestamp in range(417):
            write_event(stream, {'kind': 'console', 'level': 'error', 'text': text}, audit)
            retain_expected_outage(expected, text, timestamp, audit)
        self.assertEqual(expected, [{'text': text, 'count': 417,
                                     'first_time': 0, 'last_time': 416}])
        self.assertEqual(audit['collection_counts']['expected_outage_errors'], 417)
        self.assertEqual(len(stream.getvalue().splitlines()), 417)
        self.assertFalse(audit.get('capacity_exhausted', False))
        for index in range(256):
            retain_expected_outage(expected, f'distinct-{index}', index, audit)
        self.assertEqual(len(expected), 256)
        self.assertTrue(audit['capacity_exhausted'])
        self.assertEqual(audit['omitted_items'], 1)

    def test_finalization_failures_cannot_turn_successful_proof_into_a_pass(self):
        async def check(failure, outage_repeats=0):
            handlers = {}
            page_handlers = {}
            locator = SimpleNamespace(wait_for=AsyncMock(),
                                      element_handle=AsyncMock(return_value=None))

            async def screenshot(**kwargs):
                if failure == 'late-pageerror':
                    page_handlers['pageerror'](RuntimeError('late page failure'))

            pages = [SimpleNamespace(view=view, goto=AsyncMock(),
                         locator=lambda selector: locator,
                         screenshot=AsyncMock(side_effect=screenshot),
                         on=lambda name, handler: page_handlers.update({name: handler}))
                     for view in ('3d', '2d')]
            context = SimpleNamespace(
                add_init_script=AsyncMock(), route=AsyncMock(),
                on=lambda name, handler: handlers.update({name: handler}),
                new_page=AsyncMock(side_effect=pages))
            browser = SimpleNamespace(new_context=AsyncMock(return_value=context),
                        close=AsyncMock(side_effect=(RuntimeError('close failure')
                                                   if failure == 'close' else None)))
            manager = AsyncMock()
            manager.__aenter__.return_value = SimpleNamespace(
                chromium=SimpleNamespace(launch=AsyncMock(return_value=browser)))

            async def state(page, *args):
                frame = SimpleNamespace(url='http://127.0.0.1/rerun?view=' + page.view)
                for path in ('/proxy/MessageProxyService/ReadMessages',
                             '/assets/rerun/viewer.wasm', '/assets/rerun/index.js',
                             '/assets/rerun/re_viewer.js'):
                    handlers['response'](SimpleNamespace(
                        url='http://127.0.0.1' + path, status=200,
                        request=SimpleNamespace(frame=frame)))
                return None, {'id': 'recording'}

            async def lifecycle(command, timeout, probe, summary):
                summary['outage_seen'] = True
                for _ in range(outage_repeats):
                    handlers['console'](SimpleNamespace(type='error',
                        text='Failed to load resource: net::ERR_CONNECTION_REFUSED',
                        location={'url': 'http://127.0.0.1/proxy'}))
                if failure == 'unexpected-console':
                    handlers['console'](SimpleNamespace(type='error',
                        text='unexpected viewer failure',
                        location={'url': 'http://127.0.0.1/proxy'}))
                return 0

            with tempfile.TemporaryDirectory() as directory:
                args = SimpleNamespace(base_url='http://127.0.0.1',
                                       output_dir=Path(directory), timeout=1,
                                       lifecycle_timeout=1, command=['unused'])
                with (patch.dict(sys.modules, {'playwright.async_api': SimpleNamespace(
                          async_playwright=lambda: manager)}),
                      patch('verify_rerun_ui.wait_state', state),
                      patch('verify_rerun_ui.wait_proof', AsyncMock(return_value={})),
                      patch('verify_rerun_ui.run_lifecycle', lifecycle),
                      patch('verify_rerun_ui.api_body', AsyncMock(return_value=b'{}'))):
                    code = await run(args)
                summary = json.loads((args.output_dir / 'summary.json').read_text())
                self.assertEqual(code, 1 if failure else 0)
                self.assertEqual(summary['passed'], not bool(failure))
                if failure == 'close':
                    self.assertIn('close failure', summary['error'])
                elif failure == 'late-pageerror':
                    self.assertIn('late page failure', summary['browser_errors'])
                elif failure == 'unexpected-console':
                    self.assertIn('unexpected viewer failure', summary['browser_errors'])

        for failure in (None, 'close', 'late-pageerror', 'unexpected-console'):
            with self.subTest(failure=failure):
                asyncio.run(check(failure))
        asyncio.run(check(None, outage_repeats=417))

    def test_nested_evidence_has_one_total_capacity_budget(self):
        audit = {}
        result = bound_evidence([['x' * 5000] * 256] * 256, audit)
        self.assertLess(len(json.dumps(result)), 2 * 1024 * 1024)
        self.assertTrue(audit['capacity_exhausted'])

    def test_log_and_error_overflow_fail_with_audited_omissions(self):
        audit, errors, stream = {}, [], io.StringIO()
        for i in range(10):
            retain_evidence(errors, f'unexpected-{i}', audit, 'browser_errors', limit=2)
            write_event(stream, {'kind': 'pageerror', 'text': 'failure' * 100}, audit, limit=100)
        self.assertEqual(errors, ['unexpected-0', 'unexpected-1'])
        self.assertEqual(audit['collection_counts']['browser_errors'], 10)
        self.assertEqual(audit['omitted_events'], 10)
        self.assertEqual(stream.getvalue(), '')
        self.assertTrue(audit['capacity_exhausted'])

    def test_never_returning_capture_has_a_phase_deadline(self):
        async def hang(*args):
            await asyncio.Event().wait()

        async def check():
            with patch('verify_rerun_ui.capture', hang):
                with self.assertRaisesRegex(RuntimeError, 'phase deadline'):
                    await asyncio.wait_for(wait_proof(None, None, Path('.'), 'before', '3d', .05), 1)
        asyncio.run(check())

    def test_lifecycle_timeout_and_cancellation_reap_owned_child(self):
        async def check(cancel):
            summary = {}
            command = [sys.executable, '-c', 'import time; time.sleep(60)']
            task = asyncio.create_task(run_lifecycle(command, .1 if not cancel else 60,
                                                     lambda: True, summary, grace=.1))
            if cancel:
                while 'child_pid' not in summary:
                    await asyncio.sleep(.01)
                task.cancel()
                with self.assertRaises(asyncio.CancelledError):
                    await task
            else:
                with self.assertRaisesRegex(RuntimeError, 'exceeded'):
                    await task
                self.assertTrue(summary['lifecycle_timed_out'])
            self.assertIsNotNone(summary['child_exit'])
            with self.assertRaises(ProcessLookupError):
                os.kill(summary['child_pid'], 0)
        asyncio.run(check(False))
        asyncio.run(check(True))

    def test_api_body_exhaustion_is_rejected_before_json_decode(self):
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'x' * (4 * 1024 * 1024 + 1))

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with self.assertRaisesRegex(RuntimeError, 'exceeds 4 MiB'):
                read_api(f'http://127.0.0.1:{server.server_port}', '/api/state')
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_cancelled_native_proof_reaps_its_process(self):
        async def check():
            children = []
            spawn = asyncio.create_subprocess_exec

            async def hanging_proof(*args, **kwargs):
                child = await spawn(sys.executable, '-c', 'import time; time.sleep(60)', **kwargs)
                children.append(child)
                return child

            with patch('verify_rerun_ui.asyncio.create_subprocess_exec', hanging_proof):
                with self.assertRaises(asyncio.TimeoutError):
                    await asyncio.wait_for(recording_proof(Path('unused'), Path('unused'), '3d', 'id'), .1)
            self.assertIsNotNone(children[0].returncode)
            with self.assertRaises(ProcessLookupError):
                os.kill(children[0].pid, 0)
        asyncio.run(check())

    def test_slow_trickle_api_has_a_total_deadline(self):
        import time

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                try:
                    for _ in range(100):
                        self.wfile.write(b' ')
                        self.wfile.flush()
                        time.sleep(.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            async def check():
                with self.assertRaisesRegex(RuntimeError, 'exceeded deadline'):
                    await asyncio.wait_for(api_body(f'http://127.0.0.1:{server.server_port}',
                                                    '/api/state', timeout=.2), 1)
            asyncio.run(check())
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_only_network_failure_counts_as_scene_outage(self):
        class Handler(BaseHTTPRequestHandler):
            status = 200

            def do_GET(self):
                self.send_response(self.status)
                self.end_headers()
                self.wfile.write(b'{"url": ""}')

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"
        try:
            self.assertTrue(scene_up(url))
            Handler.status = 500
            self.assertTrue(scene_up(url))
            Handler.status = 404
            self.assertTrue(scene_up(url))
        finally:
            server.shutdown()
            server.server_close()
            thread.join()
        self.assertFalse(scene_up(url))

    def test_only_expected_stream_errors_during_lifecycle_are_exempt(self):
        source = "rerun+http://127.0.0.1:19878/proxy"
        location = "http://127.0.0.1:19878/assets/rerun/re_viewer"
        error = ("ERROR re_viewer::app::logic: Data source failed: js api error: "
                 f"TypeError: network error\n- Source: {source}\n")
        self.assertTrue(expected_outage_error(error, "lifecycle", source, location))
        self.assertFalse(expected_outage_error(error, "before", source, location))
        self.assertFalse(expected_outage_error(error, "after", source, location))
        self.assertFalse(expected_outage_error(error, "lifecycle", source + "wrong", location))
        self.assertFalse(expected_outage_error(error, "lifecycle", source, "https://third-party.invalid/viewer.js"))
        self.assertFalse(expected_outage_error("TypeError: unrelated runtime error", "lifecycle", source, location))
        for code in ("ERR_CONNECTION_REFUSED", "ERR_INCOMPLETE_CHUNKED_ENCODING"):
            message = "Failed to load resource: net::" + code
            self.assertTrue(expected_outage_error(message, "lifecycle", source, location))
            self.assertFalse(expected_outage_error(message, "before", source, location))
            self.assertFalse(expected_outage_error(message, "after", source, location))
            self.assertFalse(expected_outage_error(message, "lifecycle", source, "https://third-party.invalid/a.js"))
            self.assertFalse(expected_outage_error(message, "lifecycle", source, "http://127.0.0.1:19879/api/state"))
            self.assertFalse(expected_outage_error("Application failure: net::" + code, "lifecycle", source, location))
        for code in ("ERR_CERT_AUTHORITY_INVALID", "ERR_INVALID_AUTH_CREDENTIALS"):
            self.assertFalse(expected_outage_error("Failed to load resource: net::" + code,
                                                  "lifecycle", source, location))

    def test_lifecycle_command_is_required(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable, str(Path(__file__).with_name("verify_rerun_ui.py")),
                                     "--base-url", "http://127.0.0.1:1", "--output-dir", directory],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("lifecycle command", result.stderr)

    def test_same_browser_must_receive_new_recording_without_reload(self):
        before = {"token": "frame", "page_token": "same-page", "starts": 1, "id": "old", "range": None,
                  "events": [{"recording_id": "old"}]}
        self.assertFalse(recovered(before, before))
        after = {**before, "id": "new", "events": [{"recording_id": "new"}]}
        self.assertTrue(recovered(before, after))
        with self.assertRaisesRegex(RuntimeError, "restarted"):
            recovered(before, {**after, "page_token": "fresh-page"})
        self.assertTrue(recovered(before, {**after, "token": "fresh-iframe", "starts": 2}))


if __name__ == "__main__":
    unittest.main()
