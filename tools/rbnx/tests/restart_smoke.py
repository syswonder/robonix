"""Exercise the real CLI with an Atlas-only stack; never declare robot hardware."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--binary', required=True, help='Built rbnx executable')
parser.add_argument('--output-dir', help='Existing directory for test artifacts')
args = parser.parse_args()
BINARY = Path(args.binary).resolve()
BASE = Path(tempfile.mkdtemp(prefix='rbnx-restart-check-', dir=args.output_dir))
manifest = BASE / 'robonix_manifest.yaml'
with socket.socket() as sock:
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
manifest.write_text(f'name: lifecycle-check\nsystem:\n  atlas:\n    listen: "127.0.0.1:{port}"\n')
state = BASE / 'rbnx-boot/state.json'
processes = []


def wait_for(condition, seconds, label):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if condition():
            return
        time.sleep(.1)
    raise AssertionError('timeout: ' + label)


def boot(tag):
    log = open(BASE / f'{tag}.log', 'w')
    p = subprocess.Popen([str(BINARY), 'boot', '-f', str(manifest), '--no-update-check', '-v'],
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(p)

    def active():
        if p.poll() is not None:
            raise AssertionError(f'{tag}: boot exited {p.returncode}; see {BASE}/{tag}.log')
        try:
            d = json.loads(state.read_text())
            with socket.create_connection(('127.0.0.1', port), timeout=.2):
                return bool(d.get('components'))
        except (OSError, ValueError):
            return False
    wait_for(active, 20, 'Atlas activation')
    return p, json.loads(state.read_text())


def detached(boot_id, tag):
    log = open(BASE / f'{tag}.log', 'w')
    p = subprocess.Popen(['python3', '-u', '-c',
                          'import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                          'print("ready",flush=True); time.sleep(180)'],
                         env={**os.environ, 'RBNX_BOOT_ID': boot_id},
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(p)
    wait_for(lambda: 'ready' in (BASE / f'{tag}.log').read_text(), 5, 'detached child ready')
    return p


def shutdown():
    t0 = time.monotonic()
    p = subprocess.run([str(BINARY), 'shutdown', '-f', str(manifest)],
                       capture_output=True, text=True, timeout=40)
    (BASE / f'shutdown-{time.time_ns()}.log').write_text(p.stdout + p.stderr)
    assert p.returncode == 0, p.stdout + p.stderr
    return round(time.monotonic() - t0, 3)


report = {}
try:
    owner, d = boot('first')
    orphan = detached(d['boot_id'], 'owned-detached')
    foreign = detached('foreign-' + d['boot_id'], 'foreign-detached')
    sentinel = BASE / 'rbnx-boot/logs/sentinel.log'
    sentinel.write_text('preserve old evidence')
    t0 = time.monotonic()
    duplicate = subprocess.run([str(BINARY), 'boot', '-f', str(manifest), '--no-update-check'],
                               capture_output=True, text=True, timeout=3)
    assert duplicate.returncode != 0
    assert sentinel.read_text() == 'preserve old evidence'
    report['duplicate_refused_s'] = round(time.monotonic() - t0, 3)
    report['shutdown_with_detached_child_s'] = shutdown()
    wait_for(lambda: orphan.poll() is not None and owner.poll() is not None, 5, 'owned processes exit')
    assert foreign.poll() is None, 'shutdown killed another boot'
    assert not state.exists()
    report['foreign_child_preserved'] = True
    report['detached_child_stopped'] = True

    second, d = boot('second')
    report['immediate_restart_shutdown_s'] = shutdown()
    wait_for(lambda: second.poll() is not None, 5, 'second boot exit')

    third, d = boot('third')
    orphan2 = detached(d['boot_id'], 'crash-detached')
    third.kill()
    third.wait()
    wait_for(lambda: orphan2.poll() is not None and not state.exists(), 20, 'watchdog crash cleanup')
    report['watchdog_cleanup_verified'] = True
    fourth, d = boot('fourth')
    report['post_crash_restart_shutdown_s'] = shutdown()
    wait_for(lambda: fourth.poll() is not None, 5, 'fourth boot exit')
    report['passed'] = True
finally:
    for p in processes:
        if p.poll() is None:
            p.kill()
        p.wait(timeout=5)
    (BASE / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print('RESULT_DIR=' + str(BASE), flush=True)
    print(json.dumps(report), flush=True)
