"""Local requests to the process that owns a C30D serial session."""
import errno
import hashlib
import json
import os
import socket
import socketserver
import struct
import subprocess
import sys
from concurrent.futures import TimeoutError
from threading import Thread

OWNER_SERIAL_ENV = 'SLICER_C30D_OWNER_SERIAL'
REQUEST_TIMEOUT = 5.


def address(serial):
    key = hashlib.sha256(str(serial).encode()).hexdigest()[:24]
    return f'\0slicer-c30d-{os.geteuid()}-{key}'


def local_peer(connection):
    _, uid, _ = struct.unpack('3i', connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
    return uid == os.geteuid()


def call_owner(serial, action):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(REQUEST_TIMEOUT + 1.)
        try:
            connection.connect(address(serial))
        except OSError as error:
            if error.errno in (errno.ENOENT, errno.ECONNREFUSED):
                return None
            raise
        if not local_peer(connection):
            raise RuntimeError('底盘驱动服务不属于当前用户')
        connection.sendall(action.encode('ascii') + b'\n')
        with connection.makefile('rb') as response:
            result = json.loads(response.readline(2048))
        return tuple(result)


class _Handler(socketserver.StreamRequestHandler):
    timeout = REQUEST_TIMEOUT

    def handle(self):
        if not local_peer(self.connection):
            return
        action = self.rfile.readline(16)
        if action == b'lock\n':
            result = self.server.recovery.queue_test()
        elif action == b'unlock\n':
            result = self.server.recovery.queue_unlock()
        else:
            self.wfile.write(json.dumps((False, '未知底盘操作')).encode() + b'\n')
            return
        try:
            outcome = result.result(timeout=REQUEST_TIMEOUT)
        except TimeoutError:
            result.cancel()
            outcome = False, '等待驱动确认底盘状态超时'
        self.wfile.write(json.dumps(outcome, ensure_ascii=False).encode() + b'\n')


class CommandServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True

    def __init__(self, serial, recovery):
        super().__init__(address(serial), _Handler)
        self.serial = str(serial)
        self.recovery = recovery
        self.worker = None
        self.child = None

    def start(self, unlock_script=None):
        if unlock_script is not None:
            def start_unlock():
                if self.child is None or self.child.poll() is not None:
                    self.child = subprocess.Popen(
                        [sys.executable, '-s', str(unlock_script), '--unlock'],
                        env=dict(os.environ, **{OWNER_SERIAL_ENV: self.serial}),
                    )
            self.recovery.start_unlock_command = start_unlock
        self.worker = Thread(target=self.serve_forever, kwargs={'poll_interval': .05}, daemon=True)
        self.worker.start()
        return self

    def close(self):
        self.recovery.stop_requests()
        if self.worker is not None:
            self.shutdown()
        self.server_close()
        if self.child is not None and self.child.poll() is None:
            self.child.terminate()
            try:
                self.child.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                self.child.kill()
                self.child.wait()

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()
