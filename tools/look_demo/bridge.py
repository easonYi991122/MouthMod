#!/usr/bin/env python3
"""Serial -> HTTP bridge for the look bench (Python standard library only).

Serves this folder and relays the board's USB CDC lines to the page, so the
test page works in any browser (no Web Serial needed).

  GET  /            test page
  GET  /events      Server-Sent Events, one board line per event
  POST /cmd         body = one command line, written to the board
  GET  /bridge-info JSON status

Safety: when the last page disconnects the bridge sends "hid 0" and
"disarm"; closing the bridge closes the port (DTR low), which also disarms.

Focus: on macOS the bridge checks the frontmost app every 250 ms (lsappinfo,
no extra permission) and keeps telling the board "focus 1" while Minecraft
is in front, "focus 0" when it leaves. The board only acts on it after
"fgate 1" (the floating window's "只在 Minecraft 前台时输出").

Usage:
  python3 bridge.py                  # auto-detect /dev/cu.usbmodem*
  python3 bridge.py --serial /dev/cu.usbmodem1101 --http 8765
  python3 bridge.py --focus-app "minecraft"   # substring of the app name
"""
import argparse
import collections
import glob
import json
import os
import queue
import select
import subprocess
import sys
import termios
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))


class SerialLink:
    def __init__(self, path):
        self.path = path  # None = auto-detect
        self.fd = None
        self.lock = threading.Lock()
        self.clients = []
        self.clients_lock = threading.Lock()
        self.current = None
        self.lines = 0
        self.events = collections.deque(maxlen=8)  # last #E lines

    def detect(self):
        if self.path:
            return self.path if os.path.exists(self.path) else None
        ports = sorted(glob.glob("/dev/cu.usbmodem*")) + sorted(glob.glob("/dev/ttyACM*"))
        return ports[0] if ports else None

    def open(self, path):
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0                                             # iflag: raw
        attrs[1] = 0                                             # oflag: raw
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
        attrs[3] = 0                                             # lflag: raw
        attrs[4] = attrs[5] = termios.B115200  # CDC ignores baud; never 1200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd

    def broadcast(self, line):
        with self.clients_lock:
            for q in self.clients:
                try:
                    q.put_nowait(line)
                except queue.Full:
                    pass  # slow page: drop rather than block the reader

    def write(self, text):
        with self.lock:
            if self.fd is None:
                return False
            try:
                os.write(self.fd, text.encode())
                return True
            except OSError:
                return False

    def run(self):
        buf = b""
        while True:
            path = self.detect()
            if not path:
                time.sleep(0.5)
                continue
            try:
                fd = self.open(path)
            except OSError as e:
                self.broadcast(f"#BRIDGE,error,{path},{e.strerror}")
                time.sleep(1.0)
                continue
            with self.lock:
                self.fd, self.current = fd, path
            print(f"[bridge] connected {path}", flush=True)
            self.broadcast(f"#BRIDGE,connected,{path}")
            buf = b""
            try:
                while True:
                    r, _, _ = select.select([fd], [], [], 0.5)
                    if not r:
                        continue
                    chunk = os.read(fd, 4096)
                    if not chunk:
                        raise OSError("port closed")
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode(errors="replace").strip()
                        if line:
                            self.lines += 1
                            if line.startswith("#E,"):
                                self.events.append(line)  # replayed to late pages
                            self.broadcast(line)
            except OSError as e:
                print(f"[bridge] lost {path}: {e}", flush=True)
                self.broadcast(f"#BRIDGE,disconnected,{path}")
            finally:
                with self.lock:
                    self.fd, self.current = None, None
                try:
                    os.close(fd)
                except OSError:
                    pass
            time.sleep(0.5)

    def add_client(self):
        q = queue.Queue(maxsize=500)
        with self.clients_lock:
            self.clients.append(q)
        return q

    def remove_client(self, q):
        with self.clients_lock:
            self.clients.remove(q)
            last = not self.clients
        if last:
            self.write("hid 0\ndisarm\n")
            print("[bridge] last page closed -> hid 0, disarm", flush=True)


def frontmost_app():
    """Display name of the frontmost app on macOS, or None if unknown."""
    try:
        asn = subprocess.run(["lsappinfo", "front"], capture_output=True,
                             text=True, timeout=1).stdout.strip()
        if not asn:
            return None
        out = subprocess.run(["lsappinfo", "info", "-only", "name", asn],
                             capture_output=True, text=True, timeout=1).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    # Output looks like: "LSDisplayName"="Minecraft Education"
    _, _, value = out.partition("=")
    return value.strip().strip('"') or None


class FocusWatcher:
    """Tells the board whether Minecraft is the frontmost app."""

    def __init__(self, link, match):
        self.link = link
        self.match = match.lower()
        self.name = None
        self.front = False

    def run(self):
        last_sent = 0.0
        while True:
            name = frontmost_app()
            front = bool(name) and self.match in name.lower()
            if name != self.name or front != self.front:
                self.name, self.front = name, front
                self.link.broadcast(f"#BRIDGE,front,{name or '?'},{1 if front else 0}")
                last_sent = 0.0  # report the change at once
            now = time.monotonic()
            if front and now - last_sent >= 0.25:
                self.link.write("focus 1\n")  # heartbeat; the board times out
                last_sent = now
            elif not front and last_sent == 0.0:
                self.link.write("focus 0\n")
                last_sent = now
            time.sleep(0.25)


LINK = None
FOCUS = None


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=HERE, **kw)

    def log_message(self, fmt, *args):
        pass  # keep the console for bridge events

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self):
        if self.path == "/bridge-info":
            body = json.dumps({"bridge": True, "port": LINK.current,
                               "lines": LINK.lines,
                               "front": FOCUS.name if FOCUS else None,
                               "minecraft_front": FOCUS.front if FOCUS else None,
                               }).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.end_headers()
            q = LINK.add_client()
            try:
                state = f"#BRIDGE,connected,{LINK.current}" if LINK.current else "#BRIDGE,waiting"
                self.wfile.write(f"data: {state}\n\n".encode())
                for ev in list(LINK.events):
                    self.wfile.write(f"data: {ev}\n\n".encode())
                if FOCUS and FOCUS.name:
                    front = f"#BRIDGE,front,{FOCUS.name},{1 if FOCUS.front else 0}"
                    self.wfile.write(f"data: {front}\n\n".encode())
                self.wfile.flush()
                while True:
                    try:
                        line = q.get(timeout=1.0)
                        self.wfile.write(f"data: {line}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                LINK.remove_client(q)
            return
        super().do_GET()

    def do_POST(self):
        if self.path != "/cmd":
            self.send_error(404)
            return
        n = int(self.headers.get("Content-Length", "0"))
        text = self.rfile.read(min(n, 200)).decode(errors="replace").strip()
        ok = bool(text) and "\n" not in text and LINK.write(text + "\n")
        self.send_response(200 if ok else 503)
        self.send_header("Content-Length", "0")
        self.end_headers()


def main():
    global LINK, FOCUS
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--serial", help="serial device (default: auto-detect)")
    ap.add_argument("--http", type=int, default=8765, help="HTTP port")
    ap.add_argument("--focus-app", default="minecraft",
                    help="substring of the game's app name (default: minecraft)")
    args = ap.parse_args()
    LINK = SerialLink(args.serial)
    threading.Thread(target=LINK.run, daemon=True).start()
    if sys.platform == "darwin":
        FOCUS = FocusWatcher(LINK, args.focus_app)
        threading.Thread(target=FOCUS.run, daemon=True).start()
    else:
        print("[bridge] focus detection only on macOS; fgate would block output",
              flush=True)
    server = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    server.daemon_threads = True
    print(f"[bridge] open http://localhost:{args.http}  (Ctrl+C to stop)", flush=True)
    print(f"[bridge] floating window: http://localhost:{args.http}/float.html", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        LINK.write("hid 0\ndisarm\n")
        print("\n[bridge] stopped", flush=True)


if __name__ == "__main__":
    sys.exit(main())
