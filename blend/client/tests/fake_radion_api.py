
import base64
import json
import struct
import threading
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

COMMANDS = json.loads((Path(__file__).parent / "fixtures" / "commands.json").read_text())["commands"]
PRIMITIVES = ["box", "plane", "sphere", "cylinder", "cone", "capsule", "torus"]


def tiny_png(width=2, height=2):
    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    rows = b"".join(b"\x00" + b"\xc0\x39\x2b" * width for _ in range(height))
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b"")


class CommandFailure(Exception):
    def __init__(self, status, code, message):
        self.status, self.code, self.message = status, code, message


class FakeRadionApi:
    def __init__(self, token=None):
        self.token = token
        self.parts = []
        self.history = []
        self.redo = []
        self.calls = []
        self.lock = threading.Lock()
        self._httpd = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
        self._httpd.owner = self
        self._httpd.daemon_threads = True

    @property
    def url(self):
        return f"http://127.0.0.1:{self._httpd.server_address[1]}"

    def start(self):
        threading.Thread(target=self._httpd.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
        return self

    def stop(self):
        self._httpd.shutdown()
        self._httpd.server_close()

    def names(self):
        return [part["name"] for part in self.parts]

    def run(self, name, args):
        handler = getattr(self, f"cmd_{name}", None)
        if handler is None:
            if name not in {c["name"] for c in COMMANDS}:
                raise CommandFailure(404, "unknown_command", f"unknown command '{name}'")
            raise CommandFailure(422, "failed", f"'{name}' is not implemented by the fake editor")
        return handler(args)

    def _edit(self):
        self.history.append([dict(p) for p in self.parts])
        self.redo.clear()

    def _status(self):
        return {"hasMesh": bool(self.parts), "canUndo": bool(self.history),
                "parts": [{"index": i, **p} for i, p in enumerate(self.parts)]}

    def cmd_get_status(self, args):
        return self._status()

    def cmd_get_mesh_data(self, args):
        return {"positions": [[i * 0.001, 0.0, 1.0] for i in range(3000)], "triangles": [[0, 1, 2]] * 1000}

    def cmd_screenshot(self, args):
        return {"width": 2, "height": 2}, tiny_png()

    def cmd_add_primitive(self, args):
        kind = args.get("type")
        if kind not in PRIMITIVES:
            raise CommandFailure(400, "invalid_params",
                                 f"argument 'type' must be one of: {', '.join(PRIMITIVES)}")
        self._edit()
        part = {"name": args.get("name", f"{kind}_{len(self.parts)}"), "triangles": 12}
        self.parts.append(part)
        return {"index": len(self.parts) - 1, **part}

    def cmd_add_loft(self, args):
        if not args.get("sections"):
            raise CommandFailure(400, "invalid_params", "argument 'sections' is required")
        self._edit()
        part = {"name": args.get("name", "loft"), "triangles": 200}
        self.parts.append(part)
        return {"index": len(self.parts) - 1, **part}

    def cmd_select(self, args):
        if not self.parts:
            raise CommandFailure(422, "failed", "the document has no mesh yet")
        return {"vertexCount": 24}  # selection is not an undo step

    def cmd_set_part_visible(self, args):
        return {"visible": args.get("visible", True)}  # not an undo step either

    def cmd_save_mesh(self, args):
        return {"path": args.get("path", "")}

    def cmd_new_document(self, args):
        self.parts, self.history, self.redo = [], [], []
        return self._status()

    def cmd_delete_part(self, args):
        if not self.parts:
            raise CommandFailure(422, "failed", "no such part")
        self._edit()
        self.parts.pop()
        return self._status()

    def cmd_undo(self, args):
        steps = args.get("steps", 1)
        if not self.history:
            raise CommandFailure(422, "failed", "nothing to undo")
        done = 0
        while self.history and done < steps:
            self.redo.append([dict(p) for p in self.parts])
            self.parts = self.history.pop()
            done += 1
        return {**self._status(), "stepsUndone": done}

    def cmd_redo(self, args):
        steps = args.get("steps", 1)
        done = 0
        while self.redo and done < steps:
            self.history.append([dict(p) for p in self.parts])
            self.parts = self.redo.pop()
            done += 1
        return {**self._status(), "stepsRedone": done}


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/api/health":
            self._send(200, {"ok": True, "name": "radion_blender", "apiVersion": 1})
        elif self._authorised():
            self._send(200, {"ok": True, "commands": COMMANDS})

    def do_POST(self):
        prefix = "/api/commands/"
        if not self.path.startswith(prefix) or not self._authorised():
            return
        length = int(self.headers.get("Content-Length", 0))
        name = self.path[len(prefix):]
        server = self.server.owner
        try:
            args = json.loads(self.rfile.read(length) or b"{}")
            with server.lock:
                server.calls.append((name, args))
                outcome = server.run(name, args)
        except CommandFailure as failure:
            self._send(failure.status, {"ok": False, "error": {"code": failure.code, "message": failure.message}})
            return
        result, image = outcome if isinstance(outcome, tuple) else (outcome, None)
        payload = {"ok": True, "result": result}
        if image:
            payload["image"] = {"mimeType": "image/png", "data": base64.b64encode(image).decode()}
        self._send(200, payload)

    def _authorised(self):
        token = self.server.owner.token
        if token and self.headers.get("Authorization") != f"Bearer {token}":
            self._send(401, {"ok": False, "error": {"code": "invalid_params", "message": "missing or wrong token"}})
            return False
        return True

    def _send(self, status, payload):
        data = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)
