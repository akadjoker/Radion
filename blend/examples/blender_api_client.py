"""Minimal client for the Radion Blender HTTP API (standard library only).

    client = BlenderApi()                         # http://127.0.0.1:7420
    client.call("add_primitive", type="sphere", name="body", color="#c0392b")
"""

import base64
import json
import os
import urllib.error
import urllib.request


class BlenderApiError(RuntimeError):
    def __init__(self, command, code, message):
        super().__init__(f"{command}: {message} ({code})")
        self.command = command
        self.code = code


class BlenderApi:
    def __init__(self, base_url="http://127.0.0.1:7420", token=None):
        self.base_url = base_url.rstrip("/")
        self.token = token or os.environ.get("RADION_BLENDER_API_TOKEN")

    def _request(self, method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(self.base_url + path, data=data, method=method)
        request.add_header("Content-Type", "application/json")
        if self.token:
            request.add_header("Authorization", f"Bearer {self.token}")
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                return json.loads(response.read())
        except urllib.error.HTTPError as error:
            return json.loads(error.read())

    def commands(self):
        return self._request("GET", "/api/commands")["commands"]

    def call_raw(self, command, **arguments):
        reply = self._request("POST", f"/api/commands/{command}", arguments)
        if not reply.get("ok"):
            error = reply.get("error", {})
            raise BlenderApiError(command, error.get("code"), error.get("message"))
        return reply

    def call(self, command, **arguments):
        return self.call_raw(command, **arguments)["result"]

    def screenshot(self, path, **arguments):
        reply = self.call_raw("screenshot", **arguments)
        with open(path, "wb") as file:
            file.write(base64.b64decode(reply["image"]["data"]))
        return reply["result"]
