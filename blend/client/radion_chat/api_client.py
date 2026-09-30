"""Small REST client for the Radion Blender HTTP API (standard library only).

The editor is started with `radion_blender --api`; see blend/doc/API.md for the protocol.
"""

import base64
import json
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass

DEFAULT_API_URL = "http://127.0.0.1:7420"

# The editor is local: never route it through an HTTP(S)_PROXY from the environment.
_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


class ApiError(Exception):
    """The editor answered with an error (or something that is not a valid answer)."""

    def __init__(self, message, code="error", http_status=None, command=None):
        super().__init__(message)
        self.message = message
        self.code = code
        self.http_status = http_status
        self.command = command

    def __str__(self):
        where = f"{self.command}: " if self.command else ""
        status = f", HTTP {self.http_status}" if self.http_status else ""
        return f"{where}{self.message} [{self.code}{status}]"


class ApiConnectionError(ApiError):
    """The editor could not be reached at all (not running, wrong port, timeout)."""

    def __init__(self, message, command=None):
        super().__init__(message, code="connection_failed", command=command)


@dataclass
class CommandReply:
    """A successful command: its result object and, for `screenshot`, the PNG."""

    result: dict
    image_mime: str = ""
    image_b64: str = ""

    @property
    def image_bytes(self):
        return base64.b64decode(self.image_b64) if self.image_b64 else b""


class RadionApiClient:
    def __init__(self, base_url=DEFAULT_API_URL, token=None, timeout=60.0):
        self.base_url = (base_url or DEFAULT_API_URL).rstrip("/")
        self.token = token or None
        self.timeout = timeout

    def health(self, timeout=2.0):
        """GET /api/health. Raises ApiConnectionError if the editor is not there."""
        return self._request("GET", "/api/health", timeout=timeout)

    def commands(self):
        """Every command the editor offers: name, description, readOnly, inputSchema."""
        return self._request("GET", "/api/commands")["commands"]

    def call(self, name, arguments=None):
        """Runs one command; raises ApiError when the editor refuses or fails it."""
        path = "/api/commands/" + urllib.parse.quote(name, safe="")
        reply = self._request("POST", path, arguments or {}, command=name)
        image = reply.get("image") or {}
        return CommandReply(
            result=reply.get("result") or {},
            image_mime=image.get("mimeType", ""),
            image_b64=image.get("data", ""),
        )

    def _request(self, method, path, body=None, command=None, timeout=None):
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(self.base_url + path, data=data, method=method)
        request.add_header("Content-Type", "application/json")
        if self.token:
            request.add_header("Authorization", f"Bearer {self.token}")
        status = 200
        try:
            with _OPENER.open(request, timeout=timeout or self.timeout) as response:
                raw = response.read()
        except urllib.error.HTTPError as error:
            status, raw = error.code, error.read()
        except (urllib.error.URLError, OSError) as error:
            reason = getattr(error, "reason", error)
            raise ApiConnectionError(
                f"Cannot reach the Radion editor API at {self.base_url} ({reason}). "
                "Start the editor with `radion_blender --api`.",
                command=command,
            ) from error
        return self._parse(raw, status, command)

    @staticmethod
    def _parse(raw, status, command):
        try:
            payload = json.loads(raw)
        except ValueError:
            snippet = raw[:200].decode("utf-8", "replace")
            raise ApiError(f"Not a JSON answer: {snippet!r}", "invalid_response", status, command)
        if isinstance(payload, dict) and payload.get("ok") is True:
            return payload
        error = payload.get("error", {}) if isinstance(payload, dict) else {}
        raise ApiError(
            error.get("message") or f"HTTP {status}",
            error.get("code") or "error",
            status,
            command,
        )
