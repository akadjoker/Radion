import socket

import pytest

from radion_chat.api_client import ApiConnectionError, ApiError, RadionApiClient


def client_for(server, **options):
    return RadionApiClient(server.url, **options)


def test_health(api_server):
    assert client_for(api_server).health() == {"ok": True, "name": "radion_blender", "apiVersion": 1}


def test_commands_lists_name_description_schema(api_server):
    commands = client_for(api_server).commands()
    assert len(commands) == 39
    assert {"name", "description", "readOnly", "inputSchema"} <= set(commands[0])


def test_call_returns_result(api_server):
    reply = client_for(api_server).call("add_primitive", {"type": "box", "name": "hull"})
    assert reply.result["name"] == "hull"
    assert reply.image_bytes == b""
    assert api_server.calls == [("add_primitive", {"type": "box", "name": "hull"})]


def test_call_without_arguments_sends_an_empty_object(api_server):
    client_for(api_server).call("get_status")
    assert api_server.calls == [("get_status", {})]


def test_screenshot_image_is_decoded(api_server):
    reply = client_for(api_server).call("screenshot")
    assert reply.image_mime == "image/png"
    assert reply.image_bytes.startswith(b"\x89PNG")


@pytest.mark.parametrize("command, args, status, code", [
    ("add_primitive", {"type": "teapot"}, 400, "invalid_params"),
    ("select", {}, 422, "failed"),
    ("no_such_command", {}, 404, "unknown_command"),
])
def test_editor_errors_are_typed(api_server, command, args, status, code):
    with pytest.raises(ApiError) as caught:
        client_for(api_server).call(command, args)
    error = caught.value
    assert (error.http_status, error.code, error.command) == (status, code, command)
    assert error.message
    assert not isinstance(error, ApiConnectionError)


def test_token_is_sent_and_required(api_server):
    api_server.token = "s3cret"
    with pytest.raises(ApiError) as caught:
        client_for(api_server).commands()
    assert caught.value.http_status == 401
    assert client_for(api_server, token="s3cret").commands()
    assert client_for(api_server, token="s3cret").call("get_status").result["hasMesh"] is False


def test_health_needs_no_token(api_server):
    api_server.token = "s3cret"
    assert client_for(api_server).health()["ok"]


def unused_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_unreachable_editor_says_how_to_start_it():
    with pytest.raises(ApiConnectionError) as caught:
        RadionApiClient(f"http://127.0.0.1:{unused_port()}").health()
    assert "radion_blender --api" in str(caught.value)
    assert caught.value.code == "connection_failed"


def test_non_json_answer_is_an_api_error():
    # A plain HTTP server that is not the editor, e.g. another program on port 7420.
    import http.server
    import threading

    class Page(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"<html>hello</html>")

        def log_message(self, *args):
            pass

    httpd = http.server.HTTPServer(("127.0.0.1", 0), Page)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        with pytest.raises(ApiError) as caught:
            RadionApiClient(f"http://127.0.0.1:{httpd.server_address[1]}").health()
        assert caught.value.code == "invalid_response"
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_trailing_slash_in_base_url(api_server):
    assert RadionApiClient(api_server.url + "/").health()["ok"]
