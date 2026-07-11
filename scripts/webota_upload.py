import http.client
import os
import time
from urllib.parse import urlparse

Import("env")


def _target_from_upload_port(upload_port):
    if upload_port.startswith("http://") or upload_port.startswith("https://"):
        parsed = urlparse(upload_port)
        host = parsed.hostname
        port = parsed.port or (443 if parsed.scheme == "https" else 80)
        path = parsed.path or "/update"
        return host, port, path
    return upload_port, 80, "/update"


def _http_get(host, port, path, timeout=5):
    conn = http.client.HTTPConnection(host, port, timeout=timeout)
    try:
        conn.request("GET", path)
        response = conn.getresponse()
        response.read()
        return response.status
    finally:
        conn.close()


def _post_firmware(host, port, path, firmware_path):
    boundary = "----awtrix-webota-boundary"
    filename = os.path.basename(firmware_path)
    with open(firmware_path, "rb") as firmware:
        body = (
            f"--{boundary}\r\n"
            f'Content-Disposition: form-data; name="firmware"; filename="{filename}"\r\n'
            "Content-Type: application/octet-stream\r\n\r\n"
        ).encode("ascii")
        body += firmware.read()
        body += f"\r\n--{boundary}--\r\n".encode("ascii")

    headers = {
        "Content-Type": f"multipart/form-data; boundary={boundary}",
        "Content-Length": str(len(body)),
        "Connection": "close",
    }

    conn = http.client.HTTPConnection(host, port, timeout=25)
    try:
        conn.request("POST", path, body=body, headers=headers)
        response = conn.getresponse()
        response_body = response.read().decode("utf-8", "replace")
        return response.status, response_body
    finally:
        conn.close()


def webota_upload(source, target, env):
    firmware_path = str(source[0])
    host, port, path = _target_from_upload_port(env.subst("$UPLOAD_PORT"))
    print(f"Uploading {firmware_path} to http://{host}:{port}{path}")

    try:
        status, response_body = _post_firmware(host, port, path, firmware_path)
        print(f"HTTP OTA response status: {status}")
        if response_body:
            print(f"HTTP OTA response: {response_body}")
        if not 200 <= status < 300:
            return 1
    except Exception as exc:
        # The ESP often reboots before the HTTP client receives the final page.
        print(f"HTTP OTA connection ended during reboot: {exc}")

    poll_path = "/update"
    print("Waiting for device to return...")
    deadline = time.time() + 75
    while time.time() < deadline:
        time.sleep(3)
        try:
            status = _http_get(host, port, poll_path)
            if 200 <= status < 500:
                print("HTTP OTA complete; device is reachable again.")
                return 0
        except Exception:
            pass

    print("HTTP OTA upload may have completed, but the device did not return in time.")
    return 1


env.Replace(UPLOADCMD=webota_upload)
