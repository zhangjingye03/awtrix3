import http.client
import hashlib
import json
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
        body = response.read().decode("utf-8", "replace")
        return response.status, body
    finally:
        conn.close()


def _post_firmware(host, port, path, firmware_path):
    boundary = "----awtrix-webota-boundary"
    filename = os.path.basename(firmware_path)
    with open(firmware_path, "rb") as firmware:
        firmware_data = firmware.read()
        body = (
            f"--{boundary}\r\n"
            f'Content-Disposition: form-data; name="firmware"; filename="{filename}"\r\n'
            "Content-Type: application/octet-stream\r\n\r\n"
        ).encode("ascii")
        body += firmware_data
        body += f"\r\n--{boundary}--\r\n".encode("ascii")

    firmware_size = len(firmware_data)
    firmware_md5 = hashlib.md5(firmware_data).hexdigest()
    headers = {
        "Content-Type": f"multipart/form-data; boundary={boundary}",
        "Content-Length": str(len(body)),
        "Connection": "close",
    }

    conn = http.client.HTTPConnection(host, port, timeout=25)
    try:
        conn.request("POST", f"{path}?size={firmware_size}&md5={firmware_md5}", body=body, headers=headers)
        response = conn.getresponse()
        response_body = response.read().decode("utf-8", "replace")
        return response.status, response_body
    finally:
        conn.close()


def webota_upload(source, target, env):
    firmware_path = str(source[0])
    host, port, path = _target_from_upload_port(env.subst("$UPLOAD_PORT"))
    print(f"Uploading {firmware_path} to http://{host}:{port}{path}")

    baseline_uptime = None
    try:
        status, body = _http_get(host, port, path)
        ready = json.loads(body)
        if status != 200 or ready.get("status") != "ready" or ready.get("protocol") != 1:
            print("Device does not expose the verified HTTP OTA protocol.")
            return 1
        baseline_uptime = ready.get("uptime")
    except Exception as exc:
        print(f"Cannot contact OTA endpoint: {exc}")
        return 1

    try:
        status, response_body = _post_firmware(host, port, path, firmware_path)
        print(f"HTTP OTA response status: {status}")
        if response_body:
            print(f"HTTP OTA response: {response_body}")
        if status != 200 or response_body.strip() != "OK":
            return 1
    except Exception as exc:
        print(f"HTTP OTA upload did not complete: {exc}")
        return 1

    poll_path = path
    print("Waiting for device to return...")
    deadline = time.time() + 75
    while time.time() < deadline:
        time.sleep(3)
        try:
            status, response_body = _http_get(host, port, poll_path)
            ready = json.loads(response_body)
            if status == 200 and ready.get("status") == "ready" and ready.get("protocol") == 1:
                if baseline_uptime is None or ready.get("uptime", baseline_uptime + 1) <= baseline_uptime:
                    print("HTTP OTA complete; device rebooted and is ready.")
                    return 0
        except Exception:
            pass

    print("HTTP OTA upload may have completed, but the device did not return in time.")
    return 1


env.Replace(UPLOADCMD=webota_upload)
