"""OTA upload script for ArduinoOTA (jandrassy) library via PlatformIO."""
import sys
import requests
from base64 import b64encode
from os.path import getsize

Import("env")

def ota_upload(source, target, env):
    firmware_path = str(source[0])
    upload_port = env.GetProjectOption("upload_port")
    ota_password = env.GetProjectOption("custom_ota_password", "password")
    ota_port = int(env.GetProjectOption("custom_ota_port", "65280"))

    url = f"http://{upload_port}:{ota_port}/sketch"
    auth = b64encode(f"arduino:{ota_password}".encode()).decode()

    file_size = getsize(firmware_path)
    print(f"Uploading {file_size} bytes to {url}")

    with open(firmware_path, "rb") as f:
        resp = requests.post(
            url,
            data=f,
            headers={
                "Content-Type": "application/octet-stream",
                "Content-Length": str(file_size),
                "Authorization": f"Basic {auth}",
            },
            timeout=30,
        )

    if resp.status_code == 200:
        print("OTA upload successful! Board will reboot.")
    else:
        print(f"OTA upload failed: {resp.status_code} {resp.text}")
        sys.exit(1)

env.Replace(UPLOADCMD=ota_upload)
