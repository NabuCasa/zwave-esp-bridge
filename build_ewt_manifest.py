import argparse
import json

parser = argparse.ArgumentParser()
parser.add_argument('-v', help='Version number to use in the manifest. Example: 2025.09.2', required=True, type=str)

version = parser.parse_args().v

manifest = {
  "name": "zwa-2.usb-uart-bridge",
  "version": version,
  "new_install_prompt_erase": "false",
  "builds": [
    {
      "chipFamily": "ESP32-S3",
      "parts": [
        {
          "path": f"{version}/zwa2-esp-bridge.{version}.factory.bin",
          "offset": 0
        }
      ]
    }
  ]
}

print(json.dumps(manifest, indent=2))
