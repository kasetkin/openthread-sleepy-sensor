#!/usr/bin/env python3
"""Stage an MQTT OTA update for the openthread-sleepy-sensor firmware.

The image is staged as N retained CHUNK messages (`<id>/ota/image/<n>`), each small enough
for the device's MQTT RX buffer — the device pulls them in order and resumes a broken
download mid-way (see main/ota_updater.h for the protocol and why chunking is required).
Nothing needs to stay running after staging — Home Assistant's Update entity handles the
approve/install step from there:

    tools/ota_push.py build/openthread-sleepy-sensor.bin --device esp32-OT-MQTT-sensor-0011223344556677

Options:
    --install   also set the retained install flag (bench workflow: no HA click needed)
    --force     manifest carries "force":true (device skips its battery gate)
    --watch     stay connected and tail <id>/ota/status + <id>/ota/installed
    --clear     remove all retained OTA messages for the device instead of staging

Broker address/port/credentials are read from secrets.yaml (same file the firmware embeds);
the firmware version is read from the .bin's embedded esp_app_desc_t, so there is no manual
version bookkeeping — every `git describe`-stamped build is distinguishable.
"""

import argparse
import hashlib
import json
import re
import ssl
import struct
import sys
import time
from pathlib import Path

import paho.mqtt.client as mqtt

APP_DESC_OFFSET = 0x20  # esp_image_header_t (24) + first esp_image_segment_header_t (8)
APP_DESC_MAGIC = 0xABCD5432
# Must not exceed the firmware's OTA_MAX_CHUNK_SIZE (main/ota_updater.h) — the device
# rejects manifests with a larger chunk_size because a chunk must fit its MQTT RX buffer.
DEFAULT_CHUNK_SIZE = 8192
PROJECT_ROOT = Path(__file__).resolve().parent.parent


def read_secrets(path: Path) -> dict[str, str]:
    """Minimal parser for the flat `key: "value"` lines secrets.yaml uses."""
    secrets: dict[str, str] = {}
    for line in path.read_text().splitlines():
        m = re.match(r'^\s*([A-Za-z0-9_]+)\s*:\s*"([^"]*)"', line)
        if m:
            secrets[m.group(1)] = m.group(2)
    return secrets


def read_app_version(image: bytes) -> str:
    """Extract esp_app_desc_t.version (PROJECT_VER, i.e. `git describe`) from the .bin."""
    if len(image) < APP_DESC_OFFSET + 0x30 + 32:
        sys.exit("image too small to contain an esp_app_desc_t — not an ESP-IDF app image?")
    (magic,) = struct.unpack_from("<I", image, APP_DESC_OFFSET)
    if magic != APP_DESC_MAGIC:
        sys.exit(f"esp_app_desc_t magic mismatch at 0x{APP_DESC_OFFSET:x} "
                 f"(got 0x{magic:08x}) — not an ESP-IDF app image?")
    # esp_app_desc_t: magic(4) secure_version(4) reserv1(8) version(32) ...
    raw = struct.unpack_from("32s", image, APP_DESC_OFFSET + 16)[0]
    return raw.split(b"\0", 1)[0].decode()


def connect(secrets: dict[str, str]) -> mqtt.Client:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    username = secrets.get("mqtt_username", "")
    if username:
        client.username_pw_set(username, secrets.get("mqtt_password", ""))
    if secrets.get("mqtt_tls") == "true":
        # The firmware skips CN checks for its literal-IP broker (see README); do the same.
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        client.tls_set_context(ctx)
    host = secrets.get("mqtt_broker_address", "")
    port = int(secrets.get("mqtt_port", "1883"))
    if not host:
        sys.exit("secrets.yaml has no mqtt_broker_address")
    client.connect(host, port, keepalive=30)
    client.loop_start()
    return client


def publish_retained(client: mqtt.Client, topic: str, payload, label: str,
                     quiet: bool = False) -> None:
    info = client.publish(topic, payload, qos=1, retain=True)
    info.wait_for_publish(timeout=60)
    if not info.is_published():
        sys.exit(f"publishing {label} to {topic} timed out")
    if not quiet:
        size = len(payload) if payload is not None else 0
        print(f"  staged {label}: {topic} ({size} bytes, retained)")


def collect_retained_ota_topics(client: mqtt.Client, device: str, wait_s: float = 3.0) -> set[str]:
    """Every retained <device>/ota/* topic currently on the broker (image chunks included)."""
    topics: set[str] = set()

    def on_message(_c, _u, msg):
        if msg.retain and msg.payload:
            topics.add(msg.topic)

    client.on_message = on_message
    client.subscribe(f"{device}/ota/#", qos=0)
    time.sleep(wait_s)
    client.unsubscribe(f"{device}/ota/#")
    client.on_message = None
    return topics


def clear_stale(client: mqtt.Client, device: str, keep: set[str]) -> None:
    """Delete retained OTA messages not in `keep` — old chunks past a new image's count,
    the pre-chunking single-blob topic, manifests from other stagings, and so on."""
    for topic in sorted(collect_retained_ota_topics(client, device) - keep):
        if topic.endswith("/ota/installed") or topic.endswith("/ota/status"):
            continue  # device-owned topics, not staging artifacts
        publish_retained(client, topic, None, f"(cleared) {topic}", quiet=True)
        print(f"  cleared stale retained: {topic}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", nargs="?", type=Path,
                    default=PROJECT_ROOT / "build" / "openthread-sleepy-sensor.bin",
                    help="app image to stage (default: build/openthread-sleepy-sensor.bin)")
    ap.add_argument("--device", required=True,
                    help="full MQTT device id incl. MAC suffix (see the boot log's "
                         "'MQTT device_id:' line, or the HA device page)")
    ap.add_argument("--secrets", type=Path, default=PROJECT_ROOT / "secrets.yaml",
                    help="secrets.yaml to read the broker address/credentials from")
    ap.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE,
                    help=f"chunk payload bytes (default {DEFAULT_CHUNK_SIZE}; must not exceed "
                         "the firmware's OTA_MAX_CHUNK_SIZE)")
    ap.add_argument("--install", action="store_true",
                    help="also set the retained install flag (skip the HA Install click)")
    ap.add_argument("--force", action="store_true",
                    help='manifest "force":true — device ignores its low-battery gate')
    ap.add_argument("--watch", action="store_true",
                    help="stay connected and print <id>/ota/status + installed-version updates")
    ap.add_argument("--clear", action="store_true",
                    help="delete all retained OTA staging messages for the device and exit")
    args = ap.parse_args()

    secrets = read_secrets(args.secrets)
    t = {name: f"{args.device}/ota/{name}" for name in
         ("manifest", "install", "installed", "status")}
    chunk_topic = lambda n: f"{args.device}/ota/image/{n}"  # noqa: E731

    client = connect(secrets)
    try:
        if args.clear:
            clear_stale(client, args.device, keep=set())
            print("retained OTA staging messages cleared")
            return

        image = args.image.read_bytes()
        version = read_app_version(image)
        chunks = [image[i:i + args.chunk_size] for i in range(0, len(image), args.chunk_size)]
        manifest = json.dumps({
            "version": version,
            "size": len(image),
            "sha256": hashlib.sha256(image).hexdigest(),
            "chunk_size": args.chunk_size,
            "force": args.force,
        })
        print(f"staging {args.image.name}: version {version}, {len(image)} bytes "
              f"in {len(chunks)} chunks of {args.chunk_size}, for device {args.device}")

        # Order matters: chunks first, manifest last — a device waking mid-staging must not
        # see a manifest whose chunks aren't all retained yet. Stale leftovers (a previous
        # image's extra chunks, the old single-blob topic) are cleared before that.
        clear_stale(client, args.device,
                    keep={chunk_topic(n) for n in range(len(chunks))} | {t["install"]})
        for n, chunk in enumerate(chunks):
            publish_retained(client, chunk_topic(n), chunk, f"chunk {n}", quiet=True)
        print(f"  staged {len(chunks)} chunk messages ({chunk_topic(0)} … {chunk_topic(len(chunks) - 1)})")
        publish_retained(client, t["manifest"], manifest, "manifest")
        if args.install:
            publish_retained(client, t["install"], "install", "install flag")
            print("install flag set — device updates on its next wake cycle")
        else:
            print("staged only — approve via the device's Update entity in Home Assistant "
                  "(or re-run with --install)")

        if args.watch:
            done = {"flag": False}

            def on_message(_c, _u, msg):
                body = msg.payload.decode(errors="replace")
                print(f"  [{time.strftime('%H:%M:%S')}] {msg.topic}: {body}")
                if msg.topic == t["installed"] and body == version:
                    print("device reports the new version — OTA complete")
                    done["flag"] = True

            client.on_message = on_message
            client.subscribe([(t["status"], 0), (t["installed"], 0)])
            print("watching (Ctrl-C to stop)...")
            try:
                while not done["flag"]:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
