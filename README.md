# openthread-sleepy-sensor
Firmware for ESP32-C6 based sensor with low power consumption.

The node is a Thread sleepy end device (MTD). Each cycle it wakes, reads the SHT3x sensor,
connects to an MQTT broker to publish, then light-sleeps. The broker is reached over IPv4
mapped through the Thread Border Router's NAT64 prefix.

## MQTT transport

MQTT runs **plaintext (`mqtt://`)** on port `1883` (override with `mqtt_port` in
`secrets.yaml`; see [secrets.yaml.example](secrets.yaml.example)). The broker is reached by
IPv4 mapped through the Thread Border Router's NAT64 prefix. The device authenticates with
`mqtt_username` / `mqtt_password`.

### Why plaintext is acceptable here

Thread already encrypts **every over-the-air hop** with AES-128-CCM (the network key), so the
ESP32 → OTBR path is confidential. The only cleartext segment is the **wired OTBR → broker
hop**, which stays inside the home LAN trust boundary. Adding TLS on top would re-encrypt
traffic that's already protected on the air, at a real power cost on a sleepy device.

### Why not TLS

- **TLS-PSK** is incompatible with the **stock Home Assistant Mosquitto add-on**: it always
  loads the `go-auth` auth plugin, which intercepts PSK identity lookups and ignores
  `psk_file`, so the broker rejects every connection with TLS alert 115 `unknown_psk_identity`
  even with a correct key. PSK would require a standalone Mosquitto without `go-auth`.
- **Certificate (X.509) TLS** would run a full asymmetric handshake **every wake cycle**
  (connect → publish → disconnect, ~70 s) — the dominant per-cycle CPU and airtime cost on
  this slow link. It can't be amortized: `esp-mqtt` can't reuse a TLS session across the
  per-cycle client teardown, and a live connection can't survive light sleep (the TCP task is
  halted and the broker keepalive expires). The broker is also reached by IP through NAT64, so
  a certificate's CN/SAN couldn't be validated against a hostname anyway.

Net: plaintext over Thread keeps the link confidential on the air while preserving the power
budget. If the wired LAN hop must also be encrypted, run a standalone Mosquitto with a TLS-PSK
listener and re-introduce the PSK transport in the firmware.
