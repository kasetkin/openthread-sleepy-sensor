# openthread-sleepy-sensor
Firmware for ESP32-C6 based sensor with low power consumption.

The node is a Thread sleepy end device (MTD). Each cycle it wakes, reads the SHT3x sensor,
connects to an MQTT broker to publish, then light-sleeps. The broker is reached over IPv4
mapped through the Thread Border Router's NAT64 prefix.

## MQTT transport

MQTT runs **plaintext (`mqtt://`) by default** on port `1883` (override with `mqtt_port` in
`secrets.yaml`; see [secrets.yaml.example](secrets.yaml.example)). An optional
**MQTT-over-TLS (`mqtts://`) mode** is available via `mqtt_tls: "true"` in `secrets.yaml`
(reusing `mqtt_port` for the TLS listener — update it alongside `mqtt_tls` when switching
modes). The broker is reached by a literal IP address either way — IPv4 mapped through the
Thread Border Router's NAT64 prefix, or a direct IPv4/IPv6 address over Wi-Fi. The device
authenticates with `mqtt_username` / `mqtt_password` in both modes.

### Why plaintext is the default

Thread already encrypts **every over-the-air hop** with AES-128-CCM (the network key), so the
ESP32 → OTBR path is confidential regardless of the MQTT transport. For a broker that is only
ever reached over the **local, wired OTBR → broker hop** (i.e. inside the home LAN trust
boundary), plaintext MQTT is acceptable: the only cleartext segment stays on-LAN, and adding
TLS on top would re-encrypt traffic that's already protected on the air, at a real power/CPU
cost on a sleepy device (see below).

### Why TLS is optional, not mandatory

- **TLS-PSK** is incompatible with the **stock Home Assistant Mosquitto add-on**: it always
  loads the `go-auth` auth plugin, which intercepts PSK identity lookups and ignores
  `psk_file`, so the broker rejects every connection with TLS alert 115 `unknown_psk_identity`
  even with a correct key. PSK would require a standalone Mosquitto without `go-auth`, which is
  why this firmware uses certificate (X.509) TLS instead when TLS is enabled.
- **Certificate (X.509) TLS** runs a full asymmetric handshake **every wake cycle** (connect →
  publish → disconnect, ~70 s) — a real per-cycle CPU/airtime/power cost on a sleepy device,
  and it can't be amortized: `esp-mqtt` can't reuse a TLS session across the per-cycle client
  teardown, and a live connection can't survive light sleep (the TCP task is halted and the
  broker keepalive expires). This is why TLS stays **off by default** for the common LAN-only
  deployment, where it would only be re-protecting an already-encrypted Thread hop.
- **The broker is reached by literal IP** (NAT64-mapped IPv4, or plain IPv6/IPv4 over Wi-Fi) —
  never a DNS hostname (see `MqttConfig::broker_address`'s doc comment in
  [mqtt_sender.h](main/mqtt_sender.h)). This means a certificate's CN/SAN can never be
  validated against the connect address, on either transport. When TLS is enabled, this
  firmware therefore sets `skip_cert_common_name_check = true` and relies on chain-of-trust
  verification only (via the ESP-IDF public CA bundle, or a pinned CA/leaf cert set in
  `mqtt_tls_ca_cert`) — TLS still authenticates "signed by a CA I trust" and encrypts the link,
  but not "this is specifically the host I dialed." This is a materially weaker guarantee than
  typical browser-grade TLS and is unlikely to improve until the broker can be dialed by
  hostname.

### When to actually enable `mqtt_tls`

The plaintext-is-fine argument above assumes the broker is only ever reached over a LAN-local
wire. **That assumption does not hold for every deployment.** If your `mqtt_broker_address` is
a public-internet address (e.g. a reverse proxy in front of a remote Home Assistant instance,
reached over WAN rather than the local OTBR → broker LAN hop), MQTT credentials and sensor
data cross the public internet in cleartext with plaintext MQTT — a materially different
threat model than the LAN-only case above. Set `mqtt_tls: "true"` (and point `mqtt_port` at
your broker's TLS listener) in that case. The CN-check limitation above still applies, but a
passive WAN eavesdropper can no longer read credentials or payloads, and the connection is
still validated against a CA chain of trust by default.

If you additionally want to pin your own CA or a self-signed leaf certificate rather than
trust the full public CA bundle, set `mqtt_tls_ca_cert` in `secrets.yaml` to its base64 body
(no `-----BEGIN/END-----` markers, no embedded newlines).

## CPU frequency / power

The HP core runs at **160 MHz** (the ESP32-C6 maximum). It is deliberately **not** capped to
80 MHz, because for this radio-bound sleepy end device that saves little-to-nothing on battery
and can be net-negative:

- CPU frequency only affects power while the core is **awake**. The node sleeps almost the whole
  cycle, and sleep current (~7 µA deep / ~180 µA light) is independent of CPU frequency.
- The wake window is dominated by the **802.15.4 radio** (RX ~78 mA, TX ~251 mA), whose current
  is independent of CPU frequency. The HP-core delta between 80 and 160 MHz is only a few mA over
  a ~12 mA baseline.
- **Race to sleep**: at the C6's fixed core voltage, dynamic power scales ~linearly with frequency
  while compute time scales ~inversely, so compute energy is roughly flat. Running at 80 MHz just
  keeps the radio and peripherals powered *longer*, so a lower clock can cost *more* per cycle.

The real battery levers are maximizing sleep time and minimizing radio-on duration (poll period,
fast ACK, short wake window). Idle frequency scaling is already handled by ESP-IDF Power
Management (DFS) + automatic light sleep (`CONFIG_PM_ENABLE`, tickless idle), which drop the clock
when idle and burst to 160 MHz only when there is work — the best of both. Confirming the choice
on real hardware would take a coulomb-counter measurement over a full wake→publish→sleep cycle.
