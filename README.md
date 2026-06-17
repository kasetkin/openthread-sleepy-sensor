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
