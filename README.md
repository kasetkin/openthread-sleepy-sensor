# openthread-sleepy-sensor
Firmware for ESP32-C6 based sensor with low power consumption.

The node is a Thread sleepy end device (MTD). Each cycle it wakes, reads the SHT3x sensor,
connects to an MQTT broker to publish, then light-sleeps. The broker is reached directly
over IPv6 when `mqtt_broker_address` is an IPv6 literal (use an address the Border Router
can route — its own OMR-prefix address or the broker host's LAN ULA/GUA, never the
mesh-local prefix), or over IPv4 mapped through the Border Router's NAT64 prefix.

Configuration is split by sensitivity across two files embedded at build time: credentials
and network settings go in `secrets.yaml` (gitignored; see
[secrets.yaml.example](secrets.yaml.example)), while non-secret per-device tuning —
calibration offsets, publish cadence, heater schedule, battery-ADC setup — goes in
[device_config.yaml](device_config.yaml) (see
[device_config.yaml.example](device_config.yaml.example) for the documented key reference).
Several of those same values (plus antenna selection) can also be changed **live from Home
Assistant, without a reflash** — see "Runtime-tunable parameters over MQTT" below.

## MQTT transport

MQTT runs **plaintext (`mqtt://`) by default** on port `1883` (override with `mqtt_port` in
`secrets.yaml`; see [secrets.yaml.example](secrets.yaml.example)). An optional
**MQTT-over-TLS (`mqtts://`) mode** is available via `mqtt_tls: "true"` in `secrets.yaml`
(reusing `mqtt_port` for the TLS listener — update it alongside `mqtt_tls` when switching
modes). The broker is reached by a literal IP address either way — over Thread, a direct
IPv6 address (no NAT64) or an IPv4 address mapped through the Border Router's NAT64
prefix; over Wi-Fi, a direct IPv4/IPv6 address. The device authenticates with
`mqtt_username` / `mqtt_password` in both modes.

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
- **The broker is reached by literal IP** (direct IPv6 or NAT64-mapped IPv4 over Thread,
  plain IPv6/IPv4 over Wi-Fi) —
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

## OTA firmware updates over MQTT

Firmware can be updated over the air with MQTT as the only transport — no HTTP server, works
identically over Thread and Wi-Fi. The image is staged on the broker as **N retained chunk
messages** (`<id>/ota/image/<n>`, 8 KB each) which the device pulls strictly in order,
writing each straight into `esp_ota_write()`; a broken download **resumes** from the first
missing chunk on the next wake cycle. Chunking is a correctness requirement, not an
optimisation: esp-mqtt cannot survive a multi-second radio stall while receiving a message
larger than its RX buffer (its parser desyncs mid-message — observed on hardware over
Thread), while a chunk that fits the buffer is delivered atomically. Retained staging means
**no live host during the download** — the stager exits after publishing.

Workflow:

1. **Stage** (build machine): `tools/ota_push.py --device <device_id>` publishes the retained
   manifest + image (version is read from the .bin's `esp_app_desc_t`; the device id is
   printed in the boot log as `MQTT device_id:`).
2. **Install** (Home Assistant): the device advertises an MQTT `update` entity, so HA shows
   "Update available" with an **Install** button; the click publishes a retained install
   command the sleeping device picks up on its next wake. (Bench shortcut: `--install`.)
3. **Download**: the device verifies version/battery (≥30 % unless the manifest says
   `"force":true`), drops its Thread data-poll period to 50 ms for the session (staying a
   sleepy child — switching to rx-on-when-idle was tried and black-holes downlink during
   the mode renegotiation), pulls the chunks in order with a 2-deep pipelined subscription
   window, checks SHA-256, flips the boot partition and reboots. Expect roughly **4–8 min**
   for a ~1.9 MB image. Brief radio stalls that kill the MQTT connection are recovered
   **within the same wake cycle** (reconnect + resume from the first missing chunk), and
   while an update is pending every backstop wake runs an OTA-attempt cycle even when the
   sensors have nothing new to publish.
4. **Confirm or roll back**: the new image boots as `PENDING_VERIFY`
   (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`). Its first broker-ACKed publish marks it valid; if
   that never happens within `UNCONFIRMED_OTA_REBOOT_AFTER_CYCLES` consecutive unhealthy cycles
   (see [sensorstask.h](main/sensorstask.h)), a safety-net reboot restarts the device and the
   bootloader falls back to the previous slot automatically.

Topics live under `<device_id>/ota/*` — see [ota_updater.h](main/ota_updater.h) for the
contract. The flash layout is two 1984 K app slots (`ota_0`/`ota_1` + `otadata`,
[partitions.csv](partitions.csv)); migrating a device from the old single-`factory` layout
requires one final USB flash. `tools/ota_push.py --clear` removes the retained image from the
broker once every device is updated (harmless to leave; it is ~1.8 MB of broker storage).

## Runtime-tunable parameters over MQTT

A subset of `device_config.yaml`'s calibration/threshold/schedule keys, plus antenna
selection, can be changed from Home Assistant **without rebuilding or reflashing**. HA exposes
each as a `number` or `switch` entity (filed under the device's "Configuration" section); the
device applies an accepted change **live, with no reboot**, and persists it to NVS so it
survives a power cycle.

| HA entity | Same key as in `device_config.yaml` | Range |
|---|---|---|
| Temperature offset | `temp_offset` | −10 .. 10 °C |
| Temperature min change | `temp_min_change` | 0.05 .. 5 °C |
| Humidity offset | `rh_offset` | −20 .. 20 % |
| Humidity min change | `rh_min_change` | 0.5 .. 20 % |
| Max publish gap | `max_publish_gap_sec` | 0 .. 21600 s |
| Heater period | `heater_period_minutes` | 0 .. 10080 min |
| Heater high-RH trigger | `heater_high_rh_trigger_minutes` | 0 .. 1440 min |
| Sensor samples | `sensor_samples` | 1 .. 16 |
| External antenna | *(no YAML key — HA/NVS only)* | on/off |

Each entity is backed by one retained MQTT topic, `<device_id>/cfg/<name>` (see
[runtime_config.h](main/runtime_config.h) for the exact contract), which the device subscribes
to as a single `<device_id>/cfg/#` wildcard each wake — same "retained command a sleepy device
can't miss" idiom the OTA `install` topic uses above. `state_topic` and `command_topic` are the
same topic: HA publishes a new value there (retained), and after validating/clamping it the
device applies it and republishes its own retained echo of the value actually in effect, so
HA's display always matches reality, not just what was requested.

The first 8 rows go straight into the LP core's shared-memory config block (the same one
`lp_sensor_core_init()` populates at boot from `device_config.yaml`) and take effect on the LP
core's very next wake. The external antenna switch is a GPIO-level analog RF-switch selection
([common_utils.h](main/common_utils.h)'s `enableExtAntenna()`) — also applied immediately, no
reboot, since the switch is transparent to everything above the radio's physical layer.

Precedence at boot is **NVS override (if HA has ever set one) → `device_config.yaml` → compiled
default** — editing `device_config.yaml` still works exactly as before for a device that has
never received an MQTT override; once HA sets a value, it wins until HA (or an NVS erase)
changes it again.

Two read-only diagnostic entities report on the heater schedule above rather than configure
it: **Heater problem** (a `binary_sensor`, ON if the most recent heater run's temperature
rise never cleared its target) and **Heater run count** (a monotonically increasing counter
of completed heater runs since boot, so HA can graph/sum activations over time). Both stay
absent from HA until the first heater run since boot completes.

## Recovery: when the device reboots, and what survives a blackout

The MQTT broker/Home Assistant is wall-powered; this device isn't, so a grid blackout takes the
broker down while the sensor keeps running on battery. The device only ever reboots (a full
Thread/Wi-Fi re-attach) for a confirmed **LP-core stall** — a full safeguard window with zero LP
heartbeat progress, the one failure that's actually a local firmware wedge a restart can fix.
Link-down and broker-unreachable cycles retry forever at the normal cadence instead: rebooting
can't join a network that isn't there, and can't restart a remote broker process either, so
treating either as reboot-worthy was pure cost (a full re-attach) for no benefit — see
`SensorsTask::LP_STALL_REBOOT_THRESHOLD`'s doc comment ([sensorstask.h](main/sensorstask.h)) for
the full reasoning. The separate, narrower `UNCONFIRMED_OTA_REBOOT_AFTER_CYCLES` safety net
mentioned above still reboots a **freshly-flashed, never-yet-confirmed** image after enough
consecutive unhealthy cycles of any kind, so a genuinely broken update still rolls back via the
bootloader — it just no longer fires for an already-trusted image sitting through an ordinary
outage.

### Blackout data buffering & backfill

Readings that fail to publish live (broker unreachable) are kept in a RAM-only ring buffer
([history_log.h](main/history_log.h), ~96 KB / 8192 entries — days-to-weeks of coverage at
typical publish cadence) instead of being silently overwritten by LP's next poll. It's
deliberately not flash-backed: since a blackout no longer triggers a reboot (above), the buffer
survives essentially the entire outage in RAM, with no new flash partition and no one-time USB
reflash needed — this ships over the normal OTA path like everything else.

Once the broker's reachable again, the backlog replays over the same connection as small
batches on a plain (non-retained) topic:

```
<device_id>/backfill  ->  [{"ago":<seconds before this message>,"t":<°C>,"h":<%RH>}, ...]
```

The device has no wall clock (deliberately — see `history_log.h`'s doc comment: avoiding an
internet/DNS dependency during the exact window that's suspect during a blackout). `"ago"` is
relative; Home Assistant computes each point's real historical timestamp as
`(message arrival time − ago)` using its own clock, which is correct again by the time this runs.

HA's built-in entity History graph only ever shows a state at the time it actually arrived —
there's no supported way to hand it a backdated raw state. Getting backfilled points to show up
at their *true* time instead needs HA's long-term-statistics import, via a small one-time
automation (fires automatically on every future replay, not a manual per-outage step):

```yaml
automation:
  - alias: "Sensor backfill import"
    trigger:
      - platform: mqtt
        topic: "<device_id>/backfill"   # substitute the real device_id
    action:
      - variables:
          entries: "{{ trigger.payload_json }}"
          now_ts: "{{ as_timestamp(now()) }}"
      - repeat:
          for_each: "{{ entries }}"
          sequence:
            - variables:
                # floor to the 5-minute bucket boundary recorder.import_statistics expects
                bucket_ts: "{{ (now_ts - repeat.item.ago | int) | int
                               - ((now_ts - repeat.item.ago | int) | int % 300) }}"
            - action: recorder.import_statistics
              data:
                statistic_id: "sensor.REPLACE_WITH_REAL_ENTITY_ID_temperature"
                source: recorder
                unit_of_measurement: "°C"
                has_mean: true
                has_sum: false
                stats:
                  - start: "{{ as_datetime(bucket_ts) }}"
                    mean: "{{ repeat.item.t }}"
            # repeat the same action block against the humidity entity_id / "%" / repeat.item.h
```

Check this against **Developer Tools → Actions** on your own HA version before trusting it —
`recorder.import_statistics`'s exact schema has shifted across HA releases, the real
`statistic_id` depends on how HA slugified this device's discovery-config entity names, and
this hasn't been validated against a live instance. It also assumes HA's MQTT integration keeps
its usual persistent broker session (the default), so the few-second gap between "broker's back"
and "HA's automation is ready" doesn't drop a non-retained backfill batch — true for the common
co-located Mosquitto add-on setup this project already targets (see "MQTT transport" above).

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

## Radio TX power, and the instrumentation that has to come first

The device has never set its 802.15.4 transmit power, so it runs at the PHY power table's
maximum — **+20 dBm**, the loudest the part can transmit. Turning that down is the obvious
battery lever, and the per-level cost is steeply non-linear (ESP32-C6 datasheet, Table 5-9):

| TX power | Peak current | Saved vs. 20 dBm | dB of margin given up |
|---|---|---|---|
| **20.0 dBm (current default)** | 305 mA | — | — |
| 12.0 dBm | 187 mA | **118 mA** | 8 |
| 0 dBm | 119 mA | 186 mA | 20 |
| −15.0 dBm | 92 mA | 213 mA | 35 |
| *(RX, for reference)* | *74 mA* | | |

The first 8 dB buys 55% of the total available saving; the last 15 dB buys 13%. The efficient
knee is around **+8…+12 dBm** — dropping straight to the minimum spends 35 dB of link margin to
buy almost nothing beyond what +12 dBm already gives.

Two things make it wrong to just set it and hope:

1. **The "Signal strength" entity measures the wrong direction.** It is
   `otThreadGetParentLastRssi` — *our* receiver measuring the *border router's* transmitter — so
   it cannot respond to our own TX power at all. The entity that can is **"Uplink signal
   strength"**, fed by Thread 1.2 enhanced-ACK probing: the parent stamps its own measurement of
   our frames into the ACKs it returns. That one is absent on a Thread 1.1 border router, in
   which case "Parent link quality" (0–3) and the TX retry/CCA/no-ack counters are the fallback.
2. **The saving cannot be predicted without knowing radio duty cycle.** What reaches the battery
   is `ΔI_avg = ΔI_peak(P) × t_tx / cycle_period`. The **"Radio TX time"** diagnostic entity is
   that `t_tx` term. At ~5 ms/cycle the 20→12 dBm step is worth ~2 µA (≈1% of budget — not worth
   any link margin); at ~50 ms/cycle it is worth ~20 µA (≈10% — worth real work).

Hence the diagnostic entities (Radio TX/RX time, TX retries, CCA failures, TX no-ack expiry,
Parent link quality, Uplink signal strength), enabled by `CONFIG_OPENTHREAD_RADIO_STATS_ENABLE`
and `CONFIG_OPENTHREAD_LINK_METRICS` and read once per publish window as per-cycle deltas. They
change no radio behaviour; they exist so the TX-power decision can be made from measurements
rather than guessed. Note also that `CONFIG_OPENTHREAD_PARENT_SEARCH_RSS_THRESHOLD=-65` operates
on *downlink* RSSI and does **not** interact with our transmit power.

### Measured result: the gate says build the knob

With the instrumentation flashed and running long enough to gather real data, the `ΔI_avg =
ΔI_peak(P) × t_tx / cycle_period` decision above resolves against actual numbers instead of the
hypothetical ~5 ms/~50 ms cycle examples. From a 133.4 h, OTA-free HA export (2026-07-29 →
2026-08-04):

- **Radio TX duty cycle**: `sum(Radio TX time) / wall-clock window` = 207,195 ms / 480,205 s =
  **0.043 %** (mean 87.6 ms of TX-on time per cycle, ~2365 cycles).
- **Battery baseline**: 0.3354 mA / 1.367 mW, cross-checked two independent ways in the same
  window (a from-scratch fit of `voltage` against the etalon curve, and the firmware's own
  reported `battery` % sensor) agreeing to 3 significant figures — **~361 days** implied runtime
  on a full 3200 mAh pack. An earlier pass that happened to include an OTA update inside the
  averaging window read ~20–27 % higher — OTA's few high-power minutes are enough to skew a
  multi-day average, so any future re-measurement should confirm the window is OTA-free first.

| Step | Saves | % of total battery current |
|---|---|---|
| 20 → 12 dBm | ~50.9 µA | **15.2 %** |
| 20 → 0 dBm | ~80.3 µA | **23.9 %** |
| 20 → −15 dBm | ~91.9 µA | **27.4 %** |

Even the conservative 12 dBm step clears a >5 % "worth building" bar by 3×, so the runtime knob
(`cfg/tx_power_dbm`, with the brick-hazard revert-on-failure design already worked out) is next.

The same export refines the Phase C ("auto" mode) design: `TX retries` stayed clean (0.048/cycle
mean, only 4.1 % of cycles had any) and `TX no-ack expiry` was zero for the whole window, but `CCA
failures` hit **≥1 on 56.6 % of cycles** — frequent channel contention independent of our TX power
(likely Wi-Fi coexistence), so an adaptive controller must not gate its step-down on "zero CCA
failures for N cycles" as originally sketched; retries and no-ack expiry are the clean signals to
gate on instead. `Uplink signal strength` averaged **−48.6 dBm**, ~15 dB above the −85 dBm target
margin Phase C would step down toward — comfortable headroom. `Parent link quality` stayed almost
entirely absent (2 samples in 2365 cycles), confirming it's too sparse to gate on alone.

### The knob: built, flashed, and part of a broader power investigation

`cfg/tx_power_dbm` (HA number, −15…20 dBm) plus a read-only `TX power (active)` diagnostic are
implemented (`main/runtime_config.{h,cpp}`, `NetworkLink::setTxPowerDbm`, brick-hazard
pending/known-good/revert-after-3-cycles state machine, radio pinned to +20 dBm until first
attach and for the whole OTA window). Flashed and set to **0 dBm** as the first step of a larger
question: this device measures **~335 µA average current**, well above a commonly-cited **~35 µA**
"correctly configured ESP32-C6 light sleep" figure. Turns out the datasheet's own Table 5-11 says
**180 µA** Typ, not 35 — the 35 µA figure is a *more* optimized configuration than "light sleep
just enabled," achieved (per extensive ESP-IDF GitHub issue research) only with every
retention/clock optimization applied together. A quantitative duty-cycle model built from this
project's own timing data attributes ~229 of the 335 µA to legitimate, already-characterized radio
TX/RX activity (not a bug — directly reducible by this TX-power knob), leaving a ~106 µA residual
that points at two more candidates: the RTC clock source (`CONFIG_RTC_CLK_SRC_INT_RC` vs the
Espressif-recommended `INT_RC32K`) and possibly the shared MODEM power domain not fully powering
down for an OpenThread-only, Bluetooth-less build. A second new diagnostic sensor, `HP awake
time`, was added alongside the TX-power knob specifically to help pin this down (it measures total
HP-core wall-clock time outside of light sleep per cycle, which the existing Radio TX/RX time
entities don't capture). Full research, the ranked hypotheses, and the quantitative model are in
memory (`project_light_sleep_power_investigation`) — this is an active, sequenced investigation
(one variable changed at a time so each result stays attributable), not a closed topic.
