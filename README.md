# openthread-sleepy-sensor

Firmware for an ESP32-C6 low-power environmental sensor that joins a Thread network and
exposes its readings as a **Matter** device.

The node is a Thread end device (MTD) running the Matter stack. Each cycle it wakes, reads the
SHT3x sensor, pushes the calibrated temperature and humidity into its Matter cluster attributes,
and goes idle. A Matter controller (e.g. Home Assistant) subscribes to those attributes and is
reported the new values over Thread.

## Matter data model

The device presents a Matter root node plus two sensor endpoints:

| Endpoint | Device type | Cluster | Attribute |
|----------|-------------|---------|-----------|
| Temperature | Temperature Sensor | Temperature Measurement (`0x0402`) | `MeasuredValue` (int16, centi-°C) |
| Humidity | Humidity Sensor | Relative Humidity Measurement (`0x0405`) | `MeasuredValue` (uint16, centi-%) |

Readings are produced by `sensorstask.cpp` (SHT3x read → per-device calibration → optional
heater maintenance) and handed to `matter_sensor.cpp`, which writes them into the cluster
`MeasuredValue` attributes. The Matter subscription engine handles reporting — there is no
per-cycle connection to open or drain.

## Commissioning (Android phone over BLE → Home Assistant)

Matter commissioning bootstraps the device onto Thread over **BLE**. The commissioning window
opens automatically at boot (`CONFIG_CHIP_ENABLE_PAIRING_AUTOSTART`) because this board has no
button.

Prerequisites:
- A **Thread Border Router** in the ecosystem (e.g. Home Assistant Connect ZBT/SkyConnect or
  Home Assistant Yellow; or a Google/Apple hub if you commission via their app). The
  commissioner pushes the operational Thread dataset to the device — the phone alone can't.
- The device's **onboarding payload** (QR code + 11-digit manual pairing code). esp-matter
  prints both to the serial console at startup; scan the QR or type the manual code into the app.

Two routes into Home Assistant:
1. **HA Android Companion app** → *Add device* → *Matter* → scan/enter the code. This commissions
   the device directly into Home Assistant's Matter fabric over BLE (via Google Play Services).
2. **Google Home app** → commission the device, then **share** it to Home Assistant
   (multi-admin) with a new pairing code.

> **Attestation:** this build uses esp-matter's built-in **test** Device Attestation
> Certificates (VID `0xFFF1`). Android will warn that the device is *uncertified* — proceed past
> the warning. Home Assistant's `python-matter-server` accepts test certs. Apple Home does **not**
> — targeting Apple would require provisioning real per-device certs into the `fctry` partition.

## Build & flash

Use the esp-matter devcontainer (ESP-IDF v5.5.4 + esp-matter, with `ESP_MATTER_PATH` exported).

```sh
idf.py set-target esp32c6
idf.py build
idf.py -p <port> flash monitor
```

Watch the serial log: it traces the full lifecycle (`Matter started …`, BLE connect, PASE
session, Thread attach, `Commissioning complete`, IPv6 assigned) and logs each attribute push,
so a stuck commissioning is easy to pinpoint.

Per-device sensor calibration is read from `calibration.txt` (embedded at build time — see
[calibration.txt.example](calibration.txt.example)).

## CPU frequency / power

The HP core runs at **160 MHz** (the ESP32-C6 maximum). It is deliberately **not** capped to
80 MHz, because for this radio-bound end device that saves little-to-nothing on battery and can
be net-negative:

- CPU frequency only affects power while the core is **awake**. The node is idle almost the whole
  cycle, and sleep current (~7 µA deep / ~180 µA light) is independent of CPU frequency.
- The wake window is dominated by the **802.15.4 radio** (RX ~78 mA, TX ~251 mA), whose current
  is independent of CPU frequency. The HP-core delta between 80 and 160 MHz is only a few mA over
  a ~12 mA baseline.
- **Race to sleep**: at the C6's fixed core voltage, dynamic power scales ~linearly with frequency
  while compute time scales ~inversely, so compute energy is roughly flat. Running at 80 MHz just
  keeps the radio and peripherals powered *longer*, so a lower clock can cost *more* per cycle.

The real battery levers are maximizing idle/sleep time and minimizing radio-on duration (poll
period, fast ACK, short wake window). Idle frequency scaling is already handled by ESP-IDF Power
Management (DFS) + tickless idle, which drop the clock when idle and burst to 160 MHz only when
there is work. Confirming the choice on real hardware would take a coulomb-counter measurement
over a full wake→report→idle cycle.

## Future work

- **Full sleepy ICD/LIT power pass.** The device currently runs as an **rx-on MTD** (always
  listening). The low-power path is to register as a Matter **ICD** (Intermittent Connectivity
  Device), switch the Thread radio to a **sleepy** poll schedule (rx-off-when-idle), and enable
  automatic light sleep coordinated with the ICD poll period. It is deferred because ICD/LIT
  tuning tends to make commissioning flaky; it's the main remaining battery lever.
- **OTA updates.** The flash is already partitioned dual-slot for OTA (`ota_0`/`ota_1` +
  `otadata`, see [partitions.csv](partitions.csv)) and the Matter OTA Requestor cluster is
  enabled, so no future repartition (which would wipe commissioning) is needed. The remaining
  work is the update flow itself: an OTA Provider, signed images, and delta OTA
  (`esp_delta_ota`) if the app outgrows a single slot.
