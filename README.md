# Tesla BLE → HomeKit Bridge (XIAO ESP32-S3)

Firmware that turns a **Seeed Studio XIAO ESP32-S3** into a tiny
HomeKit-controllable Tesla key. On first boot the board pairs itself with a
Tesla vehicle over BLE (same protocol a Tesla phone-key uses), joins your
Wi-Fi, and advertises itself as a HomeKit bridge exposing lock/unlock,
climate, battery SoC, inside/outside temperature, and momentary tiles for
honk, flash, frunk, trunk, charge-port and wake.

Control your car from Apple Home — local, offline, no Fleet API, no rate
limits.

```
  ┌────────────────┐       BLE (signed AES-GCM       ┌───────────────┐
  │  XIAO ESP32-S3 │ ◀────  ECDH P-256 + HMAC) ────▶ │ Tesla vehicle │
  └────────────────┘                                  └───────────────┘
          │
          │ Wi-Fi / HAP-IP / mDNS
          ▼
  ┌────────────────┐
  │  Apple Home    │
  └────────────────┘
```

## Hardware

| Item                          | Notes                                      |
| ----------------------------- | ------------------------------------------ |
| XIAO ESP32-S3 (regular)       | ESP32-S3R8, 8 MB flash, 8 MB PSRAM         |
| USB-C cable                   | For flashing and power                     |
| 5 V USB power supply          | Keep the bridge within BLE range of the car |

The XIAO ESP32-S3 "Sense" (with camera) also works. Range from bridge to
vehicle is typically 3–10 m clear line of sight; put it in a window or
garage wall facing the driveway.

## Protocol summary

All Tesla commands ride the [Tesla Vehicle Command SDK] protocol, introduced
with 2023+ firmware. Every command is a protobuf `RoutableMessage` carrying a
signed payload (HMAC-SHA256 or AES-128-GCM) keyed by an ECDH shared secret
derived from the ESP32's NIST-P256 key pair. BLE is only the transport —
the security envelope is identical to the Fleet API's.

Two domains are established, each with its own peer session:
- **VCSEC** (`DOMAIN_VEHICLE_SECURITY`) — lock, unlock, trunk/frunk, charge
  port, wake, plus the cheap `body-controller-state` query. Works while the
  car is asleep.
- **Infotainment** (`DOMAIN_INFOTAINMENT`) — climate, charging settings,
  SoC / temperature queries. Requires the car to be awake.

The Tesla vehicle advertises as `S<hex(sha1(VIN)[0..8])>C`, so the bridge
identifies the correct car by VIN only — no MAC pinning, no prior bonding.

[Tesla Vehicle Command SDK]: https://github.com/teslamotors/vehicle-command

## First-boot setup

1. **Flash the firmware.**
   ```bash
   git clone --recurse-submodules <this repo>
   cd files
   pio run -t upload && pio device monitor
   ```

2. **Join Wi-Fi.** HomeSpan opens its own AP named `HomeSpan-Setup`; connect
   from your phone, enter your network credentials. (Or type `W` in the
   serial monitor to reconfigure later.)

3. **Set the VIN.** In the serial monitor:
   ```
   @V 5YJ30123456789ABC
   ```
   The VIN is stored in NVS.

4. **Pair with the vehicle.** Sit in the driver's seat with your Tesla key
   card handy, then in the serial monitor:
   ```
   @P
   ```
   The ESP32 scans, connects, and sends an `add-key-request` (key role:
   `DRIVER`). The car's center console displays **"Tap your Tesla key card
   on the console to approve"**. Tap it. The firmware stores the pairing.

5. **Add to Apple Home.** Open the Home app → *Add Accessory* → scan the
   QR code printed in the serial log on boot (pairing code `46637726`,
   setup ID `TSLA`).

## Serial CLI commands

HomeSpan owns the serial console. Type `?` for the full list. This firmware
adds:

| Command         | Purpose                                  |
| --------------- | ---------------------------------------- |
| `@V <vin>`      | Persist the 17-char VIN.                 |
| `@P`            | Send `add-key-request` to the vehicle.   |
| `@I`            | Print bridge + vehicle status.           |
| `@Z`            | Factory reset (erase all of NVS).        |

## HomeKit accessories exposed

- **Tesla Lock** — `LockMechanism` (Secured / Unsecured / Unknown).
- **Tesla Climate** — `HeaterCooler` with current/target temperature.
  Active toggles HVAC on/off.
- **Tesla Battery** — `BatteryService` (SoC %, charging flag, low-battery
  warning <20 %).
- **Tesla Outside** — `TemperatureSensor` for ambient temperature.
- **Tesla Wake / Honk / Flash / Frunk / Trunk / Charge Port** — momentary
  `Switch` tiles that auto-reset to *off*.

## Polling strategy

```
┌─ every 10 s ─┐   VCSEC body-controller-state  (does NOT wake the car)
│               │
│ while awake ┐│   Infotainment GetVehicleData(charge, climate)
│  every 30 s ││   — drops to every 10 s while charging.
```

The car auto-sleeps about 11 min after activity; polling through VCSEC is
safe during sleep. Infotainment queries are gated on
`vehicleSleepStatus == AWAKE`, so the bridge does not inadvertently keep
the vehicle from sleeping.

## Layout

```
firmware-xiao-s3-tesla/
├── platformio.ini               Build config, lib_deps, include paths
├── partitions.csv               8 MB: 3 MB OTA_0, 3 MB OTA_1, 64 KB NVS, …
├── components/
│   └── tesla-ble/               Git submodule → yoziru/tesla-ble (MIT)
│       ├── include/             Public headers
│       ├── src/                 C++ sources
│       └── generated/           nanopb-generated .pb.{c,h} for the Tesla protos
├── src/
│   ├── main.cpp                 Setup + loop, CLI, Wi-Fi callback
│   ├── config.h                 GPIO map, polling intervals, role
│   ├── storage.{h,cpp}          NVS wrapper: VIN, Tesla priv-key, paired flag
│   ├── led.{h,cpp}              Non-blocking status LED patterns
│   ├── tesla_transport.{h,cpp}  NimBLE central: scan-by-VIN, MTU, fragmentation
│   ├── tesla_client.{h,cpp}     Handshake, pairing, commands, polling
│   └── hk_services.{h,cpp}      HomeSpan accessory + characteristic wiring
└── README.md
```

## Build

Requires [PlatformIO](https://platformio.org/):

```bash
pio run                  # build
pio run -t upload        # flash over USB-C
pio device monitor       # serial console @ 115200
```

The `pioarduino/platform-espressif32` fork is used because the official
Espressif platform does not include the `seeed_xiao_esp32c6` board definition.
Pioarduino ships Arduino-ESP32 3.x on IDF 5.x (mbedTLS 3.x), satisfying both
the Tesla BLE library and HomeSpan's ≥ 3.3.0 requirement.

## Security notes

- The P-256 private key is stored in plaintext NVS. For production use,
  enable flash encryption (`idf.py menuconfig → Security features → Enable
  flash encryption`) and tuck the key behind a KEY partition.
- The firmware accepts any HomeKit controller that completes pair-setup
  with the printed code. Change `kDefaultSetupCode` in `config.h` or type
  `S <new_code>` at the HomeSpan CLI before enrolling.
- Tesla allows at most three simultaneous BLE clients — don't run the phone
  Tesla app and another BLE bridge alongside this one or you'll evict each
  other's connections.

## Credits

- [yoziru/tesla-ble] — the C++ Tesla BLE client library (MIT).
- [teslamotors/vehicle-command] — the upstream protocol spec.
- [HomeSpan] — the Arduino HAP implementation.
- [NimBLE-Arduino] — the BLE stack.

[yoziru/tesla-ble]: https://github.com/yoziru/tesla-ble
[teslamotors/vehicle-command]: https://github.com/teslamotors/vehicle-command
[HomeSpan]: https://github.com/HomeSpan/HomeSpan
[NimBLE-Arduino]: https://github.com/h2zero/NimBLE-Arduino
