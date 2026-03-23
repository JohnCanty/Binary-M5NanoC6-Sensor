# Binary MQTT Sensor (ESP32-C6 / M5NanoC6)

Firmware for a single binary sensor input that publishes state to MQTT and provides a built-in web setup UI.

## Features

- Binary sensor input (active-low by default)
- MQTT publishing with retained LWT
- Web setup page for:
  - Wi-Fi SSID/password
  - MQTT server
  - Device name
  - DHCP hostname (also used as MQTT client ID)
  - Topic prefix
  - Optional subscribe topic/value for NeoPixel toggle
  - Optional reverse sensing logic (treat HIGH as active)
- First-boot SoftAP provisioning when Wi-Fi credentials are missing
- NVS persistence for Wi-Fi and MQTT settings
- Wi-Fi and MQTT reconnect logic with staged recovery
- Optional daily reboot watchdog

## Hardware and Software

- Board: ESP32-C6 (tested on M5NanoC6)
- Framework: Arduino (via PlatformIO)
- Build system: PlatformIO

Main config files:

- `platformio.ini`
- `sdkconfig.m5stack-nanoc6`
- `src/main.cpp`

## MQTT Behavior

### Published topics

Given:

- `topicPrefix` (default: `BinarySensors/`)
- `deviceName` (default: `BinarySensor`)

The firmware publishes:

- State: `<topicPrefix><deviceName>/DETECT`
- LWT: `<topicPrefix><deviceName>/LWT`

### Payloads

- `DETECT`:
  - `OFF` when input is active (`LOW`)
  - `ON` when input is inactive (`HIGH`)
- `LWT`:
  - retained `Online` on successful MQTT connect
  - broker publishes `Offline` when client disconnects unexpectedly

## First-Boot Provisioning

If no Wi-Fi credentials are saved, the device starts a SoftAP and serves the setup page.

After the device is fully booted, hold the device button for 10 seconds to enter setup mode.

Current AP SSID:

- `Sensor-setup`

Typical AP IP:

- `192.168.4.1`

Provisioning flow:

1. Power the device.
2. Connect a phone/laptop to `Sensor-setup`.
3. Open `http://192.168.4.1`.
4. Enter Wi-Fi + MQTT settings and save.
5. Device switches to STA mode and joins your network.

## Web Setup Options

The setup page supports:

- `WiFi SSID`
- `WiFi Password`
  - Leave blank to keep existing password if SSID is unchanged
- `MQTT Server`
- `Device`
- `DHCP Hostname / MQTT Client ID`
- `Topic Prefix`
- `Subscribe Topic (optional)`
- `Toggle Value (optional)`
- `Reverse sensing logic`

### Optional subscribe/toggle behavior

If both optional fields are set:

- The device subscribes to `Subscribe Topic`
- When a payload exactly matches `Toggle Value`, NeoPixel toggles between red and green

If either field is blank:

- No subscribe action is taken
- No toggle action is performed

## Build and Upload

Build:

```bash
platformio run
```

Upload:

```bash
platformio run --target upload --upload-port /dev/cu.usbmodem1101
```

Serial monitor:

```bash
platformio device monitor --baud 115200
```

## Runtime Notes

- Wi-Fi hostname is derived from MAC and used in STA mode.
- Web UI is available when connected to STA, and also during SoftAP provisioning mode.
- MQTT reconnect attempts are rate-limited.
- If MQTT fails repeatedly, firmware may restart as a recovery action.

## Security Notes

- Credentials are stored in NVS.
- Flash encryption can be enabled for stronger at-rest protection.
- For production deployments, prefer isolated IoT networks/VLANs.

## Troubleshooting

### Device does not appear on your LAN

- Check serial logs for Wi-Fi status
- If no credentials are saved, join `Sensor-setup` and configure Wi-Fi

### Cannot reach setup page in SoftAP mode

- Ensure client is connected to `Sensor-setup`
- Open `http://192.168.4.1`

### MQTT is disconnected

- Verify MQTT server IP/hostname and broker availability
- Confirm network reachability from the device subnet
- Check topic prefix/device name values for expected topics

### Build fails in shell with missing Python modules

If using Homebrew PlatformIO, dependencies are installed in its own virtual environment. Install required packages with:

```bash
"$(brew --prefix platformio)/libexec/bin/python" -m pip install littlefs-python fatfs-ng pyyaml rich-click zopfli intelhex rich cryptography ecdsa bitstring "reedsolo>=1.5.3,<1.8" "esp-idf-size>=2.0.0" "esp-coredump>=1.14.0"
```

## License

No license file is currently included in this repository.
Add one before external distribution.
