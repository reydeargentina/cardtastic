# Cardtastic

Cardtastic is a simple, fast UI for Meshtastic nodes running on the M5Stack Cardputer v1.1. It connects to a Meshtastic radio over BLE and focuses on chat, nodes, and channels without changing device configuration.

## Features
- BLE scan and connect to Meshtastic radios
- Conversations list with unread markers and last-message previews
- Channel list and quick open for broadcast chats
- Node list with status and quick DM actions
- Chat view with send/receive
- Optional SD persistence for conversations

## Hardware
- M5Stack Cardputer v1.1 (ESP32-S3)
- Meshtastic node with BLE enabled
- Optional microSD card (for persistence)

## Software
- PlatformIO
- Arduino framework

## Build and Upload (PlatformIO)
```bash
pio run -e m5stack-cardputer -t upload
```

Serial monitor:
```bash
pio device monitor -e m5stack-cardputer
```

## Local PlatformIO Overrides
For local port settings, create `platformio.local.ini` (ignored by git) and override the ports there:
```ini
[env:m5stack-cardputer]
upload_port = /dev/serial/by-id/usb-...
monitor_port = /dev/serial/by-id/usb-...
```

## Usage
- Connect: use Scan to find radios, then connect.
- Conversations: open recent chats; unread messages show a `*`.
- Channels: open a broadcast chat by channel.
- Nodes: view known nodes and open a DM.

Navigation (Cardputer keyboard):
- Menu screens: Arrow keys or W/S to move, Enter or Right to select, Backspace or Left to go back.
- Chat: type to compose, Enter to send.
- Chat scroll: hold FN and use `;` (up) / `.` (down). Hold Shift for page scroll.

## Persistence (SD)
If a microSD card is present, Cardtastic stores conversations as JSON under `/cardtastic/` on the card. The app keeps the most recent 50 messages per conversation. If no card is available, conversations live in RAM only.

## License
This project is licensed under the GPLv3.
