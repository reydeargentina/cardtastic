# Cardtastic

**Cardtastic v0.6** is a lightweight Meshtastic client for the **M5Stack Cardputer v1.1**. It connects to a Meshtastic radio over Bluetooth Low Energy (BLE) and puts conversations, channels, and nearby nodes at your fingertips—without changing the radio's configuration.

## What you can do

- Scan for Meshtastic radios and connect over BLE, including PIN-protected pairing.
- Browse known nodes and channels, and start direct or channel conversations.
- Send and receive messages, see delivery progress, and spot unread conversations.
- Keep recent conversations on a microSD card, when one is available.
- See connection and initial synchronization status while the node's data loads.

Chat supports Spanish accents and UTF-8 text wrapping. Emoji are omitted from displayed messages.

## Requirements

- M5Stack Cardputer v1.1 (ESP32-S3)
- A Meshtastic radio with Bluetooth enabled
- PlatformIO (Core/CLI or the PlatformIO IDE extension)
- Optional: microSD card for conversation persistence

## Build and upload

From the project directory, build the firmware:

```sh
pio run -e m5stack-cardputer
```

Connect the Cardputer over USB and upload:

```sh
pio run -e m5stack-cardputer -t upload
```

To open the serial monitor:

```sh
pio device monitor -e m5stack-cardputer
```

The project targets the `m5stack-cardputer` PlatformIO environment. For a local serial-port override, create the git-ignored `platformio.local.ini` file:

```ini
[env:m5stack-cardputer]
upload_port = /dev/serial/by-id/your-cardputer
monitor_port = /dev/serial/by-id/your-cardputer
```

## First connection

1. Make sure Bluetooth is enabled on your already-configured Meshtastic radio.
2. On Cardtastic, choose **Connect** and scan for the radio.
3. Select the radio to connect. If it requests a BLE passkey, enter the PIN shown by the radio; an existing bond may connect without prompting.
4. Wait for the initial node and channel synchronization to finish, then open **Conversations**, **Channels**, or **Nodes**.

The status distinguishes **AUTH**, **ENTER PIN**, **SYNC**, and **CONNECTED**.
Messages can be sent only after synchronization completes; pressing Enter while
the radio is not ready keeps the chat draft intact. Configuration synchronization
stops after four failed attempts. If **SYNC FAILED** appears, press **R** on the
**Connect** screen to start another cycle. Each attempt allows four seconds
without configuration progress, with a maximum duration of 30 seconds.

Connection changes wait for the previous BLE attempt to finish closing. To scan
while connected, select **Scan for devices** to disconnect, then select it again
after the link closes.

Cardtastic is intended for messaging with an already-configured node. Configure region, channels, and frequencies with the Meshtastic app; Cardtastic does not modify those settings.

## Controls

- **Menus:** Arrow keys or W/S to move; Enter or Right to select; Backspace or Left to go back.
- **Chat:** Type to compose and press Enter to send. Backspace edits the draft.
- **Chat scrolling:** Hold FN and press `;` to scroll up or `.` to scroll down. Hold Shift for page scrolling.
- **Connect:** Press A to toggle startup auto-connect, or R to restart configuration synchronization on an authenticated connection.
- **Delete/back:** In the connection list, Backspace deletes the selected entry; FN+Backspace goes back.

## Conversation storage

When a microSD card is available, Cardtastic stores conversation data as JSON under `/cardtastic/` and keeps up to the most recent 50 messages per conversation. Without an SD card, conversations remain in RAM and are lost when the device restarts.

## Changelog and license

See [CHANGELOG.md](CHANGELOG.md) for release history. Cardtastic is licensed under the GNU General Public License v3.0; see [LICENSE](LICENSE).
