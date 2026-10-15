# Changelog

All notable changes to this project will be documented in this file.

## v0.6 - 2026-10-15
- Improved message delivery reliability and show delivery status in chat.
- Refresh conversations as messages arrive and retain node identities for sender labels.
- Improved BLE scanning, startup auto-connect retries, and initial configuration sync status.
- Improved node list activity and recency indicators.
- Render Spanish accents and resolve mentions in chat; preserve UTF-8 wrapping and omit unsupported emoji.
- Fixed deserialization of 32-bit node IDs.

## v0.5 - 2026-02-09
- Improved BLE connection stability and auth failure handling.
- Added config request retries with timeouts for more reliable node sync.
- Adjusted BLE write behavior to match characteristic capabilities.
- UI: clearer connection warnings and correct connected-device marking.
- Version bump to v0.5.
