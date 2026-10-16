# Native client-state regressions

These tests exercise the same dependency-free state helpers used by the firmware.
They do not require PlatformIO, an Arduino mock, or a connected board.

From the repository root on Linux:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -Isrc test/native/client_state_test.cpp -o /tmp/cardtastic-client-state-test
/tmp/cardtastic-client-state-test
```

Coverage: finite retry budgets for failed writes and missing responses, explicit
retry, nonce correlation, disconnect invalidation, inactivity versus total
timeout, `millis()` rollover, control-event FIFO, ring wrap/overflow and terminal
recovery, coalesced notifications/logs, session admission and deferred reuse,
and radio identity transitions independent of SD availability.

The queue is deliberately externally synchronized in production. These tests
validate its policy, not FreeRTOS scheduling or NimBLE's internals.

## Hardware integration checklist (issues #2 and #3)

- First pairing with six-digit PIN (including leading zeroes), wrong PIN,
  existing bond, cancellation and pairing timeout. The PIN screen should exit
  when the request is no longer active.
- Radio missing at startup; three bounded auto-connect attempts. Manual connect
  remains available afterwards.
- Cancel a pending connection, then immediately choose the same/another radio.
  The new attempt must wait for the old stack terminal event and cleanup.
- Disconnect during authentication, GATT discovery, config request/read and
  sending. No crash, unexpected cleared model or stale event applied to a new link.
- Sync succeeds only for the current config nonce with a known local node ID.
  Use a test radio/transport to suppress completion or reject writes: four attempts
  then SYNC FAILED. R in Connect starts another bounded cycle.
- A large node database with ongoing config progress must not hit the four-second
  inactivity timeout, but each attempt still has a 30-second total limit.
- While AUTH/SYNC/SYNC FAILED, Enter in chat preserves the draft without sending.
  After a valid sync the same draft can be sent.
- Continuous traffic/notifications must not starve keyboard or lifecycle events.
  Reception yields after eight records or approximately 30 ms between GATT calls;
  individual synchronous GATT operations may take longer.
- Request a scan while connected: the link closes first; scanning is available
  on the next selection, without changing active device indexes underneath BLE.

## Radio/session checks (issue #4)

- Change radio A -> B with SD and without SD. A's conversations, cached node
  names, unread state, active chat and selected node must not appear under B.
- Disconnect from A with a chat open, then reconnect to A: the chat/draft and
  node selection must be cleared, while A's conversations remain available.
- Disconnect from A without SD and connect to B: B starts with only its own
  in-RAM broadcast conversation; received packets are not assigned until its
  node identity is known. Return to A and verify the runtime model is loaded
  from A's SD history when present.
- Try to leave a chat open over disconnect and press Enter: no packet is sent
  and no file is saved without a current radio identity.
- Start a DM, disconnect before ACK, and reconnect/switch radio: its pending
  delivery ends as failed in the original radio's runtime history, never updates
  a message on the other radio.

## Conversation storage checks (issue #5)

- With two radios and the same broadcast channel and DM peer, save messages on
  both; verify they are stored under separate `/cardtastic/<radio-id>/`
  directories and remain distinct after switching A -> B -> A.
- Start with a legacy root-level conversation whose `radio` field matches A.
  On A's first load it should be copied to A's namespace, parsed and identity-
  checked before the legacy source is removed.
- Force a failed/partial migration (for example, remove the card during copy).
  The legacy source must remain available and the incomplete temporary file
  must not be treated as a conversation.
- A legacy root-level file with a missing/zero `radio` field is ambiguous: it
  must remain untouched and must not be loaded automatically for either radio.
- If the namespaced destination already exists, it is authoritative; migration
  must not overwrite it with the legacy file.

Record radio firmware version and serial logs. A native pass and a successful
firmware build do not replace these hardware checks.
