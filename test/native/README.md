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
radio identity transitions independent of SD availability, storage retry backoff
including rollover, RAM/SD history overlap merging, the shared message-byte limit,
UTF-8 validation, and codepoint-safe backspace boundaries.

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
- Request a scan while connected: the link closes first; scanning starts
  automatically, without changing active device indexes underneath BLE.

## Nonblocking BLE work (CT-12)

- Leave the saved radio off. Request Scan during the first auto-connect attempt,
  during retry backoff, and near the timeout boundary. One selection must cancel
  retries, wait for the terminal event and native GAP inactivity, then scan.
  There must be no `Unable to scan - connection in progress` error.
- While waiting for cancellation, cancel Scan or leave Connect with Backspace.
  No scan may start later. A subsequent manual scan/connection must work.
- During each scan attempt, navigate the live results, cancel with Enter on
  `Cancel scan`, or return home with Backspace. Select a discovered device while
  scanning: discovery stops before the connection starts. Repeat several times.
- With no nearby radios, the three eight-second attempts must finish without
  blocking navigation. With unnamed results, names can update between attempts.
- Sync a large node database while using the keyboard and receiving numbered
  messages. Compare received IDs/history against the sender: yielding between
  batches must not discard records. The existing receive budget remains eight
  records or 30 ms, checked between synchronous GATT reads.
- For physical latency measurements, build with
  `-DCARDTASTIC_DEBUG_BLE_TIMING=1`. Serial reports maximum BLE service and full
  loop duration once per second, in microseconds. Record maxima while idle,
  scanning, cancelling a connection, and synchronizing the same database, plus
  observed keyboard response. These are service durations, not a guaranteed
  keyboard latency; an individual synchronous GATT operation can exceed the
  receive budget. No hardware latency figures are claimed by the native tests.
- The previous blocking scan occupied the loop for each eight-second attempt
  (up to three attempts plus pauses). Compare with the new build on the same
  hardware; record firmware versions, node counts, timings and serial logs.
- Full GATT enumeration is diagnostic only and requires
  `-DCARDTASTIC_DEBUG_GATT=1`; normal connection discovers only the Meshtastic
  service/characteristics it needs.

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

## microSD recovery checks (issue #6)

- Fill/remove the card during conversation and node-cache writes. The current
  final file or backup must remain valid; partial temporaries must not replace
  it. The UI reports an SD warning while chat and reception continue in RAM.
- Interrupt power between writing the verified temporary, renaming the old
  final to `.bak`, and promoting the temporary. On reboot, a valid temporary
  or backup is recovered; corrupt files remain preserved as `.corrupt*`.
- Inject open, short-write, rename and read/parse failures. Verify the old
  history remains readable, the error is logged/shown, and retries are delayed
  rather than repeated on every main-loop iteration.
- Start with SD unavailable, receive messages in RAM, then insert a card with
  an existing history for that radio. The RAM and SD sequences merge using the
  longest exact SD-suffix/RAM-prefix overlap; non-overlapping records are kept,
  and the merged snapshot is eventually saved.
- Remove/reinsert the card after storage was loaded. Verify reloading and
  reconciliation do not erase new RAM messages or silently replace history.

## Message size checks (issue #7)

- Verify 199, 200, and 201 UTF-8 bytes: only the first two are sendable; the
  client rejects excess without writing a packet or adding a history entry.
- Verify multibyte text at exactly 200 bytes and one byte over. The counter is
  byte-based, backspace removes a whole UTF-8 codepoint, and the visible editor
  suffix never starts inside a codepoint.
- Force a BLE write failure at the limit. The draft stays editable, the failed
  payload is not recorded as a sent chat message, and retrying after correction
  records exactly the bytes accepted by the client.
- Long drafts keep their final text and cursor visible beside the counter.

Record radio firmware version and serial logs. A native pass and a successful
firmware build do not replace these hardware checks.
