#include "ClientState.h"
#include "StoragePolicy.h"

#include <assert.h>
#include <stdio.h>
#include <string>
#include <vector>

static void storagePolicy() {
    StorageRetrySchedule retry;
    assert(retry.due(0));
    retry.failed(100);
    assert(!retry.due(3099));
    assert(retry.due(3100));
    retry.failed(3100);
    assert(!retry.due(9099));
    assert(retry.due(9100));
    retry.succeeded();
    assert(retry.due(9100));
    retry.failed(UINT32_MAX - 1000);
    assert(!retry.due(UINT32_MAX - 1));
    assert(retry.due(1999));

    std::vector<std::string> persisted = {"one", "two"};
    std::vector<std::string> memory = {"two", "three"};
    mergeStorageHistories(persisted, memory, 10,
        [](const std::string& a, const std::string& b) { return a == b; });
    assert((persisted == std::vector<std::string>{"one", "two", "three"}));

    std::vector<std::string> empty;
    std::vector<std::string> fresh = {"ram-1", "ram-2"};
    mergeStorageHistories(empty, fresh, 10,
        [](const std::string& a, const std::string& b) { return a == b; });
    assert(empty == fresh);

    std::vector<std::string> disjoint = {"disk-1", "disk-2"};
    std::vector<std::string> ram = {"ram-1", "ram-2"};
    mergeStorageHistories(disjoint, ram, 3,
        [](const std::string& a, const std::string& b) { return a == b; });
    assert((disjoint == std::vector<std::string>{"disk-2", "ram-1", "ram-2"}));
}

static void finiteAttempts(bool writeFails, uint32_t now) {
    MeshConfigSync sync;
    sync.start(now);
    uint32_t previousId = 0;
    for (unsigned attempt = 1; attempt <= MeshConfigSync::maxAttempts; ++attempt) {
        assert(sync.due(now));
        assert(sync.beginAttempt(now));
        assert(sync.attempts == attempt);
        assert(sync.expectedId != 0 && sync.expectedId != previousId);
        previousId = sync.expectedId;
        assert(!sync.beginAttempt(now));
        if (!writeFails) {
            assert(!sync.timedOut(now + MeshConfigSync::idleTimeoutMs - 1));
            now += MeshConfigSync::idleTimeoutMs;
            assert(sync.timedOut(now));
        }
        sync.failAttempt(now);
        assert(!sync.complete(previousId, true, now));
        assert(!sync.due(now + MeshConfigSync::retryMs - 1));
        now += MeshConfigSync::retryMs;
    }
    assert(sync.phase == MeshSyncPhase::FAILED);
    assert(!sync.due(now + 1000000));
    assert(!sync.beginAttempt(now));
    sync.start(now); // explicit manual retry gets a new budget, not an old nonce
    assert(sync.attempts == 0);
    assert(sync.beginAttempt(now));
    assert(!sync.complete(previousId, true, now));
    assert(sync.complete(sync.expectedId, true, now));
}

static void synchronization() {
    MeshConfigSync sync;
    assert(!sync.complete(0, true, 0));
    sync.start(0);
    assert(sync.beginAttempt(0)); // millis()==0 is a valid request timestamp
    uint32_t id = sync.expectedId;
    assert(!sync.complete(id + 1, true, 1));
    assert(!sync.complete(id, false, 1));
    assert(sync.complete(id, true, 1));
    assert(!sync.complete(id, true, 2)); // duplicate completion
    sync.reset(); // disconnect must invalidate a pending or ready configuration
    assert(!sync.complete(id, true, 2));
    sync.start(10);
    assert(sync.beginAttempt(10));
    assert(sync.expectedId != id);
    assert(!sync.complete(id, true, 10));

    // Valid config records extend inactivity, but cannot extend the hard limit.
    uint32_t started = 10;
    for (uint32_t elapsed = 1000; elapsed < MeshConfigSync::attemptTimeoutMs; elapsed += 1000) {
        sync.progress(started + elapsed);
        assert(!sync.timedOut(started + elapsed));
    }
    uint32_t end = started + MeshConfigSync::attemptTimeoutMs;
    sync.progress(end);
    assert(sync.timedOut(end));
    assert(!sync.complete(sync.expectedId, true, end));

    sync.start(100);
    assert(sync.beginAttempt(100));
    sync.progress(4100); // a late response must not revive an expired request
    assert(sync.timedOut(4100));
    assert(!sync.complete(sync.expectedId, true, 4100));

    finiteAttempts(false, 0);
    finiteAttempts(true, 100);
    finiteAttempts(false, UINT32_MAX - 1000); // inactivity and backoff wrap
    finiteAttempts(true, UINT32_MAX - 500);

    sync.start(UINT32_MAX - 100);
    assert(sync.beginAttempt(UINT32_MAX - 100));
    for (uint32_t elapsed = 1000; elapsed < MeshConfigSync::attemptTimeoutMs; elapsed += 1000) {
        sync.progress(UINT32_MAX - 100 + elapsed);
    }
    assert(sync.timedOut(UINT32_MAX - 100 + MeshConfigSync::attemptTimeoutMs));
}

static MeshBleEvent makeEvent(MeshBleEventType type, uint32_t session) {
    MeshBleEvent event;
    event.type = type;
    event.session = session;
    return event;
}

static void eventQueue() {
    MeshBleEventBuffer queue;
    MeshBleEvent out;
    assert(!queue.pop(out));
    assert(!queue.takeOverflow());
    // A notification/log flood cannot displace connection/auth/disconnect events.
    for (unsigned i = 0; i < 10000; ++i) {
        auto notify = makeEvent(MeshBleEventType::FROM_NUM, 1);
        notify.fromNum = i;
        assert(queue.push(notify));
        assert(queue.push(makeEvent(MeshBleEventType::LOG, 1)));
    }
    assert(queue.push(makeEvent(MeshBleEventType::CONNECTED, 1)));
    assert(queue.push(makeEvent(MeshBleEventType::PASSKEY, 1)));
    assert(queue.push(makeEvent(MeshBleEventType::AUTH, 1)));
    assert(queue.push(makeEvent(MeshBleEventType::DISCONNECTED, 1)));
    assert(queue.pop(out) && out.type == MeshBleEventType::CONNECTED);
    assert(queue.pop(out) && out.type == MeshBleEventType::PASSKEY);
    assert(queue.pop(out) && out.type == MeshBleEventType::AUTH);
    assert(queue.pop(out) && out.type == MeshBleEventType::DISCONNECTED);
    assert(queue.pop(out) && out.type == MeshBleEventType::FROM_NUM && out.fromNum == 9999);
    assert(queue.pop(out) && out.type == MeshBleEventType::LOG);
    assert(!queue.pop(out));
    assert(!queue.takeOverflow());

    // Wrap the ring several times, checking FIFO and session preservation.
    for (unsigned cycle = 1; cycle <= 3; ++cycle) {
        for (size_t i = 0; i < MeshBleEventBuffer::capacity; ++i) {
            auto event = makeEvent(MeshBleEventType::AUTH, cycle);
            event.reason = i;
            assert(queue.push(event));
        }
        assert(!queue.push(makeEvent(MeshBleEventType::AUTH, cycle)));
        assert(!queue.push(makeEvent(MeshBleEventType::DISCONNECTED, cycle)));
        assert(queue.takeOverflow());
        assert(!queue.takeOverflow());
        for (size_t i = 0; i < MeshBleEventBuffer::capacity; ++i) {
            assert(queue.pop(out) && out.reason == static_cast<int>(i) && out.session == cycle);
        }
        assert(queue.pop(out) && out.type == MeshBleEventType::DISCONNECTED && out.session == cycle);
        assert(!queue.pop(out));
    }
}

static void lifecycle() {
    MeshBleSession session;
    MeshConfigSync sync;
    assert(session.begin());
    uint32_t old = session.id;
    sync.start(0);
    assert(sync.beginAttempt(0));
    uint32_t oldConfig = sync.expectedId;

    session.closing = true; // disconnect/cancel requested while synchronizing
    sync.reset();
    assert(!session.begin());
    assert(!session.canRelease(false)); // disconnected flag alone isn't a terminal event
    assert(!sync.complete(oldConfig, true, 1));
    session.terminalSeen = true;
    assert(!session.canRelease(true)); // callback arrives before stack handle is cleared
    assert(!session.begin());
    assert(session.canRelease(false));
    session.release();
    assert(!session.accepts(old));
    assert(session.begin());
    assert(session.id != old);
    assert(!session.accepts(old)); // old auth/notify/terminal cannot reset the new session
    assert(session.accepts(session.id));
    sync.start(2);
    assert(sync.beginAttempt(2));
    assert(!sync.complete(oldConfig, true, 3));
    assert(sync.complete(sync.expectedId, true, 3));

    // Failed attempts (no connection handle) also require terminal acknowledgement.
    session.closing = true;
    assert(!session.canRelease(false));
    session.terminalSeen = true;
    assert(session.canRelease(false));
    session.release();
    session.id = UINT32_MAX;
    assert(session.begin() && session.id == 1);
}

static void radioIdentityIsolation() {
    RadioIdentityTracker radios;
    assert(radios.radioNode() == 0 && !radios.connected());
    assert(radios.observe(0) == RadioStateTransition::UNCHANGED);
    assert(radios.observe(0xAABBCCDD) == RadioStateTransition::IDENTITY_CHANGED);
    assert(radios.radioNode() == 0xAABBCCDD && radios.connected());
    assert(radios.observe(0xAABBCCDD) == RadioStateTransition::UNCHANGED);
    assert(radios.observe(0) == RadioStateTransition::DISCONNECTED);
    assert(radios.radioNode() == 0xAABBCCDD && !radios.connected());
    // Reconnecting to the same radio retains radio-owned conversations but
    // the caller still clears session-only selections/drafts on DISCONNECTED.
    assert(radios.observe(0xAABBCCDD) == RadioStateTransition::RECONNECTED);
    assert(radios.observe(0x01020304) == RadioStateTransition::IDENTITY_CHANGED);
    assert(radios.radioNode() == 0x01020304 && radios.connected());
    // The transition policy contains no SD dependency: A -> no SD -> B always
    // changes identity and tells the caller to discard A's RAM-owned model.
    RadioIdentityTracker noSd;
    assert(noSd.observe(10) == RadioStateTransition::IDENTITY_CHANGED);
    assert(noSd.observe(0) == RadioStateTransition::DISCONNECTED);
    assert(noSd.observe(20) == RadioStateTransition::IDENTITY_CHANGED);
}

int main() {
    storagePolicy();
    synchronization();
    eventQueue();
    lifecycle();
    radioIdentityIsolation();
    puts("client_state: all regression checks passed");
}
