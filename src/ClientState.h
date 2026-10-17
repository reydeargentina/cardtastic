#pragma once

#include <stddef.h>
#include <stdint.h>

// Pure state helpers: no BLE, Arduino, or wall-clock dependency.
inline bool meshDeadlineReached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

enum class MeshSyncPhase { IDLE, WAITING, BACKOFF, READY, FAILED };

class MeshConfigSync {
public:
    static constexpr uint8_t maxAttempts = 4;
    static constexpr uint32_t retryMs = 1000;
    static constexpr uint32_t idleTimeoutMs = 4000;
    static constexpr uint32_t attemptTimeoutMs = 30000;

    MeshSyncPhase phase = MeshSyncPhase::IDLE;
    uint8_t attempts = 0;
    uint32_t expectedId = 0;

    void reset() {
        phase = MeshSyncPhase::IDLE;
        attempts = 0;
        expectedId = 0;
    }

    void start(uint32_t now) {
        reset();
        phase = MeshSyncPhase::BACKOFF;
        _nextAttempt = now;
    }

    bool due(uint32_t now) const {
        return phase == MeshSyncPhase::BACKOFF && meshDeadlineReached(now, _nextAttempt);
    }

    bool beginAttempt(uint32_t now) {
        if (!due(now)) return false;
        ++attempts;
        // Keep the nonce sequence across resets/reconnects. Avoid special API nonces.
        if (++_nonce == 0 || _nonce >= 0xFFFFFF00u) _nonce = 1;
        expectedId = _nonce;
        _started = _progress = now;
        phase = MeshSyncPhase::WAITING;
        return true;
    }

    void progress(uint32_t now) {
        if (phase == MeshSyncPhase::WAITING && !timedOut(now)) _progress = now;
    }

    bool timedOut(uint32_t now) const {
        return phase == MeshSyncPhase::WAITING &&
               (now - _progress >= idleTimeoutMs || now - _started >= attemptTimeoutMs);
    }

    void failAttempt(uint32_t now) {
        expectedId = 0;
        phase = attempts >= maxAttempts ? MeshSyncPhase::FAILED : MeshSyncPhase::BACKOFF;
        _nextAttempt = now + retryMs;
    }

    bool complete(uint32_t id, bool hasIdentity, uint32_t now) {
        if (phase != MeshSyncPhase::WAITING || timedOut(now) || id != expectedId || !hasIdentity) return false;
        phase = MeshSyncPhase::READY;
        return true;
    }

private:
    uint32_t _nonce = 0;
    uint32_t _started = 0;
    uint32_t _progress = 0;
    uint32_t _nextAttempt = 0;
};

// Application-side ownership gate. The stack itself remains owned by NimBLE.
class MeshBleSession {
public:
    uint32_t id = 0;
    bool active = false;
    bool closing = false;
    bool terminalSeen = false;

    bool begin() {
        if (active || closing) return false;
        if (++id == 0) ++id;
        active = true;
        terminalSeen = false;
        return true;
    }

    bool accepts(uint32_t eventSession) const { return active && eventSession == id; }
    bool canRelease(bool stackConnected, bool stackConnecting = false) const {
        return closing && terminalSeen && !stackConnected && !stackConnecting;
    }
    void release() { active = closing = terminalSeen = false; }
};

enum class RadioStateTransition { UNCHANGED, IDENTITY_CHANGED, DISCONNECTED, RECONNECTED };

// Tracks radio identity independently from microSD availability or UI screen.
class RadioIdentityTracker {
public:
    RadioStateTransition observe(uint32_t radioNode) {
        if (radioNode == 0) {
            if (!_connected) return RadioStateTransition::UNCHANGED;
            _connected = false;
            return RadioStateTransition::DISCONNECTED;
        }
        if (radioNode != _radioNode) {
            _radioNode = radioNode;
            _connected = true;
            return RadioStateTransition::IDENTITY_CHANGED;
        }
        if (!_connected) {
            _connected = true;
            return RadioStateTransition::RECONNECTED;
        }
        return RadioStateTransition::UNCHANGED;
    }

    uint32_t radioNode() const { return _radioNode; }
    bool connected() const { return _connected; }

private:
    uint32_t _radioNode = 0;
    bool _connected = false;
};

enum class MeshBleEventType { CONNECTED, CONNECT_FAILED, DISCONNECTED, AUTH, PASSKEY, FROM_NUM, LOG };

struct MeshBleEvent {
    MeshBleEventType type = MeshBleEventType::CONNECTED;
    uint32_t session = 0;
    uint16_t handle = 0xFFFF;
    int reason = 0;
    bool encrypted = false;
    uint32_t fromNum = 0;
    char log[96] = {};
};

// The caller protects ALL access, including overflow state, across tasks.
class MeshBleEventBuffer {
public:
    static constexpr size_t capacity = 16;

    bool push(const MeshBleEvent& event) {
        // Notifications cannot consume the control-event queue. Only the latest
        // packet counter matters; polling remains a fallback even without notify.
        if (event.type == MeshBleEventType::FROM_NUM) {
            _fromNum = event;
            _hasFromNum = true;
            return true;
        }
        if (event.type == MeshBleEventType::LOG) {
            _log = event;
            _hasLog = true; // debug logs are deliberately lossy
            return true;
        }
        if (_count == capacity) {
            _overflow = true;
            // Preserve a terminal event for recovery if the control ring filled.
            if (event.type == MeshBleEventType::DISCONNECTED ||
                event.type == MeshBleEventType::CONNECT_FAILED) {
                _terminal = event;
                _hasTerminal = true;
            }
            return false;
        }
        _events[(_read + _count) % capacity] = event;
        ++_count;
        return true;
    }

    bool pop(MeshBleEvent& event) {
        if (_count) {
            event = _events[_read];
            _read = (_read + 1) % capacity;
            --_count;
        } else if (_hasTerminal) {
            event = _terminal;
            _hasTerminal = false;
        } else if (_hasFromNum) {
            event = _fromNum;
            _hasFromNum = false;
        } else if (_hasLog) {
            event = _log;
            _hasLog = false;
        } else {
            return false;
        }
        return true;
    }

    bool takeOverflow() {
        bool overflow = _overflow;
        _overflow = false;
        return overflow;
    }

private:
    MeshBleEvent _events[capacity];
    MeshBleEvent _fromNum, _log, _terminal;
    size_t _read = 0;
    size_t _count = 0;
    bool _hasFromNum = false, _hasLog = false, _hasTerminal = false;
    bool _overflow = false;
};
