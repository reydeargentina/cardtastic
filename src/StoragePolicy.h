#pragma once

#include <stddef.h>
#include <stdint.h>
#include <vector>

inline bool storageDeadlineReached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

// Shared backoff for storage retries. It is intentionally time-source agnostic
// so rollover and throttling behavior can be tested natively.
class StorageRetrySchedule {
public:
    bool due(uint32_t now) const {
        return !_scheduled || storageDeadlineReached(now, _nextAttempt);
    }

    void failed(uint32_t now) {
        if (_failures < 6) ++_failures;
        uint32_t delay = 3000UL << (_failures - 1);
        if (delay > 60000UL) delay = 60000UL;
        _nextAttempt = now + delay;
        _scheduled = true;
    }

    void succeeded() {
        _failures = 0;
        _scheduled = false;
        _nextAttempt = 0;
    }

private:
    uint8_t _failures = 0;
    uint32_t _nextAttempt = 0;
    bool _scheduled = false;
};

// Merge two chronologically ordered histories by their longest exact
// persisted-suffix/RAM-prefix overlap. No non-overlapping records are discarded.
template <typename T, typename Equal>
void mergeStorageHistories(std::vector<T>& persisted,
                           const std::vector<T>& memory,
                           size_t maximum,
                           Equal equal) {
    size_t overlap = persisted.size() < memory.size() ? persisted.size() : memory.size();
    while (overlap > 0) {
        bool matches = true;
        size_t start = persisted.size() - overlap;
        for (size_t i = 0; i < overlap; ++i) {
            if (!equal(persisted[start + i], memory[i])) {
                matches = false;
                break;
            }
        }
        if (matches) break;
        --overlap;
    }
    for (size_t i = overlap; i < memory.size(); ++i) persisted.push_back(memory[i]);
    if (maximum > 0 && persisted.size() > maximum) {
        persisted.erase(persisted.begin(), persisted.begin() + (persisted.size() - maximum));
    }
}
