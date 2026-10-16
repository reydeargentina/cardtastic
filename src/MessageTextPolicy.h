#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr size_t kMaxMessageTextBytes = 200;

inline bool messageTextFits(size_t byteCount) { return byteCount <= kMaxMessageTextBytes; }

inline size_t previousUtf8Boundary(const char* text, size_t length, size_t offset) {
    if (!text || offset == 0) return 0;
    if (offset > length) offset = length;
    size_t start = offset - 1;
    while (start > 0 && (static_cast<uint8_t>(text[start]) & 0xC0) == 0x80) --start;
    return start;
}

inline bool isValidUtf8(const char* text, size_t length) {
    if (!text && length != 0) return false;
    for (size_t i = 0; i < length;) {
        const uint8_t first = static_cast<uint8_t>(text[i]);
        if (first <= 0x7F) {
            ++i;
            continue;
        }

        size_t count = 0;
        uint32_t codepoint = 0;
        if ((first & 0xE0) == 0xC0) {
            count = 2;
            codepoint = first & 0x1F;
        } else if ((first & 0xF0) == 0xE0) {
            count = 3;
            codepoint = first & 0x0F;
        } else if ((first & 0xF8) == 0xF0) {
            count = 4;
            codepoint = first & 0x07;
        } else {
            return false;
        }
        if (i + count > length) return false;
        for (size_t j = 1; j < count; ++j) {
            const uint8_t continuation = static_cast<uint8_t>(text[i + j]);
            if ((continuation & 0xC0) != 0x80) return false;
            codepoint = (codepoint << 6) | (continuation & 0x3F);
        }
        if ((count == 2 && codepoint < 0x80) ||
            (count == 3 && codepoint < 0x800) ||
            (count == 4 && codepoint < 0x10000) ||
            codepoint > 0x10FFFF ||
            (codepoint >= 0xD800 && codepoint <= 0xDFFF)) return false;
        i += count;
    }
    return true;
}

inline bool copyMessageTextPayload(const char* text, size_t length, uint8_t* payload, size_t capacity) {
    if (!payload || !messageTextFits(length) || length > capacity || !isValidUtf8(text, length)) return false;
    for (size_t i = 0; i < length; ++i) payload[i] = static_cast<uint8_t>(text[i]);
    return true;
}
