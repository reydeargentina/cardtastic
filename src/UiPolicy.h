#pragma once

inline bool shouldMarkMessageUnread(bool fromMe, bool visibleAtChatBottom) {
    return !fromMe && !visibleAtChatBottom;
}
