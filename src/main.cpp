#include <SD.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <algorithm>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include "M5Cardputer.h"
#include <vector>
#include <NimBLEDevice.h> 
#include <string>
#include "MeshtasticClient.h"
#include "MessageTextPolicy.h"
#include "StoragePolicy.h"
#include "meshtastic/channel.pb.h"

// -------------------- Basic navigation --------------------

enum class NavKey {
    NONE,
    UP,
    UP_FAST,
    DOWN,
    DOWN_FAST,
    ENTER,
    ESC,
    RETRY_SYNC,
    TOGGLE_AUTOCONNECT
};

// -------------------- Conversation / Message model --------------------

constexpr uint32_t kBroadcastAddr = 0xFFFFFFFF;
constexpr size_t kConversationMaxMessages = 50;
constexpr size_t kConversationJsonMax = 32768;
constexpr uint32_t kPacketDedupWindowSeconds = 10 * 60;
const char* kStorageDir = "/cardtastic";
constexpr uint32_t kScreenIdleMs = 60000;

constexpr uint16_t kColorBodyBg = BLACK;
constexpr uint16_t kColorBodyText = WHITE;
constexpr uint16_t kColorHeaderBg = DARKGREEN;
constexpr uint16_t kColorHeaderText = WHITE;
constexpr uint16_t kColorFooterBg = DARKGREEN;
constexpr uint16_t kColorFooterText = WHITE;
constexpr uint16_t kColorSeparator = GREEN;
constexpr int kFooterH = 10;

uint32_t decodeUtf8Codepoint(const String& text, size_t index, size_t& byteCount) {
    const size_t length = text.length();
    if (index >= length) {
        byteCount = 0;
        return 0;
    }

    const uint8_t first = static_cast<uint8_t>(text[index]);
    if (first < 0x80) {
        byteCount = 1;
        return first;
    }

    uint8_t expected = 0;
    uint32_t codepoint = 0;
    if ((first & 0xE0) == 0xC0) {
        expected = 2;
        codepoint = first & 0x1F;
    } else if ((first & 0xF0) == 0xE0) {
        expected = 3;
        codepoint = first & 0x0F;
    } else if ((first & 0xF8) == 0xF0) {
        expected = 4;
        codepoint = first & 0x07;
    } else {
        byteCount = 1;
        return 0xFFFD;
    }

    if (index + expected > length) {
        byteCount = 1;
        return 0xFFFD;
    }
    for (uint8_t i = 1; i < expected; ++i) {
        uint8_t continuation = static_cast<uint8_t>(text[index + i]);
        if ((continuation & 0xC0) != 0x80) {
            byteCount = 1;
            return 0xFFFD;
        }
        codepoint = (codepoint << 6) | (continuation & 0x3F);
    }

    if ((expected == 2 && codepoint < 0x80) ||
        (expected == 3 && codepoint < 0x800) ||
        (expected == 4 && codepoint < 0x10000) ||
        codepoint > 0x10FFFF ||
        (codepoint >= 0xD800 && codepoint <= 0xDFFF)) {
        byteCount = 1;
        return 0xFFFD;
    }

    byteCount = expected;
    return codepoint;
}

bool isEmojiCodepoint(uint32_t codepoint) {
    return (codepoint >= 0x1F000 && codepoint <= 0x1FAFF) ||
           (codepoint >= 0x2600 && codepoint <= 0x27BF) ||
           (codepoint >= 0x1F1E6 && codepoint <= 0x1F1FF) ||
           (codepoint >= 0x1F3FB && codepoint <= 0x1F3FF) ||
           (codepoint >= 0xFE00 && codepoint <= 0xFE0F) ||
           (codepoint >= 0xE0020 && codepoint <= 0xE007F) ||
           codepoint == 0x200D || codepoint == 0x20E3;
}

String omitEmoji(const String& text) {
    String result;
    result.reserve(text.length());
    for (size_t i = 0; i < text.length();) {
        size_t byteCount = 0;
        uint32_t codepoint = decodeUtf8Codepoint(text, i, byteCount);
        if (!isEmojiCodepoint(codepoint)) {
            result.concat(text.substring(i, i + byteCount));
        }
        i += byteCount;
    }
    return result;
}

uint32_t displayCodepoint(uint32_t codepoint, bool customUppercase) {
    switch (codepoint) {
    case 0x00E1: return 0xA0; // á
    case 0x00E9: return 0x82; // é
    case 0x00ED: return 0xA1; // í
    case 0x00F3: return 0xA2; // ó
    case 0x00FA: return 0xA3; // ú
    case 0x00FC: return 0x81; // ü
    case 0x00F1: return 0xA4; // ñ
    case 0x00C1: return customUppercase ? 0xE000 : 0xA0; // Á
    case 0x00C9: return customUppercase ? 0xE001 : 0x82; // É
    case 0x00CD: return customUppercase ? 0xE002 : 0xA1; // Í
    case 0x00D3: return customUppercase ? 0xE003 : 0xA2; // Ó
    case 0x00DA: return customUppercase ? 0xE004 : 0xA3; // Ú
    case 0x00DC: return 0x9A; // Ü
    case 0x00D1: return 0xA5; // Ñ
    case 0x00BF: return 0xA8; // ¿
    case 0x00A1: return 0xAD; // ¡
    default: return 0;
    }
}

void appendUtf8Codepoint(String& output, uint32_t codepoint) {
    char encoded[5] = {0};
    size_t length = 0;
    if (codepoint < 0x80) {
        encoded[length++] = static_cast<char>(codepoint);
    } else if (codepoint < 0x800) {
        encoded[length++] = static_cast<char>(0xC0 | (codepoint >> 6));
        encoded[length++] = static_cast<char>(0x80 | (codepoint & 0x3F));
    } else if (codepoint < 0x10000) {
        encoded[length++] = static_cast<char>(0xE0 | (codepoint >> 12));
        encoded[length++] = static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F));
        encoded[length++] = static_cast<char>(0x80 | (codepoint & 0x3F));
    } else {
        encoded[length++] = static_cast<char>(0xF0 | (codepoint >> 18));
        encoded[length++] = static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F));
        encoded[length++] = static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F));
        encoded[length++] = static_cast<char>(0x80 | (codepoint & 0x3F));
    }
    encoded[length] = '\0';
    output.concat(encoded);
}

String mapSpanishGlyphsForDisplay(const String& text, bool customUppercase = false) {
    String result;
    result.reserve(text.length());
    for (size_t i = 0; i < text.length();) {
        size_t byteCount = 0;
        uint32_t codepoint = decodeUtf8Codepoint(text, i, byteCount);
        uint32_t displayCode = displayCodepoint(codepoint, customUppercase);
        if (displayCode != 0) {
            appendUtf8Codepoint(result, displayCode);
        } else {
            result.concat(text.substring(i, i + byteCount));
        }
        i += byteCount;
    }
    return result;
}

bool isHexDigit(char c) {
    return (c >= '0' && c <= '9') ||
           (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}

int hexDigitValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return c - 'A' + 10;
}

String knownNodeName(uint32_t nodeNum);

String resolveNodeMentionsForDisplay(const String& text) {
    String result;
    result.reserve(text.length());
    for (size_t i = 0; i < text.length();) {
        if (text[i] == '@' && i + 10 <= text.length() && text[i + 1] == '!') {
            uint32_t nodeNum = 0;
            bool valid = true;
            for (size_t j = i + 2; j < i + 10; ++j) {
                if (!isHexDigit(text[j])) {
                    valid = false;
                    break;
                }
                nodeNum = (nodeNum << 4) | static_cast<uint32_t>(hexDigitValue(text[j]));
            }
            if (valid && (i + 10 == text.length() || !isHexDigit(text[i + 10]))) {
                String name = knownNodeName(nodeNum);
                if (name.length() > 0) {
                    result += "@";
                    result += name;
                    i += 10;
                    continue;
                }
            }
        }

        size_t byteCount = 0;
        decodeUtf8Codepoint(text, i, byteCount);
        result.concat(text.substring(i, i + byteCount));
        i += byteCount;
    }
    return result;
}

String prepareMessageForDisplay(const String& text) {
    return mapSpanishGlyphsForDisplay(resolveNodeMentionsForDisplay(omitEmoji(text)), true);
}

bool drawCustomSpanishGlyph(M5GFX& display, uint32_t codepoint, uint16_t color) {
    static const uint8_t glyphs[][8] = {
        {0x02, 0x04, 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11}, // Á
        {0x02, 0x04, 0x1F, 0x10, 0x1E, 0x10, 0x10, 0x1F}, // É
        {0x02, 0x04, 0x1F, 0x04, 0x04, 0x04, 0x04, 0x1F}, // Í
        {0x02, 0x04, 0x0E, 0x11, 0x11, 0x11, 0x11, 0x0E}, // Ó
        {0x02, 0x04, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}  // Ú
    };
    if (codepoint < 0xE000 || codepoint > 0xE004) return false;

    const uint8_t* glyph = glyphs[codepoint - 0xE000];
    int x = display.getCursorX();
    int y = display.getCursorY();
    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 5; ++col) {
            if (glyph[row] & (1U << (4 - col))) {
                display.drawPixel(x + col, y + row, color);
            }
        }
    }
    display.setCursor(x + 6, y);
    return true;
}

void drawDisplayText(M5GFX& display, const String& text, uint16_t color) {
    size_t segmentStart = 0;
    for (size_t i = 0; i < text.length();) {
        size_t byteCount = 0;
        uint32_t codepoint = decodeUtf8Codepoint(text, i, byteCount);
        if (codepoint >= 0xE000 && codepoint <= 0xE004) {
            if (i > segmentStart) {
                display.print(text.substring(segmentStart, i));
            }
            drawCustomSpanishGlyph(display, codepoint, color);
            i += byteCount;
            segmentStart = i;
        } else {
            i += byteCount;
        }
    }
    if (segmentStart < text.length()) {
        display.print(text.substring(segmentStart));
    }
}

size_t nextUtf8Boundary(const String& text, size_t index) {
    size_t byteCount = 0;
    decodeUtf8Codepoint(text, index, byteCount);
    return index + (byteCount > 0 ? byteCount : 1);
}

void drawHeaderBackground(int height) {
    if (height <= 0) return;
    auto& d = M5Cardputer.Display;
    d.fillRect(0, 0, d.width(), height, kColorHeaderBg);
    d.drawFastHLine(0, height - 1, d.width(), kColorSeparator);
}

void drawFooterBackground() {
    auto& d = M5Cardputer.Display;
    int h = d.height();
    d.fillRect(0, h - kFooterH, d.width(), kFooterH, kColorFooterBg);
    d.drawFastHLine(0, h - kFooterH, d.width(), kColorSeparator);
}

void drawCardtasticLogo(int centerX, int topY, int maxW, int maxH, uint16_t color) {
    const float baseW = 56.0f;
    const float baseH = 40.0f;
    if (maxW <= 0 || maxH <= 0) return;

    float scale = std::min(maxW / baseW, maxH / baseH);
    if (scale <= 0.0f) return;

    int w = (int)lroundf(baseW * scale);
    int h = (int)lroundf(baseH * scale);
    if (w < 12 || h < 8) return;

    int x = centerX - (w / 2);
    int y = topY + (maxH - h) / 2;

    int rx = std::max(2, (int)lroundf(6.0f * scale));
    int nodeR = std::max(2, (int)lroundf(3.0f * scale));

    auto& d = M5Cardputer.Display;
    d.drawRoundRect(x, y, w, h, rx, color);
    if (w > 12 && h > 12 && rx > 2) {
        d.drawRoundRect(x + 1, y + 1, w - 2, h - 2, std::max(1, rx - 1), color);
    }

    auto sx = [&](float px) { return x + (int)lroundf(px * scale); };
    auto sy = [&](float py) { return y + (int)lroundf(py * scale); };

    int leftX   = sx(16.0f);
    int rightX  = sx(40.0f);
    int topYc   = sy(14.0f);
    int bottomX = sx(28.0f);
    int bottomY = sy(26.0f);

    d.drawLine(leftX, topYc, rightX, topYc, color);
    d.drawLine(leftX, topYc, bottomX, bottomY, color);
    d.drawLine(rightX, topYc, bottomX, bottomY, color);
    d.fillCircle(leftX, topYc, nodeR, color);
    d.fillCircle(rightX, topYc, nodeR, color);
    d.fillCircle(bottomX, bottomY, nodeR, color);
}

void drawSplashScreen() {
    auto& d = M5Cardputer.Display;
    d.fillScreen(kColorBodyBg);

    d.setTextSize(2);
    d.setTextColor(kColorHeaderText, kColorBodyBg);
    String title = "CARDTASTIC";
    int titleX = (d.width() - d.textWidth(title)) / 2;
    if (titleX < 0) titleX = 0;
    d.setCursor(titleX, 10);
    d.print(title);

    int logoTop = 28;
    int logoMaxH = d.height() - logoTop - 8;
    int logoMaxW = d.width() - 16;
    if (logoMaxH > 0 && logoMaxW > 0) {
        drawCardtasticLogo(d.width() / 2, logoTop, logoMaxW, logoMaxH, kColorSeparator);
    }
}

enum class MessageDeliveryState {
    UNKNOWN,
    PENDING,
    MESH_CONFIRMED,
    DELIVERED,
    QUEUE_REJECTED,
    FAILED,
    TIMED_OUT,
    DISCONNECTED,
    UNCONFIRMED
};

struct Message {
    bool   fromMe;  // true = this UI, false = remote node
    uint32_t from;
    uint32_t packetId;
    uint32_t rxTime;
    uint8_t channel;
    String text;
    MessageDeliveryState delivery = MessageDeliveryState::UNKNOWN;
    uint32_t deliveryToken = 0;
};

struct Conversation {
    uint32_t             peer;        // nodenum for DM, or broadcast addr
    bool                 isBroadcast;
    uint8_t              channel;
    std::vector<Message> messages;
    int                  firstVisibleIndex;  // -1 = auto-bottom (line index)
    bool                 hasUnread;
    uint32_t             lastActivitySeq;
    bool                 storageDirty = false;
};

struct CachedNodeIdentity {
    uint32_t nodeNum = 0;
    String shortName;
    String longName;
};

static std::vector<Conversation> gConversations;
static std::vector<CachedNodeIdentity> gCachedNodeIdentities;
static int gActiveConversation = -1;
static uint32_t gSelectedNodeNum = 0;
static uint32_t gNextActivitySeq = 1;
static uint32_t gNextDeliveryToken = 1;
static ConnectionStatus gLastRenderedConnectionStatus = ConnectionStatus::DISCONNECTED;
static bool gLastRenderedConfigComplete = false;
static String gLastRenderedSessionStatus;
static String gLastRenderedError;
static String gLastRenderedStorageError;

constexpr uint32_t kNodeActiveMs = 5UL * 60UL * 1000UL;
static bool gSdReady = false;
static StorageRetrySchedule gStorageRetry;
static String gStorageError;
static bool gStorageWarningSticky = false;
static bool gStorageLoaded = false;
static uint32_t gLoadedRadioNode = 0;
static uint32_t gConversationRadioNode = 0;
static RadioIdentityTracker gRadioIdentity;
static uint32_t gNodeIdentityCacheRadio = 0;
static bool gNodeIdentityCacheLoaded = false;
static bool gNodeIdentityCacheDirty = false;
static uint32_t gLastInputMs = 0;
static bool gScreenSleeping = false;
static int gPendingPinDeviceIndex = -1;

Conversation* getConversation(int index);
int ensureConversation(uint32_t peer, bool isBroadcast, uint8_t channel);
void initConversations();
String resolvedNodeLabel(uint32_t nodeNum);
void handleRadioIdentityChanged();
void handleRadioDisconnected();
void applyRadioStateTransition(uint32_t radioNode);

bool initStorage() {
    int cs = M5.getPin(m5::sd_spi_cs);
    int sck = M5.getPin(m5::sd_spi_sclk);
    int mosi = M5.getPin(m5::sd_spi_mosi);
    int miso = M5.getPin(m5::sd_spi_miso);

    if (cs < 0 || cs == 255 || sck < 0 || sck == 255 || mosi < 0 || mosi == 255 || miso < 0 || miso == 255) {
        // Fallback to Cardputer SPI SD pins (per M5Cardputer SD example).
        sck = 40;
        miso = 39;
        mosi = 14;
        cs = 12;
    }

    SPI.begin(sck, miso, mosi, cs);
    if (!SD.begin(cs, SPI, 20000000)) {
        if (!SD.begin(cs, SPI, 10000000)) {
            return false;
        }
    }

    if (!SD.exists(kStorageDir)) {
        if (!SD.mkdir(kStorageDir)) {
            return false;
        }
    } else {
        File dir = SD.open(kStorageDir);
        if (!dir || !dir.isDirectory()) {
            if (dir) dir.close();
            return false;
        }
        dir.close();
    }
    return true;
}

void storageFailure(const char* operation) {
    uint32_t now = millis();
    gSdReady = false;
    gStorageLoaded = false;
    gLoadedRadioNode = 0;
    gNodeIdentityCacheLoaded = false;
    gStorageError = String("SD ") + operation;
    gStorageRetry.failed(now);
    Serial.printf("[Storage] %s failed; retry scheduled\n", operation);
}

void storageSuccess() {
    gStorageRetry.succeeded();
    if (!gStorageWarningSticky) gStorageError = "";
    gSdReady = true;
}

bool ensureStorage() {
    if (gSdReady) return true;
    uint32_t now = millis();
    if (!gStorageRetry.due(now)) return false;
    gSdReady = initStorage();
    if (gSdReady) {
        storageSuccess();
    } else {
        gStorageError = "SD unavailable";
        gStorageRetry.failed(now);
        Serial.println("[Storage] SD mount failed; retry scheduled");
    }
    return gSdReady;
}

String nodeHexId(uint32_t num) {
    char buf[9];
    snprintf(buf, sizeof(buf), "%08lX", (unsigned long)num);
    return String(buf);
}

String nodeIdentityCachePath(uint32_t radioNode) {
    return String(kStorageDir) + "/nodes_" + nodeHexId(radioNode) + ".json";
}

String radioStoragePath(uint32_t radioNode) {
    return String(kStorageDir) + "/" + nodeHexId(radioNode);
}

bool ensureRadioStorage(uint32_t radioNode) {
    if (radioNode == 0 || !ensureStorage()) return false;
    String path = radioStoragePath(radioNode);
    if (SD.exists(path)) {
        File dir = SD.open(path);
        bool valid = dir && dir.isDirectory();
        if (dir) dir.close();
        if (!valid) storageFailure("radio directory check");
        return valid;
    }
    if (!SD.mkdir(path)) {
        storageFailure("directory creation");
        return false;
    }
    return true;
}

bool validStorageJson(const String& path, uint32_t radioNode, bool identityCache) {
    File f = SD.open(path, FILE_READ);
    if (!f) return false;
    size_t capacity = f.size() * 2 + 512;
    if (capacity < 2048) capacity = 2048;
    size_t maximum = identityCache ? 49152 : kConversationJsonMax;
    if (capacity > maximum) capacity = maximum;
    DynamicJsonDocument doc(capacity);
    DeserializationError error = deserializeJson(doc, f);
    f.close();
    if (error || doc.overflowed() || (doc["radio"] | uint32_t{0}) != radioNode) return false;
    return identityCache ? !doc["nodes"].as<JsonArray>().isNull()
                         : !doc["messages"].as<JsonArray>().isNull();
}

bool preserveInvalidStorageFile(const String& path) {
    if (!SD.exists(path)) return true;
    String preserved = path + ".corrupt";
    for (unsigned suffix = 1; SD.exists(preserved) && suffix < 100; ++suffix) {
        preserved = path + ".corrupt" + String(suffix);
    }
    return !SD.exists(preserved) && SD.rename(path, preserved);
}

bool recoverStorageFile(const String& path, uint32_t radioNode, bool identityCache) {
    if (validStorageJson(path, radioNode, identityCache)) return true;
    String temporary = path + ".tmp";
    String backup = path + ".bak";
    String candidate;
    // A fully validated temporary is the newest complete transaction when the
    // final path is missing/corrupt; otherwise retain the last valid backup.
    if (validStorageJson(temporary, radioNode, identityCache)) candidate = temporary;
    else if (validStorageJson(backup, radioNode, identityCache)) candidate = backup;
    if (candidate.length() == 0) {
        if (!SD.exists(path)) return true;
        if (!preserveInvalidStorageFile(path)) return false;
        gStorageWarningSticky = true;
        gStorageError = "SD corrupt file preserved";
        Serial.printf("[Storage] preserved invalid file: %s\n", path.c_str());
        return true;
    }
    if (SD.exists(path)) {
        if (!preserveInvalidStorageFile(path)) return false;
        gStorageWarningSticky = true;
        gStorageError = "SD corrupt file preserved";
    }
    if (!SD.rename(candidate, path)) return false;
    Serial.printf("[Storage] recovered %s from %s\n", path.c_str(), candidate.c_str());
    return validStorageJson(path, radioNode, identityCache);
}

bool writeJsonTransaction(const String& path,
                          DynamicJsonDocument& doc,
                          uint32_t radioNode,
                          bool identityCache) {
    if (!gStorageRetry.due(millis())) return false;
    if (doc.overflowed()) {
        gStorageError = "SD data too large";
        gStorageRetry.failed(millis());
        Serial.println("[Storage] JSON document overflow; write skipped");
        return false;
    }
    if (!recoverStorageFile(path, radioNode, identityCache)) {
        storageFailure("recovery");
        return false;
    }

    String temporary = path + ".tmp";
    String backup = path + ".bak";
    if (SD.exists(temporary) && !SD.remove(temporary)) {
        storageFailure("temporary cleanup");
        return false;
    }
    File f = SD.open(temporary, FILE_WRITE);
    if (!f) {
        storageFailure("temporary open");
        return false;
    }
    size_t expected = measureJson(doc);
    size_t written = serializeJson(doc, f);
    f.flush();
    f.close();
    File check = SD.open(temporary, FILE_READ);
    bool complete = check && written == expected && check.size() == written;
    if (check) check.close();
    if (!complete || !validStorageJson(temporary, radioNode, identityCache)) {
        SD.remove(temporary);
        storageFailure("temporary verification");
        return false;
    }

    if (SD.exists(path)) {
        if (validStorageJson(path, radioNode, identityCache)) {
            // The final file is valid, so replacing an older backup still
            // leaves a complete copy on the card.
            if (SD.exists(backup) && !SD.remove(backup)) {
                storageFailure("backup cleanup");
                return false;
            }
            if (!SD.rename(path, backup)) {
                storageFailure("backup rename");
                return false;
            }
        } else if (!preserveInvalidStorageFile(path)) {
            storageFailure("corrupt file preservation");
            return false;
        }
    }
    if (!SD.rename(temporary, path)) {
        if (!SD.exists(path) && SD.exists(backup)) SD.rename(backup, path);
        storageFailure("commit rename");
        return false;
    }
    if (!validStorageJson(path, radioNode, identityCache)) {
        preserveInvalidStorageFile(path);
        if (!SD.exists(path) && SD.exists(backup)) SD.rename(backup, path);
        storageFailure("final verification");
        return false;
    }
    storageSuccess();
    return true;
}

bool loadNodeIdentityCache(uint32_t radioNode) {
    if (radioNode == 0) return false;
    if (gNodeIdentityCacheRadio != radioNode) {
        gCachedNodeIdentities.clear();
        gNodeIdentityCacheDirty = false;
        gNodeIdentityCacheLoaded = false;
        gNodeIdentityCacheRadio = radioNode;
    }
    if (gNodeIdentityCacheLoaded) return false;
    if (!ensureStorage()) return false;

    String path = nodeIdentityCachePath(radioNode);
    if (!recoverStorageFile(path, radioNode, true)) {
        storageFailure("identity cache recovery");
        return false;
    }
    if (SD.exists(path)) {
        File f = SD.open(path, FILE_READ);
        if (!f) {
            storageFailure("identity cache open");
            return false;
        }
        size_t cap = f.size() * 2 + 512;
        if (cap < 2048) cap = 2048;
        if (cap > 49152) cap = 49152;
        DynamicJsonDocument doc(cap);
        DeserializationError err = deserializeJson(doc, f);
        f.close();
        if (err || doc.overflowed() || (doc["radio"] | uint32_t{0}) != radioNode ||
            doc["nodes"].isNull()) {
            storageFailure("identity cache read");
            return false;
        }
        {
            JsonArray nodes = doc["nodes"].as<JsonArray>();
            for (JsonObject obj : nodes) {
                CachedNodeIdentity identity;
                identity.nodeNum = obj["num"] | uint32_t{0};
                identity.shortName = obj["short"] | "";
                identity.longName = obj["long"] | "";
                if (identity.nodeNum != 0 &&
                    (identity.shortName.length() > 0 || identity.longName.length() > 0)) {
                    auto existing = gCachedNodeIdentities.begin();
                    while (existing != gCachedNodeIdentities.end() &&
                           existing->nodeNum != identity.nodeNum) ++existing;
                    if (existing == gCachedNodeIdentities.end()) {
                        gCachedNodeIdentities.push_back(identity);
                    } else {
                        if (existing->shortName.length() == 0) existing->shortName = identity.shortName;
                        if (existing->longName.length() == 0) existing->longName = identity.longName;
                    }
                }
            }
        }
    }

    gNodeIdentityCacheLoaded = true;
    storageSuccess();
    return true;
}

bool persistNodeIdentityCache() {
    uint32_t radioNode = gMesh.myNodeNum();
    if (radioNode == 0 || radioNode != gNodeIdentityCacheRadio || !ensureStorage()) return false;

    size_t capacity = 2048 + gCachedNodeIdentities.size() * 128;
    if (capacity > 49152) capacity = 49152;
    DynamicJsonDocument doc(capacity);
    doc["version"] = 1;
    doc["radio"] = radioNode;
    JsonArray nodes = doc.createNestedArray("nodes");
    for (const auto& identity : gCachedNodeIdentities) {
        JsonObject obj = nodes.createNestedObject();
        obj["num"] = identity.nodeNum;
        obj["short"] = identity.shortName;
        obj["long"] = identity.longName;
    }
    String path = nodeIdentityCachePath(radioNode);
    return writeJsonTransaction(path, doc, radioNode, true);
}

bool refreshNodeIdentityCache() {
    uint32_t radioNode = gMesh.myNodeNum();
    if (radioNode == 0 || radioNode != gNodeIdentityCacheRadio) return false;

    bool changed = false;
    for (int i = 0; i < gMesh.getNodeCount(); ++i) {
        MeshNodeInfo node = gMesh.getNodeInfo(i);
        if (node.num == 0 || (node.shortName.length() == 0 && node.longName.length() == 0)) continue;

        CachedNodeIdentity* cached = nullptr;
        for (auto& identity : gCachedNodeIdentities) {
            if (identity.nodeNum == node.num) {
                cached = &identity;
                break;
            }
        }
        if (!cached) {
            CachedNodeIdentity identity;
            identity.nodeNum = node.num;
            identity.shortName = node.shortName;
            identity.longName = node.longName;
            gCachedNodeIdentities.push_back(identity);
            changed = true;
            continue;
        }

        if (node.shortName.length() > 0 && cached->shortName != node.shortName) {
            cached->shortName = node.shortName;
            changed = true;
        }
        if (node.longName.length() > 0 && cached->longName != node.longName) {
            cached->longName = node.longName;
            changed = true;
        }
    }

    if (changed) gNodeIdentityCacheDirty = true;
    if (gNodeIdentityCacheDirty && persistNodeIdentityCache()) {
        gNodeIdentityCacheDirty = false;
    }
    return changed;
}

String knownNodeName(uint32_t nodeNum) {
    if (nodeNum == 0) return String();
    for (int i = 0; i < gMesh.getNodeCount(); ++i) {
        MeshNodeInfo node = gMesh.getNodeInfo(i);
        if (node.num != nodeNum) continue;
        if (node.shortName.length() > 0) return node.shortName;
        if (node.longName.length() > 0) return node.longName;
        break;
    }
    for (const auto& identity : gCachedNodeIdentities) {
        if (identity.nodeNum != nodeNum) continue;
        if (identity.shortName.length() > 0) return identity.shortName;
        return identity.longName;
    }
    return String();
}

String resolvedNodeLabel(uint32_t nodeNum) {
    if (nodeNum == 0) return String("Unknown");
    String name = knownNodeName(nodeNum);
    if (name.length() > 0) return name;
    return String("0x") + nodeHexId(nodeNum);
}

uint32_t parsePeerFromFilename(const String& name) {
    int slash = name.lastIndexOf('/');
    String base = (slash >= 0) ? name.substring(slash + 1) : name;
    if (!base.startsWith("dm_")) return 0;
    int dot = base.lastIndexOf('.');
    String hex = (dot > 3) ? base.substring(3, dot) : base.substring(3);
    if (hex.length() == 0) return 0;
    char* end = nullptr;
    uint32_t value = strtoul(hex.c_str(), &end, 16);
    if (!end || *end != '\0') return 0;
    return value;
}

String conversationFilePath(const Conversation& conv) {
    uint32_t radio = gConversationRadioNode;
    String directory = radioStoragePath(radio);
    if (conv.isBroadcast) {
        return directory + "/ch_" + String(conv.channel) + ".json";
    }
    return directory + "/dm_" + nodeHexId(conv.peer) + ".json";
}

String conversationFilePath(uint32_t radio, bool isBroadcast, uint8_t channel, uint32_t peer) {
    String directory = radioStoragePath(radio);
    if (isBroadcast) return directory + "/ch_" + String(channel) + ".json";
    return directory + "/dm_" + nodeHexId(peer) + ".json";
}

void migrateLegacyConversations(uint32_t radioNode) {
    if (radioNode == 0 || !ensureRadioStorage(radioNode)) return;

    File dir = SD.open(kStorageDir);
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return;
    }
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        if (f.isDirectory()) {
            f.close();
            continue;
        }
        String sourcePath = String(f.name());
        if (sourcePath.lastIndexOf('/') < 0) sourcePath = String(kStorageDir) + "/" + sourcePath;
        int slash = sourcePath.lastIndexOf('/');
        String baseName = slash >= 0 ? sourcePath.substring(slash + 1) : sourcePath;
        if (!baseName.endsWith(".json") || baseName.startsWith("nodes_")) {
            f.close();
            continue;
        }

        size_t capacity = f.size() * 2 + 512;
        if (capacity < 2048) capacity = 2048;
        if (capacity > kConversationJsonMax) capacity = kConversationJsonMax;
        DynamicJsonDocument doc(capacity);
        DeserializationError error = deserializeJson(doc, f);
        f.close();
        // Legacy records without an exact radio ID are ambiguous. Leave them
        // untouched for an explicit/manual recovery instead of guessing.
        if (error || (doc["radio"] | uint32_t{0}) != radioNode) continue;

        uint32_t peer = doc["peer"] | uint32_t{0};
        bool isBroadcast = doc["broadcast"] | false;
        uint8_t channel = doc["channel"] | 0;
        if (isBroadcast && peer == 0) peer = kBroadcastAddr;
        if (!isBroadcast && peer == 0) peer = parsePeerFromFilename(baseName);
        if (peer == 0) continue;

        String destination = conversationFilePath(radioNode, isBroadcast, channel, peer);
        if (SD.exists(destination)) continue; // The namespaced copy is authoritative.
        String temporary = destination + ".migrate.tmp";
        SD.remove(temporary);

        File source = SD.open(sourcePath, FILE_READ);
        File target = SD.open(temporary, FILE_WRITE);
        if (!source || !target) {
            if (source) source.close();
            if (target) target.close();
            SD.remove(temporary);
            continue;
        }
        size_t sourceSize = source.size();
        uint8_t buffer[256];
        bool copied = true;
        while (source.available()) {
            int count = source.read(buffer, sizeof(buffer));
            if (count <= 0 || target.write(buffer, static_cast<size_t>(count)) != static_cast<size_t>(count)) {
                copied = false;
                break;
            }
        }
        source.close();
        target.close();
        if (!copied) {
            SD.remove(temporary);
            continue;
        }

        File verify = SD.open(temporary, FILE_READ);
        bool verified = false;
        if (verify) {
            DynamicJsonDocument verifyDoc(capacity);
            DeserializationError verifyError = deserializeJson(verifyDoc, verify);
            verified = verify.size() == sourceSize && !verifyError &&
                       (verifyDoc["radio"] | uint32_t{0}) == radioNode &&
                       (verifyDoc["peer"] | uint32_t{0}) == (doc["peer"] | uint32_t{0}) &&
                       (verifyDoc["broadcast"] | false) == isBroadcast &&
                       (verifyDoc["channel"] | 0) == channel;
            verify.close();
        }
        if (verified && SD.rename(temporary, destination)) {
            // Remove the legacy source only after the namespaced file is complete
            // and can be parsed with the expected identity.
            SD.remove(sourcePath);
        } else {
            SD.remove(temporary);
        }
    }
    dir.close();
}

bool recoverRadioConversations(uint32_t radioNode) {
    String directoryPath = radioStoragePath(radioNode);
    File dir = SD.open(directoryPath);
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return false;
    }
    std::vector<String> paths;
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        if (f.isDirectory()) {
            f.close();
            continue;
        }
        String candidate = String(f.name());
        if (candidate.lastIndexOf('/') < 0) candidate = directoryPath + "/" + candidate;
        f.close();
        String path = candidate;
        if (candidate.endsWith(".json.tmp")) path = candidate.substring(0, candidate.length() - 4);
        else if (candidate.endsWith(".json.bak")) path = candidate.substring(0, candidate.length() - 4);
        else if (!candidate.endsWith(".json")) continue;
        bool alreadyListed = false;
        for (const String& existing : paths) {
            if (existing == path) {
                alreadyListed = true;
                break;
            }
        }
        if (!alreadyListed) paths.push_back(path);
    }
    dir.close();
    for (const String& path : paths) {
        if (!recoverStorageFile(path, radioNode, false)) return false;
    }
    return true;
}

bool persistConversation(int convIndex) {
    uint32_t radio = gMesh.myNodeNum();
    if (radio == 0 || radio != gConversationRadioNode) return false;
    Conversation* conv = getConversation(convIndex);
    if (!conv) return false;
    conv->storageDirty = true;
    if (!ensureRadioStorage(radio)) return false;

    DynamicJsonDocument doc(kConversationJsonMax);
    doc["version"] = 2;
    doc["radio"] = radio;
    doc["peer"] = conv->peer;
    doc["broadcast"] = conv->isBroadcast;
    doc["channel"] = conv->channel;
    doc["unread"] = conv->hasUnread;
    doc["lastActivity"] = conv->lastActivitySeq;

    JsonArray arr = doc.createNestedArray("messages");
    size_t total = conv->messages.size();
    size_t start = (total > kConversationMaxMessages) ? (total - kConversationMaxMessages) : 0;
    for (size_t i = start; i < total; ++i) {
        const auto& msg = conv->messages[i];
        JsonObject obj = arr.createNestedObject();
        obj["from"] = msg.from;
        obj["fromMe"] = msg.fromMe;
        obj["packetId"] = msg.packetId;
        obj["channel"] = msg.channel;
        obj["rxTime"] = msg.rxTime;
        obj["text"] = msg.text;
        obj["delivery"] = static_cast<uint8_t>(msg.delivery);
    }

    String path = conversationFilePath(*conv);
    if (!writeJsonTransaction(path, doc, radio, false)) return false;
    conv->storageDirty = false;
    return true;
}

bool loadConversationsFromStorage() {
    if (!ensureStorage()) return false;
    uint32_t currentRadio = gMesh.myNodeNum();
    if (currentRadio == 0 || currentRadio != gConversationRadioNode || !ensureRadioStorage(currentRadio)) return false;
    migrateLegacyConversations(currentRadio);
    if (!recoverRadioConversations(currentRadio)) {
        storageFailure("conversation recovery");
        return false;
    }
    uint32_t maxActivity = 0;

    String radioDir = radioStoragePath(currentRadio);
    File dir = SD.open(radioDir);
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        storageFailure("conversation directory read");
        return false;
    }

    bool hadReadError = false;
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        if (f.isDirectory()) {
            f.close();
            continue;
        }
        String name = String(f.name());
        int slash = name.lastIndexOf('/');
        String baseName = (slash >= 0) ? name.substring(slash + 1) : name;
        if (!baseName.endsWith(".json") || baseName.startsWith("nodes_")) {
            f.close();
            continue;
        }

        size_t cap = f.size() * 2 + 512;
        if (cap < 2048) cap = 2048;
        if (cap > kConversationJsonMax) cap = kConversationJsonMax;
        DynamicJsonDocument doc(cap);

        DeserializationError err = deserializeJson(doc, f);
        f.close();
        if (err || doc.overflowed()) {
            hadReadError = true;
            Serial.printf("[Storage] invalid conversation file: %s\n", baseName.c_str());
            continue;
        }

        uint32_t peer = doc["peer"] | uint32_t{0};
        bool isBroadcast = doc["broadcast"] | false;
        uint8_t channel = doc["channel"] | 0;
        bool unread = doc["unread"] | false;
        uint32_t radio = doc["radio"] | uint32_t{0};
        bool hasLastActivity = doc.containsKey("lastActivity");
        uint32_t lastActivity = doc["lastActivity"] | uint32_t{0};
        if (isBroadcast && peer == 0) peer = kBroadcastAddr;
        if (!isBroadcast && peer == 0) {
            peer = parsePeerFromFilename(name);
        }
        if (peer == 0) {
            continue;
        }
        if (radio != currentRadio) {
            continue;
        }
        JsonArray msgs = doc["messages"].as<JsonArray>();
        if (msgs.isNull()) {
            hadReadError = true;
            Serial.printf("[Storage] missing message array: %s\n", baseName.c_str());
            continue;
        }

        int convIndex = ensureConversation(peer, isBroadcast, channel);
        Conversation* conv = getConversation(convIndex);
        if (!conv) continue;

        std::vector<Message> inMemory = conv->messages;
        bool memoryUnread = conv->hasUnread;
        uint32_t memoryActivity = conv->lastActivitySeq;
        bool migratedPendingDelivery = false;

        conv->peer = peer;
        conv->isBroadcast = isBroadcast;
        conv->channel = channel;
        conv->messages.clear();
        conv->hasUnread = unread || memoryUnread;
        conv->firstVisibleIndex = -1;

        std::vector<Message> fromStorage;
        for (JsonObject obj : msgs) {
            Message m;
            m.from = obj["from"] | uint32_t{0};
            m.fromMe = obj["fromMe"] | false;
            m.packetId = obj["packetId"] | uint32_t{0};
            m.channel = obj["channel"] | channel;
            m.rxTime = obj["rxTime"] | uint32_t{0};
            const char* txt = obj["text"] | "";
            m.text = String(txt);
            uint8_t delivery = obj["delivery"] | static_cast<uint8_t>(MessageDeliveryState::UNKNOWN);
            if (delivery <= static_cast<uint8_t>(MessageDeliveryState::UNCONFIRMED)) {
                m.delivery = static_cast<MessageDeliveryState>(delivery);
            }
            // Pending delivery tracking belongs to the previous process/session.
            if (m.delivery == MessageDeliveryState::PENDING) {
                m.delivery = MessageDeliveryState::UNCONFIRMED;
                migratedPendingDelivery = true;
            }
            fromStorage.push_back(m);
        }
        conv->messages.swap(fromStorage);
        mergeStorageHistories(conv->messages, inMemory, kConversationMaxMessages,
            [](const Message& a, const Message& b) {
                if (a.packetId != 0 || b.packetId != 0) {
                    if (a.packetId == 0 || b.packetId == 0 ||
                        a.packetId != b.packetId || a.from != b.from ||
                        a.channel != b.channel) {
                        return false;
                    }
                    if (a.rxTime != 0 && b.rxTime != 0) {
                        uint32_t delta = a.rxTime > b.rxTime
                                             ? a.rxTime - b.rxTime
                                             : b.rxTime - a.rxTime;
                        return delta <= kPacketDedupWindowSeconds;
                    }
                    return true;
                }
                return a.from == b.from && a.fromMe == b.fromMe && a.text == b.text;
            });
        conv->storageDirty = !inMemory.empty() || migratedPendingDelivery;
        if (!hasLastActivity && !conv->messages.empty()) {
            lastActivity = 1;
        }
        conv->lastActivitySeq = lastActivity > memoryActivity ? lastActivity : memoryActivity;
        if (lastActivity > maxActivity) {
            maxActivity = conv->lastActivitySeq;
        }
    }
    dir.close();
    // Keep valid conversations available even if one file was corrupt, but
    // do not report the storage load complete if any read failed. Retry after
    // backoff; recovery will preserve invalid files or restore their backup.
    if (hadReadError) {
        gStorageError = "SD history read error";
        Serial.println("[Storage] one or more conversations could not be read");
        storageFailure("conversation read");
        return false;
    } else {
        storageSuccess();
    }
    if (maxActivity >= gNextActivitySeq) {
        gNextActivitySeq = maxActivity + 1;
    }
    return true;
}

bool tryLoadConversationsOnce() {
    uint32_t currentRadio = gMesh.myNodeNum();
    if (currentRadio == 0) return false;
    bool changed = false;
    if (gConversationRadioNode != currentRadio) {
        // RAM state belongs to one radio even when there is no SD card.
        initConversations();
        gCachedNodeIdentities.clear();
        gNodeIdentityCacheRadio = 0;
        gNodeIdentityCacheLoaded = false;
        gNodeIdentityCacheDirty = false;
        gStorageLoaded = false;
        gLoadedRadioNode = 0;
        gConversationRadioNode = currentRadio;
        handleRadioIdentityChanged();
        changed = true;
    }
    bool identitiesLoaded = loadNodeIdentityCache(currentRadio);
    if (gStorageLoaded && gLoadedRadioNode == currentRadio) return changed || identitiesLoaded;
    if (!ensureStorage()) return changed || identitiesLoaded;

    if (!loadConversationsFromStorage()) return changed || identitiesLoaded;
    gLoadedRadioNode = currentRadio;
    gStorageLoaded = true;
    return true;
}

void flushDirtyConversations() {
    if (gMesh.myNodeNum() == 0 || gMesh.myNodeNum() != gConversationRadioNode ||
        !gStorageRetry.due(millis())) return;
    for (size_t i = 0; i < gConversations.size(); ++i) {
        if (!gConversations[i].storageDirty) continue;
        if (!persistConversation(static_cast<int>(i))) return;
    }
}

void applyRadioStateTransition(uint32_t radioNode) {
    RadioStateTransition transition = gRadioIdentity.observe(radioNode);
    if (transition == RadioStateTransition::DISCONNECTED) {
        handleRadioDisconnected();
        return;
    }
    if (transition != RadioStateTransition::IDENTITY_CHANGED) return;

    // This reset is based on the radio's node ID, never on whether SD mounted.
    initConversations();
    gCachedNodeIdentities.clear();
    gNodeIdentityCacheRadio = 0;
    gNodeIdentityCacheLoaded = false;
    gNodeIdentityCacheDirty = false;
    gStorageLoaded = false;
    gLoadedRadioNode = 0;
    gConversationRadioNode = radioNode;
    handleRadioIdentityChanged();
}

int findConversationIndex(uint32_t peer, bool isBroadcast, uint8_t channel) {
    for (int i = 0; i < (int)gConversations.size(); ++i) {
        const auto& c = gConversations[i];
        if (c.peer == peer && c.isBroadcast == isBroadcast) {
            if (!isBroadcast || c.channel == channel) {
                return i;
            }
        }
    }
    return -1;
}

int ensureConversation(uint32_t peer, bool isBroadcast, uint8_t channel) {
    int idx = findConversationIndex(peer, isBroadcast, channel);
    if (idx >= 0) return idx;
    Conversation c;
    c.peer = peer;
    c.isBroadcast = isBroadcast;
    c.channel = channel;
    c.firstVisibleIndex = -1;
    c.hasUnread = false;
    c.lastActivitySeq = 0;
    gConversations.push_back(c);
    return (int)gConversations.size() - 1;
}

Conversation* getConversation(int index) {
    if (index < 0 || index >= (int)gConversations.size()) return nullptr;
    return &gConversations[index];
}

String conversationTitle(const Conversation& conv) {
    if (conv.isBroadcast) return mapSpanishGlyphsForDisplay(gMesh.channelLabel(conv.channel), true);
    return mapSpanishGlyphsForDisplay(resolvedNodeLabel(conv.peer), true);
}

String conversationListTitle(const Conversation& conv) {
    if (conv.isBroadcast) {
        return mapSpanishGlyphsForDisplay(String("[ch") + String(conv.channel) + "] " +
                                          gMesh.channelLabel(conv.channel));
    }
    return mapSpanishGlyphsForDisplay(resolvedNodeLabel(conv.peer));
}

String lastMessagePreview(const Conversation& conv) {
    if (conv.messages.empty()) return String("");
    const Message& msg = conv.messages.back();
    String text = prepareMessageForDisplay(msg.text);
    text.replace('\n', ' ');
    if (msg.fromMe) {
        return String("Me: ") + text;
    }
    return text;
}

String truncateLabel(const String& text, size_t maxChars) {
    if (maxChars == 0) return String("");
    size_t end = 0;
    size_t chars = 0;
    while (end < text.length() && chars < maxChars) {
        end = nextUtf8Boundary(text, end);
        ++chars;
    }
    if (end == text.length()) return text;
    if (maxChars <= 3) return text.substring(0, end);

    size_t prefixEnd = 0;
    for (size_t i = 0; i < maxChars - 3 && prefixEnd < text.length(); ++i) {
        prefixEnd = nextUtf8Boundary(text, prefixEnd);
    }
    return text.substring(0, prefixEnd) + "...";
}

String nodeHex(uint32_t num) {
    String hex = String(num, HEX);
    hex.toUpperCase();
    return String("0x") + hex;
}

String nodeDisplayName(const MeshNodeInfo& node) {
    if (node.shortName.length() > 0) return mapSpanishGlyphsForDisplay(node.shortName);
    if (node.longName.length() > 0) return mapSpanishGlyphsForDisplay(node.longName);
    return nodeHex(node.num);
}

String channelDisplayName(const MeshChannelInfo& ch) {
    if (ch.name.length() > 0) return mapSpanishGlyphsForDisplay(ch.name);
    if (ch.index == 0) return "LongFast";
    return String("Ch ") + ch.index;
}

const char* channelRoleLabel(uint8_t role) {
    switch (role) {
    case meshtastic_Channel_Role_PRIMARY:
        return "P";
    case meshtastic_Channel_Role_SECONDARY:
        return "S";
    case meshtastic_Channel_Role_DISABLED:
        return "D";
    default:
        break;
    }
    return "?";
}

bool nodeActivityAgeSecondsAt(const MeshNodeInfo& node,
                              uint32_t& ageSec,
                              uint32_t nowMs,
                              bool hasRadioTime,
                              uint32_t radioNowEpoch) {
    if (node.activitySource == MeshActivitySource::RADIO_TIME) {
        if (node.lastHeard == 0 || !hasRadioTime || node.lastHeard > radioNowEpoch) {
            return false;
        }
        ageSec = radioNowEpoch - node.lastHeard;
        return true;
    }
    if (node.activitySource == MeshActivitySource::LOCAL_OBSERVATION) {
        ageSec = (nowMs - node.lastActivityObservedMs) / 1000;
        return true;
    }
    return false;
}

bool nodeActivityAgeSeconds(const MeshNodeInfo& node, uint32_t& ageSec) {
    uint32_t radioNowEpoch = 0;
    bool hasRadioTime = gMesh.radioTimeNow(radioNowEpoch);
    return nodeActivityAgeSecondsAt(node, ageSec, millis(), hasRadioTime, radioNowEpoch);
}

String formatAge(const MeshNodeInfo& node) {
    uint32_t ageSec = 0;
    if (!nodeActivityAgeSeconds(node, ageSec)) return "?";
    if (ageSec < 60) {
        return String(ageSec) + "s";
    }
    uint32_t ageMin = ageSec / 60;
    if (ageMin < 60) {
        return String(ageMin) + "m";
    }
    uint32_t ageHr = ageMin / 60;
    return String(ageHr) + "h";
}

const char* nodeActivityLabel(const MeshNodeInfo& node) {
    uint32_t ageSec = 0;
    if (!nodeActivityAgeSeconds(node, ageSec)) return "[UNKNOWN]";
    return ageSec <= kNodeActiveMs / 1000 ? "[ACTIVE]" : "[STALE]";
}

String nodeNameForRow(const String& name, const String& suffix, M5GFX& display, int maxWidth) {
    String full = name + suffix;
    if (display.textWidth(full) <= maxWidth) return full;

    const String ellipsis = "...";
    size_t end = 0;
    String prefix;
    while (end < name.length()) {
        size_t next = nextUtf8Boundary(name, end);
        String candidate = name.substring(0, next);
        if (display.textWidth(candidate + ellipsis + suffix) > maxWidth) break;
        end = next;
        prefix = candidate;
    }
    if (prefix.length() > 0) return prefix + ellipsis + suffix;
    return suffix;
}

String batteryStatusLabel() {
    int32_t level = M5.Power.getBatteryLevel();
    if (level < 0 || level > 100) {
        return "BAT --";
    }
    String label = "BAT " + String(level) + "%";
    auto charging = M5.Power.isCharging();
    if (charging == m5::Power_Class::is_charging_t::is_charging) {
        label += "+";
    }
    return label;
}

void drawHeaderRightStatus(const String& line1, const String& line2) {
    auto& d = M5Cardputer.Display;
    d.setTextSize(1);
    d.setTextColor(kColorHeaderText, kColorHeaderBg);

    int clearWidth = d.textWidth("Connecting");
    int scanWidth  = d.textWidth("Scan 99s");
    if (scanWidth > clearWidth) clearWidth = scanWidth;
    int x = d.width() - clearWidth - 4;

    d.fillRect(x, 4, clearWidth + 4, 10, kColorHeaderBg);
    d.fillRect(x, 14, clearWidth + 4, 10, kColorHeaderBg);

    if (line1.length() > 0) {
        d.setCursor(d.width() - d.textWidth(line1) - 4, 4);
        d.print(line1);
    }
    if (line2.length() > 0) {
        d.setCursor(d.width() - d.textWidth(line2) - 4, 14);
        d.print(line2);
    }
}

bool getSelectedNodeInfo(MeshNodeInfo& out) {
    if (gSelectedNodeNum == 0) return false;
    for (int i = 0; i < gMesh.getNodeCount(); ++i) {
        MeshNodeInfo node = gMesh.getNodeInfo(i);
        if (node.num == gSelectedNodeNum) {
            out = node;
            return true;
        }
    }
    return false;
}

const char* connectionStatusLabel() {
    return gMesh.sessionStatusLabel();
}

const char* connectionOrSyncStatusLabel() {
    return connectionStatusLabel();
}

uint16_t connectionStatusColor() {
    switch (gMesh.status()) {
    case ConnectionStatus::CONNECTED:
        if (gMesh.syncFailed()) return RED;
        if (!gMesh.readyToSend() || gMesh.lastError().length() > 0) {
            return YELLOW;
        }
        return GREEN;
    case ConnectionStatus::SCANNING:
    case ConnectionStatus::CONNECTING:
        return YELLOW;
    case ConnectionStatus::ERROR:
        return RED;
    case ConnectionStatus::DISCONNECTED:
    default:
        return DARKGREY;
    }
}

bool hasUnreadConversations() {
    if (gMesh.myNodeNum() == 0) return false;
    for (const auto& conv : gConversations) {
        if (conv.hasUnread) return true;
    }
    return false;
}

void initConversations() {
    gConversations.clear();
    ensureConversation(kBroadcastAddr, true, 0);
    gActiveConversation = -1;
    gNextActivitySeq = 1;
}

void invalidateRadioTransientState() {
    gActiveConversation = -1;
    gSelectedNodeNum = 0;
    gPendingPinDeviceIndex = -1;
}

void appendConversationMessage(int convIndex,
                               bool fromMe,
                               const String& text,
                               uint32_t from,
                               bool markUnread,
                               MessageDeliveryState delivery = MessageDeliveryState::UNKNOWN,
                               uint32_t deliveryToken = 0,
                               uint32_t packetId = 0,
                               uint8_t channel = 0,
                               uint32_t rxTime = 0) {
    if (gMesh.myNodeNum() == 0 || gMesh.myNodeNum() != gConversationRadioNode) return;
    Conversation* conv = getConversation(convIndex);
    if (!conv) return;
    bool autoBottom = (conv->firstVisibleIndex < 0);
    Message msg;
    msg.fromMe = fromMe;
    msg.from = from;
    msg.packetId = packetId;
    msg.rxTime = rxTime;
    msg.channel = channel;
    msg.text = text;
    msg.delivery = delivery;
    msg.deliveryToken = deliveryToken;
    conv->messages.push_back(msg);
    if (conv->messages.size() > kConversationMaxMessages) {
        size_t remove = conv->messages.size() - kConversationMaxMessages;
        conv->messages.erase(conv->messages.begin(), conv->messages.begin() + remove);
    }
    if (autoBottom) {
        conv->firstVisibleIndex = -1;
    }
    if (markUnread) {
        conv->hasUnread = true;
    }
    conv->lastActivitySeq = gNextActivitySeq++;
    persistConversation(convIndex);
}

bool reconcileOrDeduplicatePacket(const MeshTextMessage& incoming) {
    if (incoming.packetId == 0 || incoming.from == 0) return false;

    for (size_t convIndex = 0; convIndex < gConversations.size(); ++convIndex) {
        auto& conv = gConversations[convIndex];
        for (auto& message : conv.messages) {
            if (message.packetId != incoming.packetId ||
                message.from != incoming.from ||
                message.channel != incoming.channel) {
                continue;
            }

            if (message.rxTime != 0 && incoming.rxTime != 0) {
                uint32_t delta = message.rxTime > incoming.rxTime
                                     ? message.rxTime - incoming.rxTime
                                     : incoming.rxTime - message.rxTime;
                if (delta > kPacketDedupWindowSeconds) continue;
            }

            // A local radio echo enriches the sent row instead of creating a
            // second copy. The same path upgrades a duplicate's unknown time.
            if (message.rxTime == 0 && incoming.rxTime != 0) {
                message.rxTime = incoming.rxTime;
                persistConversation(static_cast<int>(convIndex));
            }
            return true;
        }
    }
    return false;
}

void applyDeliveryUpdate(const MeshDeliveryUpdate& update) {
    MessageDeliveryState state = MessageDeliveryState::UNKNOWN;
    switch (update.stage) {
    case MeshDeliveryStage::MESH_CONFIRMED:
        state = MessageDeliveryState::MESH_CONFIRMED;
        break;
    case MeshDeliveryStage::DESTINATION_ACKED:
        state = MessageDeliveryState::DELIVERED;
        break;
    case MeshDeliveryStage::QUEUE_REJECTED:
        state = MessageDeliveryState::QUEUE_REJECTED;
        break;
    case MeshDeliveryStage::ROUTING_FAILED:
        state = MessageDeliveryState::FAILED;
        break;
    case MeshDeliveryStage::TIMED_OUT:
        state = MessageDeliveryState::TIMED_OUT;
        break;
    case MeshDeliveryStage::DISCONNECTED:
        state = MessageDeliveryState::DISCONNECTED;
        break;
    case MeshDeliveryStage::UNCONFIRMED:
        state = MessageDeliveryState::UNCONFIRMED;
        break;
    }

    for (size_t convIndex = 0; convIndex < gConversations.size(); ++convIndex) {
        auto& conv = gConversations[convIndex];
        for (auto& msg : conv.messages) {
            if (msg.deliveryToken == update.token) {
                msg.delivery = state;
                persistConversation(static_cast<int>(convIndex));
                return;
            }
        }
    }
}

// -------------------- Generic list menu helper --------------------

struct MenuOption {
    String label;
    bool   enabled;
};

struct ListMenuState {
    int selected = 0;
    int top = 0;

    void reset() {
        selected = 0;
        top = 0;
    }

    int current() const { return selected; }
    int topIndex() const { return top; }

    void handleNav(NavKey k, int itemCount) {
        if (itemCount <= 0) return;
        int step = (k == NavKey::UP_FAST || k == NavKey::DOWN_FAST) ? 4 : 1;
        switch (k) {
        case NavKey::UP:
        case NavKey::UP_FAST:
            if (selected > 0) {
                selected -= step;
                if (selected < 0) selected = 0;
            }
            break;
        case NavKey::DOWN:
        case NavKey::DOWN_FAST:
            if (selected + 1 < itemCount) {
                selected += step;
                if (selected + 1 > itemCount) selected = itemCount - 1;
            }
            break;
        default:
            break;
        }
    }

    void ensureVisible(int itemCount, int visibleRows) {
        if (itemCount <= 0) {
            top = 0;
            return;
        }
        if (visibleRows <= 0) return;
        if (selected < 0) selected = 0;
        if (selected >= itemCount) selected = itemCount - 1;

        if (selected < top) {
            top = selected;
        } else if (selected >= top + visibleRows) {
            top = selected - visibleRows + 1;
        }

        if (top < 0) top = 0;
        int maxTop = itemCount - visibleRows;
        if (maxTop < 0) maxTop = 0;
        if (top > maxTop) top = maxTop;
    }

    template <typename DisplayT>
    void drawMenu(DisplayT& d,
                  const std::vector<MenuOption>& items,
                  int startY,
                  int lineH,
                  int maxRows = -1,
                  int textSize = 2) {
        d.setTextSize(textSize);
        int visibleRows = maxRows;
        if (visibleRows <= 0) {
            visibleRows = (d.height() - startY) / lineH;
        }
        if (visibleRows < 1) visibleRows = 1;

        ensureVisible((int)items.size(), visibleRows);

        int end = top + visibleRows;
        if (end > (int)items.size()) end = (int)items.size();
        for (int i = top; i < end; ++i) {
            int y = startY + (i - top) * lineH;
            const auto& opt = items[i];

            uint16_t textColor;
            if (i == selected) {
                d.fillRect(0, y - 2, d.width(), lineH, DARKGREY);
                textColor = opt.enabled ? BLACK : DARKGREY;
                d.setTextColor(textColor, DARKGREY);
            } else {
                textColor = opt.enabled ? WHITE : DARKGREY;
                d.setTextColor(textColor, BLACK);
            }
            d.setCursor(8, y);
            drawDisplayText(d, opt.label, textColor);
            d.println();
        }
    }
};

// -------------------- Screen interface --------------------

class Screen {
public:
    virtual ~Screen() {}
    virtual void onEnter() {}
    virtual void onExit() {}

    // Abstract navigation
    virtual void handleNav(NavKey k) = 0;

    // Optional text input (ignored by default)
    virtual void handleChars(const std::vector<char>& chars) {
        (void)chars;
    }

    virtual void draw() = 0;
};

// Screen IDs

enum class ScreenId {
    HOME,
    CONNECT,
    PIN_ENTRY,
    NODES,
    CHANNELS,
    NODE_ACTIONS,
    NODE_INFO,
    CONVERSATIONS,
    CHAT   // conversation view
};

// Forward declarations
class HomeScreen;
class ConnectScreen;
class PinEntryScreen;
class NodesScreen;
class ChannelsScreen;
class NodeActionsScreen;
class NodeInfoScreen;
class ConversationsScreen;
class ChatScreen;

// -------------------- ScreenManager --------------------

class ScreenManager {
public:
    ScreenManager() : currentId(ScreenId::HOME),
                      currentScreen(nullptr),
                      needRedraw(true) {}

    void begin();
    void switchTo(ScreenId id);
    void handleNav(NavKey k);
    void loop();
    void drawConnectScanUpdates(bool redrawHeader, bool redrawDevices);
    void handleRadioIdentityChanged();
    void handleRadioDisconnected();

private:
    ScreenId currentId;
    Screen*  currentScreen;
    bool     needRedraw;

    // Static screen instances (simple for now)
    HomeScreen*          home;
    ConnectScreen*       connect;
    PinEntryScreen*      pinEntry;
    NodesScreen*         nodes;
    ChannelsScreen*      channels;
    NodeActionsScreen*   nodeActions;
    NodeInfoScreen*      nodeInfo;
    ConversationsScreen* convs;
    ChatScreen*          chat;

    Screen* getScreenById(ScreenId id);
    void    redrawIfNeeded();
};

extern ScreenManager gScreens;

// -------------------- Home screen (list) --------------------

class HomeScreen : public Screen {
public:
    HomeScreen() {
        items.push_back({"Connect", true});
        items.push_back({"Nodes", true});
        items.push_back({"Channels", true});
        items.push_back({"Conversations", true});
    }

    void onEnter() override {
        menu.reset();
    }

    void handleNav(NavKey k) override {
        menu.handleNav(k, (int)items.size());
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(kColorBodyBg);
        const int headerH = 24;
        drawHeaderBackground(headerH);

        // --- Header (small text) ---
        d.setTextSize(1);
        d.setTextColor(kColorHeaderText, kColorHeaderBg);
        String title = "Cardtastic 0.6";
        int titleX = (d.width() - d.textWidth(title)) / 2;
        if (titleX < 0) titleX = 0;
        d.setCursor(titleX, 4);
        d.println(title);
        const int bleY = 14;
        const int bleDotR = 3;
        const int bleDotX = 4 + bleDotR;
        const int bleDotY = bleY + 4;
        d.fillCircle(bleDotX, bleDotY, bleDotR, connectionStatusColor());
        d.setCursor(bleDotX + bleDotR + 2, bleY);
        d.print("BLE: ");
        d.println(connectionOrSyncStatusLabel());

        String batt = batteryStatusLabel();
        int battX = d.width() - d.textWidth(batt) - 4;
        if (battX < 0) battX = 0;
        int titleRight = titleX + d.textWidth(title);
        int battY = (battX <= titleRight + 4) ? 14 : 4;
        d.setCursor(battX, battY);
        d.print(batt);

        // --- Menu (bigger text) ---
        d.setTextColor(kColorBodyText, kColorBodyBg);
        const int startY = 32;
        const int lineH  = 22;
        int visibleRows = (d.height() - kFooterH - startY) / lineH;
        if (visibleRows < 1) visibleRows = 1;
        if (items.size() >= 4) {
            items[3].label = hasUnreadConversations() ? "Conversations *" : "Conversations";
        }
        menu.drawMenu(d, items, startY, lineH, visibleRows);

        // --- Bottom hint bar (small text again) ---
        d.setTextSize(1);
        drawFooterBackground();
        d.setTextColor(kColorFooterText, kColorFooterBg);
        int h = d.height();
        d.setCursor(4, h - kFooterH + 1);
        d.print("Arrows/W,S: move  Enter/Right: select  Backspace/Left: back");
    }

    int getSelected() const { return menu.current(); }

private:
    ListMenuState           menu;
    std::vector<MenuOption> items;
};

// -------------------- Connect screen --------------------

class ConnectScreen : public Screen {
public:
    ConnectScreen() {}

    void onEnter() override {
        menu.reset();
    }

    void onExit() override {
        gMesh.cancelScan();
    }

    void drawScanUpdates(bool redrawHeader, bool redrawDevices) {
        auto& d = M5Cardputer.Display;
        if (redrawHeader) drawStatusHeader(d);
        if (redrawDevices) {
            const int startY = 28;
            d.fillRect(0, startY - 2, d.width(), d.height() - kFooterH - startY + 2,
                       kColorBodyBg);
            drawDeviceRows(d);
        }
    }

    void handleNav(NavKey k) override {
        int itemCount = getItemCount();

        switch (k) {
        case NavKey::UP:
        case NavKey::DOWN:
        case NavKey::UP_FAST:
        case NavKey::DOWN_FAST:
            menu.handleNav(k, itemCount);
            break;
        case NavKey::ENTER:
            handleEnter();
            break;
        case NavKey::TOGGLE_AUTOCONNECT:
            gMesh.toggleAutoConnect();
            break;
        case NavKey::RETRY_SYNC:
            gMesh.retryConfig();
            break;
        default:
            break;
        }
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(kColorBodyBg);
        drawStatusHeader(d);
        drawDeviceRows(d);

        // --- Bottom help bar ---
        d.setTextSize(1);
        drawFooterBackground();
        d.setTextColor(kColorFooterText, kColorFooterBg);
        int h = d.height();
        d.setCursor(4, h - kFooterH + 1);
        String footer = String("A:Auto ") + (gMesh.autoConnectEnabled() ? "ON" : "OFF");
        footer += gMesh.isScanning() ? " Enter:cancel Del:back" : " R:Sync Enter:select";
        d.print(footer);
    }

private:
    ListMenuState menu;

    void drawStatusHeader(M5GFX& d) const {
        drawHeaderBackground(24);
        d.setTextSize(1);
        d.setTextColor(kColorHeaderText, kColorHeaderBg);
        d.setCursor(4, 4);
        d.print("Status: ");
        d.println(gMesh.sessionStatusLabel());

        d.setCursor(4, 14);
        if (gMesh.isScanning()) {
            int seconds = gMesh.scanSecondsLeft();
            if (gMesh.scanAttempt() == 0) {
                d.print("Waiting for BLE to close...");
            } else {
                d.print(String("Scan ") + gMesh.scanAttempt() + "/3");
                if (seconds >= 0) d.print(String("  ") + seconds + "s");
            }
        } else if (gMesh.status() == ConnectionStatus::CONNECTED) {
            if (gMesh.lastError().length() > 0) {
                d.print("Warn: ");
                d.println(gMesh.lastError());
            } else {
                d.print("Radio: ");
                d.println(gMesh.radioName());
            }
        } else if (gMesh.status() == ConnectionStatus::ERROR) {
            d.print("Error: ");
            d.println(gMesh.lastError());
        } else {
            d.print("Devices: ");
            d.println(gMesh.getDeviceCount());
        }
    }

    void drawDeviceRows(M5GFX& d) {
        d.setTextColor(kColorBodyText, kColorBodyBg);
        d.setTextSize(2);
        const int startY = 28;
        const int lineH = 22;
        int visibleRows = (d.height() - kFooterH - startY) / lineH;
        if (visibleRows < 1) visibleRows = 1;
        int itemCount = getItemCount();
        menu.ensureVisible(itemCount, visibleRows);
        int top = menu.topIndex();
        int selected = menu.current();
        for (int i = top; i < itemCount && i < top + visibleRows; ++i) {
            int y = startY + (i - top) * lineH;
            if (i == selected) {
                d.fillRect(0, y - 2, d.width(), lineH, DARKGREY);
                d.setTextColor(BLACK, DARKGREY);
            } else {
                d.setTextColor(WHITE, BLACK);
            }
            d.setCursor(8, y);
            if (i == 0) {
                d.println(gMesh.isScanning() ? "Cancel scan" : "Scan for devices");
            } else {
                int devIndex = i - 1;
                MeshDeviceInfo info = gMesh.getDeviceInfo(devIndex);
                String label = info.name;
                if (gMesh.isConnected() && gMesh.connectedDeviceIndex() == devIndex) label += " [C]";
                d.println(label);
            }
        }
    }

    int getItemCount() const {
        return 1 + gMesh.getDeviceCount();
    }

    void handleEnter() {
        int sel       = menu.current();
        int itemCount = getItemCount();
        if (sel < 0 || sel >= itemCount) return;

        if (sel == 0) {
            if (gMesh.isScanning()) gMesh.cancelScan();
            else gMesh.startScan();
            menu.reset();
        } else {
            int devIndex = sel - 1;
            drawHeaderRightStatus("Connecting", "");
            gMesh.cancelAutoConnectRetries();
            gMesh.clearPasskey();
            gMesh.connectToIndex(devIndex);
        }
    }
};

void ScreenManager::drawConnectScanUpdates(bool redrawHeader, bool redrawDevices) {
    if (currentId == ScreenId::CONNECT && connect) {
        connect->drawScanUpdates(redrawHeader, redrawDevices);
    }
}

// -------------------- PIN entry screen --------------------

class PinEntryScreen : public Screen {
public:
    void onEnter() override {
        pinBuffer = "";
    }

    void onExit() override {
        pinBuffer = "";
        gPendingPinDeviceIndex = -1;
    }

    void handleNav(NavKey k) override {
        if (k == NavKey::ENTER) {
            attemptConnect();
        }
    }

    void handleChars(const std::vector<char>& chars) override {
        for (char c : chars) {
            if (c == '\b') {
                if (pinBuffer.length() > 0) {
                    pinBuffer.remove(pinBuffer.length() - 1);
                }
            } else if (c >= '0' && c <= '9') {
                if (pinBuffer.length() < 6) {
                    pinBuffer += c;
                }
            }
        }
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(kColorBodyBg);
        const int headerH = 24;
        drawHeaderBackground(headerH);

        d.setTextSize(1);
        d.setTextColor(kColorHeaderText, kColorHeaderBg);
        d.setCursor(4, 4);
        d.println("Enter BLE PIN");
        d.setCursor(4, 14);
        d.print("Device: ");
        d.println(pendingDeviceLabel());

        int y = headerH + 4;
        d.setTextColor(kColorBodyText, kColorBodyBg);
        if (gMesh.status() == ConnectionStatus::ERROR && gMesh.lastError().length() > 0) {
            d.setCursor(4, y);
            d.print("Error: ");
            d.println(gMesh.lastError());
            y += 12;
        }

        d.setTextSize(2);
        String pinDisplay = pinBuffer;
        for (size_t i = pinDisplay.length(); i < 6; ++i) {
            pinDisplay += "-";
        }
        String line = String("PIN: ") + pinDisplay;
        int x = (d.width() - d.textWidth(line)) / 2;
        if (x < 0) x = 0;
        d.setCursor(x, y + 12);
        d.println(line);

        d.setTextSize(1);
        drawFooterBackground();
        d.setTextColor(kColorFooterText, kColorFooterBg);
        int h = d.height();
        d.setCursor(4, h - kFooterH + 1);
        d.print("Enter: connect  Backspace: delete  FN+Backspace: back");
    }

private:
    String pinBuffer;

    String pendingDeviceLabel() const {
        if (gPendingPinDeviceIndex < 0 || gPendingPinDeviceIndex >= gMesh.getDeviceCount()) {
            return String("Unknown");
        }
        MeshDeviceInfo info = gMesh.getDeviceInfo(gPendingPinDeviceIndex);
        if (info.name.length() > 0) return info.name;
        if (info.id.length() > 0) return info.id;
        return String("Unknown");
    }

    void attemptConnect() {
        if (pinBuffer.length() != 6) {
            return;
        }
        uint32_t pin = (uint32_t)pinBuffer.toInt();
        drawHeaderRightStatus("Connecting", "");
        bool ok = gMesh.submitPasskey(pin);
        if (ok) {
            gScreens.switchTo(ScreenId::CONNECT);
        }
    }
};

// -------------------- Nodes screen --------------------

class NodesScreen : public Screen {
public:
    void onEnter() override {
        menu.reset();
        rebuildList();
    }

    void handleNav(NavKey k) override {
        int itemCount = getItemCount();
        menu.handleNav(k, itemCount);
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(kColorBodyBg);
        const int headerH = 44;
        drawHeaderBackground(headerH);
        d.setTextColor(kColorHeaderText, kColorHeaderBg);

        rebuildList();

        d.setTextSize(1);
        d.setCursor(4, 4);
        d.print("Nodes: ");
        d.print((int)nodeIndexMap.size());
        d.print("  BLE: ");
        d.println(connectionStatusLabel());

        MeshNodeInfo selNode;
        bool hasSel = false;
        if (!nodeIndexMap.empty()) {
            int sel = menu.current();
            if (sel >= 0 && sel < (int)nodeIndexMap.size()) {
                selNode = gMesh.getNodeInfo(nodeIndexMap[sel]);
                hasSel = (selNode.num != 0);
            }
        }

        if (hasSel) {
            d.setCursor(4, 14);
            String identity = nodeNameForRow(nodeDisplayName(selNode),
                                             String(" ") + nodeHex(selNode.num),
                                             d,
                                             d.width() - 8);
            d.println(identity);

            d.setCursor(4, 24);
            d.print(nodeActivityLabel(selNode));
            d.print(" Age:");
            d.println(formatAge(selNode));

            d.setCursor(4, 34);
            d.print("SNR:");
            d.print(selNode.snr, 1);
            d.print(" Hops:");
            if (selNode.hasHops) {
                d.print(selNode.hopsAway);
            } else {
                d.print("-");
            }
            d.print(" Batt:");
            if (selNode.hasBattery) {
                if (selNode.batteryLevel > 100) {
                    d.print("PWR");
                } else {
                    d.print(selNode.batteryLevel);
                    d.print("%");
                }
            } else {
                d.print("-");
            }
            d.println();
        } else {
            d.setCursor(4, 14);
            d.println("No nodes yet");
        }

        const int startY = 46;
        const int lineH  = 18;
        int screenH = d.height();
        int visibleRows = (screenH - kFooterH - startY) / lineH;
        if (visibleRows < 1) visibleRows = 1;
        menu.ensureVisible((int)nodeIndexMap.size(), visibleRows);
        int top = menu.topIndex();
        d.setTextSize(2);
        for (int i = top; i < (int)nodeIndexMap.size() && i < top + visibleRows; ++i) {
            int y = startY + (i - top) * lineH;
            MeshNodeInfo info = gMesh.getNodeInfo(nodeIndexMap[i]);

            if (i == menu.current()) {
                d.fillRect(0, y - 2, d.width(), lineH, DARKGREY);
                d.setTextColor(BLACK, DARKGREY);
            } else {
                d.setTextColor(WHITE, BLACK);
            }

            String suffix;
            if (info.num == gMesh.myNodeNum()) {
                suffix += " [me]";
            } else if (info.via_mqtt) {
                suffix += " [m]";
            }
            suffix += " ";
            suffix += nodeActivityLabel(info);
            String label = nodeNameForRow(nodeDisplayName(info), suffix, d, d.width() - 12);
            d.setCursor(6, y);
            d.println(label);
        }

        // Bottom hint
        d.setTextSize(1);
        drawFooterBackground();
        d.setTextColor(kColorFooterText, kColorFooterBg);
        int h = d.height();
        d.setCursor(4, h - kFooterH + 1);
        d.print("Backspace/Left: back   Shift: page");
    }

    int getSelectedNodeIndex() {
        rebuildList();
        if (nodeIndexMap.empty()) return -1;
        if (gSelectedNodeNum != 0) {
            for (int i = 0; i < (int)nodeIndexMap.size(); ++i) {
                if (gMesh.getNodeInfo(nodeIndexMap[i]).num == gSelectedNodeNum) {
                    menu.selected = i;
                    return nodeIndexMap[i];
                }
            }
        }
        int sel = menu.current();
        if (sel < 0 || sel >= (int)nodeIndexMap.size()) return -1;
        return nodeIndexMap[sel];
    }

private:
    ListMenuState menu;
    std::vector<int> nodeIndexMap;

    int getItemCount() const {
        return (int)nodeIndexMap.size();
    }

    void rebuildList() {
        uint32_t selectedNodeNum = 0;
        int selected = menu.current();
        if (selected >= 0 && selected < (int)nodeIndexMap.size()) {
            selectedNodeNum = gMesh.getNodeInfo(nodeIndexMap[selected]).num;
        }

        nodeIndexMap.clear();
        int count = gMesh.getNodeCount();
        for (int i = 0; i < count; ++i) {
            nodeIndexMap.push_back(i);
        }

        uint32_t sortNowMs = millis();
        uint32_t sortRadioNowEpoch = 0;
        bool hasSortRadioTime = gMesh.radioTimeNow(sortRadioNowEpoch);
        std::sort(nodeIndexMap.begin(), nodeIndexMap.end(),
                  [sortNowMs, hasSortRadioTime, sortRadioNowEpoch](int a, int b) {
            MeshNodeInfo nodeA = gMesh.getNodeInfo(a);
            MeshNodeInfo nodeB = gMesh.getNodeInfo(b);
            uint32_t ageA = 0;
            uint32_t ageB = 0;
            bool knownA = nodeActivityAgeSecondsAt(nodeA, ageA, sortNowMs,
                                                   hasSortRadioTime, sortRadioNowEpoch);
            bool knownB = nodeActivityAgeSecondsAt(nodeB, ageB, sortNowMs,
                                                   hasSortRadioTime, sortRadioNowEpoch);
            if (knownA != knownB) return knownA;
            if (knownA && ageA != ageB) return ageA < ageB;

            auto activityRank = [](const MeshNodeInfo& node) {
                if (node.activitySource == MeshActivitySource::LOCAL_OBSERVATION) return 2;
                if (node.activitySource == MeshActivitySource::RADIO_TIME && node.lastHeard != 0) return 1;
                return 0;
            };
            int rankA = activityRank(nodeA);
            int rankB = activityRank(nodeB);
            if (rankA != rankB) {
                // Without a radio clock reference, observed local traffic is
                // still comparable and sorts ahead of absolute unknown-age data.
                return rankA > rankB;
            }
            if (nodeA.activitySource == MeshActivitySource::RADIO_TIME &&
                nodeA.lastHeard != nodeB.lastHeard) {
                return nodeA.lastHeard > nodeB.lastHeard;
            }
            if (nodeA.activitySource == MeshActivitySource::LOCAL_OBSERVATION) {
                uint32_t elapsedA = sortNowMs - nodeA.lastActivityObservedMs;
                uint32_t elapsedB = sortNowMs - nodeB.lastActivityObservedMs;
                if (elapsedA != elapsedB) return elapsedA < elapsedB;
            }
            return nodeA.num < nodeB.num;
        });

        if (selectedNodeNum != 0) {
            for (int i = 0; i < (int)nodeIndexMap.size(); ++i) {
                if (gMesh.getNodeInfo(nodeIndexMap[i]).num == selectedNodeNum) {
                    menu.selected = i;
                    return;
                }
            }
        }
        if (menu.current() >= (int)nodeIndexMap.size()) {
            menu.reset();
        }
    }
};

// -------------------- Channels screen --------------------

class ChannelsScreen : public Screen {
public:
    void onEnter() override {
        menu.reset();
    }

    void handleNav(NavKey k) override {
        int count = getItemCount();
        switch (k) {
        case NavKey::UP:
        case NavKey::DOWN:
        case NavKey::UP_FAST:
        case NavKey::DOWN_FAST:
            menu.handleNav(k, count);
            break;
        case NavKey::ENTER:
            openChannel();
            break;
        default:
            break;
        }
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(kColorBodyBg);
        const int headerH = 24;
        drawHeaderBackground(headerH);

        d.setTextSize(1);
        d.setTextColor(kColorHeaderText, kColorHeaderBg);
        d.setCursor(4, 4);
        d.println("Channels");
        d.setCursor(4, 14);
        d.print("BLE: ");
        d.println(connectionStatusLabel());

        int count = gMesh.getChannelCount();
        d.setTextColor(kColorBodyText, kColorBodyBg);
        if (count == 0) {
            d.setCursor(4, 26);
            d.println("No channels yet");
        }

        const int startY = 32;
        const int lineH  = 22;
        int screenH = d.height();
        int visibleRows = (screenH - kFooterH - startY) / lineH;
        if (visibleRows < 1) visibleRows = 1;

        menu.ensureVisible(count, visibleRows);
        int top = menu.topIndex();

        d.setTextSize(2);
        for (int i = top; i < count && i < top + visibleRows; ++i) {
            int y = startY + (i - top) * lineH;
            MeshChannelInfo info = gMesh.getChannelInfo(i);

            if (i == menu.current()) {
                d.fillRect(0, y - 2, d.width(), lineH, DARKGREY);
                d.setTextColor(BLACK, DARKGREY);
            } else {
                d.setTextColor(WHITE, BLACK);
            }

            String label = String(info.index) + " " + channelDisplayName(info);
            label += " ";
            label += channelRoleLabel(info.role);
            if (info.muted) {
                label += " [M]";
            }

            d.setCursor(6, y);
            d.println(label);
        }

        d.setTextSize(1);
        drawFooterBackground();
        d.setTextColor(kColorFooterText, kColorFooterBg);
        int h = d.height();
        d.setCursor(4, h - kFooterH + 1);
        d.print("Enter/Right: open  Backspace/Left: back  Shift: page");
    }

private:
    ListMenuState menu;

    int getItemCount() const {
        return gMesh.getChannelCount();
    }

    void openChannel() {
        int count = getItemCount();
        if (count <= 0) return;
        int sel = menu.current();
        if (sel < 0 || sel >= count) return;

        MeshChannelInfo info = gMesh.getChannelInfo(sel);
        if (info.index < 0) return;

        int convIndex = ensureConversation(kBroadcastAddr, true, (uint8_t)info.index);
        gActiveConversation = convIndex;
        Conversation* conv = getConversation(convIndex);
        if (conv) {
            conv->hasUnread = false;
            persistConversation(convIndex);
        }
        gScreens.switchTo(ScreenId::CHAT);
    }
};

// -------------------- Node actions screen --------------------

class NodeActionsScreen : public Screen {
public:
    NodeActionsScreen() {
        items.push_back({"DM", true});
        items.push_back({"Info", true});
        items.push_back({"Back", true});
    }

    void onEnter() override {
        menu.reset();
    }

    void handleNav(NavKey k) override {
        int itemCount = (int)items.size();
        switch (k) {
        case NavKey::UP:
        case NavKey::DOWN:
        case NavKey::UP_FAST:
        case NavKey::DOWN_FAST:
            menu.handleNav(k, itemCount);
            break;
        case NavKey::ENTER:
            handleEnter();
            break;
        default:
            break;
        }
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(kColorBodyBg);
        const int headerH = 34;
        drawHeaderBackground(headerH);
        d.setTextColor(kColorHeaderText, kColorHeaderBg);

        MeshNodeInfo node;
        bool hasNode = getSelectedNodeInfo(node);
        bool isSelf = hasNode && node.num == gMesh.myNodeNum();
        items[0].enabled = !isSelf;

        d.setTextSize(1);
        d.setCursor(4, 4);
        d.println("Node Actions");
        if (hasNode) {
            d.setCursor(4, 14);
            d.print(nodeDisplayName(node));
            d.print(" ");
            d.println(nodeHex(node.num));
            d.setCursor(4, 24);
            d.print("Seen: ");
            d.println(formatAge(node));
        } else {
            d.setCursor(4, 14);
            d.println("No node selected");
        }

        const int startY = 36;
        const int lineH  = 22;
        int screenH = d.height();
        int visibleRows = (screenH - kFooterH - startY) / lineH;
        if (visibleRows < 1) visibleRows = 1;

        d.setTextSize(2);
        d.setTextColor(kColorBodyText, kColorBodyBg);
        menu.drawMenu(d, items, startY, lineH, visibleRows);

        d.setTextSize(1);
        drawFooterBackground();
        d.setTextColor(kColorFooterText, kColorFooterBg);
        int h = d.height();
        d.setCursor(4, h - kFooterH + 1);
        d.print("Enter/Right: select  Backspace/Left: back");
    }

private:
    ListMenuState           menu;
    std::vector<MenuOption> items;

    void handleEnter() {
        MeshNodeInfo node;
        bool hasNode = getSelectedNodeInfo(node);
        int sel = menu.current();
        if (!hasNode) {
            gScreens.switchTo(ScreenId::NODES);
            return;
        }

        if (sel == 0) {
            if (node.num == gMesh.myNodeNum()) {
                return;
            }
            int convIndex = ensureConversation(node.num, false, 0);
            gActiveConversation = convIndex;
            Conversation* conv = getConversation(convIndex);
            if (conv) {
                conv->hasUnread = false;
                persistConversation(convIndex);
            }
            gScreens.switchTo(ScreenId::CHAT);
        } else if (sel == 1) {
            gScreens.switchTo(ScreenId::NODE_INFO);
        } else {
            gScreens.switchTo(ScreenId::NODES);
        }
    }
};

// -------------------- Node info screen --------------------

class NodeInfoScreen : public Screen {
public:
    void handleNav(NavKey k) override {
        if (k == NavKey::ENTER || k == NavKey::ESC) {
            gScreens.switchTo(ScreenId::NODE_ACTIONS);
        }
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(kColorBodyBg);
        const int headerH = 24;
        drawHeaderBackground(headerH);

        MeshNodeInfo node;
        bool hasNode = getSelectedNodeInfo(node);

        d.setTextSize(1);
        d.setTextColor(kColorHeaderText, kColorHeaderBg);
        d.setCursor(4, 4);
        d.println("Node Info");

        d.setTextColor(kColorBodyText, kColorBodyBg);
        if (!hasNode) {
            d.setCursor(4, 14);
            d.println("No node selected");
        } else {
            String name = nodeDisplayName(node);
            if (name.length() > 18) {
                name = truncateLabel(name, 18);
            }

            d.setCursor(4, 14);
            d.print("Name: ");
            d.println(name);

            d.setCursor(4, 24);
            d.print("Num: ");
            d.println(nodeHex(node.num));

            d.setCursor(4, 34);
            d.print("Seen: ");
            d.print(formatAge(node));
            d.print("  Active: ");
            uint32_t activityAge = 0;
            if (!nodeActivityAgeSeconds(node, activityAge)) {
                d.println("?");
            } else {
                d.println(activityAge <= kNodeActiveMs / 1000 ? "yes" : "no");
            }

            d.setCursor(4, 44);
            d.print("SNR:");
            d.print(node.snr, 1);
            d.print(" Hops:");
            if (node.hasHops) {
                d.print(node.hopsAway);
            } else {
                d.print("-");
            }
            d.print(" Ch:");
            d.println(node.channel);

            d.setCursor(4, 54);
            d.print("MQTT: ");
            d.println(node.via_mqtt ? "yes" : "no");

            d.setCursor(4, 64);
            d.print("Batt: ");
            if (node.hasBattery) {
                if (node.batteryLevel > 100) {
                    d.print("PWR");
                } else {
                    d.print(node.batteryLevel);
                    d.print("%");
                }
            } else {
                d.print("-");
            }
            d.print("  V: ");
            if (node.hasVoltage) {
                d.println(node.voltage, 2);
            } else {
                d.println("-");
            }

            d.setCursor(4, 74);
            d.print("LastHeard: ");
            d.println(node.lastHeard);
        }

        d.setTextSize(1);
        drawFooterBackground();
        d.setTextColor(kColorFooterText, kColorFooterBg);
        int h = d.height();
        d.setCursor(4, h - kFooterH + 1);
        d.print("Backspace/Left: back");
    }
};

// -------------------- Conversations screen (list) --------------------

class ConversationsScreen : public Screen {
public:
    void onEnter() override {
        menu.reset();
        rebuildOrder();
    }

    void handleNav(NavKey k) override {
        if (!isRadioReady()) return;
        menu.handleNav(k, (int)convIndexMap.size());
        (void)k;
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(kColorBodyBg);
        const int headerH = 24;
        drawHeaderBackground(headerH);

        if (!isRadioReady()) {
            d.setTextSize(1);
            d.setTextColor(kColorHeaderText, kColorHeaderBg);
            d.setCursor(4, 4);
            d.println("Conversations");
            d.setCursor(4, 14);
            d.print("BLE: ");
            d.println(connectionStatusLabel());
            d.setTextColor(kColorBodyText, kColorBodyBg);
            d.setCursor(4, 28);
            d.println("Connect to a node to load conversations");

            d.setTextSize(1);
            drawFooterBackground();
            d.setTextColor(kColorFooterText, kColorFooterBg);
            int h = d.height();
            d.setCursor(4, h - kFooterH + 1);
            d.print(gStorageError.length() ? "SD WARNING - check history" : "Backspace/Left: back");
            return;
        }

        rebuildOrder();

        d.setTextSize(1);
        d.setTextColor(kColorHeaderText, kColorHeaderBg);
        d.setCursor(4, 4);
        d.println("Conversations");
        d.setCursor(4, 14);
        d.print("BLE: ");
        d.println(connectionStatusLabel());

        if (convIndexMap.empty()) {
            d.setTextColor(kColorBodyText, kColorBodyBg);
            d.setCursor(4, 28);
            d.println("No conversations yet");

            d.setTextSize(1);
            drawFooterBackground();
            d.setTextColor(kColorFooterText, kColorFooterBg);
            int h = d.height();
            d.setCursor(4, h - kFooterH + 1);
            d.print(gStorageError.length() ? "SD WARNING - check history" : "Backspace/Left: back");
            return;
        }

        const int startY = 28;
        const int lineH  = 16;
        int screenH = d.height();
        int visibleRows = (screenH - kFooterH - 10 - startY) / lineH;
        if (visibleRows < 1) visibleRows = 1;

        d.setTextColor(kColorBodyText, kColorBodyBg);
        int charWidth = d.textWidth("A");
        if (charWidth <= 0) charWidth = 12;
        int maxChars = (d.width() - 12) / charWidth;
        if (maxChars < 6) maxChars = 6;

        items.clear();
        for (int idx : convIndexMap) {
            Conversation* conv = getConversation(idx);
            if (!conv) continue;
            String label = conversationListTitle(*conv);
            if (conv->hasUnread) {
                label += " *";
            }
            String preview = lastMessagePreview(*conv);
            if (preview.length() > 0) {
                label += ": " + preview;
            }
            items.push_back({truncateLabel(label, (size_t)maxChars), true});
        }

        menu.drawMenu(d, items, startY, lineH, visibleRows, 1);

        // Bottom hint
        d.setTextSize(1);
        drawFooterBackground();
        d.setTextColor(kColorFooterText, kColorFooterBg);
        int h = d.height();
        d.setCursor(4, h - kFooterH + 1);
        d.print(gStorageError.length() ? "SD WARNING - check history" :
                "Arrows/W,S: move  Enter/Right: open  Backspace/Left: back");
    }

    int getSelectedConversationIndex() const {
        if (!isRadioReady()) return -1;
        int sel = menu.current();
        if (sel < 0 || sel >= (int)convIndexMap.size()) return -1;
        return convIndexMap[sel];
    }

private:
    ListMenuState           menu;
    std::vector<MenuOption> items;
    std::vector<int>        convIndexMap;

    bool isRadioReady() const {
        return gMesh.myNodeNum() != 0;
    }

    void rebuildOrder() {
        convIndexMap.clear();
        for (int i = 0; i < (int)gConversations.size(); ++i) {
            Conversation* conv = getConversation(i);
            if (!conv || conv->messages.empty()) {
                continue;
            }
            if (!conv->isBroadcast && conv->peer == gMesh.myNodeNum()) {
                continue;
            }
            convIndexMap.push_back(i);
        }

        std::sort(convIndexMap.begin(), convIndexMap.end(), [](int a, int b) {
            const Conversation* ca = getConversation(a);
            const Conversation* cb = getConversation(b);
            if (!ca || !cb) return a < b;
            if (ca->hasUnread != cb->hasUnread) return ca->hasUnread > cb->hasUnread;
            if (ca->lastActivitySeq != cb->lastActivitySeq) {
                return ca->lastActivitySeq > cb->lastActivitySeq;
            }
            String ta = conversationListTitle(*ca);
            String tb = conversationListTitle(*cb);
            return ta < tb;
        });

        if (menu.current() >= (int)convIndexMap.size()) {
            menu.reset();
        }
    }
};

// -------------------- Chat screen (conversation view) --------------------

class ChatScreen : public Screen {
    struct ChatLine {
        String text;
        size_t coloredPrefixLength = 0;
        uint16_t prefixColor = WHITE;
        bool isDateSeparator = false;
    };

public:
    ChatScreen() : inputBuffer(""), sendFeedback("") {}

    void onEnter() override {
        Conversation* conv = getConversation(gActiveConversation);
        if (conv) {
            conv->hasUnread = false;
            conv->firstVisibleIndex = -1;
            persistConversation(gActiveConversation);
        }
    }

    void onExit() override {
        gActiveConversation = -1;
        inputBuffer = "";
        sendFeedback = "";
    }

    void handleNav(NavKey k) override {
        auto& d = M5Cardputer.Display;
        int h = d.height();

        // Layout: small header at top, messages in the middle, input at bottom
        const int headerH = 20;   // two small lines
        const int inputH  = 12;   // one input line
        const int topY    = headerH + 4;
        const int bottomY = h - inputH - 2;
        const int lineH   = 10;

        Conversation* conv = getConversation(gActiveConversation);
        if (!conv) return;

        int maxRows = (bottomY - topY) / lineH;
        if (maxRows < 1) maxRows = 1;

        std::vector<ChatLine> lines;
        buildWrappedLines(*conv, d, lines);
        int total    = (int)lines.size();
        int maxStart = (total > maxRows) ? (total - maxRows) : 0;
        int step     = (k == NavKey::UP_FAST || k == NavKey::DOWN_FAST) ? maxRows : 1;

        int currentStart = conv->firstVisibleIndex < 0 ? maxStart : conv->firstVisibleIndex;

        switch (k) {
        case NavKey::UP:
        case NavKey::UP_FAST:
            currentStart -= step;
            if (currentStart < 0) currentStart = 0;
            conv->firstVisibleIndex = currentStart;
            break;

        case NavKey::DOWN:
        case NavKey::DOWN_FAST:
            currentStart += step;
            if (currentStart >= maxStart) {
                conv->firstVisibleIndex = -1;
            } else {
                conv->firstVisibleIndex = currentStart;
            }
            break;

        case NavKey::ENTER:
            // Send message if there is input
            if (inputBuffer.length() > 0) {
                // Keep the draft intact until authentication/configuration is ready.
                if (!gMesh.readyToSend()) break;
                if (!messageTextFits(inputBuffer.length()) ||
                    !isValidUtf8(inputBuffer.c_str(), inputBuffer.length())) {
                    sendFeedback = !messageTextFits(inputBuffer.length()) ? "Message too long" : "Invalid UTF-8";
                    break;
                }
                uint32_t dest = conv->isBroadcast ? kBroadcastAddr : conv->peer;
                uint32_t deliveryToken = gNextDeliveryToken++;
                if (deliveryToken == 0) deliveryToken = gNextDeliveryToken++;
                uint32_t packetId = 0;
                String outgoingText = inputBuffer;
                bool sent = gMesh.sendTextMessage(outgoingText,
                                                  dest,
                                                  conv->channel,
                                                  deliveryToken,
                                                  &packetId);
                if (sent) {
                    appendConversationMessage(gActiveConversation,
                                              true,
                                              outgoingText,
                                              gMesh.myNodeNum(),
                                              false,
                                              MessageDeliveryState::PENDING,
                                              deliveryToken,
                                              packetId,
                                              conv->channel);
                    if (!conv->isBroadcast && dest != 0) {
                        gMesh.ensureNode(dest);
                    }
                    inputBuffer = "";
                    sendFeedback = "";
                    conv->firstVisibleIndex = -1;
                } else {
                    sendFeedback = gMesh.lastError().length() > 0 ? gMesh.lastError() : "Send failed; draft kept";
                }
            }
            break;

        default:
            break;
        }
    }

    void handleChars(const std::vector<char>& chars) override {
        for (char c : chars) {
            if (c == '\b') {
                // Backspace removes the previous UTF-8 codepoint, not just its last byte.
                if (inputBuffer.length() > 0) {
                    size_t start = previousUtf8Boundary(inputBuffer.c_str(), inputBuffer.length(), inputBuffer.length());
                    inputBuffer.remove(start);
                }
                sendFeedback = "";
            } else if (c >= 32 && c <= 126) {
                // Printable ASCII character
                inputBuffer += c;
                sendFeedback = "";
            }
        }
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(kColorBodyBg);

        int w = d.width();
        int h = d.height();

        // Layout: small header, messages, input at bottom
        const int headerH = 20;
        const int inputH  = 12;
        const int topY    = headerH + 4;
        const int bottomY = h - inputH - 2;
        const int lineH   = 10;

        // Header
        d.setTextSize(1);
        drawHeaderBackground(headerH);
        d.setTextColor(kColorHeaderText, kColorHeaderBg);
        d.setCursor(4, 4);
        Conversation* conv = getConversation(gActiveConversation);
        if (!conv) {
            d.println("Chat");
            d.setCursor(4, 14);
            d.println("No conversation selected");
        } else {
            String title = conversationTitle(*conv);
            drawDisplayText(d, title, kColorHeaderText);
            d.println();
            d.setCursor(4, 14);
            String status = sendFeedback;
            if (status.length() == 0 && inputBuffer.length() > kMaxMessageTextBytes) {
                status = String("Too long: ") + String(inputBuffer.length()) + "/" + String(kMaxMessageTextBytes) + " B";
            } else if (status.length() == 0) {
                status = gStorageError.length() ? "SD WARNING - check history" : "FN+;/. scroll DEL | ACK!=read";
            }
            drawDisplayText(d, truncateToWidth(status, d.width() - 8, d), kColorHeaderText);
        }

        int maxRows = (bottomY - topY) / lineH;
        if (maxRows < 1) maxRows = 1;

        if (!conv) {
            return;
        }

        std::vector<ChatLine> lines;
        buildWrappedLines(*conv, d, lines);
        int total    = (int)lines.size();
        int maxStart = (total > maxRows) ? (total - maxRows) : 0;

        int start = conv->firstVisibleIndex < 0 ? maxStart : conv->firstVisibleIndex;
        if (start > maxStart) start = maxStart;
        if (start < 0) start = 0;
        int end   = start + maxRows;
        if (end > total) end = total;

        // Draw messages
        d.setTextSize(1);
        d.setTextColor(kColorBodyText, kColorBodyBg);
        int y = topY;
        for (int i = start; i < end; ++i) {
            d.setCursor(2, y);
            const ChatLine& line = lines[i];
            if (line.isDateSeparator) {
                d.setTextColor(CYAN, kColorBodyBg);
                int x = (w - d.textWidth(line.text)) / 2;
                d.setCursor(x > 2 ? x : 2, y);
                drawDisplayText(d, line.text, CYAN);
                d.println();
            } else if (line.coloredPrefixLength > 0 && line.coloredPrefixLength <= line.text.length()) {
                d.setTextColor(line.prefixColor, kColorBodyBg);
                drawDisplayText(d, line.text.substring(0, line.coloredPrefixLength), line.prefixColor);
                d.setTextColor(kColorBodyText, kColorBodyBg);
                drawDisplayText(d, line.text.substring(line.coloredPrefixLength), kColorBodyText);
                d.println();
            } else {
                d.setTextColor(kColorBodyText, kColorBodyBg);
                drawDisplayText(d, line.text, kColorBodyText);
                d.println();
            }
            y += lineH;
        }

        // Composer: reserve space for the byte counter and show the suffix at
        // the end of the draft so the active cursor never scrolls off-screen.
        d.fillRect(0, h - inputH, w, inputH, kColorFooterBg);
        d.drawFastHLine(0, h - inputH, w, kColorSeparator);
        String prompt = gMesh.readyToSend() ? String("> ") : String("[not ready] ");
        String counter = String(inputBuffer.length()) + "/" + String(kMaxMessageTextBytes) + "B";
        int promptWidth = d.textWidth(prompt);
        int counterWidth = d.textWidth(counter);
        int cursorWidth = d.textWidth("_");
        int textX = 2 + promptWidth;
        int counterX = w - 2 - counterWidth;
        int availableTextWidth = counterX - textX - cursorWidth - 2;
        if (availableTextWidth < 0) availableTextWidth = 0;

        size_t visibleStart = inputBuffer.length();
        const int ellipsisWidth = d.textWidth("...");
        while (visibleStart > 0) {
            size_t candidateStart = previousUtf8Boundary(inputBuffer.c_str(), inputBuffer.length(), visibleStart);
            String candidate = mapSpanishGlyphsForDisplay(inputBuffer.substring(candidateStart), true);
            int hiddenPrefixWidth = candidateStart > 0 ? ellipsisWidth : 0;
            if (d.textWidth(candidate) + hiddenPrefixWidth > availableTextWidth) break;
            visibleStart = candidateStart;
        }

        d.setTextColor(kColorFooterText, kColorFooterBg);
        d.setCursor(2, h - inputH + 1);
        d.print(prompt);
        if (visibleStart > 0) d.print("...");
        drawDisplayText(d, mapSpanishGlyphsForDisplay(inputBuffer.substring(visibleStart), true), kColorFooterText);
        d.print("_");
        d.setTextColor(inputBuffer.length() > kMaxMessageTextBytes ? RED : kColorFooterText, kColorFooterBg);
        d.setCursor(counterX, h - inputH + 1);
        d.print(counter);
    }

    bool isInputEmpty() const {
        return inputBuffer.length() == 0;
    }

private:
    bool formatMessageTime(uint32_t epoch, String& timeText, String& dateText) const {
        const String& tz = gMesh.timeZone();
        if (epoch == 0 || tz.length() == 0) return false;

        static String activeTimezone;
        if (activeTimezone != tz) {
            if (setenv("TZ", tz.c_str(), 1) != 0) return false;
            tzset();
            activeTimezone = tz;
        }

        time_t rawTime = static_cast<time_t>(epoch);
        struct tm localTime;
        if (!localtime_r(&rawTime, &localTime)) return false;
        char timeBuffer[6];
        char dateBuffer[11];
        if (strftime(timeBuffer, sizeof(timeBuffer), "%H:%M", &localTime) == 0 ||
            strftime(dateBuffer, sizeof(dateBuffer), "%d/%m/%Y", &localTime) == 0) {
            return false;
        }
        timeText = timeBuffer;
        dateText = dateBuffer;
        return true;
    }

    String truncateToWidth(const String& text, int maxWidth, M5GFX& d) const {
        if (maxWidth <= 0) return String("");
        if (d.textWidth(text) <= maxWidth) return text;
        const String ellipsis = "...";
        int ellipsisW = d.textWidth(ellipsis);
        if (ellipsisW >= maxWidth) return text.substring(0, 1);
        size_t end = 0;
        String sub;
        while (end < text.length()) {
            size_t next = nextUtf8Boundary(text, end);
            String candidate = text.substring(0, next);
            if (d.textWidth(candidate) + ellipsisW > maxWidth) break;
            end = next;
            sub = candidate;
        }
        if (end > 0) {
            if (d.textWidth(sub) + ellipsisW <= maxWidth) {
                return sub + ellipsis;
            }
        }
        return ellipsis;
    }

    void appendWrappedText(const String& prefix,
                           const String& text,
                           int maxWidth,
                           M5GFX& d,
                           size_t coloredPrefixLength,
                           uint16_t prefixColor,
                           std::vector<ChatLine>& out) const {
        String effectivePrefix = prefix;
        if (d.textWidth(effectivePrefix) > maxWidth) {
            effectivePrefix = truncateToWidth(effectivePrefix, maxWidth, d);
        }
        size_t firstOutputLine = out.size();
        auto appendLine = [&](const String& value) {
            ChatLine line;
            line.text = value;
            if (out.size() == firstOutputLine && coloredPrefixLength > 0) {
                line.coloredPrefixLength = std::min(coloredPrefixLength, effectivePrefix.length());
                line.prefixColor = prefixColor;
            }
            out.push_back(line);
        };
        int prefixWidth = d.textWidth(effectivePrefix);
        const String indent = "  ";

        String clean = text;
        clean.replace('\n', ' ');

        String current = effectivePrefix;
        int currentWidth = prefixWidth;
        bool hasText = false;
        int wordCount = 0;

        int len = clean.length();
        int i = 0;
        while (i < len) {
            while (i < len && clean[i] == ' ') {
                i++;
            }
            int start = i;
            while (i < len && clean[i] != ' ') {
                i++;
            }
            if (start >= i) {
                break;
            }
            String word = clean.substring(start, i);
            wordCount++;

            int wordWidth = d.textWidth(word);
            int spaceWidth = hasText ? d.textWidth(" ") : 0;

            if (currentWidth + spaceWidth + wordWidth <= maxWidth) {
                if (hasText) {
                    current += " ";
                    currentWidth += spaceWidth;
                }
                current += word;
                currentWidth += wordWidth;
                hasText = true;
                continue;
            }

            if (hasText) {
                appendLine(current);
                current = indent;
                currentWidth = d.textWidth(current);
                hasText = false;
            }

            if (currentWidth + wordWidth > maxWidth) {
                size_t pos = 0;
                while (pos < word.length()) {
                    size_t chunkEnd = pos;
                    String chunk;
                    while (chunkEnd < word.length()) {
                        size_t next = nextUtf8Boundary(word, chunkEnd);
                        String candidate = word.substring(pos, next);
                        if (chunkEnd > pos && currentWidth + d.textWidth(candidate) > maxWidth) break;
                        chunkEnd = next;
                        chunk = candidate;
                        if (currentWidth + d.textWidth(chunk) > maxWidth) break;
                    }
                    if (chunkEnd == pos) {
                        chunkEnd = nextUtf8Boundary(word, pos);
                        chunk = word.substring(pos, chunkEnd);
                    }
                    current += chunk;
                    currentWidth += d.textWidth(chunk);
                    hasText = true;
                    pos = chunkEnd;
                    if (pos < word.length()) {
                        appendLine(current);
                        current = indent;
                        currentWidth = d.textWidth(current);
                        hasText = false;
                    }
                }
            } else {
                current += word;
                currentWidth += wordWidth;
                hasText = true;
            }
        }

        if (wordCount == 0) {
            appendLine(effectivePrefix);
        } else if (hasText) {
            appendLine(current);
        }
    }

    void buildWrappedLines(const Conversation& conv,
                           M5GFX& d,
                           std::vector<ChatLine>& out) const {
        out.clear();
        d.setTextSize(1);
        int maxWidth = d.width() - 4;
        String previousDate;
        for (const auto& msg : conv.messages) {
            String timeText = "--:--";
            String dateText;
            if (formatMessageTime(msg.rxTime, timeText, dateText) &&
                dateText != previousDate) {
                ChatLine dateLine;
                dateLine.text = String("--- ") + dateText + " ---";
                dateLine.isDateSeparator = true;
                out.push_back(dateLine);
                previousDate = dateText;
            }
            String label = msg.fromMe ? String("Me") : resolvedNodeLabel(msg.from);
            label = mapSpanishGlyphsForDisplay(label, true);
            String prefix = timeText + " " + label + "> ";
            size_t coloredPrefixLength = msg.fromMe ? prefix.length() : 0;
            uint16_t prefixColor = kColorBodyText;
            if (msg.fromMe) {
                switch (msg.delivery) {
                case MessageDeliveryState::PENDING:
                    prefixColor = YELLOW;
                    break;
                case MessageDeliveryState::MESH_CONFIRMED:
                    prefixColor = conv.isBroadcast ? GREEN : CYAN;
                    break;
                case MessageDeliveryState::DELIVERED:
                    prefixColor = GREEN;
                    break;
                case MessageDeliveryState::FAILED:
                    prefixColor = RED;
                    break;
                case MessageDeliveryState::QUEUE_REJECTED:
                    prefixColor = RED;
                    break;
                case MessageDeliveryState::UNCONFIRMED:
                case MessageDeliveryState::TIMED_OUT:
                case MessageDeliveryState::DISCONNECTED:
                    prefixColor = ORANGE;
                    break;
                case MessageDeliveryState::UNKNOWN:
                default:
                    coloredPrefixLength = 0;
                    break;
                }
            }
            String displayText = prepareMessageForDisplay(msg.text);
            appendWrappedText(prefix,
                              displayText,
                              maxWidth,
                              d,
                              coloredPrefixLength,
                              prefixColor,
                              out);
        }
    }

    String inputBuffer;
    String sendFeedback;
};

// -------------------- ScreenManager implementation --------------------

void ScreenManager::begin() {
    home    = new HomeScreen();
    connect = new ConnectScreen();
    pinEntry = new PinEntryScreen();
    nodes   = new NodesScreen();
    channels = new ChannelsScreen();
    nodeActions = new NodeActionsScreen();
    nodeInfo = new NodeInfoScreen();
    convs   = new ConversationsScreen();
    chat    = new ChatScreen();

    initConversations();

    currentId     = ScreenId::HOME;
    currentScreen = getScreenById(currentId);
    if (currentScreen) currentScreen->onEnter();
    needRedraw = true;
}

Screen* ScreenManager::getScreenById(ScreenId id) {
    switch (id) {
    case ScreenId::HOME:          return home;
    case ScreenId::CONNECT:       return connect;
    case ScreenId::PIN_ENTRY:     return pinEntry;
    case ScreenId::NODES:         return nodes;
    case ScreenId::CHANNELS:      return channels;
    case ScreenId::NODE_ACTIONS:  return nodeActions;
    case ScreenId::NODE_INFO:     return nodeInfo;
    case ScreenId::CONVERSATIONS: return convs;
    case ScreenId::CHAT:          return chat;
    }
    return nullptr;
}

void ScreenManager::switchTo(ScreenId id) {
    if (id == currentId) return;
    if (currentScreen) currentScreen->onExit();
    currentId     = id;
    currentScreen = getScreenById(id);
    if (currentScreen) currentScreen->onEnter();
    needRedraw = true;
}

void ScreenManager::handleRadioIdentityChanged() {
    gSelectedNodeNum = 0;
    gPendingPinDeviceIndex = -1;
    if (currentId == ScreenId::CHAT || currentId == ScreenId::PIN_ENTRY ||
        currentId == ScreenId::NODE_ACTIONS || currentId == ScreenId::NODE_INFO) {
        switchTo(ScreenId::HOME);
    } else {
        needRedraw = true;
    }
}

void ScreenManager::handleRadioDisconnected() {
    gSelectedNodeNum = 0;
    gPendingPinDeviceIndex = -1;
    if (currentId == ScreenId::CHAT || currentId == ScreenId::PIN_ENTRY ||
        currentId == ScreenId::NODE_ACTIONS || currentId == ScreenId::NODE_INFO) {
        switchTo(ScreenId::HOME);
    } else {
        needRedraw = true;
    }
}

void handleRadioIdentityChanged() {
    invalidateRadioTransientState();
    gScreens.handleRadioIdentityChanged();
}

void handleRadioDisconnected() {
    invalidateRadioTransientState();
    gScreens.handleRadioDisconnected();
}

void ScreenManager::handleNav(NavKey k) {
    if (!currentScreen) return;

    // Global "back": go to Home from any screen except Home
    if (k == NavKey::ESC) {
        if (currentId == ScreenId::PIN_ENTRY) {
            gMesh.cancelAutoConnectRetries();
            gMesh.disconnect();
            switchTo(ScreenId::CONNECT);
            return;
        } else if (currentId == ScreenId::CHAT) {
            switchTo(ScreenId::CONVERSATIONS);
            return;
        } else if (currentId == ScreenId::NODE_INFO) {
            switchTo(ScreenId::NODE_ACTIONS);
            return;
        } else if (currentId == ScreenId::NODE_ACTIONS) {
            switchTo(ScreenId::NODES);
            return;
        } else if (currentId != ScreenId::HOME) {
            switchTo(ScreenId::HOME);
            return;
        }
    }

    // HOME-specific behavior for ENTER
    if (currentId == ScreenId::HOME && k == NavKey::ENTER) {
        auto* hs  = static_cast<HomeScreen*>(currentScreen);
        int   sel = hs->getSelected();
        if (sel == 0) {
            switchTo(ScreenId::CONNECT);
            return;
        } else if (sel == 1) {
            switchTo(ScreenId::NODES);
            return;
        } else if (sel == 2) {
            switchTo(ScreenId::CHANNELS);
            return;
        } else if (sel == 3) {
            switchTo(ScreenId::CONVERSATIONS);
            return;
        }
        // Channels handled above
    }

    // CONVERSATIONS: ENTER opens the chat (single conversation for now)
    if (currentId == ScreenId::CONVERSATIONS && k == NavKey::ENTER) {
        auto* cs = static_cast<ConversationsScreen*>(currentScreen);
        int convIndex = cs->getSelectedConversationIndex();
        if (convIndex < 0) return;
        gActiveConversation = convIndex;
        Conversation* conv = getConversation(convIndex);
        if (conv) {
            conv->hasUnread = false;
            persistConversation(convIndex);
        }
        switchTo(ScreenId::CHAT);
        return;
    }

    // NODES: ENTER opens actions for selected node
    if (currentId == ScreenId::NODES && k == NavKey::ENTER) {
        auto* ns = static_cast<NodesScreen*>(currentScreen);
        int nodeIndex = ns->getSelectedNodeIndex();
        if (nodeIndex >= 0) {
            gSelectedNodeNum = gMesh.getNodeInfo(nodeIndex).num;
            switchTo(ScreenId::NODE_ACTIONS);
        }
        return;
    }

    currentScreen->handleNav(k);
    needRedraw = true;
}

void ScreenManager::redrawIfNeeded() {
    if (needRedraw && currentScreen) {
        currentScreen->draw();
        needRedraw = false;
    }
}

void ScreenManager::loop() {
    // Update Cardputer (keyboard, etc.)
    M5Cardputer.update();

    uint32_t now = millis();
    bool keyChanged = M5Cardputer.Keyboard.isChange();
    static bool scanWasRunning = false;
    static int renderedScanAttempt = -1;
    static int renderedScanSeconds = -2;
    static uint32_t renderedScanRevision = UINT32_MAX;
    bool scanning = gMesh.isScanning();
    if (currentId == ScreenId::CONNECT && scanning && !gScreenSleeping) {
        int attempt = gMesh.scanAttempt();
        int seconds = gMesh.scanSecondsLeft();
        uint32_t revision = gMesh.scanRevision();
        bool redrawHeader = !scanWasRunning || attempt != renderedScanAttempt ||
                            seconds != renderedScanSeconds;
        bool redrawDevices = !scanWasRunning || revision != renderedScanRevision;
        if (redrawHeader || redrawDevices) {
            drawConnectScanUpdates(redrawHeader, redrawDevices);
        }
        renderedScanAttempt = attempt;
        renderedScanSeconds = seconds;
        renderedScanRevision = revision;
    } else if (scanWasRunning) {
        // One normal full draw settles the screen after scan completion/cancel.
        needRedraw = true;
        renderedScanAttempt = -1;
        renderedScanSeconds = -2;
        renderedScanRevision = UINT32_MAX;
    }
    scanWasRunning = scanning;
    if (keyChanged) {
        gLastInputMs = now;
        if (gScreenSleeping) {
            M5Cardputer.Display.wakeup();
            gScreenSleeping = false;
            needRedraw = true;
            return;
        }
    }

    if (!gScreenSleeping && kScreenIdleMs > 0 && (now - gLastInputMs) > kScreenIdleMs) {
        M5Cardputer.Display.sleep();
        gScreenSleeping = true;
        return;
    }

    applyRadioStateTransition(gMesh.myNodeNum());

    if (currentId != ScreenId::PIN_ENTRY && gMesh.awaitingPasskey()) {
        gPendingPinDeviceIndex = gMesh.connectedDeviceIndex();
        switchTo(ScreenId::PIN_ENTRY);
        return;
    }
    if (currentId == ScreenId::PIN_ENTRY && !gMesh.awaitingPasskey()) {
        switchTo(ScreenId::CONNECT);
    }

    bool connectionStateChanged = gLastRenderedConnectionStatus != gMesh.status() ||
                                  gLastRenderedConfigComplete != gMesh.configComplete() ||
                                  gLastRenderedSessionStatus != gMesh.sessionStatusLabel() ||
                                  gLastRenderedError != gMesh.lastError();
    if (connectionStateChanged) {
        needRedraw = true;
    }
    gLastRenderedConnectionStatus = gMesh.status();
    gLastRenderedConfigComplete = gMesh.configComplete();
    gLastRenderedSessionStatus = gMesh.sessionStatusLabel();
    gLastRenderedError = gMesh.lastError();
    if (gLastRenderedStorageError != gStorageError) needRedraw = true;
    gLastRenderedStorageError = gStorageError;

    if (tryLoadConversationsOnce()) {
        needRedraw = true;
    }
    if (refreshNodeIdentityCache()) {
        needRedraw = true;
    }
    flushDirtyConversations();

    MeshDeliveryUpdate deliveryUpdate;
    while (gMesh.popDeliveryUpdate(deliveryUpdate)) {
        applyDeliveryUpdate(deliveryUpdate);
        needRedraw = true;
    }

    bool gotMessage = false;
    MeshTextMessage msg;
    while (gMesh.popTextMessage(msg)) {
        if (gMesh.myNodeNum() == 0 || gMesh.myNodeNum() != gConversationRadioNode) continue;
        if (reconcileOrDeduplicatePacket(msg)) {
            gotMessage = true;
            continue;
        }
        bool isBroadcast = (msg.to == kBroadcastAddr || msg.to == 0);
        uint32_t sender = msg.from;
        if (!msg.fromMe && !isBroadcast && sender == 0) {
            Conversation* active = getConversation(gActiveConversation);
            if (active && !active->isBroadcast) {
                sender = active->peer;
            }
        }
        uint32_t peer = isBroadcast ? kBroadcastAddr : (msg.fromMe ? msg.to : sender);
        int convIndex = ensureConversation(peer, isBroadcast, msg.channel);
        bool isActive = (convIndex == gActiveConversation);
        bool markUnread = (!msg.fromMe && !isActive);
        Serial.printf("[App] RX route: from=0x%08lX peer=0x%08lX broadcast=%d ch=%u conv=%d active=%d unread=%d\n",
                      (unsigned long)sender,
                      (unsigned long)peer,
                      isBroadcast ? 1 : 0,
                      (unsigned)msg.channel,
                      convIndex,
                      isActive ? 1 : 0,
                      markUnread ? 1 : 0);
        appendConversationMessage(convIndex,
                                 msg.fromMe,
                                 msg.text,
                                 sender,
                                 markUnread,
                                 MessageDeliveryState::UNKNOWN,
                                 0,
                                 msg.packetId,
                                 msg.channel,
                                 msg.rxTime);
        gotMessage = true;
    }
    if (gotMessage) {
        needRedraw = true;
    }

    NavKey nk = NavKey::NONE;
    std::vector<char> textChars;   // chars sent to the current screen (chat)

    if (keyChanged && M5Cardputer.Keyboard.isPressed()) {
        Keyboard_Class::KeysState st = M5Cardputer.Keyboard.keysState();

        bool isChat = (currentId == ScreenId::CHAT);
        bool isPin  = (currentId == ScreenId::PIN_ENTRY);

        if (isChat || isPin) {
            // --- TEXT INPUT MODE ---
            if (st.enter) {
                nk = NavKey::ENTER;
            }
            else if (st.del) {
                // Backspace: with FN = back, without FN = delete text
                if (st.fn) {
                    nk = NavKey::ESC;
                } else {
                    if (isChat) {
                        auto* chatScreen = static_cast<ChatScreen*>(currentScreen);
                        if (chatScreen && chatScreen->isInputEmpty()) {
                            nk = NavKey::ESC;
                        } else {
                            textChars.push_back('\b');
                        }
                    } else {
                        textChars.push_back('\b');
                    }
                }
            }
            else {
                if (st.fn && isChat) {
                    // Chat only: FN + key => navigation
                    bool fast = st.shift;
                    for (auto c : st.word) {
                        switch (c) {
                        case ';':
                        case ':':
                            nk = fast ? NavKey::UP_FAST : NavKey::UP;
                            break;
                        case '.':
                        case '>':
                            nk = fast ? NavKey::DOWN_FAST : NavKey::DOWN;
                            break;
                        case ',':
                        case '<':
                            nk = NavKey::ESC;
                            break;
                        case '/':
                        case '?':
                            nk = NavKey::ENTER;
                            break;
                        default:
                            break;
                        }
                        if (nk != NavKey::NONE) {
                            break;
                        }
                    }
                } else if (isPin) {
                    // PIN entry: digits only, allow left/right shortcuts
                    for (auto c : st.word) {
                        if (c >= '0' && c <= '9') {
                            textChars.push_back(c);
                        } else if (c == ',' || c == '<') {
                            nk = NavKey::ESC;
                        } else if (c == '/' || c == '?') {
                            nk = NavKey::ENTER;
                        }
                        if (nk != NavKey::NONE) {
                            break;
                        }
                    }
                } else {
                    // Chat without FN: all characters are text
                    for (auto c : st.word) {
                        textChars.push_back(c);
                    }
                }
            }
        } else {
            // --- MENU / NON-CHAT MODE ---
            // 1) Enter
            if (st.enter) {
                nk = NavKey::ENTER;
            }
            // 2) Backspace/Delete -> back
            else if (st.del) {
                nk = NavKey::ESC;
            }
            else {
                // 3) Interpret printable keys (arrow legends share keys with punctuation)
                //    Left-arrow  -> ','  (we'll treat as "back")
                //    Down-arrow  -> '.'  (NEXT / move down)
                //    Up-arrow    -> ';'  (PREV / move up)
                //    Right-arrow -> '/'  (SELECT / like Enter)

                bool fast = st.shift;
                for (auto c : st.word) {
                    switch (c) {

                    case 'r':
                    case 'R':
                        if (currentId == ScreenId::CONNECT) nk = NavKey::RETRY_SYNC;
                        break;

                    case 'a':
                    case 'A':
                        if (currentId == ScreenId::CONNECT) {
                            nk = NavKey::TOGGLE_AUTOCONNECT;
                        }
                        break;

                    // UP: Up-arrow / ';' + fallback W
                    case ';':
                    case ':':
                    case 'w':
                    case 'W':
                        nk = fast ? NavKey::UP_FAST : NavKey::UP;
                        break;

                    // DOWN: Down-arrow / '.' + fallback S
                    case '.':
                    case '>':
                    case 's':
                    case 'S':
                        nk = fast ? NavKey::DOWN_FAST : NavKey::DOWN;
                        break;

                    // LEFT: Left-arrow / ',' -> treat as "back"
                    case ',':
                    case '<':
                        nk = NavKey::ESC;
                        break;

                    // RIGHT: Right-arrow / '/' -> treat as "enter/select"
                    case '/':
                    case '?':
                        nk = NavKey::ENTER;
                        break;

                    default:
                        break;
                    }

                    if (nk != NavKey::NONE) {
                        break;
                    }
                }
            }
        }
    }

    // First, pass raw characters to the current screen (if it cares)
    if (!textChars.empty() && currentScreen) {
        currentScreen->handleChars(textChars);
        needRedraw = true;
    }

    // Then handle navigation
    if (nk != NavKey::NONE) {
        handleNav(nk);
    }

    redrawIfNeeded();
}

// -------------------- Global and Arduino setup/loop --------------------

ScreenManager gScreens;

void setup() {
    Serial.begin(115200);      // serial debug

    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);
    M5Cardputer.Display.setAttribute(lgfx::CP437_SWITCH, true);

    M5Cardputer.Display.setRotation(1);
    drawSplashScreen();
    delay(1000);

    gMesh.begin();     // init BLE
    gScreens.begin();
    gSdReady = initStorage();
    if (gSdReady) {
        storageSuccess();
    } else {
        gStorageError = "SD unavailable";
        gStorageRetry.failed(millis());
        Serial.println("[Storage] initial SD mount failed; retry scheduled");
    }
    gLastInputMs = millis();
    tryLoadConversationsOnce();
    gMesh.connectToLastDevice();
}


void loop() {
#if defined(CARDTASTIC_DEBUG_BLE_TIMING) && CARDTASTIC_DEBUG_BLE_TIMING
    const uint32_t startedUs = micros();
#endif
    gMesh.loop();      // BLE event handling
#if defined(CARDTASTIC_DEBUG_BLE_TIMING) && CARDTASTIC_DEBUG_BLE_TIMING
    const uint32_t bleUs = micros() - startedUs;
#endif
    gScreens.loop();
#if defined(CARDTASTIC_DEBUG_BLE_TIMING) && CARDTASTIC_DEBUG_BLE_TIMING
    static uint32_t maxBleUs = 0;
    static uint32_t maxLoopUs = 0;
    static uint32_t lastReportMs = 0;
    maxBleUs = std::max(maxBleUs, bleUs);
    maxLoopUs = std::max(maxLoopUs, static_cast<uint32_t>(micros() - startedUs));
    if (millis() - lastReportMs >= 1000) {
        Serial.printf("[Timing] max BLE=%lu us, max loop=%lu us\n",
                      static_cast<unsigned long>(maxBleUs), static_cast<unsigned long>(maxLoopUs));
        maxBleUs = maxLoopUs = 0;
        lastReportMs = millis();
    }
#endif
}
