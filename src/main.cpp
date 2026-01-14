#include <SD.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include "M5Cardputer.h"
#include <vector>
#include <NimBLEDevice.h> 
#include <string>
#include "MeshtasticClient.h"
#include "meshtastic/channel.pb.h"

// -------------------- Basic navigation --------------------

enum class NavKey {
    NONE,
    UP,
    UP_FAST,
    DOWN,
    DOWN_FAST,
    ENTER,
    ESC
};

// -------------------- Conversation / Message model (RAM only for now) --------------------

constexpr uint32_t kBroadcastAddr = 0xFFFFFFFF;
constexpr size_t kConversationMaxMessages = 50;
constexpr size_t kConversationJsonMax = 32768;
const char* kStorageDir = "/cardtastic";

struct Message {
    bool   fromMe;  // true = this UI, false = remote node
    uint32_t from;
    String text;
};

struct Conversation {
    uint32_t             peer;        // nodenum for DM, or broadcast addr
    bool                 isBroadcast;
    uint8_t              channel;
    std::vector<Message> messages;
    int                  firstVisibleIndex;  // -1 = auto-bottom
    bool                 hasUnread;
};

static std::vector<Conversation> gConversations;
static int gActiveConversation = -1;
static int gSelectedNodeIndex = -1;

constexpr uint32_t kNodeActiveMs = 5UL * 60UL * 1000UL;
static bool gSdReady = false;
static uint32_t gLastSdAttemptMs = 0;
static bool gStorageLoaded = false;
static uint32_t gLoadedRadioNode = 0;

Conversation* getConversation(int index);
int ensureConversation(uint32_t peer, bool isBroadcast, uint8_t channel);
void initConversations();

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

bool ensureStorage() {
    if (gSdReady) return true;
    uint32_t now = millis();
    if (now - gLastSdAttemptMs < 3000) return false;
    gLastSdAttemptMs = now;
    gSdReady = initStorage();
    return gSdReady;
}

String nodeHexId(uint32_t num) {
    char buf[9];
    snprintf(buf, sizeof(buf), "%08lX", (unsigned long)num);
    return String(buf);
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
    if (conv.isBroadcast) {
        return String(kStorageDir) + "/ch_" + String(conv.channel) + ".json";
    }
    return String(kStorageDir) + "/dm_" + nodeHexId(conv.peer) + ".json";
}

void persistConversation(int convIndex) {
    if (!ensureStorage()) return;
    Conversation* conv = getConversation(convIndex);
    if (!conv) return;

    DynamicJsonDocument doc(kConversationJsonMax);
    doc["version"] = 1;
    uint32_t radio = gMesh.myNodeNum();
    if (radio != 0) {
        doc["radio"] = radio;
    }
    doc["peer"] = conv->peer;
    doc["broadcast"] = conv->isBroadcast;
    doc["channel"] = conv->channel;
    doc["unread"] = conv->hasUnread;

    JsonArray arr = doc.createNestedArray("messages");
    size_t total = conv->messages.size();
    size_t start = (total > kConversationMaxMessages) ? (total - kConversationMaxMessages) : 0;
    for (size_t i = start; i < total; ++i) {
        const auto& msg = conv->messages[i];
        JsonObject obj = arr.createNestedObject();
        obj["from"] = msg.from;
        obj["fromMe"] = msg.fromMe;
        obj["text"] = msg.text;
    }

    String path = conversationFilePath(*conv);
    if (SD.exists(path)) {
        SD.remove(path);
    }
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        gSdReady = false;
        gStorageLoaded = false;
        return;
    }
    serializeJson(doc, f);
    f.close();
}

void loadConversationsFromStorage() {
    if (!ensureStorage()) return;
    uint32_t currentRadio = gMesh.myNodeNum();

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
        String name = String(f.name());
        if (!name.endsWith(".json")) {
            f.close();
            continue;
        }

        size_t cap = f.size() * 2 + 512;
        if (cap < 2048) cap = 2048;
        if (cap > kConversationJsonMax) cap = kConversationJsonMax;
        DynamicJsonDocument doc(cap);

        DeserializationError err = deserializeJson(doc, f);
        f.close();
        if (err) {
            continue;
        }

        uint32_t peer = doc["peer"] | 0;
        bool isBroadcast = doc["broadcast"] | false;
        uint8_t channel = doc["channel"] | 0;
        bool unread = doc["unread"] | false;
        uint32_t radio = doc["radio"] | 0;
        if (isBroadcast && peer == 0) peer = kBroadcastAddr;
        if (!isBroadcast && peer == 0) {
            peer = parsePeerFromFilename(name);
        }
        if (peer == 0) {
            continue;
        }
        if (radio != 0 && currentRadio != 0 && radio != currentRadio) {
            continue;
        }

        int convIndex = ensureConversation(peer, isBroadcast, channel);
        Conversation* conv = getConversation(convIndex);
        if (!conv) continue;

        if (!conv->messages.empty()) {
            continue;
        }

        conv->peer = peer;
        conv->isBroadcast = isBroadcast;
        conv->channel = channel;
        conv->messages.clear();
        conv->hasUnread = unread;
        conv->firstVisibleIndex = -1;

        JsonArray msgs = doc["messages"].as<JsonArray>();
        if (msgs.isNull()) continue;
        for (JsonObject obj : msgs) {
            Message m;
            m.from = obj["from"] | 0;
            m.fromMe = obj["fromMe"] | false;
            const char* txt = obj["text"] | "";
            m.text = String(txt);
            conv->messages.push_back(m);
        }

        if (conv->messages.size() > kConversationMaxMessages) {
            size_t remove = conv->messages.size() - kConversationMaxMessages;
            conv->messages.erase(conv->messages.begin(), conv->messages.begin() + remove);
        }
    }
    dir.close();
}

bool tryLoadConversationsOnce() {
    uint32_t currentRadio = gMesh.myNodeNum();
    if (currentRadio == 0) return false;
    if (gStorageLoaded && gLoadedRadioNode == currentRadio) return false;
    if (!ensureStorage()) return false;

    if (gLoadedRadioNode != 0 && gLoadedRadioNode != currentRadio) {
        initConversations();
    }

    gLoadedRadioNode = currentRadio;
    loadConversationsFromStorage();
    gStorageLoaded = true;
    return true;
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
    gConversations.push_back(c);
    return (int)gConversations.size() - 1;
}

Conversation* getConversation(int index) {
    if (index < 0 || index >= (int)gConversations.size()) return nullptr;
    return &gConversations[index];
}

String conversationTitle(const Conversation& conv) {
    if (conv.isBroadcast) return gMesh.channelLabel(conv.channel);
    return gMesh.nodeLabel(conv.peer);
}

String nodeHex(uint32_t num) {
    String hex = String(num, HEX);
    hex.toUpperCase();
    return String("0x") + hex;
}

String nodeDisplayName(const MeshNodeInfo& node) {
    if (node.shortName.length() > 0) return node.shortName;
    if (node.longName.length() > 0) return node.longName;
    return nodeHex(node.num);
}

String channelDisplayName(const MeshChannelInfo& ch) {
    if (ch.name.length() > 0) return ch.name;
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

String formatAge(uint32_t lastUpdateMs) {
    if (lastUpdateMs == 0) return "-";
    uint32_t ageSec = (millis() - lastUpdateMs) / 1000;
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

bool isNodeActive(const MeshNodeInfo& node) {
    if (node.lastUpdateMs == 0) return false;
    return (millis() - node.lastUpdateMs) <= kNodeActiveMs;
}

const char* nodeActiveTag(const MeshNodeInfo& node) {
    if (node.lastUpdateMs == 0) return " [?]";
    return isNodeActive(node) ? " [A]" : "";
}

void drawHeaderRightStatus(const String& line1, const String& line2) {
    auto& d = M5Cardputer.Display;
    d.setTextSize(1);
    d.setTextColor(WHITE, BLACK);

    int clearWidth = d.textWidth("Connecting");
    int scanWidth  = d.textWidth("Scan 99s");
    if (scanWidth > clearWidth) clearWidth = scanWidth;
    int x = d.width() - clearWidth - 4;

    d.fillRect(x, 4, clearWidth + 4, 10, BLACK);
    d.fillRect(x, 14, clearWidth + 4, 10, BLACK);

    if (line1.length() > 0) {
        d.setCursor(d.width() - d.textWidth(line1) - 4, 4);
        d.print(line1);
    }
    if (line2.length() > 0) {
        d.setCursor(d.width() - d.textWidth(line2) - 4, 14);
        d.print(line2);
    }
}

void drawScanCountdown(int secondsLeft, int attempt, int attempts) {
    if (secondsLeft < 0) {
        drawHeaderRightStatus("", "");
        return;
    }
    String line1 = String("Scan ") + secondsLeft + "s";
    String line2 = String(attempt) + "/" + attempts;
    drawHeaderRightStatus(line1, line2);
}

bool getSelectedNodeInfo(MeshNodeInfo& out) {
    if (gSelectedNodeIndex < 0 || gSelectedNodeIndex >= gMesh.getNodeCount()) return false;
    out = gMesh.getNodeInfo(gSelectedNodeIndex);
    return out.num != 0;
}

const char* connectionStatusLabel() {
    switch (gMesh.status()) {
    case ConnectionStatus::DISCONNECTED:
        return "DISCONNECTED";
    case ConnectionStatus::SCANNING:
        return "SCANNING";
    case ConnectionStatus::CONNECTING:
        return "CONNECTING";
    case ConnectionStatus::CONNECTED:
        return "CONNECTED";
    case ConnectionStatus::ERROR:
        return "ERROR";
    }
    return "UNKNOWN";
}

bool hasUnreadConversations() {
    for (const auto& conv : gConversations) {
        if (conv.hasUnread) return true;
    }
    return false;
}

void initConversations() {
    gConversations.clear();
    ensureConversation(kBroadcastAddr, true, 0);
    gActiveConversation = -1;
}

void appendConversationMessage(int convIndex,
                               bool fromMe,
                               const String& text,
                               uint32_t from,
                               bool markUnread) {
    Conversation* conv = getConversation(convIndex);
    if (!conv) return;
    bool autoBottom = (conv->firstVisibleIndex < 0);
    conv->messages.push_back({fromMe, from, text});
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
    persistConversation(convIndex);
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
                  int maxRows = -1) {
        d.setTextSize(2);
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

            if (i == selected) {
                d.fillRect(0, y - 2, d.width(), lineH, DARKGREY);
                d.setTextColor(opt.enabled ? BLACK : DARKGREY, DARKGREY);
            } else {
                d.setTextColor(opt.enabled ? WHITE : DARKGREY, BLACK);
            }
            d.setCursor(8, y);
            d.println(opt.label);
        }
    }
};

// -------------------- Screen interface --------------------

class Screen {
public:
    virtual ~Screen() {}
    virtual void onEnter() {}
    virtual void onExit() {}

    // Navegación abstracta
    virtual void handleNav(NavKey k) = 0;

    // Entrada de texto opcional (por defecto ignora)
    virtual void handleChars(const std::vector<char>& chars) {
        (void)chars;
    }

    virtual void draw() = 0;
};

// Screen IDs

enum class ScreenId {
    HOME,
    CONNECT,
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

private:
    ScreenId currentId;
    Screen*  currentScreen;
    bool     needRedraw;

    // Static screen instances (simple for now)
    HomeScreen*          home;
    ConnectScreen*       connect;
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
        d.fillScreen(BLACK);
        d.setTextColor(WHITE, BLACK);

        // --- Header (small text) ---
        d.setTextSize(1);
        String title = "Cardtastic 0.1";
        int titleX = (d.width() - d.textWidth(title)) / 2;
        if (titleX < 0) titleX = 0;
        d.setCursor(titleX, 4);
        d.println(title);
        d.setCursor(4, 14);
        d.print("BLE: ");
        d.println(connectionStatusLabel());

        // --- Menu (bigger text) ---
        const int startY = 32;
        const int lineH  = 22;
        int visibleRows = (d.height() - 10 - startY) / lineH;
        if (visibleRows < 1) visibleRows = 1;
        if (items.size() >= 4) {
            items[3].label = hasUnreadConversations() ? "Conversations *" : "Conversations";
        }
        menu.drawMenu(d, items, startY, lineH, visibleRows);

        // --- Bottom hint bar (small text again) ---
        d.setTextSize(1);
        d.setTextColor(WHITE, BLACK);
        int h = d.height();
        d.fillRect(0, h - 10, d.width(), 10, BLACK);
        d.setCursor(4, h - 9);
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
        default:
            break;
        }
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(BLACK);
        d.setTextColor(WHITE, BLACK);

        // --- Header compacto: sólo estado en 2 líneas pequeñas ---
        d.setTextSize(1);
        d.setCursor(4, 4);
        d.print("Status: ");
        switch (gMesh.status()) {
        case ConnectionStatus::DISCONNECTED:
            d.println("DISCONNECTED");
            break;
        case ConnectionStatus::SCANNING:
            d.println("SCANNING");
            break;
        case ConnectionStatus::CONNECTING:
            d.println("CONNECTING");
            break;
        case ConnectionStatus::CONNECTED:
            d.println("CONNECTED");
            break;
        case ConnectionStatus::ERROR:
            d.println("ERROR");
            break;
        }

        d.setCursor(4, 14);
        if (gMesh.status() == ConnectionStatus::CONNECTED) {
            d.print("Radio: ");
            d.println(gMesh.radioName());
        } else if (gMesh.status() == ConnectionStatus::ERROR) {
            d.print("Error: ");
            d.println(gMesh.lastError());
        } else {
            int count = gMesh.getDeviceCount();
            d.print("Devices: ");
            d.println(count);
        }

        // --- Lista: Scan + dispositivos ---
        d.setTextSize(2);
        const int startY = 28;   // más arriba, aprovechando el espacio ganado
        const int lineH  = 22;
        int screenH = d.height();
        int visibleRows = (screenH - 10 - startY) / lineH;
        if (visibleRows < 1) visibleRows = 1;

        int itemCount = getItemCount();
        int sel       = menu.current();
        menu.ensureVisible(itemCount, visibleRows);
        int top = menu.topIndex();

        for (int i = top; i < itemCount && i < top + visibleRows; ++i) {
            int y = startY + (i - top) * lineH;

            if (i == sel) {
                d.fillRect(0, y - 2, d.width(), lineH, DARKGREY);
                d.setTextColor(BLACK, DARKGREY);
            } else {
                d.setTextColor(WHITE, BLACK);
            }

            d.setCursor(8, y);
            if (i == 0) {
                d.println("Scan for devices");
            } else {
                int devIndex = i - 1;
                MeshDeviceInfo info = gMesh.getDeviceInfo(devIndex);

                String label = info.name;
                if (gMesh.isConnected() && gMesh.radioName() == info.name) {
                    label += " [C]";
                }
                d.println(label);
            }
        }

        // --- Barra inferior de ayuda ---
        d.setTextSize(1);
        d.setTextColor(WHITE, BLACK);
        int h = d.height();
        d.fillRect(0, h - 10, d.width(), 10, BLACK);
        d.setCursor(4, h - 9);
        d.print("Enter/Right: scan/connect   Backspace/Left: back");
    }

private:
    ListMenuState menu;

    int getItemCount() const {
        return 1 + gMesh.getDeviceCount();
    }

    void handleEnter() {
        int sel       = menu.current();
        int itemCount = getItemCount();
        if (sel < 0 || sel >= itemCount) return;

        if (sel == 0) {
            // Scan for devices
            gMesh.setScanProgressCallback(drawScanCountdown);
            gMesh.startScan();
            gMesh.setScanProgressCallback(nullptr);
            menu.reset();
        } else {
            int devIndex = sel - 1;
            drawHeaderRightStatus("Connecting", "");
            gMesh.connectToIndex(devIndex);
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
        d.fillScreen(BLACK);
        d.setTextColor(WHITE, BLACK);

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
            d.print("Sel: ");
            d.print(nodeDisplayName(selNode));
            d.print(" ");
            d.println(nodeHex(selNode.num));

            d.setCursor(4, 24);
            d.print("SNR:");
            d.print(selNode.snr, 1);
            d.print(" H:");
            if (selNode.hasHops) {
                d.print(selNode.hopsAway);
            } else {
                d.print("-");
            }
            d.print(" Ch:");
            d.print(selNode.channel);

            d.setCursor(4, 34);
            d.print("Batt:");
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
            d.print(" V:");
            if (selNode.hasVoltage) {
                d.print(selNode.voltage, 2);
            } else {
                d.print("-");
            }
            d.print(" Age:");
            d.println(formatAge(selNode.lastUpdateMs));
        } else {
            d.setCursor(4, 14);
            d.println("No nodes yet");
        }

        const int startY = 46;
        const int lineH  = 18;
        int screenH = d.height();
        int visibleRows = (screenH - 10 - startY) / lineH;
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

            String label = nodeDisplayName(info);
            if (label.length() > 14) {
                label = label.substring(0, 13) + ".";
            }
            if (info.num == gMesh.myNodeNum()) {
                label += " [me]";
            } else if (info.via_mqtt) {
                label += " [m]";
            }
            label += nodeActiveTag(info);
            d.setCursor(6, y);
            d.println(label);
        }

        // Bottom hint
        d.setTextSize(1);
        int h = d.height();
        d.fillRect(0, h - 10, d.width(), 10, BLACK);
        d.setCursor(4, h - 9);
        d.print("Backspace/Left: back   Shift: page");
    }

    int getSelectedNodeIndex() {
        rebuildList();
        if (nodeIndexMap.empty()) return -1;
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
        nodeIndexMap.clear();
        int count = gMesh.getNodeCount();
        for (int i = 0; i < count; ++i) {
            nodeIndexMap.push_back(i);
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
        d.fillScreen(BLACK);
        d.setTextColor(WHITE, BLACK);

        d.setTextSize(1);
        d.setCursor(4, 4);
        d.println("Channels");
        d.setCursor(4, 14);
        d.print("BLE: ");
        d.println(connectionStatusLabel());

        int count = gMesh.getChannelCount();
        if (count == 0) {
            d.setCursor(4, 26);
            d.println("No channels yet");
        }

        const int startY = 32;
        const int lineH  = 22;
        int screenH = d.height();
        int visibleRows = (screenH - 10 - startY) / lineH;
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
        int h = d.height();
        d.fillRect(0, h - 10, d.width(), 10, BLACK);
        d.setCursor(4, h - 9);
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
        d.fillScreen(BLACK);
        d.setTextColor(WHITE, BLACK);

        MeshNodeInfo node;
        bool hasNode = getSelectedNodeInfo(node);

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
            d.println(formatAge(node.lastUpdateMs));
        } else {
            d.setCursor(4, 14);
            d.println("No node selected");
        }

        const int startY = 36;
        const int lineH  = 22;
        int screenH = d.height();
        int visibleRows = (screenH - 10 - startY) / lineH;
        if (visibleRows < 1) visibleRows = 1;

        d.setTextSize(2);
        menu.drawMenu(d, items, startY, lineH, visibleRows);

        d.setTextSize(1);
        int h = d.height();
        d.fillRect(0, h - 10, d.width(), 10, BLACK);
        d.setCursor(4, h - 9);
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
        d.fillScreen(BLACK);
        d.setTextColor(WHITE, BLACK);

        MeshNodeInfo node;
        bool hasNode = getSelectedNodeInfo(node);

        d.setTextSize(1);
        d.setCursor(4, 4);
        d.println("Node Info");

        if (!hasNode) {
            d.setCursor(4, 14);
            d.println("No node selected");
        } else {
            String name = nodeDisplayName(node);
            if (name.length() > 18) {
                name = name.substring(0, 17) + ".";
            }

            d.setCursor(4, 14);
            d.print("Name: ");
            d.println(name);

            d.setCursor(4, 24);
            d.print("Num: ");
            d.println(nodeHex(node.num));

            d.setCursor(4, 34);
            d.print("Seen: ");
            d.print(formatAge(node.lastUpdateMs));
            d.print("  Active: ");
            if (node.lastUpdateMs == 0) {
                d.println("?");
            } else {
                d.println(isNodeActive(node) ? "yes" : "no");
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
        int h = d.height();
        d.fillRect(0, h - 10, d.width(), 10, BLACK);
        d.setCursor(4, h - 9);
        d.print("Backspace/Left: back");
    }
};

// -------------------- Conversations screen (list) --------------------

class ConversationsScreen : public Screen {
public:
    void onEnter() override {
        menu.reset();
        rebuildList();
    }

    void handleNav(NavKey k) override {
        menu.handleNav(k, (int)items.size());
        (void)k;
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(BLACK);
        d.setTextColor(WHITE, BLACK);

        rebuildList();

        d.setTextSize(1);
        d.setCursor(4, 4);
        d.println("Conversations");
        d.setCursor(4, 14);
        d.print("BLE: ");
        d.println(connectionStatusLabel());

        const int startY = 28;
        const int lineH  = 22;
        int screenH = d.height();
        int visibleRows = (screenH - 10 - 10 - startY) / lineH;
        if (visibleRows < 1) visibleRows = 1;

        d.setTextSize(2);
        menu.drawMenu(d, items, startY, lineH, visibleRows);

        // Hint debajo de la lista
        d.setTextSize(1);
        int hintY = startY + lineH * ((int)items.size() < visibleRows ? (int)items.size() : visibleRows) + 2;
        d.setCursor(4, hintY);
        d.println("Enter/Right: open chat");

        // Bottom hint
        int h = d.height();
        d.fillRect(0, h - 10, d.width(), 10, BLACK);
        d.setCursor(4, h - 9);
        d.print("Arrows/W,S: move  Enter/Right: open  Backspace/Left: back");
    }

    int getSelectedConversationIndex() const {
        int sel = menu.current();
        if (sel < 0 || sel >= (int)convIndexMap.size()) return -1;
        return convIndexMap[sel];
    }

private:
    ListMenuState           menu;
    std::vector<MenuOption> items;
    std::vector<int>        convIndexMap;

    void rebuildList() {
        items.clear();
        convIndexMap.clear();

        std::vector<int> ordered;
        int broadcastIdx = ensureConversation(kBroadcastAddr, true, 0);
        ordered.push_back(broadcastIdx);

        uint32_t myNode = gMesh.myNodeNum();
        int nodeCount = gMesh.getNodeCount();
        for (int i = 0; i < nodeCount; ++i) {
            MeshNodeInfo info = gMesh.getNodeInfo(i);
            if (info.num == 0 || info.num == myNode) continue;
            int convIdx = ensureConversation(info.num, false, 0);
            bool exists = false;
            for (int idx : ordered) {
                if (idx == convIdx) {
                    exists = true;
                    break;
                }
            }
            if (!exists) ordered.push_back(convIdx);
        }

        for (int i = 0; i < (int)gConversations.size(); ++i) {
            bool exists = false;
            for (int idx : ordered) {
                if (idx == i) {
                    exists = true;
                    break;
                }
            }
            if (!exists) ordered.push_back(i);
        }

        for (int idx : ordered) {
            Conversation* conv = getConversation(idx);
            if (!conv) continue;
            String label = conversationTitle(*conv);
            if (conv->hasUnread) {
                label += " *";
            }
            items.push_back({label, true});
            convIndexMap.push_back(idx);
        }

        if (menu.current() >= (int)items.size()) {
            menu.reset();
        }
    }
};

// -------------------- Chat screen (conversation view) --------------------

class ChatScreen : public Screen {
public:
    ChatScreen() : inputBuffer("") {}

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
    }

    void handleNav(NavKey k) override {
        auto& d = M5Cardputer.Display;
        int h = d.height();

        // Layout: header pequeño arriba, mensajes en el medio, input abajo
        const int headerH = 30;   // tres líneas pequeñas
        const int inputH  = 12;   // una línea de input
        const int topY    = headerH + 4;
        const int bottomY = h - inputH - 2;
        const int lineH   = 10;

        Conversation* conv = getConversation(gActiveConversation);
        if (!conv) return;

        int maxRows = (bottomY - topY) / lineH;
        if (maxRows < 1) maxRows = 1;

        int total    = (int)conv->messages.size();
        int maxStart = (total > maxRows) ? (total - maxRows) : 0;
        int step     = (k == NavKey::UP_FAST || k == NavKey::DOWN_FAST) ? maxRows : 1;

        if (conv->firstVisibleIndex < 0) {
            conv->firstVisibleIndex = maxStart;
        }

        switch (k) {
        case NavKey::UP:
        case NavKey::UP_FAST:
            if (conv->firstVisibleIndex > 0) {
                conv->firstVisibleIndex -= step;
                if (conv->firstVisibleIndex < 0) conv->firstVisibleIndex = 0;
            }
            break;

        case NavKey::DOWN:
        case NavKey::DOWN_FAST:
            if (conv->firstVisibleIndex < maxStart) {
                conv->firstVisibleIndex += step;
                if (conv->firstVisibleIndex > maxStart) conv->firstVisibleIndex = maxStart;
            }
            break;

        case NavKey::ENTER:
            // Enviar mensaje si hay algo escrito
            if (inputBuffer.length() > 0) {
                uint32_t dest = conv->isBroadcast ? kBroadcastAddr : conv->peer;
                if (!gMesh.isConnected()) {
                    appendConversationMessage(gActiveConversation,
                                              true,
                                              String("[not connected] ") + inputBuffer,
                                              gMesh.myNodeNum(),
                                              false);
                } else {
                    bool sent = gMesh.sendTextMessage(inputBuffer, dest, conv->channel);
                    if (sent) {
                        appendConversationMessage(gActiveConversation, true, inputBuffer, gMesh.myNodeNum(), false);
                        if (!conv->isBroadcast && dest != 0) {
                            gMesh.noteNode(dest);
                        }
                    } else {
                        appendConversationMessage(gActiveConversation,
                                                  true,
                                                  String("[send failed] ") + inputBuffer,
                                                  gMesh.myNodeNum(),
                                                  false);
                    }
                }

                inputBuffer = "";
                // Ir al final para ver el mensaje enviado
                conv->firstVisibleIndex = -1;
            }
            break;

        default:
            break;
        }
    }

    void handleChars(const std::vector<char>& chars) override {
        for (char c : chars) {
            if (c == '\b') {
                // Backspace: borrar último carácter del input
                if (inputBuffer.length() > 0) {
                    inputBuffer.remove(inputBuffer.length() - 1);
                }
            } else if (c >= 32 && c <= 126) {
                // Caracter imprimible ASCII
                inputBuffer += c;
            }
        }
    }

    void draw() override {
        auto& d = M5Cardputer.Display;
        d.fillScreen(BLACK);
        d.setTextColor(WHITE, BLACK);

        int w = d.width();
        int h = d.height();

        // Layout: header pequeño, mensajes, input abajo
        const int headerH = 30;
        const int inputH  = 12;
        const int topY    = headerH + 4;
        const int bottomY = h - inputH - 2;
        const int lineH   = 10;

        // Header
        d.setTextSize(1);
        d.setCursor(4, 4);
        Conversation* conv = getConversation(gActiveConversation);
        if (!conv) {
            d.println("Chat");
            d.setCursor(4, 14);
            d.println("No conversation selected");
        } else {
            d.print("Chat: ");
            d.println(conversationTitle(*conv));
            d.setCursor(4, 14);
            d.print("BLE: ");
            d.println(connectionStatusLabel());
            d.setCursor(4, 24);
            d.println("FN+;/. scroll  Shift=page");
        }

        int maxRows = (bottomY - topY) / lineH;
        if (maxRows < 1) maxRows = 1;

        if (!conv) {
            return;
        }

        int total    = (int)conv->messages.size();
        int maxStart = (total > maxRows) ? (total - maxRows) : 0;

        if (conv->firstVisibleIndex < 0) {
            conv->firstVisibleIndex = maxStart;
        }
        if (conv->firstVisibleIndex > maxStart) {
            conv->firstVisibleIndex = maxStart;
        }
        if (conv->firstVisibleIndex < 0) {
            conv->firstVisibleIndex = 0;
        }

        int start = conv->firstVisibleIndex;
        int end   = start + maxRows;
        if (end > total) end = total;

        // Draw messages
        d.setTextSize(1);
        int y = topY;
        for (int i = start; i < end; ++i) {
            const Message& msg = conv->messages[i];
            d.setCursor(2, y);

            // Prefijo simple
            if (msg.fromMe) {
                d.print("Me> ");
            } else {
                String label = gMesh.nodeLabel(msg.from);
                d.print(label);
                d.print("> ");
            }
            d.println(msg.text);

            y += lineH;
        }

        // Input line at bottom: muestra el buffer
        d.fillRect(0, h - inputH, w, inputH, BLACK);
        d.setCursor(2, h - inputH + 1);
        d.print("> ");
        d.print(inputBuffer);
    }

private:
    String inputBuffer;
};

// -------------------- ScreenManager implementation --------------------

void ScreenManager::begin() {
    home    = new HomeScreen();
    connect = new ConnectScreen();
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

void ScreenManager::handleNav(NavKey k) {
    if (!currentScreen) return;

    // Global "back": go to Home from any screen except Home
    if (k == NavKey::ESC) {
        if (currentId == ScreenId::NODE_INFO) {
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
        if (convIndex >= 0) {
            gActiveConversation = convIndex;
            Conversation* conv = getConversation(convIndex);
            if (conv) {
                conv->hasUnread = false;
                persistConversation(convIndex);
            }
        }
        switchTo(ScreenId::CHAT);
        return;
    }

    // NODES: ENTER opens actions for selected node
    if (currentId == ScreenId::NODES && k == NavKey::ENTER) {
        auto* ns = static_cast<NodesScreen*>(currentScreen);
        int nodeIndex = ns->getSelectedNodeIndex();
        if (nodeIndex >= 0) {
            gSelectedNodeIndex = nodeIndex;
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

    if (tryLoadConversationsOnce()) {
        needRedraw = true;
    }

    bool gotMessage = false;
    MeshTextMessage msg;
    while (gMesh.popTextMessage(msg)) {
        bool isBroadcast = (msg.to == kBroadcastAddr || msg.to == 0);
        uint32_t peer = isBroadcast ? kBroadcastAddr : (msg.fromMe ? msg.to : msg.from);
        int convIndex = ensureConversation(peer, isBroadcast, msg.channel);
        bool isActive = (convIndex == gActiveConversation);
        bool markUnread = (!msg.fromMe && !isActive);
        appendConversationMessage(convIndex, msg.fromMe, msg.text, msg.from, markUnread);
        gotMessage = true;
    }
    if (gotMessage) {
        needRedraw = true;
    }

    NavKey nk = NavKey::NONE;
    std::vector<char> textChars;   // chars que se envían a la pantalla (chat)

    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
        Keyboard_Class::KeysState st = M5Cardputer.Keyboard.keysState();

        bool isChat = (currentId == ScreenId::CHAT);

        if (isChat) {
            // --- MODO CHAT ---
            if (st.enter) {
                nk = NavKey::ENTER;   // enviar mensaje
            }
            else if (st.del) {
                // Backspace: con FN = salir, sin FN = borrar texto
                if (st.fn) {
                    nk = NavKey::ESC;     // volver al Home
                } else {
                    textChars.push_back('\b'); // backspace en el input
                }
            }
            else {
                // En chat:
                // - Si FN está apretado: usamos ; . , / como flechas (no se agregan a texto)
                // - Si NO está FN: TODO lo que llegue va a texto (incluidos ; . , /, w, s, etc.)

                if (st.fn) {
                    // FN + key => navegación
                    bool fast = st.shift;
                    for (auto c : st.word) {
                        switch (c) {
                        case ';':   // tecla con flecha ARRIBA
                        case ':':   // shift + ;
                            nk = fast ? NavKey::UP_FAST : NavKey::UP;
                            break;
                        case '.':   // tecla con flecha ABAJO
                        case '>':   // shift + .
                            nk = fast ? NavKey::DOWN_FAST : NavKey::DOWN;
                            break;
                        case ',':   // tecla con flecha IZQ => back
                        case '<':   // shift + ,
                            nk = NavKey::ESC;
                            break;
                        case '/':   // tecla con flecha DER => ENTER de navegación
                        case '?':   // shift + /
                            nk = NavKey::ENTER;
                            break;
                        default:
                            break;
                        }
                        if (nk != NavKey::NONE) {
                            break;
                        }
                    }
                } else {
                    // Sin FN: todos los caracteres son texto
                    for (auto c : st.word) {
                        textChars.push_back(c);
                    }
                }
            }
        } else {
            // --- MODO MENÚ / NO-CHAT ---
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

    // Primero pasamos las letras crudas a la pantalla actual (si le interesan)
    if (!textChars.empty() && currentScreen) {
        currentScreen->handleChars(textChars);
        needRedraw = true;
    }

    // Luego manejo de navegación
    if (nk != NavKey::NONE) {
        handleNav(nk);
    }

    redrawIfNeeded();
}

// -------------------- Global and Arduino setup/loop --------------------

ScreenManager gScreens;

void setup() {
    Serial.begin(115200);      // <<< agregar

    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);

    M5Cardputer.Display.setRotation(1);

    gMesh.begin();     // futuro: init BLE
    gScreens.begin();
    gSdReady = initStorage();
    gLastSdAttemptMs = millis();
    tryLoadConversationsOnce();
}


void loop() {
    gMesh.loop();      // futuro: event handling BLE
    gScreens.loop();
}
