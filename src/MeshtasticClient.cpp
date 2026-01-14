#include "MeshtasticClient.h"

#include <pb_decode.h>
#include <pb_encode.h>
#include "meshtastic/mesh.pb.h"

extern MeshtasticClient gMesh;

namespace {
constexpr uint32_t kDefaultBlePasskey = 123456;
uint32_t gBlePasskey = kDefaultBlePasskey;
constexpr int kScanAttempts = 3;
constexpr int kScanSeconds = 8;

class MeshClientCallbacks : public NimBLEClientCallbacks {
    void onPassKeyEntry(NimBLEConnInfo& connInfo) override {
        Serial.println("[Mesh] Passkey requested, injecting");
        NimBLEDevice::injectPassKey(connInfo, gBlePasskey);
    }

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
        Serial.printf("[Mesh] Auth complete: enc=%d auth=%d bond=%d\n",
                      connInfo.isEncrypted() ? 1 : 0,
                      connInfo.isAuthenticated() ? 1 : 0,
                      connInfo.isBonded() ? 1 : 0);
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override {
        (void)pClient;
        Serial.printf("[Mesh] Disconnected (reason=%d)\n", reason);
        gMesh.handleDisconnect(reason);
    }
};

MeshClientCallbacks gClientCallbacks;
} // namespace

// Global instance
MeshtasticClient gMesh;

MeshtasticClient::MeshtasticClient()
    : _status(ConnectionStatus::DISCONNECTED),
      _radioName(""),
      _lastError(""),
      _scanning(false),
      _connectedIndex(-1),
      _myNodeNum(0),
      _needsPull(false),
      _configRequested(false),
      _configComplete(false),
      _nextPacketId(1),
      _lastPollMs(0),
      _lastFromNum(0),
      _client(nullptr),
      _scanProgressCb(nullptr),
      _svc(nullptr),
      _charToRadio(nullptr),
      _charFromRadio(nullptr),
      _charFromNum(nullptr),
      _charLog(nullptr)
{
}

void MeshtasticClient::begin() {
    NimBLEDevice::init("Cardtastic");

    // MTU recommended by Meshtastic
    NimBLEDevice::setMTU(512);
    NimBLEDevice::setSecurityAuth(true, true, false);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY);

    _status         = ConnectionStatus::DISCONNECTED;
    _radioName      = "";
    _lastError      = "";
    _scanning       = false;
    _connectedIndex = -1;
    _myNodeNum      = 0;
    _needsPull      = false;
    _configRequested = false;
    _configComplete = false;
    _nextPacketId   = 1;
    _lastPollMs     = 0;
    _lastFromNum    = 0;
    _rxTextQueue.clear();
    _devices.clear();
    _nodes.clear();
    _channels.clear();
    _scanProgressCb = nullptr;

    _client         = nullptr;
    _svc            = nullptr;
    _charToRadio    = nullptr;
    _charFromRadio  = nullptr;
    _charFromNum    = nullptr;
    _charLog        = nullptr;
}

void MeshtasticClient::loop() {
    if (!_client || !_client->isConnected()) return;

    uint32_t now = millis();
    uint32_t pollIntervalMs = (_configRequested && !_configComplete) ? 50 : 1000;
    if (_needsPull) {
        _needsPull = false;
        pullFromRadioUntilEmpty(true);
        _lastPollMs = now;
    } else if (now - _lastPollMs > pollIntervalMs) {
        pullFromRadioUntilEmpty(false);
        _lastPollMs = now;
    }
}

// ---- Scan and devices ----

void MeshtasticClient::startScan() {
    _devices.clear();
    _lastError = "";
    _scanning  = true;
    _status    = ConnectionStatus::SCANNING;

    Serial.println("[Mesh] Starting BLE scan...");

    NimBLEScan* pScan = NimBLEDevice::getScan();

    pScan->setActiveScan(true);        // request more info (name, etc.)
    pScan->setInterval(160);           // 100 ms
    pScan->setWindow(80);              //  50 ms (<= interval)
    pScan->setDuplicateFilter(false);  // keep duplicates; we dedupe by MAC
    pScan->clearResults();             // clear previous results

    int totalFound = 0;
    static NimBLEUUID meshSvcUuid("6ba1b218-15a8-461f-9fa8-5dcae273eafd");
    for (int attempt = 0; attempt < kScanAttempts; ++attempt) {
        Serial.printf("[Mesh] Scan attempt %d/%d\n", attempt + 1, kScanAttempts);
        uint32_t startMs = millis();
        int lastRemaining = kScanSeconds;
        if (_scanProgressCb) {
            _scanProgressCb(lastRemaining, attempt + 1, kScanAttempts);
        }

        bool ok = pScan->start(kScanSeconds * 1000UL, false);
        if (!ok) {
            Serial.println("[Mesh] pScan->start() returned false");
            _scanning  = false;
            _status    = ConnectionStatus::ERROR;
            _lastError = "Scan start failed";
            return;
        }

        while (pScan->isScanning()) {
            delay(50);
            if (_scanProgressCb) {
                int elapsed = (int)((millis() - startMs) / 1000UL);
                int remaining = kScanSeconds - elapsed;
                if (remaining < 0) remaining = 0;
                if (remaining != lastRemaining) {
                    lastRemaining = remaining;
                    _scanProgressCb(remaining, attempt + 1, kScanAttempts);
                }
            }
        }
        if (_scanProgressCb) {
            _scanProgressCb(0, attempt + 1, kScanAttempts);
        }

        NimBLEScanResults results = pScan->getResults();
        int count = results.getCount();
        Serial.printf("[Mesh] Scan done, results=%d\n", count);

        for (int i = 0; i < count; ++i) {
            const NimBLEAdvertisedDevice* dev = results.getDevice(i);
            if (!dev) continue;

            std::string name = dev->getName();
            std::string addr = dev->getAddress().toString();
            bool isMesh = dev->isAdvertisingService(meshSvcUuid);

            Serial.printf("[Mesh] Dev %d: name='%s' addr=%s mesh=%d\n",
                          i, name.c_str(), addr.c_str(), isMesh ? 1 : 0);

            if (!isMesh) {
                continue;
            }

            if (name.empty()) {
                name = addr;
            }

            bool exists = false;
            for (auto& existing : _devices) {
                if (existing.id == String(addr.c_str())) {
                    exists = true;
                    if (existing.name == existing.id && name != addr) {
                        existing.name = String(name.c_str());
                    }
                    break;
                }
            }
            if (!exists) {
                MeshDeviceInfo info;
                info.name = String(name.c_str());
                info.id   = String(addr.c_str());
                _devices.push_back(info);
            }
        }

        pScan->clearResults();
        totalFound = (int)_devices.size();
        if (totalFound > 0) break;
        delay(150);
    }

    _scanning       = false;
    _connectedIndex = -1;
    _radioName      = "";

    if (totalFound == 0) {
        _status    = ConnectionStatus::ERROR;
        _lastError = "No devices";
    } else {
        _status    = ConnectionStatus::DISCONNECTED;
        _lastError = "";
    }

    Serial.printf("[Mesh] Stored %d devices in list\n", _devices.size());
}

int MeshtasticClient::getDeviceCount() const {
    return (int)_devices.size();
}

MeshDeviceInfo MeshtasticClient::getDeviceInfo(int index) const {
    MeshDeviceInfo dummy{"<invalid>", ""};
    if (index < 0 || index >= (int)_devices.size()) return dummy;
    return _devices[index];
}

int MeshtasticClient::getNodeCount() const {
    return (int)_nodes.size();
}

MeshNodeInfo MeshtasticClient::getNodeInfo(int index) const {
    MeshNodeInfo dummy;
    if (index < 0 || index >= (int)_nodes.size()) return dummy;
    return _nodes[index];
}

void MeshtasticClient::noteNode(uint32_t num) {
    MeshNodeInfo* node = getOrCreateNode(num);
    if (!node) return;
    node->lastUpdateMs = millis();
}

MeshNodeInfo* MeshtasticClient::getOrCreateNode(uint32_t num) {
    if (num == 0) return nullptr;
    for (auto& node : _nodes) {
        if (node.num == num) return &node;
    }
    MeshNodeInfo entry;
    entry.num = num;
    entry.lastUpdateMs = millis();
    _nodes.push_back(entry);
    return &_nodes.back();
}

int MeshtasticClient::getChannelCount() const {
    return (int)_channels.size();
}

MeshChannelInfo MeshtasticClient::getChannelInfo(int index) const {
    MeshChannelInfo dummy;
    if (index < 0 || index >= (int)_channels.size()) return dummy;
    return _channels[index];
}

String MeshtasticClient::channelLabel(uint8_t index) const {
    for (const auto& ch : _channels) {
        if (ch.index == (int8_t)index) {
            if (ch.name.length() > 0) return ch.name;
            break;
        }
    }
    if (index == 0) return String("LongFast");
    return String("Channel ") + index;
}

MeshChannelInfo* MeshtasticClient::getOrCreateChannel(int8_t index) {
    if (index < 0) return nullptr;
    for (auto& ch : _channels) {
        if (ch.index == index) return &ch;
    }
    MeshChannelInfo entry;
    entry.index = index;
    for (auto it = _channels.begin(); it != _channels.end(); ++it) {
        if (it->index > index) {
            _channels.insert(it, entry);
            return &(*it);
        }
    }
    _channels.push_back(entry);
    return &_channels.back();
}

void MeshtasticClient::resetConnectionState(bool clearNodes) {
    _svc           = nullptr;
    _charToRadio   = nullptr;
    _charFromRadio = nullptr;
    _charFromNum   = nullptr;
    _charLog       = nullptr;

    _status         = ConnectionStatus::DISCONNECTED;
    _connectedIndex = -1;
    _radioName      = "";
    _myNodeNum      = 0;
    _needsPull      = false;
    _configRequested = false;
    _configComplete = false;
    _nextPacketId   = 1;
    _lastPollMs     = 0;
    _lastFromNum    = 0;
    _rxTextQueue.clear();
    _channels.clear();
    if (clearNodes) {
        _nodes.clear();
    }
}

// ---- Discover Meshtastic service ----

bool MeshtasticClient::discoverMeshtasticService() {
    if (!_client || !_client->isConnected()) {
        Serial.println("[Mesh] discoverMeshtasticService: client not connected");
        _lastError = "Not connected";
        _status    = ConnectionStatus::ERROR;
        return false;
    }

    Serial.println("[Mesh] Discovering Meshtastic MeshBluetoothService...");

    static NimBLEUUID svcUuid      ("6ba1b218-15a8-461f-9fa8-5dcae273eafd");
    static NimBLEUUID toRadioUuid  ("f75c76d2-129e-4dad-a1dd-7866124401e7");
    static NimBLEUUID fromRadioUuid("2c55e69e-4993-11ed-b878-0242ac120002");
    static NimBLEUUID fromNumUuid  ("ed9da18c-a800-4f66-a670-aa7547e34453");
    static NimBLEUUID logUuid      ("5a3d6e49-06e6-4423-9944-e9de8cdf9547");

    _svc = _client->getService(svcUuid);
    if (!_svc) {
        Serial.println("[Mesh] MeshBluetoothService not found");
        _lastError = "Mesh service not found";
        _status    = ConnectionStatus::ERROR;
        return false;
    }

    _charToRadio   = _svc->getCharacteristic(toRadioUuid);
    _charFromRadio = _svc->getCharacteristic(fromRadioUuid);
    _charFromNum   = _svc->getCharacteristic(fromNumUuid);
    _charLog       = _svc->getCharacteristic(logUuid);

    if (!_charToRadio || !_charFromRadio || !_charFromNum) {
        Serial.println("[Mesh] Missing one or more core Mesh chars (ToRadio/FromRadio/FromNum)");
        _lastError = "Mesh chars missing";
        _status    = ConnectionStatus::ERROR;
        return false;
    }

    Serial.println("[Mesh] Mesh service + core characteristics OK");

    // --- Subscribe to FromNum (packet counter) ---
    if (_charFromNum->canNotify()) {
        auto cbFromNum = [this](NimBLERemoteCharacteristic* chr,
                                uint8_t* data,
                                size_t   len,
                                bool     isNotify) {
            this->handleFromNumNotify(chr, data, len, isNotify);
        };

        if (_charFromNum->subscribe(true, cbFromNum, true)) {
            Serial.println("[Mesh] Subscribed to FromNum notifications");
        } else {
            Serial.println("[Mesh] Failed to subscribe FromNum");
        }
    } else {
        Serial.println("[Mesh] FromNum characteristic cannot notify");
    }

    // --- Subscribe to LogRecord (optional, debug only) ---
    if (_charLog && _charLog->canNotify()) {
        auto cbLog = [this](NimBLERemoteCharacteristic* chr,
                            uint8_t* data,
                            size_t   len,
                            bool     isNotify) {
            this->handleLogNotify(chr, data, len, isNotify);
        };

        if (_charLog->subscribe(true, cbLog, true)) {
            Serial.println("[Mesh] Subscribed to LogRecord notifications");
        } else {
            Serial.println("[Mesh] Failed to subscribe LogRecord");
        }
    }

    Serial.println("[Mesh] Meshtastic service & subscriptions OK");
    return true;
}

// ---- Connect by index ----

bool MeshtasticClient::connectToIndex(int index) {
    if (index < 0 || index >= (int)_devices.size()) {
        _lastError = "Invalid device index";
        _status    = ConnectionStatus::ERROR;
        return false;
    }

    // Toggle: if already connected to this index, disconnect
    if (_status == ConnectionStatus::CONNECTED && _connectedIndex == index) {
        Serial.println("[Mesh] Already connected to this index, toggling -> disconnect");
        disconnect();
        return true;
    }

    // If connected to another device, disconnect first
    if (_status == ConnectionStatus::CONNECTED && _connectedIndex != index) {
        Serial.println("[Mesh] Was connected to other index, disconnecting first");
        disconnect();
    }

    MeshDeviceInfo info = _devices[index];

    Serial.printf("[Mesh] Connecting to idx=%d name='%s' addr=%s\n",
                  index,
                  info.name.c_str(),
                  info.id.c_str());

    _status         = ConnectionStatus::CONNECTING;
    _lastError      = "";
    _radioName      = info.name;
    _connectedIndex = -1;

    // Create a client if needed
    if (_client == nullptr) {
        _client = NimBLEDevice::createClient();
        Serial.println("[Mesh] Created new NimBLEClient");
    }
    _client->setClientCallbacks(&gClientCallbacks, false);

    // Build address from string "00:4b:12:b1:19:f6"
    std::string addrStr(info.id.c_str());
    NimBLEAddress addr(addrStr, BLE_ADDR_PUBLIC);

    // Blocking connect for now (MVP)
    bool ok = _client->connect(addr);
    if (!ok) {
        Serial.println("[Mesh] client->connect() returned false");

        _status         = ConnectionStatus::ERROR;
        _lastError      = "Connect failed";
        _connectedIndex = -1;
        return false;
    }

    if (!_client->isConnected()) {
        Serial.println("[Mesh] client reports not connected after connect()");
        _status         = ConnectionStatus::ERROR;
        _lastError      = "Not connected";
        _connectedIndex = -1;
        return false;
    }

    Serial.println("[Mesh] Connected OK");

    _status         = ConnectionStatus::CONNECTED;
    _connectedIndex = index;
    _lastError      = "";

    if (!_client->secureConnection()) {
        Serial.println("[Mesh] secureConnection failed");
    }

    _rxTextQueue.clear();
    _nodes.clear();
    _channels.clear();
    _needsPull = false;
    _configRequested = false;
    _configComplete = false;

    // Debug: dump all services
    debugDumpServices();

    // Discover and subscribe to MeshBluetoothService
    if (!discoverMeshtasticService()) {
        Serial.println("[Mesh] discoverMeshtasticService() failed after connect");
        // discoverMeshtasticService sets ERROR if something fails
        return false;
    }

    if (!requestConfig()) {
        Serial.println("[Mesh] Failed to request config");
    }

    return true;
}

// ---- Service/characteristic dump ----

void MeshtasticClient::debugDumpServices() {
    if (!_client || !_client->isConnected()) {
        Serial.println("[Mesh] debugDumpServices: not connected");
        return;
    }

    Serial.println("[Mesh] Discovering all services...");

    const std::vector<NimBLERemoteService*>& services = _client->getServices(true);  // true = refresh

    if (services.empty()) {
        Serial.println("[Mesh] No services found");
        return;
    }

    for (size_t i = 0; i < services.size(); ++i) {
        NimBLERemoteService* svc = services[i];
        if (!svc) continue;

        std::string svcUuid = svc->getUUID().toString();
        Serial.printf("[Mesh] Service: %s\n", svcUuid.c_str());

        const std::vector<NimBLERemoteCharacteristic*>& chars = svc->getCharacteristics(true);

        if (chars.empty()) {
            Serial.println("  [Mesh]  (no characteristics)");
            continue;
        }

        for (size_t j = 0; j < chars.size(); ++j) {
            NimBLERemoteCharacteristic* ch = chars[j];
            if (!ch) continue;

            std::string chUuid = ch->getUUID().toString();

            bool rd  = ch->canRead();
            bool wr  = ch->canWrite();
            bool wnr = ch->canWriteNoResponse();
            bool nt  = ch->canNotify();
            bool ind = ch->canIndicate();

            Serial.printf("  [Mesh]  Char: %s  R=%d W=%d WNR=%d N=%d I=%d\n",
                          chUuid.c_str(), rd, wr, wnr, nt, ind);
        }
    }
}

// ---- Disconnect ----

void MeshtasticClient::disconnect() {
    Serial.println("[Mesh] Disconnect requested");

    if (_client && _client->isConnected()) {
        Serial.println("[Mesh] Calling client->disconnect()");
        _client->disconnect();
    }

    resetConnectionState(false);
    _lastError = "";
}

void MeshtasticClient::handleDisconnect(int reason) {
    resetConnectionState(false);
    _lastError = String("Disconnected (") + reason + ")";
}

bool MeshtasticClient::popTextMessage(MeshTextMessage& out) {
    if (_rxTextQueue.empty()) return false;
    out = _rxTextQueue.front();
    _rxTextQueue.pop_front();
    return true;
}

String MeshtasticClient::nodeLabel(uint32_t nodeNum) const {
    if (nodeNum == 0) return String("Node");
    for (const auto& node : _nodes) {
        if (node.num == nodeNum) {
            if (node.shortName.length() > 0) return node.shortName;
            if (node.longName.length() > 0) return node.longName;
            break;
        }
    }
    String hex = String(nodeNum, HEX);
    hex.toUpperCase();
    return String("0x") + hex;
}

bool MeshtasticClient::requestConfig() {
    if (!_client || !_client->isConnected() || !_charToRadio) return false;
    if (_configRequested) return true;

    NimBLEConnInfo info = _client->getConnInfo();
    if (!info.isEncrypted()) {
        _client->secureConnection();
    }

    meshtastic_ToRadio tr = meshtastic_ToRadio_init_zero;
    tr.which_payload_variant = meshtastic_ToRadio_want_config_id_tag;
    tr.want_config_id = (uint32_t)millis();

    uint8_t buf[meshtastic_ToRadio_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&stream, &meshtastic_ToRadio_msg, &tr)) {
        Serial.printf("[Mesh] ToRadio(want_config) encode failed: %s\n", PB_GET_ERROR(&stream));
        return false;
    }

    bool ok = _charToRadio->writeValue(buf, stream.bytes_written, true);
    if (ok) {
        _configRequested = true;
        _configComplete = false;
        _needsPull = true;
    }
    return ok;
}

bool MeshtasticClient::sendTextMessage(const String& text, uint32_t dest, uint8_t channel) {
    if (!_client || !_client->isConnected()) {
        _lastError = "Not connected";
        return false;
    }
    if (!_charToRadio) {
        _lastError = "Mesh char missing";
        return false;
    }
    if (text.length() == 0) {
        _lastError = "Empty message";
        return false;
    }

    NimBLEConnInfo info = _client->getConnInfo();
    if (!info.isEncrypted()) {
        _client->secureConnection();
    }

    meshtastic_ToRadio tr = meshtastic_ToRadio_init_zero;
    tr.which_payload_variant = meshtastic_ToRadio_packet_tag;

    meshtastic_MeshPacket& pkt = tr.packet;
    pkt = meshtastic_MeshPacket_init_zero;
    pkt.to = dest;
    pkt.channel = channel;
    // Let the node assign a unique packet ID to avoid PhoneAPI duplicate filtering.
    pkt.id = 0;

    pkt.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    pkt.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    pkt.decoded.want_response = false;

    size_t maxLen = sizeof(pkt.decoded.payload.bytes);
    size_t len = (text.length() > maxLen) ? maxLen : text.length();
    pkt.decoded.payload.size = static_cast<pb_size_t>(len);
    memcpy(pkt.decoded.payload.bytes, text.c_str(), len);

    uint8_t buf[meshtastic_ToRadio_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&stream, &meshtastic_ToRadio_msg, &tr)) {
        Serial.printf("[Mesh] ToRadio(packet) encode failed: %s\n", PB_GET_ERROR(&stream));
        _lastError = "Encode failed";
        return false;
    }

    bool ok = _charToRadio->writeValue(buf, stream.bytes_written, true);
    Serial.printf("[Mesh] Send text len=%u -> %s\n",
                  (unsigned)len,
                  ok ? "ok" : "fail");
    if (!ok) {
        _lastError = "Write failed";
    }
    return ok;
}

void MeshtasticClient::setPasskey(uint32_t passkey) {
    gBlePasskey = passkey;
}

// ---- Notifications ----

void MeshtasticClient::handleFromNumNotify(NimBLERemoteCharacteristic* chr,
                                           uint8_t* data, size_t len, bool isNotify)
{
    (void)chr;
    (void)isNotify;

    if (data && len >= 4) {
        uint32_t val = (uint32_t)data[0] |
                       ((uint32_t)data[1] << 8) |
                       ((uint32_t)data[2] << 16) |
                       ((uint32_t)data[3] << 24);
        _lastFromNum = val;
        Serial.printf("[Mesh] FromNum notify: %lu\n", (unsigned long)val);
    } else {
        Serial.println("[Mesh] FromNum notify");
    }

    _needsPull = true;
}

void MeshtasticClient::handleLogNotify(NimBLERemoteCharacteristic* chr,
                                       uint8_t* data, size_t len, bool isNotify)
{
    (void)chr;
    (void)isNotify;

    String line;
    for (size_t i = 0; i < len; ++i) {
        line += static_cast<char>(data[i]);
    }
    Serial.print("[Mesh LOG] ");
    Serial.println(line);
}

// ---- Drain FromRadio ----

bool MeshtasticClient::decodeFromRadioAndQueue(const uint8_t* data, size_t len)
{
    static meshtastic_FromRadio fr;
    fr = meshtastic_FromRadio_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(data, len);
    if (!pb_decode(&stream, &meshtastic_FromRadio_msg, &fr)) {
        Serial.printf("[Mesh] FromRadio decode failed: %s\n", PB_GET_ERROR(&stream));
        return false;
    }

    if (fr.which_payload_variant == meshtastic_FromRadio_my_info_tag) {
        _myNodeNum = fr.my_info.my_node_num;
        noteNode(_myNodeNum);
        Serial.printf("[Mesh] My node num = %lu\n", (unsigned long)_myNodeNum);
        return true;
    }

    if (fr.which_payload_variant == meshtastic_FromRadio_config_complete_id_tag) {
        _configComplete = true;
        Serial.printf("[Mesh] Config complete (id=%lu)\n",
                      (unsigned long)fr.config_complete_id);
        return true;
    }

    if (fr.which_payload_variant == meshtastic_FromRadio_node_info_tag) {
        MeshNodeInfo* node = getOrCreateNode(fr.node_info.num);
        if (!node) return true;

        if (fr.node_info.has_user) {
            node->shortName = String(fr.node_info.user.short_name);
            node->longName  = String(fr.node_info.user.long_name);
        }

        node->lastHeard = fr.node_info.last_heard;
        node->snr = fr.node_info.snr;
        node->channel = fr.node_info.channel;
        node->via_mqtt = fr.node_info.via_mqtt;
        node->hasHops = fr.node_info.has_hops_away;
        node->hopsAway = fr.node_info.hops_away;

        if (fr.node_info.has_device_metrics) {
            node->hasDeviceMetrics = true;
            node->hasBattery = fr.node_info.device_metrics.has_battery_level;
            node->batteryLevel = fr.node_info.device_metrics.battery_level;
            node->hasVoltage = fr.node_info.device_metrics.has_voltage;
            node->voltage = fr.node_info.device_metrics.voltage;
        }

        node->lastUpdateMs = millis();
        return true;
    }

    if (fr.which_payload_variant == meshtastic_FromRadio_channel_tag) {
        const meshtastic_Channel& ch = fr.channel;
        MeshChannelInfo* entry = getOrCreateChannel(ch.index);
        if (!entry) return true;

        entry->role = ch.role;
        entry->hasSettings = ch.has_settings;
        if (ch.has_settings) {
            entry->name = String(ch.settings.name);
            entry->uplink = ch.settings.uplink_enabled;
            entry->downlink = ch.settings.downlink_enabled;
            entry->muted = ch.settings.has_module_settings && ch.settings.module_settings.is_muted;
        } else {
            entry->name = "";
            entry->uplink = false;
            entry->downlink = false;
            entry->muted = false;
        }
        return true;
    }

    if (fr.which_payload_variant != meshtastic_FromRadio_packet_tag) {
        return true;
    }

    const meshtastic_MeshPacket& pkt = fr.packet;
    if (pkt.which_payload_variant != meshtastic_MeshPacket_decoded_tag) {
        Serial.printf("[Mesh] Packet not decoded (variant=%u)\n",
                      (unsigned)pkt.which_payload_variant);
        return true;
    }

    if (pkt.from != 0) {
        MeshNodeInfo* node = getOrCreateNode(pkt.from);
        if (node) {
            node->snr = pkt.rx_snr;
            node->channel = static_cast<uint8_t>(pkt.channel);
            node->lastUpdateMs = millis();
        }
    }

    const meshtastic_Data& decoded = pkt.decoded;
    Serial.printf("[Mesh] Packet from=0x%lX to=0x%lX ch=%u port=%u len=%u\n",
                  (unsigned long)pkt.from,
                  (unsigned long)pkt.to,
                  (unsigned)pkt.channel,
                  (unsigned)decoded.portnum,
                  (unsigned)decoded.payload.size);

    if (decoded.portnum != meshtastic_PortNum_TEXT_MESSAGE_APP) {
        return true;
    }

    if (decoded.payload.size == 0) {
        return true;
    }

    String text;
    text.reserve(decoded.payload.size);
    for (size_t i = 0; i < decoded.payload.size; ++i) {
        text += static_cast<char>(decoded.payload.bytes[i]);
    }

    MeshTextMessage msg;
    msg.from   = pkt.from;
    msg.to     = pkt.to;
    msg.channel = pkt.channel;
    msg.fromMe = (_myNodeNum != 0 && pkt.from == _myNodeNum);
    msg.text   = text;

    _rxTextQueue.push_back(msg);
    return true;
}

void MeshtasticClient::pullFromRadioUntilEmpty(bool logEmpty)
{
    if (!_charFromRadio || !_client || !_client->isConnected()) {
        Serial.println("[Mesh] pullFromRadioUntilEmpty: not ready");
        return;
    }

    while (true) {
        std::string val = _charFromRadio->readValue();  // returns current buffer or empty
        if (val.empty()) {
            if (logEmpty) {
                Serial.println("[Mesh] FromRadio empty");
            }
            break;
        }

        Serial.printf("[Mesh] FromRadio %d bytes\n", (int)val.size());
        if (!decodeFromRadioAndQueue(reinterpret_cast<const uint8_t*>(val.data()), val.size())) {
            for (size_t i = 0; i < val.size(); ++i) {
                uint8_t b = static_cast<uint8_t>(val[i]);
                if (i % 16 == 0) Serial.print("\n  ");
                char buf[4];
                snprintf(buf, sizeof(buf), "%02X ", b);
                Serial.print(buf);
            }
            Serial.println();
        }
    }
}
