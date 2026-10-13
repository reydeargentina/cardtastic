#include "MeshtasticClient.h"

#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include "meshtastic/mesh.pb.h"

extern MeshtasticClient gMesh;

namespace {
constexpr int kScanAttempts = 3;
constexpr int kScanSeconds = 8;
constexpr uint8_t kMaxAutoConnectAttempts = 3;
constexpr uint32_t kAutoConnectAttemptTimeoutMs = 15000;
constexpr uint32_t kAutoConnectRetryDelaysMs[] = {1000, 2500};
constexpr uint8_t kMaxConfigAttempts = 4;
constexpr uint32_t kConfigRetryMs = 1000;
constexpr uint32_t kConfigRequestTimeoutMs = 4000;
constexpr size_t kScanObservationCapacity = 24;

struct ScanObservation {
    char address[18];
    char name[64];
    uint8_t addrType;
};

ScanObservation gScanObservations[kScanObservationCapacity];
volatile size_t gScanObservationRead = 0;
volatile size_t gScanObservationWrite = 0;
volatile size_t gScanObservationCount = 0;
portMUX_TYPE gScanObservationMux = portMUX_INITIALIZER_UNLOCKED;

bool popScanObservation(ScanObservation& out) {
    bool found = false;
    portENTER_CRITICAL(&gScanObservationMux);
    if (gScanObservationCount > 0) {
        out = gScanObservations[gScanObservationRead];
        gScanObservationRead = (gScanObservationRead + 1) % kScanObservationCapacity;
        --gScanObservationCount;
        found = true;
    }
    portEXIT_CRITICAL(&gScanObservationMux);
    return found;
}

class MeshScanCallbacks : public NimBLEScanCallbacks {
    void queueObservation(const NimBLEAdvertisedDevice* dev) {
        if (!dev) return;
        static NimBLEUUID meshSvcUuid("6ba1b218-15a8-461f-9fa8-5dcae273eafd");
        if (!dev->isAdvertisingService(meshSvcUuid)) return;

        std::string address = dev->getAddress().toString();
        std::string name = dev->getName();
        if (name.empty()) name = dev->getPayloadByType(0x09);
        if (name.empty()) name = dev->getPayloadByType(0x08);

        ScanObservation observation{};
        snprintf(observation.address, sizeof(observation.address), "%s", address.c_str());
        snprintf(observation.name, sizeof(observation.name), "%s", name.c_str());
        observation.addrType = dev->getAddressType();

        portENTER_CRITICAL(&gScanObservationMux);
        if (gScanObservationCount < kScanObservationCapacity) {
            gScanObservations[gScanObservationWrite] = observation;
            gScanObservationWrite = (gScanObservationWrite + 1) % kScanObservationCapacity;
            ++gScanObservationCount;
        }
        portEXIT_CRITICAL(&gScanObservationMux);
    }

    void onDiscovered(const NimBLEAdvertisedDevice* dev) override {
        queueObservation(dev);
    }

    void onResult(const NimBLEAdvertisedDevice* dev) override {
        queueObservation(dev);
    }
};

class MeshClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override {
        gMesh.handleConnect(pClient);
    }

    void onConnectFail(NimBLEClient* pClient, int reason) override {
        gMesh.handleConnectFail(pClient, reason);
    }

    void onPassKeyEntry(NimBLEConnInfo& connInfo) override {
        Serial.println("[Mesh] Passkey requested");
        gMesh.handlePasskeyRequest(connInfo);
    }

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
        Serial.printf("[Mesh] Auth complete: enc=%d auth=%d bond=%d\n",
                      connInfo.isEncrypted() ? 1 : 0,
                      connInfo.isAuthenticated() ? 1 : 0,
                      connInfo.isBonded() ? 1 : 0);
        gMesh.handleAuthComplete(connInfo);
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override {
        (void)pClient;
        Serial.printf("[Mesh] Disconnected (reason=%d)\n", reason);
        gMesh.handleDisconnect(reason);
    }
};

MeshClientCallbacks gClientCallbacks;
MeshScanCallbacks gScanCallbacks;
} // namespace

// Global instance
MeshtasticClient gMesh;

MeshtasticClient::MeshtasticClient()
    : _status(ConnectionStatus::DISCONNECTED),
      _radioName(""),
      _lastError(""),
      _authFailed(false),
      _awaitingPasskey(false),
      _autoPasskeyEnabled(false),
      _autoPasskey(0),
      _pendingPasskeyHandle(0xFFFF),
      _needsServiceDiscovery(false),
      _pendingConnectIndex(-1),
      _scanning(false),
      _connectedIndex(-1),
      _autoConnectEnabled(true),
      _lastDeviceAddress(""),
      _lastDeviceName(""),
      _lastDeviceAddrType(BLE_ADDR_PUBLIC),
      _autoConnectStartup(false),
      _autoConnectRetryPending(false),
      _autoConnectAttempts(0),
      _autoConnectAttemptStartedMs(0),
      _nextAutoConnectAttemptMs(0),
      _myNodeNum(0),
      _needsPull(false),
      _configRequested(false),
      _configComplete(false),
      _configAttempts(0),
      _configRequestMs(0),
      _nextConfigAttemptMs(0),
      _nextPacketId(1),
      _lastPollMs(0),
      _lastFromNum(0),
      _awaitingQueueStatus(),
      _pendingDeliveries(),
      _deliveryUpdates(),
      _client(nullptr),
      _scanProgressCb(nullptr),
      _scanResultsCb(nullptr),
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
    _authFailed     = false;
    _awaitingPasskey = false;
    _autoPasskeyEnabled = false;
    _autoPasskey = 0;
    _pendingPasskeyHandle = 0xFFFF;
    _needsServiceDiscovery = false;
    _pendingConnectIndex = -1;
    _scanning       = false;
    _connectedIndex = -1;
    _autoConnectStartup = false;
    _autoConnectRetryPending = false;
    _autoConnectAttempts = 0;
    _autoConnectAttemptStartedMs = 0;
    _nextAutoConnectAttemptMs = 0;
    _myNodeNum      = 0;
    _needsPull      = false;
    _configRequested = false;
    _configComplete = false;
    _configAttempts = 0;
    _configRequestMs = 0;
    _nextConfigAttemptMs = 0;
    _nextPacketId   = 1;
    _lastPollMs     = 0;
    _lastFromNum    = 0;
    _rxTextQueue.clear();
    _awaitingQueueStatus.clear();
    _pendingDeliveries.clear();
    _deliveryUpdates.clear();
    _devices.clear();
    _nodes.clear();
    _channels.clear();
    _knownDevices.clear();
    _scanProgressCb = nullptr;
    _scanResultsCb = nullptr;
    loadConnectionPreferences();

    _client         = nullptr;
    _svc            = nullptr;
    _charToRadio    = nullptr;
    _charFromRadio  = nullptr;
    _charFromNum    = nullptr;
    _charLog        = nullptr;
}

void MeshtasticClient::loop() {
    uint32_t now = millis();
    processAutoConnectRetries(now);
    if (!_client || !_client->isConnected()) {
        expirePendingDeliveries(now);
        return;
    }

    if (_awaitingPasskey) {
        expirePendingDeliveries(now);
        return;
    }

    if (_needsServiceDiscovery) {
        NimBLEConnInfo info = _client->getConnInfo();
        if (!info.isEncrypted()) {
            return;
        }
        _needsServiceDiscovery = false;

        // Debug: dump all services
        debugDumpServices();

        // Discover and subscribe to MeshBluetoothService
        if (!discoverMeshtasticService()) {
            Serial.println("[Mesh] discoverMeshtasticService() failed after connect");
            if (_client && _client->isConnected()) {
                _client->disconnect();
            }
            resetConnectionState(false);
            _status = ConnectionStatus::ERROR;
            _lastError = "Mesh service discovery failed";
            scheduleAutoConnectRetry("Meshtastic service discovery failed");
            return;
        }
        saveLastConnectedDevice();
        _autoConnectStartup = false;
        _autoConnectRetryPending = false;
        _configAttempts = 0;
        _nextConfigAttemptMs = now;
    }

    if (!_configComplete) {
        if (_configRequested) {
            if (_configRequestMs > 0 && (now - _configRequestMs) > kConfigRequestTimeoutMs) {
                Serial.println("[Mesh] Config request timed out");
                _configRequested = false;
                _configRequestMs = 0;
                _configAttempts++;
                _nextConfigAttemptMs = now + kConfigRetryMs;
                _lastError = "Config timeout";
            }
        } else if (_configAttempts < kMaxConfigAttempts && now >= _nextConfigAttemptMs) {
            if (!requestConfig()) {
                _configAttempts++;
                _nextConfigAttemptMs = now + kConfigRetryMs;
                Serial.printf("[Mesh] requestConfig failed (attempt %u/%u)\n",
                              (unsigned)_configAttempts,
                              (unsigned)kMaxConfigAttempts);
                _lastError = "Config request failed";
            }
        }
    }

    uint32_t pollIntervalMs = (_configRequested && !_configComplete) ? 50 : 1000;
    if (_needsPull) {
        _needsPull = false;
        pullFromRadioUntilEmpty(true);
        _lastPollMs = now;
    } else if (now - _lastPollMs > pollIntervalMs) {
        pullFromRadioUntilEmpty(false);
        _lastPollMs = now;
    }

    expirePendingDeliveries(millis());
}

// ---- Scan and devices ----

bool MeshtasticClient::ingestScanObservation(const char* address,
                                            const char* name,
                                            uint8_t addrType) {
    if (!address || address[0] == '\0') return false;

    String addrStr(address);
    String nameStr(name ? name : "");
    if (nameStr.length() == 0) {
        nameStr = lookupKnownDeviceName(addrStr);
    }
    if (nameStr.length() == 0 && addrStr == _lastDeviceAddress) {
        nameStr = _lastDeviceName;
    }
    if (nameStr.length() == 0) {
        nameStr = addrStr;
    }

    for (auto& existing : _devices) {
        if (existing.id != addrStr) continue;

        bool changed = false;
        if (existing.addrType != addrType) {
            existing.addrType = addrType;
            changed = true;
        }
        if (existing.name == existing.id && nameStr != addrStr) {
            existing.name = nameStr;
            changed = true;
            Serial.printf("[Mesh] BLE name resolved: '%s' (%s)\n",
                          nameStr.c_str(), addrStr.c_str());
        }
        if (nameStr != addrStr) {
            updateKnownDeviceName(addrStr, nameStr);
        }
        return changed;
    }

    MeshDeviceInfo info;
    info.name = nameStr;
    info.id = addrStr;
    info.addrType = addrType;
    _devices.push_back(info);
    if (nameStr != addrStr) {
        updateKnownDeviceName(addrStr, nameStr);
    }
    Serial.printf("[Mesh] Found Meshtastic device: name='%s' addr=%s type=%u\n",
                  info.name.c_str(),
                  info.id.c_str(),
                  (unsigned)info.addrType);
    return true;
}

void MeshtasticClient::startScan() {
    _devices.clear();
    ScanObservation staleObservation;
    while (popScanObservation(staleObservation)) {
    }
    _lastError = "";
    _scanning  = true;
    _status    = ConnectionStatus::SCANNING;

    Serial.println("[Mesh] Starting BLE scan...");

    NimBLEScan* pScan = NimBLEDevice::getScan();

    pScan->setActiveScan(true);        // request more info (name, etc.)
    pScan->setInterval(160);           // 100 ms
    pScan->setWindow(80);              //  50 ms (<= interval)
    pScan->setScanCallbacks(&gScanCallbacks, true); // accept duplicates so scan responses can supply the name
    pScan->clearResults();             // clear previous results

    int totalFound = 0;
    for (int attempt = 0; attempt < kScanAttempts; ++attempt) {
        Serial.printf("[Mesh] Scan attempt %d/%d\n", attempt + 1, kScanAttempts);
        uint32_t startMs = millis();
        uint32_t lastResultsCheckMs = 0;
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
            uint32_t now = millis();
            if (now - lastResultsCheckMs >= 50) {
                lastResultsCheckMs = now;
                ScanObservation observation;
                bool changed = false;
                while (popScanObservation(observation)) {
                    changed = ingestScanObservation(observation.address,
                                                    observation.name,
                                                    observation.addrType) || changed;
                }
                if (changed && _scanResultsCb) {
                    _scanResultsCb();
                }
            }
            if (_scanProgressCb) {
                int elapsed = (int)((now - startMs) / 1000UL);
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

        ScanObservation observation;
        bool changed = false;
        while (popScanObservation(observation)) {
            changed = ingestScanObservation(observation.address,
                                            observation.name,
                                            observation.addrType) || changed;
        }
        if (changed && _scanResultsCb) {
            _scanResultsCb();
        }
        Serial.printf("[Mesh] Scan done, Meshtastic devices=%u\n", (unsigned)_devices.size());

        pScan->clearResults();
        totalFound = (int)_devices.size();
        bool anyUnnamed = false;
        for (const auto& dev : _devices) {
            if (dev.name == dev.id) {
                anyUnnamed = true;
                break;
            }
        }
        if (totalFound > 0 && !anyUnnamed) break;
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
    MeshDeviceInfo dummy;
    dummy.name = "<invalid>";
    dummy.id = "";
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

String MeshtasticClient::lookupKnownDeviceName(const String& id) const {
    for (const auto& dev : _knownDevices) {
        if (dev.id == id) {
            return dev.name;
        }
    }
    return String();
}

void MeshtasticClient::loadConnectionPreferences() {
    Preferences prefs;
    if (!prefs.begin("cardtastic", true)) {
        Serial.println("[Mesh] Could not open connection preferences");
        return;
    }
    _autoConnectEnabled = prefs.getBool("autoConnect", true);
    _lastDeviceAddress = prefs.getString("lastAddress", "");
    _lastDeviceName = prefs.getString("lastName", "");
    _lastDeviceAddrType = prefs.getUChar("addrType", BLE_ADDR_PUBLIC);
    prefs.end();

    Serial.printf("[Mesh] Auto-connect %s; saved device='%s' addr=%s type=%u\n",
                  _autoConnectEnabled ? "enabled" : "disabled",
                  _lastDeviceName.c_str(),
                  _lastDeviceAddress.c_str(),
                  (unsigned)_lastDeviceAddrType);
}

void MeshtasticClient::saveAutoConnectPreference() {
    Preferences prefs;
    if (!prefs.begin("cardtastic", false)) {
        Serial.println("[Mesh] Could not save auto-connect preference");
        return;
    }
    prefs.putBool("autoConnect", _autoConnectEnabled);
    prefs.end();
}

void MeshtasticClient::toggleAutoConnect() {
    _autoConnectEnabled = !_autoConnectEnabled;
    saveAutoConnectPreference();
    if (!_autoConnectEnabled) {
        cancelAutoConnectRetries();
    }
    Serial.printf("[Mesh] Auto-connect %s\n", _autoConnectEnabled ? "enabled" : "disabled");
}

bool MeshtasticClient::connectToLastDevice() {
    if (!_autoConnectEnabled || _lastDeviceAddress.length() == 0) {
        Serial.println("[Mesh] Auto-connect skipped (disabled or no saved device)");
        return false;
    }

    MeshDeviceInfo info;
    info.name = _lastDeviceName.length() > 0 ? _lastDeviceName : _lastDeviceAddress;
    info.id = _lastDeviceAddress;
    info.addrType = _lastDeviceAddrType;
    _devices.clear();
    _devices.push_back(info);

    _autoConnectStartup = true;
    _autoConnectRetryPending = false;
    _autoConnectAttempts = 0;
    _autoConnectAttemptStartedMs = 0;
    _nextAutoConnectAttemptMs = millis();
    return attemptAutoConnect();
}

bool MeshtasticClient::attemptAutoConnect() {
    if (!_autoConnectStartup || !_autoConnectEnabled || _devices.empty()) return false;
    if (_autoConnectAttempts >= kMaxAutoConnectAttempts) {
        _autoConnectStartup = false;
        _autoConnectRetryPending = false;
        Serial.println("[Mesh] Auto-connect retries exhausted; manual connection remains available");
        return false;
    }

    ++_autoConnectAttempts;
    _autoConnectRetryPending = false;
    _autoConnectAttemptStartedMs = millis();
    const MeshDeviceInfo& info = _devices[0];
    Serial.printf("[Mesh] Auto-connect attempt %u/%u: '%s' (%s, type=%u)\n",
                  (unsigned)_autoConnectAttempts,
                  (unsigned)kMaxAutoConnectAttempts,
                  info.name.c_str(),
                  info.id.c_str(),
                  (unsigned)info.addrType);

    bool ok = connectToIndex(0);
    if (!ok) {
        scheduleAutoConnectRetry("connect start failed");
    }
    return ok;
}

void MeshtasticClient::scheduleAutoConnectRetry(const char* reason) {
    if (!_autoConnectStartup || !_autoConnectEnabled || _autoConnectRetryPending) return;
    if (_autoConnectAttempts >= kMaxAutoConnectAttempts) {
        _autoConnectStartup = false;
        Serial.printf("[Mesh] Auto-connect stopped after %u attempts (%s); manual connection remains available\n",
                      (unsigned)_autoConnectAttempts,
                      reason ? reason : "unknown error");
        return;
    }

    uint8_t delayIndex = _autoConnectAttempts > 0 ? _autoConnectAttempts - 1 : 0;
    if (delayIndex >= (sizeof(kAutoConnectRetryDelaysMs) / sizeof(kAutoConnectRetryDelaysMs[0]))) {
        delayIndex = (sizeof(kAutoConnectRetryDelaysMs) / sizeof(kAutoConnectRetryDelaysMs[0])) - 1;
    }
    uint32_t delayMs = kAutoConnectRetryDelaysMs[delayIndex];
    _autoConnectRetryPending = true;
    _nextAutoConnectAttemptMs = millis() + delayMs;
    Serial.printf("[Mesh] Auto-connect retry in %lu ms (%s)\n",
                  (unsigned long)delayMs,
                  reason ? reason : "connection failed");
}

void MeshtasticClient::processAutoConnectRetries(uint32_t now) {
    if (!_autoConnectStartup || !_autoConnectEnabled) return;
    if (_awaitingPasskey) return;

    if (_autoConnectRetryPending) {
        if (now >= _nextAutoConnectAttemptMs) {
            attemptAutoConnect();
        }
        return;
    }

    bool stillConnecting = (_status == ConnectionStatus::CONNECTING) ||
                           (_status == ConnectionStatus::CONNECTED && _needsServiceDiscovery);
    if (!stillConnecting || (now - _autoConnectAttemptStartedMs) < kAutoConnectAttemptTimeoutMs) {
        return;
    }

    Serial.println("[Mesh] Auto-connect attempt timed out");
    if (_client) {
        if (_client->isConnected()) {
            _client->disconnect();
        } else {
            _client->cancelConnect();
        }
    }
    resetConnectionState(false);
    _status = ConnectionStatus::ERROR;
    _lastError = "Auto-connect timeout";
    scheduleAutoConnectRetry("attempt timeout");
}

void MeshtasticClient::cancelAutoConnectRetries() {
    _autoConnectStartup = false;
    _autoConnectRetryPending = false;
}

void MeshtasticClient::saveLastConnectedDevice() {
    if (_connectedIndex < 0 || _connectedIndex >= (int)_devices.size()) return;

    const MeshDeviceInfo& info = _devices[_connectedIndex];
    if (_lastDeviceAddress != info.id) {
        _lastDeviceName = "";
    }
    _lastDeviceAddress = info.id;
    _lastDeviceAddrType = info.addrType;
    if (info.name.length() > 0 && info.name != info.id) {
        _lastDeviceName = info.name;
    }

    Preferences prefs;
    if (!prefs.begin("cardtastic", false)) {
        Serial.println("[Mesh] Could not save last connected device");
        return;
    }
    prefs.putString("lastAddress", _lastDeviceAddress);
    prefs.putString("lastName", _lastDeviceName);
    prefs.putUChar("addrType", _lastDeviceAddrType);
    prefs.end();

    Serial.printf("[Mesh] Saved last device '%s' (%s, type=%u)\n",
                  _lastDeviceName.c_str(),
                  _lastDeviceAddress.c_str(),
                  (unsigned)_lastDeviceAddrType);
}

void MeshtasticClient::updateKnownDeviceName(const String& id, const String& name) {
    if (name.length() == 0 || name == id) {
        return;
    }
    for (auto& dev : _knownDevices) {
        if (dev.id == id) {
            if (dev.name != name) {
                dev.name = name;
            }
            if (id == _lastDeviceAddress && _lastDeviceName != name) {
                _lastDeviceName = name;
                Preferences prefs;
                if (prefs.begin("cardtastic", false)) {
                    prefs.putString("lastName", _lastDeviceName);
                    prefs.end();
                }
            }
            return;
        }
    }
    MeshDeviceInfo info;
    info.id = id;
    info.name = name;
    _knownDevices.push_back(info);

    if (id == _lastDeviceAddress && _lastDeviceName != name) {
        _lastDeviceName = name;
        Preferences prefs;
        if (prefs.begin("cardtastic", false)) {
            prefs.putString("lastName", _lastDeviceName);
            prefs.end();
        }
    }
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
    _configAttempts = 0;
    _configRequestMs = 0;
    _nextConfigAttemptMs = 0;
    _nextPacketId   = 1;
    _lastPollMs     = 0;
    _lastFromNum    = 0;
    _rxTextQueue.clear();
    _channels.clear();
    _authFailed = false;
    _awaitingPasskey = false;
    _pendingPasskeyHandle = 0xFFFF;
    _needsServiceDiscovery = false;
    _pendingConnectIndex = -1;
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
    _authFailed = false;

    if (_client && _client->isConnected() && _status != ConnectionStatus::CONNECTED) {
        Serial.println("[Mesh] Client connected while status not CONNECTED, disconnecting");
        disconnect();
    }

    // Toggle: if already connected to this index, disconnect
    if (_status == ConnectionStatus::CONNECTED && _connectedIndex == index) {
        Serial.println("[Mesh] Already connected to this index, toggling -> disconnect");
        disconnect();
        return true;
    }

    if (_status == ConnectionStatus::CONNECTING && _client) {
        _client->cancelConnect();
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
    _radioName      = info.name.length() ? info.name : info.id;
    _connectedIndex = -1;
    _pendingConnectIndex = index;

    // Create a client if needed
    if (_client == nullptr) {
        _client = NimBLEDevice::createClient();
        Serial.println("[Mesh] Created new NimBLEClient");
    }
    _client->setClientCallbacks(&gClientCallbacks, false);

    // Build address from string "00:4b:12:b1:19:f6"
    std::string addrStr(info.id.c_str());
    NimBLEAddress addr(addrStr, info.addrType);

    // Async connect to keep UI responsive during pairing
    bool ok = _client->connect(addr, true, true);
    if (!ok) {
        Serial.println("[Mesh] client->connect(async) returned false");
        _status               = ConnectionStatus::ERROR;
        _lastError            = "Connect failed";
        _connectedIndex       = -1;
        _pendingConnectIndex  = -1;
        return false;
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

    if (_client) {
        if (_client->isConnected()) {
            Serial.println("[Mesh] Calling client->disconnect()");
            _client->disconnect();
        } else if (_status == ConnectionStatus::CONNECTING) {
            Serial.println("[Mesh] Cancelling pending connect");
            _client->cancelConnect();
        }
    }

    resetConnectionState(false);
    _lastError = "";
}

void MeshtasticClient::handleDisconnect(int reason) {
    bool wasAuthFailed = _authFailed;
    String priorError = _lastError;
    bool shouldRetryAutoConnect = _autoConnectStartup && !wasAuthFailed;
    resetConnectionState(false);
    if (wasAuthFailed) {
        _autoConnectStartup = false;
        _autoConnectRetryPending = false;
        _authFailed = true;
        _status = ConnectionStatus::ERROR;
        _lastError = priorError.length() > 0 ? priorError : "Auth failed";
    } else {
        _lastError = String("Disconnected (") + reason + ")";
        if (shouldRetryAutoConnect) {
            scheduleAutoConnectRetry("BLE disconnected during startup");
        }
    }
}

void MeshtasticClient::handleConnect(NimBLEClient* pClient) {
    if (!pClient || !pClient->isConnected()) {
        _status = ConnectionStatus::ERROR;
        _lastError = "Connect failed";
        _pendingConnectIndex = -1;
        scheduleAutoConnectRetry("invalid connect callback");
        return;
    }

    Serial.println("[Mesh] Connected OK");
    _status         = ConnectionStatus::CONNECTED;
    _connectedIndex = _pendingConnectIndex;
    _pendingConnectIndex = -1;
    _lastError      = "";
    _authFailed     = false;
    _awaitingPasskey = false;
    _pendingPasskeyHandle = 0xFFFF;
    _needsServiceDiscovery = true;

    if (!pClient->secureConnection(true)) {
        Serial.println("[Mesh] secureConnection async failed");
    }

    _rxTextQueue.clear();
    _nodes.clear();
    _channels.clear();
    _needsPull = false;
    _configRequested = false;
    _configComplete = false;
    _configAttempts = 0;
    _configRequestMs = 0;
    _nextConfigAttemptMs = 0;
}

void MeshtasticClient::handleConnectFail(NimBLEClient* pClient, int reason) {
    (void)pClient;
    Serial.printf("[Mesh] Connect failed (reason=%d)\n", reason);
    _status               = ConnectionStatus::ERROR;
    _lastError            = String("Connect failed (") + reason + ")";
    _connectedIndex       = -1;
    _pendingConnectIndex  = -1;
    _awaitingPasskey      = false;
    _pendingPasskeyHandle = 0xFFFF;
    _needsServiceDiscovery = false;
    scheduleAutoConnectRetry("BLE connection failed");
}

void MeshtasticClient::handleAuthComplete(const NimBLEConnInfo& connInfo) {
    _awaitingPasskey = false;
    if (!connInfo.isEncrypted()) {
        _authFailed = true;
        _autoConnectStartup = false;
        _autoConnectRetryPending = false;
        _status = ConnectionStatus::ERROR;
        _lastError = "Auth failed";
        if (_client && _client->isConnected()) {
            _client->disconnect();
        }
    } else {
        _authFailed = false;
    }
}

bool MeshtasticClient::popTextMessage(MeshTextMessage& out) {
    if (_rxTextQueue.empty()) return false;
    out = _rxTextQueue.front();
    _rxTextQueue.pop_front();
    return true;
}

bool MeshtasticClient::popDeliveryUpdate(MeshDeliveryUpdate& out) {
    if (_deliveryUpdates.empty()) return false;
    out = _deliveryUpdates.front();
    _deliveryUpdates.pop_front();
    return true;
}

void MeshtasticClient::handleQueueStatus(uint32_t packetId, int result) {
    if (packetId == 0) return;

    auto pending = _pendingDeliveries.end();
    for (auto it = _pendingDeliveries.begin(); it != _pendingDeliveries.end(); ++it) {
        if (it->packetId == packetId) {
            pending = it;
            break;
        }
    }

    if (pending == _pendingDeliveries.end()) {
        while (!_awaitingQueueStatus.empty()) {
            uint32_t token = _awaitingQueueStatus.front();
            _awaitingQueueStatus.pop_front();
            for (auto it = _pendingDeliveries.begin(); it != _pendingDeliveries.end(); ++it) {
                if (it->token == token && it->packetId == 0) {
                    it->packetId = packetId;
                    pending = it;
                    break;
                }
            }
            if (pending != _pendingDeliveries.end()) break;
        }
    }

    if (pending == _pendingDeliveries.end()) return;

    Serial.printf("[Mesh] Delivery tracking: token=%lu packet_id=%lu queue_result=%d\n",
                  (unsigned long)pending->token,
                  (unsigned long)packetId,
                  result);
    if (result != 0) {
        finishDelivery(pending->token, MeshDeliveryStage::FAILED);
    }
}

void MeshtasticClient::handleRoutingResult(uint32_t packetId,
                                            uint32_t from,
                                            uint32_t to,
                                            bool hasError,
                                            int errorReason) {
    if (packetId == 0) return;

    for (auto it = _pendingDeliveries.begin(); it != _pendingDeliveries.end(); ++it) {
        if (it->packetId != packetId) continue;

        if (hasError && errorReason != meshtastic_Routing_Error_NONE) {
            Serial.printf("[Mesh] Delivery failed: token=%lu packet_id=%lu reason=%d\n",
                          (unsigned long)it->token,
                          (unsigned long)packetId,
                          errorReason);
            finishDelivery(it->token, MeshDeliveryStage::FAILED);
            return;
        }

        if (from == _myNodeNum && to == _myNodeNum) {
            Serial.printf("[Mesh] Mesh relay confirmed: token=%lu packet_id=%lu\n",
                          (unsigned long)it->token,
                          (unsigned long)packetId);
            _deliveryUpdates.push_back({it->token, MeshDeliveryStage::MESH_CONFIRMED});
            if (it->destination == 0xFFFFFFFF) {
                _pendingDeliveries.erase(it);
            }
            return;
        }

        if (it->destination != 0xFFFFFFFF && from == it->destination) {
            Serial.printf("[Mesh] Destination ACK: token=%lu packet_id=%lu from=0x%08lX\n",
                          (unsigned long)it->token,
                          (unsigned long)packetId,
                          (unsigned long)from);
            finishDelivery(it->token, MeshDeliveryStage::DESTINATION_ACKED);
            return;
        }
    }
}

void MeshtasticClient::finishDelivery(uint32_t token, MeshDeliveryStage stage) {
    _deliveryUpdates.push_back({token, stage});
    for (auto it = _pendingDeliveries.begin(); it != _pendingDeliveries.end(); ++it) {
        if (it->token == token) {
            _pendingDeliveries.erase(it);
            break;
        }
    }
    for (auto it = _awaitingQueueStatus.begin(); it != _awaitingQueueStatus.end();) {
        if (*it == token) {
            it = _awaitingQueueStatus.erase(it);
        } else {
            ++it;
        }
    }
}

void MeshtasticClient::expirePendingDeliveries(uint32_t now) {
    constexpr uint32_t kDeliveryTimeoutMs = 90000;
    for (size_t i = 0; i < _pendingDeliveries.size();) {
        if (now - _pendingDeliveries[i].startedMs < kDeliveryTimeoutMs) {
            ++i;
            continue;
        }
        uint32_t token = _pendingDeliveries[i].token;
        Serial.printf("[Mesh] Delivery timeout: token=%lu packet_id=%lu\n",
                      (unsigned long)token,
                      (unsigned long)_pendingDeliveries[i].packetId);
        finishDelivery(token, MeshDeliveryStage::FAILED);
        i = 0;
    }
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

bool MeshtasticClient::writeToRadio(const uint8_t* data, size_t len, const char* context) {
    if (!_client || !_client->isConnected()) {
        _lastError = "Not connected";
        return false;
    }
    if (!_charToRadio) {
        _lastError = "Mesh char missing";
        return false;
    }

    bool canWrite = _charToRadio->canWrite();
    bool canWriteNr = _charToRadio->canWriteNoResponse();
    if (!canWrite && !canWriteNr) {
        _lastError = "Mesh char not writable";
        return false;
    }

    bool useResponse = canWrite;
    bool ok = _charToRadio->writeValue(data, len, useResponse);
    if (!ok) {
        _lastError = String(context) + " write failed";
    }
    return ok;
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

    bool ok = writeToRadio(buf, stream.bytes_written, "Config");
    if (ok) {
        _configRequested = true;
        _configComplete = false;
        _needsPull = true;
        _configRequestMs = millis();
        _configAttempts = 0;
        _nextConfigAttemptMs = 0;
        if (_lastError.startsWith("Config")) {
            _lastError = "";
        }
    }
    return ok;
}

bool MeshtasticClient::sendTextMessage(const String& text,
                                       uint32_t dest,
                                       uint8_t channel,
                                       uint32_t deliveryToken) {
    if (text.length() == 0) {
        _lastError = "Empty message";
        return false;
    }
    if (!_client || !_client->isConnected()) {
        _lastError = "Not connected";
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
    // ReliableRouter uses explicit ACKs for DMs and implicit ACK/retries for broadcasts.
    pkt.want_ack = true;

    size_t maxLen = sizeof(pkt.decoded.payload.bytes);
    size_t len = (text.length() > maxLen) ? maxLen : text.length();
    pkt.decoded.payload.size = static_cast<pb_size_t>(len);
    memcpy(pkt.decoded.payload.bytes, text.c_str(), len);

    Serial.printf("[Mesh] TX packet: id=%lu from=0x%08lX to=0x%08lX ch=%u port=%u payload=%u/%u want_ack=%d want_response=%d\n",
                  (unsigned long)pkt.id,
                  (unsigned long)pkt.from,
                  (unsigned long)pkt.to,
                  (unsigned)pkt.channel,
                  (unsigned)pkt.decoded.portnum,
                  (unsigned)pkt.decoded.payload.size,
                  (unsigned)sizeof(pkt.decoded.payload.bytes),
                  pkt.want_ack ? 1 : 0,
                  pkt.decoded.want_response ? 1 : 0);

    uint8_t buf[meshtastic_ToRadio_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&stream, &meshtastic_ToRadio_msg, &tr)) {
        Serial.printf("[Mesh] ToRadio(packet) encode failed: %s\n", PB_GET_ERROR(&stream));
        _lastError = "Encode failed";
        return false;
    }

    bool ok = writeToRadio(buf, stream.bytes_written, "Send");
    if (ok && deliveryToken != 0) {
        PendingDelivery pending;
        pending.token = deliveryToken;
        pending.packetId = 0;
        pending.destination = dest;
        pending.startedMs = millis();
        _pendingDeliveries.push_back(pending);
        _awaitingQueueStatus.push_back(deliveryToken);
    }
    Serial.printf("[Mesh] Send text len=%u -> %s\n",
                  (unsigned)len,
                  ok ? "ok" : "fail");
    return ok;
}

void MeshtasticClient::setPasskey(uint32_t passkey) {
    _autoPasskeyEnabled = true;
    _autoPasskey = passkey;
}

void MeshtasticClient::clearPasskey() {
    _autoPasskeyEnabled = false;
    _autoPasskey = 0;
}

bool MeshtasticClient::submitPasskey(uint32_t passkey) {
    if (_pendingPasskeyHandle == 0xFFFF) return false;
    NimBLEClient* client = NimBLEDevice::getClientByHandle(_pendingPasskeyHandle);
    if (!client || !client->isConnected()) {
        _pendingPasskeyHandle = 0xFFFF;
        _awaitingPasskey = false;
        return false;
    }
    NimBLEConnInfo info = client->getConnInfo();
    bool ok = NimBLEDevice::injectPassKey(info, passkey);
    if (ok) {
        _awaitingPasskey = false;
        _pendingPasskeyHandle = 0xFFFF;
    }
    return ok;
}

void MeshtasticClient::handlePasskeyRequest(const NimBLEConnInfo& connInfo) {
    if (_autoPasskeyEnabled) {
        Serial.println("[Mesh] Auto passkey enabled, injecting");
        NimBLEDevice::injectPassKey(connInfo, _autoPasskey);
        return;
    }
    _awaitingPasskey = true;
    _pendingPasskeyHandle = connInfo.getConnHandle();
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

    Serial.printf("[Mesh] FromRadio decoded: variant=%u len=%u\n",
                  (unsigned)fr.which_payload_variant,
                  (unsigned)len);

    if (fr.which_payload_variant != meshtastic_FromRadio_packet_tag && len <= 32) {
        Serial.print("[Mesh] FromRadio raw:");
        for (size_t i = 0; i < len; ++i) {
            Serial.printf(" %02X", data[i]);
        }
        Serial.println();
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
        if (_lastError.startsWith("Config")) {
            _lastError = "";
        }
        return true;
    }

    if (fr.which_payload_variant == meshtastic_FromRadio_queueStatus_tag) {
        Serial.printf("[Mesh] Queue status: result=%d free=%u/%u mesh_packet_id=0x%08lX (%lu)\n",
                      (int)fr.queueStatus.res,
                      (unsigned)fr.queueStatus.free,
                      (unsigned)fr.queueStatus.maxlen,
                      (unsigned long)fr.queueStatus.mesh_packet_id,
                      (unsigned long)fr.queueStatus.mesh_packet_id);
        handleQueueStatus(fr.queueStatus.mesh_packet_id, (int)fr.queueStatus.res);
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
        Serial.printf("[Mesh] Packet not decoded: id=%lu from=0x%08lX to=0x%08lX ch=%u payload_variant=%u\n",
                      (unsigned long)pkt.id,
                      (unsigned long)pkt.from,
                      (unsigned long)pkt.to,
                      (unsigned)pkt.channel,
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
    Serial.printf("[Mesh] Packet: id=%lu from=0x%08lX to=0x%08lX ch=%u port=%u len=%u request_id=%lu reply_id=%lu rx_snr=%.1f hop_limit=%u want_ack=%d\n",
                  (unsigned long)pkt.id,
                  (unsigned long)pkt.from,
                  (unsigned long)pkt.to,
                  (unsigned)pkt.channel,
                  (unsigned)decoded.portnum,
                  (unsigned)decoded.payload.size,
                  (unsigned long)decoded.request_id,
                  (unsigned long)decoded.reply_id,
                  pkt.rx_snr,
                  (unsigned)pkt.hop_limit,
                  pkt.want_ack ? 1 : 0);

    if (decoded.portnum == meshtastic_PortNum_ROUTING_APP) {
        if (decoded.payload.size == 0) {
            Serial.printf("[Mesh] Routing packet without payload: request_id=%lu reply_id=%lu\n",
                          (unsigned long)decoded.request_id,
                          (unsigned long)decoded.reply_id);
            handleRoutingResult(decoded.request_id, pkt.from, pkt.to, false,
                                meshtastic_Routing_Error_NONE);
        } else {
            meshtastic_Routing routing = meshtastic_Routing_init_zero;
            pb_istream_t routingStream = pb_istream_from_buffer(decoded.payload.bytes,
                                                                 decoded.payload.size);
            if (pb_decode(&routingStream, &meshtastic_Routing_msg, &routing)) {
                if (routing.which_variant == meshtastic_Routing_error_reason_tag) {
                    if (routing.error_reason == meshtastic_Routing_Error_NONE) {
                        Serial.printf("[Mesh] Routing ACK: request_id=%lu result=success\n",
                                      (unsigned long)decoded.request_id);
                    } else {
                        Serial.printf("[Mesh] Routing error: request_id=%lu reason=%d\n",
                                      (unsigned long)decoded.request_id,
                                      (int)routing.error_reason);
                    }
                    handleRoutingResult(decoded.request_id,
                                        pkt.from,
                                        pkt.to,
                                        true,
                                        (int)routing.error_reason);
                } else {
                    Serial.printf("[Mesh] Routing message: request_id=%lu variant=%u\n",
                                  (unsigned long)decoded.request_id,
                                  (unsigned)routing.which_variant);
                }
            } else {
                Serial.printf("[Mesh] Routing payload decode failed: %s\n",
                              PB_GET_ERROR(&routingStream));
            }
        }
    }

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

    Serial.printf("[Mesh] RX text queued: from=0x%08lX to=0x%08lX ch=%u fromMe=%d len=%u\n",
                  (unsigned long)msg.from,
                  (unsigned long)msg.to,
                  (unsigned)msg.channel,
                  msg.fromMe ? 1 : 0,
                  (unsigned)msg.text.length());
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
