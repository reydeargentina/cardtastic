#pragma once

#include <Arduino.h>
#include <deque>
#include <vector>
#include <NimBLEDevice.h>

// BLE connection state to the Meshtastic node
enum class ConnectionStatus {
    DISCONNECTED,
    SCANNING,
    CONNECTING,
    CONNECTED,
    ERROR
};

// Minimal info for a BLE device found during scan
struct MeshDeviceInfo {
    String name;
    String id;   // MAC address as text ("00:4b:12:b1:19:f6")
    uint8_t addrType = BLE_ADDR_PUBLIC; // Use address type from scan (public/random)
};

// Minimal info for a known Meshtastic node
struct MeshNodeInfo {
    uint32_t num = 0;
    String   longName;
    String   shortName;
    uint32_t lastHeard = 0;
    float    snr = 0.0f;
    uint8_t  channel = 0;
    bool     via_mqtt = false;
    bool     hasHops = false;
    uint8_t  hopsAway = 0;
    bool     hasDeviceMetrics = false;
    bool     hasBattery = false;
    uint32_t batteryLevel = 0;
    bool     hasVoltage = false;
    float    voltage = 0.0f;
    uint32_t lastUpdateMs = 0;
};

// Text message received from Meshtastic (queued for the UI)
struct MeshTextMessage {
    uint32_t from;
    uint32_t to;
    uint8_t  channel;
    bool     fromMe;
    String   text;
};

// Known channel info (from node config)
struct MeshChannelInfo {
    int8_t   index = -1;
    String  name;
    uint8_t role = 0;
    bool    muted = false;
    bool    uplink = false;
    bool    downlink = false;
    bool    hasSettings = false;
};

class MeshtasticClient {
public:
    MeshtasticClient();

    void begin();
    void loop();

    using ScanProgressCallback = void (*)(int secondsLeft, int attempt, int attempts);
    void setScanProgressCallback(ScanProgressCallback cb) { _scanProgressCb = cb; }

    // ---- General state ----
    ConnectionStatus status()      const { return _status; }
    const String&    radioName()   const { return _radioName; }
    const String&    lastError()   const { return _lastError; }
    bool             authFailed() const { return _authFailed; }
    bool             awaitingPasskey() const { return _awaitingPasskey; }
    bool             isScanning()  const { return _scanning; }
    bool             isConnected() const { return _status == ConnectionStatus::CONNECTED; }

    // ---- Scan and devices ----
    void          startScan();
    bool          discoverMeshtasticService();
    int           getDeviceCount() const;
    MeshDeviceInfo getDeviceInfo(int index) const;

    // ---- Known nodes ----
    int           getNodeCount() const;
    MeshNodeInfo  getNodeInfo(int index) const;
    void          noteNode(uint32_t num);

    // ---- Known channels ----
    int            getChannelCount() const;
    MeshChannelInfo getChannelInfo(int index) const;
    String         channelLabel(uint8_t index) const;

    // ---- Connection / debug ----
    bool connectToIndex(int index);
    int  connectedDeviceIndex() const { return _connectedIndex; }
    void debugDumpServices();
    void disconnect();
    void handleDisconnect(int reason);
    void handleConnect(NimBLEClient* pClient);
    void handleConnectFail(NimBLEClient* pClient, int reason);
    void handleAuthComplete(const NimBLEConnInfo& connInfo);

    // ---- Incoming messages (text) ----
    bool     popTextMessage(MeshTextMessage& out);
    uint32_t myNodeNum() const { return _myNodeNum; }
    String   nodeLabel(uint32_t nodeNum) const;

    // ---- Outgoing messages ----
    bool requestConfig();
    bool sendTextMessage(const String& text, uint32_t dest = 0xFFFFFFFF, uint8_t channel = 0);
    void setPasskey(uint32_t passkey);
    void clearPasskey();
    bool submitPasskey(uint32_t passkey);
    void handlePasskeyRequest(const NimBLEConnInfo& connInfo);

private:
    // --- Internal helpers ---
    void pullFromRadioUntilEmpty(bool logEmpty = true);
    bool decodeFromRadioAndQueue(const uint8_t* data, size_t len);
    MeshNodeInfo* getOrCreateNode(uint32_t num);
    MeshChannelInfo* getOrCreateChannel(int8_t index);
    void resetConnectionState(bool clearNodes);
    String lookupKnownDeviceName(const String& id) const;
    void updateKnownDeviceName(const String& id, const String& name);
    void handleFromNumNotify(NimBLERemoteCharacteristic* chr,
                             uint8_t* data, size_t len, bool isNotify);
    void handleLogNotify(NimBLERemoteCharacteristic* chr,
                         uint8_t* data, size_t len, bool isNotify);

    ConnectionStatus            _status;
    String                      _radioName;
    String                      _lastError;
    bool                        _authFailed;
    bool                        _awaitingPasskey;
    bool                        _autoPasskeyEnabled;
    uint32_t                    _autoPasskey;
    uint16_t                    _pendingPasskeyHandle;
    bool                        _needsServiceDiscovery;
    int                         _pendingConnectIndex;
    bool                        _scanning;
    int                         _connectedIndex;
    uint32_t                    _myNodeNum;
    bool                        _needsPull;
    bool                        _configRequested;
    bool                        _configComplete;
    uint32_t                    _nextPacketId;
    uint32_t                    _lastPollMs;
    uint32_t                    _lastFromNum;
    std::deque<MeshTextMessage> _rxTextQueue;
    std::vector<MeshDeviceInfo> _devices;
    std::vector<MeshDeviceInfo> _knownDevices;
    std::vector<MeshNodeInfo>   _nodes;
    std::vector<MeshChannelInfo> _channels;
    NimBLEClient*               _client;
    ScanProgressCallback        _scanProgressCb = nullptr;

    NimBLERemoteService*        _svc;
    NimBLERemoteCharacteristic* _charToRadio;    // f75c... (write)
    NimBLERemoteCharacteristic* _charFromRadio;  // 2c55... (read)
    NimBLERemoteCharacteristic* _charFromNum;    // ed9d... (read/notify)
    NimBLERemoteCharacteristic* _charLog;        // 5a3d... (notify)
};

// Global instance (same gMesh usage as before)
extern MeshtasticClient gMesh;
