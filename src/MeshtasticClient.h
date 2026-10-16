#pragma once

#include <Arduino.h>
#include <deque>
#include <vector>
#include <NimBLEDevice.h>
#include "ClientState.h"

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

enum class MeshDeliveryStage {
    MESH_CONFIRMED,
    DESTINATION_ACKED,
    QUEUE_REJECTED,
    ROUTING_FAILED,
    TIMED_OUT,
    DISCONNECTED,
    UNCONFIRMED
};

struct MeshDeliveryUpdate {
    uint32_t token;
    MeshDeliveryStage stage;
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
    using ScanResultsCallback = void (*)();
    void setScanResultsCallback(ScanResultsCallback cb) { _scanResultsCb = cb; }

    // ---- General state ----
    ConnectionStatus status()      const { return _status; }
    const String&    radioName()   const { return _radioName; }
    const String&    lastError()   const { return _lastError; }
    bool             authFailed() const { return _authFailed; }
    bool             awaitingPasskey() const { return _awaitingPasskey; }
    bool             isScanning()  const { return _scanning; }
    bool             isConnected() const { return _status == ConnectionStatus::CONNECTED; }
    bool             configComplete() const { return _sync.phase == MeshSyncPhase::READY; }
    bool             syncFailed() const { return _sync.phase == MeshSyncPhase::FAILED; }
    const char*      sessionStatusLabel() const;
    bool             readyToSend() const;
    bool             retryConfig();

    // ---- Scan and devices ----
    void          startScan();
    bool          discoverMeshtasticService();
    int           getDeviceCount() const;
    MeshDeviceInfo getDeviceInfo(int index) const;
    bool          autoConnectEnabled() const { return _autoConnectEnabled; }
    void          toggleAutoConnect();
    bool          connectToLastDevice();
    void          cancelAutoConnectRetries();

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

    // ---- Incoming messages (text) ----
    bool     popTextMessage(MeshTextMessage& out);
    bool     popDeliveryUpdate(MeshDeliveryUpdate& out);
    uint32_t myNodeNum() const { return _myNodeNum; }
    String   nodeLabel(uint32_t nodeNum) const;

    // ---- Outgoing messages ----
    bool requestConfig();
    bool sendTextMessage(const String& text,
                         uint32_t dest = 0xFFFFFFFF,
                         uint8_t channel = 0,
                         uint32_t deliveryToken = 0);
    void setPasskey(uint32_t passkey);
    void clearPasskey();
    bool submitPasskey(uint32_t passkey);

private:
    // --- Internal helpers ---
    void processBleEvents();
    void handleDisconnect(int reason);
    void handleConnect();
    void handleConnectFail(int reason);
    void handleAuthComplete(bool encrypted);
    void handlePasskeyRequest(uint16_t handle);
    bool startConnection(int index);
    void closeTransport();
    void processTransportClose();
    void pullFromRadioUntilEmpty(bool logEmpty = true);
    bool decodeFromRadioAndQueue(const uint8_t* data, size_t len);
    bool writeToRadio(const uint8_t* data, size_t len, const char* context);
    bool ingestScanObservation(const char* address, const char* name, uint8_t addrType);
    MeshNodeInfo* getOrCreateNode(uint32_t num);
    MeshChannelInfo* getOrCreateChannel(int8_t index);
    void resetConnectionState(bool clearNodes);
    String lookupKnownDeviceName(const String& id) const;
    void updateKnownDeviceName(const String& id, const String& name);
    void handleQueueStatus(uint32_t packetId, int result);
    void handleRoutingResult(uint32_t packetId,
                            uint32_t from,
                            uint32_t to,
                            bool hasError,
                            int errorReason);
    void finishDelivery(uint32_t token, MeshDeliveryStage stage);
    void expirePendingDeliveries(uint32_t now);
    void loadConnectionPreferences();
    void saveAutoConnectPreference();
    void saveLastConnectedDevice();
    bool attemptAutoConnect();
    void scheduleAutoConnectRetry(const char* reason);
    void processAutoConnectRetries(uint32_t now);

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
    bool                        _autoConnectEnabled;
    String                      _lastDeviceAddress;
    String                      _lastDeviceName;
    uint8_t                     _lastDeviceAddrType;
    bool                        _autoConnectStartup;
    bool                        _autoConnectRetryPending;
    uint8_t                     _autoConnectAttempts;
    uint32_t                    _autoConnectAttemptStartedMs;
    uint32_t                    _nextAutoConnectAttemptMs;
    uint32_t                    _myNodeNum;
    bool                        _needsPull;
    MeshConfigSync              _sync;
    MeshBleSession              _ble;
    int                         _nextConnectIndex = -1;
    uint32_t                    _phaseStartedMs = 0;
    uint32_t                    _closeStartedMs = 0;
    uint32_t                    _nextPacketId;
    uint32_t                    _lastPollMs;
    uint32_t                    _lastFromNum;
    std::deque<MeshTextMessage> _rxTextQueue;
    struct PendingDelivery {
        uint32_t token;
        uint32_t packetId;
        uint32_t destination;
        uint32_t startedMs;
    };
    std::vector<PendingDelivery> _pendingDeliveries;
    std::deque<MeshDeliveryUpdate> _deliveryUpdates;
    std::vector<MeshDeviceInfo> _devices;
    std::vector<MeshDeviceInfo> _knownDevices;
    std::vector<MeshNodeInfo>   _nodes;
    std::vector<MeshChannelInfo> _channels;
    NimBLEClient*               _client;
    ScanProgressCallback        _scanProgressCb = nullptr;
    ScanResultsCallback         _scanResultsCb = nullptr;

    NimBLERemoteService*        _svc;
    NimBLERemoteCharacteristic* _charToRadio;    // f75c... (write)
    NimBLERemoteCharacteristic* _charFromRadio;  // 2c55... (read)
    NimBLERemoteCharacteristic* _charFromNum;    // ed9d... (read/notify)
    NimBLERemoteCharacteristic* _charLog;        // 5a3d... (notify)
};

// Global instance (same gMesh usage as before)
extern MeshtasticClient gMesh;
