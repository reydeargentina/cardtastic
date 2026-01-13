#pragma once

#include <Arduino.h>
#include <deque>
#include <vector>
#include <NimBLEDevice.h>

// Estado de la conexión BLE con el nodo Meshtastic
enum class ConnectionStatus {
    DISCONNECTED,
    SCANNING,
    CONNECTING,
    CONNECTED,
    ERROR
};

// Info mínima de un dispositivo BLE encontrado en el scan
struct MeshDeviceInfo {
    String name;
    String id;   // MAC en texto ("00:4b:12:b1:19:f6")
};

// Info mínima de un nodo Meshtastic conocido
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

// Mensaje de texto recibido desde Meshtastic (cola para la UI)
struct MeshTextMessage {
    uint32_t from;
    uint32_t to;
    uint8_t  channel;
    bool     fromMe;
    String   text;
};

class MeshtasticClient {
public:
    MeshtasticClient();

    void begin();
    void loop();

    using ScanProgressCallback = void (*)(int secondsLeft, int attempt, int attempts);
    void setScanProgressCallback(ScanProgressCallback cb) { _scanProgressCb = cb; }

    // ---- Estado general ----
    ConnectionStatus status()      const { return _status; }
    const String&    radioName()   const { return _radioName; }
    const String&    lastError()   const { return _lastError; }
    bool             isScanning()  const { return _scanning; }
    bool             isConnected() const { return _status == ConnectionStatus::CONNECTED; }

    // ---- Escaneo y dispositivos ----
    void          startScan();
    bool          discoverMeshtasticService();
    int           getDeviceCount() const;
    MeshDeviceInfo getDeviceInfo(int index) const;

    // ---- Nodos conocidos ----
    int           getNodeCount() const;
    MeshNodeInfo  getNodeInfo(int index) const;
    void          noteNode(uint32_t num);

    // ---- Conexión / debug ----
    bool connectToIndex(int index);
    void debugDumpServices();
    void disconnect();
    void handleDisconnect(int reason);

    // ---- Mensajes entrantes (texto) ----
    bool     popTextMessage(MeshTextMessage& out);
    uint32_t myNodeNum() const { return _myNodeNum; }
    String   nodeLabel(uint32_t nodeNum) const;

    // ---- Mensajes salientes ----
    bool requestConfig();
    bool sendTextMessage(const String& text, uint32_t dest = 0xFFFFFFFF, uint8_t channel = 0);
    void setPasskey(uint32_t passkey);

private:
    // --- helpers internos ---
    void pullFromRadioUntilEmpty(bool logEmpty = true);
    bool decodeFromRadioAndQueue(const uint8_t* data, size_t len);
    MeshNodeInfo* getOrCreateNode(uint32_t num);
    void resetConnectionState(bool clearNodes);
    void handleFromNumNotify(NimBLERemoteCharacteristic* chr,
                             uint8_t* data, size_t len, bool isNotify);
    void handleLogNotify(NimBLERemoteCharacteristic* chr,
                         uint8_t* data, size_t len, bool isNotify);

    ConnectionStatus            _status;
    String                      _radioName;
    String                      _lastError;
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
    std::vector<MeshNodeInfo>   _nodes;
    NimBLEClient*               _client;
    ScanProgressCallback        _scanProgressCb = nullptr;

    NimBLERemoteService*        _svc;
    NimBLERemoteCharacteristic* _charToRadio;    // f75c... (write)
    NimBLERemoteCharacteristic* _charFromRadio;  // 2c55... (read)
    NimBLERemoteCharacteristic* _charFromNum;    // ed9d... (read/notify)
    NimBLERemoteCharacteristic* _charLog;        // 5a3d... (notify)
};

// Instancia global, igual que antes usabas gMesh
extern MeshtasticClient gMesh;
