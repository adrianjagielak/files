#include "tesla_transport.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <atomic>

#include "config.h"
#include "vin_utils.h"

namespace tesla_transport {
namespace {

// Tesla BLE service / characteristic UUIDs (see teslamotors/vehicle-command
// pkg/connector/ble/ble.go).
const NimBLEUUID kSvcUuid("00000211-b2d1-43f0-9b88-960cebf8b91e");
const NimBLEUUID kTxUuid("00000212-b2d1-43f0-9b88-960cebf8b91e");
const NimBLEUUID kRxUuid("00000213-b2d1-43f0-9b88-960cebf8b91e");

String g_vin;
MessageCallback g_on_msg;
ConnectionCallback g_on_conn;

NimBLEClient *g_client = nullptr;
NimBLERemoteCharacteristic *g_tx = nullptr;
NimBLERemoteCharacteristic *g_rx = nullptr;
NimBLEAdvertisedDevice g_found_device;
bool g_device_found = false;

enum class State { Idle, Scanning, Connecting, Discovering, Ready, Disconnecting };
std::atomic<State> g_state{State::Idle};

// Received-message queue. Each entry is a heap-allocated std::vector<uint8_t>*.
QueueHandle_t g_rx_queue = nullptr;

// Pending-connection flag — set by connect/disconnect callbacks, drained by loop().
std::atomic<int> g_conn_event{0};  // +1 = connected, -1 = disconnected

// Reassembly buffer (BLE callback context only).
std::vector<uint8_t> g_reasm;
size_t g_reasm_expected = 0;

uint32_t g_last_attempt_ms = 0;

// Set by onScanEnd when a device is found; tryConnect() is called from loop()
// so it runs on the main task, not the NimBLE host task.
std::atomic<bool> g_connect_pending{false};

// Forward decls.
void startScan();
void tryConnect();

// -----------------------------------------------------------------------------
// NimBLE callbacks
// -----------------------------------------------------------------------------
class ScanCb : public NimBLEScanCallbacks {
 public:
  void onResult(const NimBLEAdvertisedDevice *adv) override {
    if (!adv->haveName()) return;
    const std::string name = adv->getName();
    Serial.printf("[BLE] Seen: %s\n", name.c_str());
    if (!TeslaBLE::matches_vin(name, g_vin.c_str())) return;
    Serial.printf("[BLE] Vehicle found: %s (%s)\n", name.c_str(), adv->getAddress().toString().c_str());
    g_found_device = *adv;
    g_device_found = true;
    NimBLEDevice::getScan()->stop();
  }
  void onScanEnd(const NimBLEScanResults & /*res*/, int /*reason*/) override {
    if (g_device_found) {
      g_connect_pending = true;  // connect from main loop, not NimBLE host task
    } else if (g_state == State::Scanning) {
      Serial.printf("[BLE] Scan done — vehicle not found (is it awake?), retrying in %u s\n",
                    cfg::kConnectRetryMs / 1000);
      g_state = State::Idle;
    }
  }
};

class ClientCb : public NimBLEClientCallbacks {
 public:
  void onConnect(NimBLEClient *c) override {
    log_i("BLE connected (mtu=%u)", c->getMTU());
    c->exchangeMTU();  // request the largest MTU the controller will allow
  }
  void onMTUChange(NimBLEClient *c, uint16_t mtu) override { log_i("MTU negotiated: %u", mtu); }
  void onDisconnect(NimBLEClient * /*c*/, int reason) override {
    log_w("BLE disconnected (reason=%d)", reason);
    g_tx = nullptr;
    g_rx = nullptr;
    g_state = State::Idle;
    g_reasm.clear();
    g_reasm_expected = 0;
    g_conn_event.fetch_sub(1);
  }
};

ScanCb g_scan_cb;
ClientCb g_client_cb;

void onRxNotify(NimBLERemoteCharacteristic * /*c*/, uint8_t *data, size_t len, bool /*isNotify*/) {
  // Tesla fragments large messages across indications; the first fragment is
  // prefixed with a 2-byte big-endian total length.
  size_t offset = 0;
  if (g_reasm_expected == 0) {
    if (len < 2) return;
    g_reasm_expected = (static_cast<size_t>(data[0]) << 8) | data[1];
    offset = 2;
    g_reasm.clear();
    g_reasm.reserve(g_reasm_expected);
  }
  g_reasm.insert(g_reasm.end(), data + offset, data + len);
  if (g_reasm.size() >= g_reasm_expected) {
    // Hand off to main loop via queue.
    auto *copy = new std::vector<uint8_t>(std::move(g_reasm));
    if (xQueueSend(g_rx_queue, &copy, 0) != pdTRUE) {
      log_e("RX queue full; dropping message");
      delete copy;
    }
    g_reasm.clear();
    g_reasm_expected = 0;
  }
}

// -----------------------------------------------------------------------------
// Connection state machine
// -----------------------------------------------------------------------------
void startScan() {
  g_device_found = false;
  g_state = State::Scanning;
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&g_scan_cb, /*wantDuplicates=*/false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
  scan->start(20'000, /*is_continue=*/false, /*restart=*/false);
  Serial.printf("[BLE] Scanning 20 s for Tesla VIN %s (wake the car first if asleep)\n", g_vin.c_str());
}

void tryConnect() {
  if (!g_device_found) return;
  g_state = State::Connecting;
  Serial.printf("[BLE] Connecting to %s...\n", g_found_device.getAddress().toString().c_str());
  if (!g_client) {
    g_client = NimBLEDevice::createClient();
    g_client->setClientCallbacks(&g_client_cb, /*deleteCallbacks=*/false);
    g_client->setConnectionParams(12, 24, 0, 400);  // 15-30ms interval, 4s timeout
    g_client->setConnectTimeout(5'000);
  }
  if (!g_client->connect(&g_found_device, /*deleteAttributes=*/true)) {
    Serial.println("[BLE] Connect failed — will retry");
    g_state = State::Idle;
    return;
  }
  g_state = State::Discovering;
  NimBLERemoteService *svc = g_client->getService(kSvcUuid);
  if (!svc) {
    Serial.println("[BLE] Tesla service not found on device");
    g_client->disconnect();
    g_state = State::Idle;
    return;
  }
  g_tx = svc->getCharacteristic(kTxUuid);
  g_rx = svc->getCharacteristic(kRxUuid);
  if (!g_tx || !g_rx) {
    Serial.println("[BLE] Tesla TX/RX characteristics missing");
    g_client->disconnect();
    g_state = State::Idle;
    return;
  }
  if (!g_rx->subscribe(/*notifications=*/false, onRxNotify, /*response=*/true)) {
    Serial.println("[BLE] Failed to subscribe to RX characteristic");
    g_client->disconnect();
    g_state = State::Idle;
    return;
  }
  g_state = State::Ready;
  g_conn_event.fetch_add(1);
  Serial.printf("[BLE] Link ready (mtu=%u) — starting session handshake\n", g_client->getMTU());
}

}  // namespace

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
void begin() {
  g_rx_queue = xQueueCreate(8, sizeof(std::vector<uint8_t> *));
  NimBLEDevice::init("TeslaBLE-Bridge");
  NimBLEDevice::setPower(3);      // +3 dBm
  NimBLEDevice::setMTU(517);
}

void setVin(const String &vin) { g_vin = vin; }
void setOnMessage(MessageCallback cb)    { g_on_msg  = std::move(cb); }
void setOnConnection(ConnectionCallback cb) { g_on_conn = std::move(cb); }

bool isConnected() { return g_client && g_client->isConnected(); }
bool isReady()     { return g_state == State::Ready && isConnected(); }

void requestConnect() {
  if (g_vin.length() != 17) return;
  if (g_state != State::Idle) return;
  const uint32_t now = millis();
  if (now - g_last_attempt_ms < cfg::kConnectRetryMs && g_last_attempt_ms != 0) return;
  g_last_attempt_ms = now;
  startScan();
}

void disconnect() {
  if (g_client && g_client->isConnected()) {
    g_state = State::Disconnecting;
    g_client->disconnect();
  }
}

bool sendMessage(const uint8_t *data, size_t len) {
  if (!isReady() || !g_tx) return false;
  if (len == 0 || len > 1024) return false;

  // Prefix with 2-byte BE length, then fragment to (MTU-3).
  const uint16_t mtu = g_client->getMTU();
  const size_t chunk = (mtu > 3) ? (mtu - 3) : 20;
  std::vector<uint8_t> framed;
  framed.reserve(len + 2);
  framed.push_back(static_cast<uint8_t>(len >> 8));
  framed.push_back(static_cast<uint8_t>(len & 0xFF));
  framed.insert(framed.end(), data, data + len);

  for (size_t off = 0; off < framed.size(); off += chunk) {
    const size_t n = std::min(chunk, framed.size() - off);
    if (!g_tx->writeValue(framed.data() + off, n, /*response=*/true)) {
      log_e("TX write failed at offset %u", (unsigned)off);
      return false;
    }
  }
  return true;
}

void loop() {
  if (!g_rx_queue) return;
  // Connect from main task (not NimBLE host task) to avoid deadlocks.
  if (g_connect_pending.exchange(false)) tryConnect();
  // Drain RX queue.
  std::vector<uint8_t> *msg = nullptr;
  while (xQueueReceive(g_rx_queue, &msg, 0) == pdTRUE) {
    if (g_on_msg) g_on_msg(msg->data(), msg->size());
    delete msg;
  }
  // Drain connection events.
  int ev = g_conn_event.exchange(0);
  if (ev != 0 && g_on_conn) g_on_conn(ev > 0);
}

}  // namespace tesla_transport
