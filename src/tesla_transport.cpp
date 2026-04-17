#include "tesla_transport.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <atomic>

#include "config.h"
#include "vin_utils.h"

namespace tesla_transport {
namespace {

const NimBLEUUID kSvcUuid("00000211-b2d1-43f0-9b88-960cebf8b91e");
const NimBLEUUID kTxUuid("00000212-b2d1-43f0-9b88-960cebf8b91e");
const NimBLEUUID kRxUuid("00000213-b2d1-43f0-9b88-960cebf8b91e");

String g_vin;
MessageCallback    g_on_msg;
ConnectionCallback g_on_conn;

NimBLEClient               *g_client = nullptr;
NimBLERemoteCharacteristic *g_tx     = nullptr;
NimBLERemoteCharacteristic *g_rx     = nullptr;
NimBLEAdvertisedDevice      g_found_device;

enum class State { Idle, Scanning, Connecting, Ready };
std::atomic<State> g_state{State::Idle};

QueueHandle_t       g_rx_queue   = nullptr;
std::atomic<int>    g_conn_event{0};

std::vector<uint8_t> g_reasm;
size_t               g_reasm_expected = 0;

uint32_t g_last_scan_ms = 0;

// ---------------------------------------------------------------------------
// Connect task — runs on its own stack, completely outside NimBLE callbacks.
// ---------------------------------------------------------------------------
void connectTask(void *) {
  // Wait until the scanner has fully stopped.
  NimBLEScan *scan = NimBLEDevice::getScan();
  for (int i = 0; i < 40 && scan->isScanning(); ++i)
    vTaskDelay(pdMS_TO_TICKS(50));

  Serial.printf("[BLE] Connecting to %s\n",
                g_found_device.getAddress().toString().c_str());

  if (!g_client) {
    g_client = NimBLEDevice::createClient();
    g_client->setConnectionParams(12, 24, 0, 400);
    g_client->setConnectTimeout(8'000);
  }

  if (!g_client->connect(&g_found_device, /*deleteAttributes=*/true)) {
    Serial.println("[BLE] Connect failed — will rescan");
    g_state = State::Idle;
    vTaskDelete(nullptr);
    return;
  }

  NimBLERemoteService *svc = g_client->getService(kSvcUuid);
  if (!svc) {
    Serial.println("[BLE] Tesla service not found");
    g_client->disconnect();
    g_state = State::Idle;
    vTaskDelete(nullptr);
    return;
  }

  g_tx = svc->getCharacteristic(kTxUuid);
  g_rx = svc->getCharacteristic(kRxUuid);
  if (!g_tx || !g_rx) {
    Serial.println("[BLE] Tesla TX/RX characteristics missing");
    g_client->disconnect();
    g_state = State::Idle;
    vTaskDelete(nullptr);
    return;
  }

  auto rxCb = [](NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
    size_t offset = 0;
    if (g_reasm_expected == 0) {
      if (len < 2) return;
      g_reasm_expected = (size_t(data[0]) << 8) | data[1];
      offset = 2;
      g_reasm.clear();
      g_reasm.reserve(g_reasm_expected);
    }
    g_reasm.insert(g_reasm.end(), data + offset, data + len);
    if (g_reasm.size() >= g_reasm_expected) {
      auto *copy = new std::vector<uint8_t>(std::move(g_reasm));
      if (xQueueSend(g_rx_queue, &copy, 0) != pdTRUE) delete copy;
      g_reasm.clear();
      g_reasm_expected = 0;
    }
  };

  if (!g_rx->subscribe(/*notifications=*/false, rxCb, /*response=*/true)) {
    Serial.println("[BLE] RX subscribe failed");
    g_client->disconnect();
    g_state = State::Idle;
    vTaskDelete(nullptr);
    return;
  }

  g_state = State::Ready;
  g_conn_event.fetch_add(1);
  Serial.printf("[BLE] Connected (mtu=%u)\n", g_client->getMTU());
  vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// NimBLE callbacks
// ---------------------------------------------------------------------------
class ScanCb : public NimBLEScanCallbacks {
 public:
  void onResult(const NimBLEAdvertisedDevice *adv) override {
    if (!adv->haveName()) return;
    const std::string name = adv->getName();
    if (!TeslaBLE::matches_vin(name, g_vin.c_str())) return;
    Serial.printf("[BLE] Found %s — connecting\n", name.c_str());
    g_found_device = *adv;
    NimBLEDevice::getScan()->stop();
    g_state = State::Connecting;
    xTaskCreate([](void *) { connectTask(nullptr); },
                "ble_conn", 6144, nullptr, 5, nullptr);
  }
  void onScanEnd(const NimBLEScanResults &, int) override {
    if (g_state == State::Scanning) {
      Serial.println("[BLE] Scan done — vehicle not found, will retry");
      g_state = State::Idle;
    }
  }
};

class ClientCb : public NimBLEClientCallbacks {
 public:
  void onConnect(NimBLEClient *c) override { c->exchangeMTU(); }
  void onDisconnect(NimBLEClient *, int reason) override {
    Serial.printf("[BLE] Disconnected (reason=%d) — rescanning\n", reason);
    g_tx = nullptr;
    g_rx = nullptr;
    g_reasm.clear();
    g_reasm_expected = 0;
    g_state = State::Idle;
    g_conn_event.fetch_sub(1);
  }
};

ScanCb   g_scan_cb;
ClientCb g_client_cb;

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void begin() {
  g_rx_queue = xQueueCreate(8, sizeof(std::vector<uint8_t> *));
  NimBLEDevice::init("TeslaBLE-Bridge");
  NimBLEDevice::setPower(3);
  NimBLEDevice::setMTU(517);
  NimBLEDevice::getScan()->setScanCallbacks(&g_scan_cb, false);

  // Register disconnect callback via a dummy client holder.
  // (ClientCb is set per-client in connectTask; we store it globally so it
  //  persists across reconnects without heap-allocating each time.)
}

void setVin(const String &vin) { g_vin = vin; }
void setOnMessage(MessageCallback cb)       { g_on_msg  = std::move(cb); }
void setOnConnection(ConnectionCallback cb) { g_on_conn = std::move(cb); }

bool isConnected() { return g_client && g_client->isConnected(); }
bool isReady()     { return g_state == State::Ready && isConnected(); }

void requestConnect() {
  if (g_vin.length() != 17)        return;
  if (g_state != State::Idle)      return;
  const uint32_t now = millis();
  if (g_last_scan_ms && (now - g_last_scan_ms) < cfg::kConnectRetryMs) return;
  g_last_scan_ms = now;
  g_state = State::Scanning;
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
  scan->start(20'000, false, false);
  Serial.printf("[BLE] Scanning for Tesla %s\n", g_vin.c_str());
}

void disconnect() {
  if (g_client && g_client->isConnected()) g_client->disconnect();
}

bool sendMessage(const uint8_t *data, size_t len) {
  if (!isReady() || !g_tx || len == 0 || len > 1024) return false;
  const uint16_t mtu   = g_client->getMTU();
  const size_t   chunk = (mtu > 3) ? (mtu - 3) : 20;
  std::vector<uint8_t> framed;
  framed.reserve(len + 2);
  framed.push_back(uint8_t(len >> 8));
  framed.push_back(uint8_t(len & 0xFF));
  framed.insert(framed.end(), data, data + len);
  for (size_t off = 0; off < framed.size(); off += chunk) {
    const size_t n = std::min(chunk, framed.size() - off);
    if (!g_tx->writeValue(framed.data() + off, n, true)) return false;
  }
  return true;
}

void loop() {
  if (!g_rx_queue) return;
  std::vector<uint8_t> *msg = nullptr;
  while (xQueueReceive(g_rx_queue, &msg, 0) == pdTRUE) {
    if (g_on_msg) g_on_msg(msg->data(), msg->size());
    delete msg;
  }
  int ev = g_conn_event.exchange(0);
  if (ev != 0 && g_on_conn) g_on_conn(ev > 0);
}

}  // namespace tesla_transport
