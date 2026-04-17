#include "tesla_client.h"

#include <Preferences.h>
#include <memory>
#include <vector>

#include "adapters.h"
#include "car_server.pb.h"
#include "config.h"
#include "defs.h"
#include "keys.pb.h"
#include "storage.h"
#include "tesla_transport.h"
#include "vehicle.h"
#include "vcsec.pb.h"

namespace tesla_client {
namespace {

// ---------------------------------------------------------------------------
// StorageAdapter — NVS via Arduino Preferences
// ---------------------------------------------------------------------------
class NvsStorage : public TeslaBLE::StorageAdapter {
  Preferences prefs_;

  const char *nvsKey(const std::string &key) {
    if (key == "private_key")           return "pk";
    if (key == "session_vcsec")         return "sess_vcsec";
    if (key == "session_infotainment")  return "sess_info";
    return nullptr;
  }

 public:
  bool load(const std::string &key, std::vector<uint8_t> &buf) override {
    const char *nk = nvsKey(key);
    if (!nk) return false;
    prefs_.begin("tesla", /*readOnly=*/true);
    size_t len = prefs_.getBytesLength(nk);
    prefs_.end();
    if (len == 0) return false;
    buf.resize(len);
    prefs_.begin("tesla", /*readOnly=*/true);
    prefs_.getBytes(nk, buf.data(), len);
    prefs_.end();
    return true;
  }

  bool save(const std::string &key, const std::vector<uint8_t> &buf) override {
    const char *nk = nvsKey(key);
    if (!nk) return false;
    prefs_.begin("tesla", /*readOnly=*/false);
    bool ok = prefs_.putBytes(nk, buf.data(), buf.size()) > 0;
    prefs_.end();
    return ok;
  }

  bool remove(const std::string &key) override {
    const char *nk = nvsKey(key);
    if (!nk) return false;
    prefs_.begin("tesla", /*readOnly=*/false);
    bool ok = prefs_.remove(nk);
    prefs_.end();
    return ok;
  }
};

// ---------------------------------------------------------------------------
// BleAdapter — wraps tesla_transport
// The build_* functions already prepend the 2-byte length header, so we
// pass data through to sendMessage without any additional framing.
// ---------------------------------------------------------------------------
class TransportBle : public TeslaBLE::BleAdapter {
 public:
  void connect(const std::string &) override {}  // transport manages connection
  void disconnect() override { tesla_transport::disconnect(); }
  bool write(const std::vector<uint8_t> &data) override {
    return tesla_transport::sendMessage(data.data(), data.size());
  }
};

// ---------------------------------------------------------------------------
// Module-level state
// ---------------------------------------------------------------------------
std::shared_ptr<NvsStorage>        g_storage;
std::shared_ptr<TransportBle>      g_ble;
std::shared_ptr<TeslaBLE::Vehicle> g_vehicle;

String g_vin;
State  g_state;
StateChangedCallback g_state_cb;
PairingCallback      g_pair_cb;

bool      g_paired        = false;
bool      g_connected     = false;
PairStage g_pair_stage    = PairStage::Idle;
uint32_t  g_last_vcsec_ms = 0;
uint32_t  g_last_info_ms  = 0;

void fireStateChanged() { if (g_state_cb) g_state_cb(); }

void setPairStage(PairStage s, const char *msg) {
  g_pair_stage = s;
  if (g_pair_cb) g_pair_cb(s, msg);
}

// ---------------------------------------------------------------------------
// Transport callbacks
// ---------------------------------------------------------------------------
void onMessage(const uint8_t *data, size_t len) {
  if (g_vehicle)
    g_vehicle->on_rx_data(std::vector<uint8_t>(data, data + len));
}

void onTransportConnection(bool up) {
  g_connected = up;
  if (!g_vehicle) return;

  if (up) {
    Serial.println("[tesla] BLE connected");
    g_vehicle->set_connected(true);

    if (!g_paired) {
      Serial.println("[tesla] Not paired — sending whitelist request.");
      Serial.println("[tesla] Tap your Tesla key card on the center console to approve.");
      setPairStage(PairStage::Sending, "requesting vehicle to add key");
      // pair() persists the private key and queues the whitelist command
      g_vehicle->pair(static_cast<Keys_Role>(TESLA_KEY_ROLE));
    }
    // vcsec_poll queued after pair (or immediately if already paired).
    // If pairing is pending, this may fail until the user taps — the Vehicle
    // will retry automatically.
    g_vehicle->vcsec_poll();
  } else {
    Serial.println("[tesla] BLE disconnected");
    g_vehicle->set_connected(false);
    if (g_pair_stage == PairStage::Sending || g_pair_stage == PairStage::WaitingForTap)
      setPairStage(PairStage::Idle, nullptr);
  }
}

// ---------------------------------------------------------------------------
// Vehicle state callbacks
// ---------------------------------------------------------------------------
void setupCallbacks() {
  g_vehicle->set_vehicle_status_callback([](const VCSEC_VehicleStatus &vs) {
    g_state.lockState    = (int)vs.vehicleLockState;
    g_state.sleepStatus  = (int)vs.vehicleSleepStatus;
    g_state.userPresence = (int)vs.userPresence;
    if (vs.has_closureStatuses)
      g_state.chargeFlapState = (int)vs.closureStatuses.chargePort;
    g_state.lastVcsecUpdateMs = millis();
    Serial.printf("[tesla] VCSEC: lock=%d sleep=%d presence=%d\n",
                  g_state.lockState, g_state.sleepStatus, g_state.userPresence);

    // First successful vehicleStatus proves the key is whitelisted.
    if (!g_paired) {
      g_paired = true;
      storage::setPaired(true);
      setPairStage(PairStage::Succeeded, "key accepted by vehicle");
      Serial.println("[tesla] Pairing confirmed — key is whitelisted!");
    }
    fireStateChanged();
  });

  g_vehicle->set_charge_state_callback([](const CarServer_ChargeState &cs) {
    if (cs.which_optional_battery_level == CarServer_ChargeState_battery_level_tag)
      g_state.batteryLevelPct = cs.optional_battery_level.battery_level;
    if (cs.which_optional_battery_range == CarServer_ChargeState_battery_range_tag)
      g_state.batteryRangeKm = cs.optional_battery_range.battery_range * 1.60934f;
    if (cs.which_optional_charger_power == CarServer_ChargeState_charger_power_tag)
      g_state.chargerPowerKw = cs.optional_charger_power.charger_power;
    if (cs.has_charging_state)
      g_state.charging = (cs.charging_state.which_type ==
                          CarServer_ChargeState_ChargingState_Charging_tag);
    g_state.lastInfotainUpdateMs = millis();
    fireStateChanged();
  });

  g_vehicle->set_climate_state_callback([](const CarServer_ClimateState &cls) {
    if (cls.which_optional_inside_temp_celsius == CarServer_ClimateState_inside_temp_celsius_tag)
      g_state.insideTempC = cls.optional_inside_temp_celsius.inside_temp_celsius;
    if (cls.which_optional_outside_temp_celsius == CarServer_ClimateState_outside_temp_celsius_tag)
      g_state.outsideTempC = cls.optional_outside_temp_celsius.outside_temp_celsius;
    if (cls.which_optional_is_climate_on == CarServer_ClimateState_is_climate_on_tag)
      g_state.climateOn = cls.optional_is_climate_on.is_climate_on;
    if (cls.which_optional_driver_temp_setting == CarServer_ClimateState_driver_temp_setting_tag)
      g_state.driverTempSetpointC = cls.optional_driver_temp_setting.driver_temp_setting;
    g_state.lastInfotainUpdateMs = millis();
    fireStateChanged();
  });
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void begin() {
  // Route TeslaBLE library logs to Serial so they appear in the monitor.
  TeslaBLE::set_log_callback([](TeslaBLE::LogLevel level, const char *tag, int,
                                const char *fmt, va_list args) {
    char buf[512];
    vsnprintf(buf, sizeof(buf), fmt, args);
    const char *lvl;
    switch (level) {
      case TeslaBLE::LogLevel::ERROR:   lvl = "E"; break;
      case TeslaBLE::LogLevel::WARN:    lvl = "W"; break;
      case TeslaBLE::LogLevel::INFO:    lvl = "I"; break;
      default:                          lvl = "D"; break;
    }
    Serial.printf("[TBLE/%s][%s] %s\n", lvl, tag ? tag : "?", buf);
  });

  g_storage = std::make_shared<NvsStorage>();
  g_ble     = std::make_shared<TransportBle>();
  // Vehicle constructor loads private key + sessions from storage automatically.
  g_vehicle = std::make_shared<TeslaBLE::Vehicle>(g_ble, g_storage);

  g_paired = storage::getPaired();
  setupCallbacks();

  tesla_transport::setOnMessage(onMessage);
  tesla_transport::setOnConnection(onTransportConnection);
}

void setVin(const String &vin) {
  g_vin = vin;
  if (g_vehicle) g_vehicle->set_vin(std::string(vin.c_str()));
  tesla_transport::setVin(vin);
}

bool isPaired()            { return g_paired; }
bool isSessionReady()      { return g_connected && g_state.lastVcsecUpdateMs > 0; }
bool isInfotainmentReady() { return g_connected && g_state.lastInfotainUpdateMs > 0; }
const State &getState()    { return g_state; }

void onStateChanged(StateChangedCallback cb)  { g_state_cb = std::move(cb); }
void onPairingProgress(PairingCallback cb)    { g_pair_cb  = std::move(cb); }

void startPairing() {
  if (!g_vehicle || !g_connected) {
    setPairStage(PairStage::Failed, "not connected to vehicle");
    return;
  }
  Serial.println("[tesla] Manual pair requested");
  setPairStage(PairStage::Sending, "requesting vehicle to add key");
  g_vehicle->pair(static_cast<Keys_Role>(TESLA_KEY_ROLE));
}

void cancelPairing() {
  if (g_pair_stage != PairStage::Idle) setPairStage(PairStage::Idle, "cancelled");
}

bool lockDoors()   { if (!g_vehicle) return false; g_vehicle->lock();   return true; }
bool unlockDoors() { if (!g_vehicle) return false; g_vehicle->unlock(); return true; }
bool wakeVehicle() { if (!g_vehicle) return false; g_vehicle->wake();   return true; }

bool honk() {
  if (!g_vehicle) return false;
  g_vehicle->honk_horn();
  return true;
}
bool flashLights() {
  if (!g_vehicle) return false;
  g_vehicle->flash_lights();
  return true;
}
bool frunkOpen() {
  if (!g_vehicle) return false;
  g_vehicle->open_frunk();
  return true;
}
bool trunkOpen() {
  if (!g_vehicle) return false;
  g_vehicle->open_trunk();
  return true;
}
bool trunkClose() {
  if (!g_vehicle) return false;
  g_vehicle->close_trunk();
  return true;
}
bool chargePortOpen() {
  if (!g_vehicle) return false;
  g_vehicle->open_charge_port();
  return true;
}
bool chargePortClose() {
  if (!g_vehicle) return false;
  g_vehicle->close_charge_port();
  return true;
}
bool hvacOn() {
  if (!g_vehicle) return false;
  g_vehicle->set_climate(true);
  return true;
}
bool hvacOff() {
  if (!g_vehicle) return false;
  g_vehicle->set_climate(false);
  return true;
}
bool setHvacTempCelsius(float target) {
  if (!g_vehicle) return false;
  g_vehicle->set_climate_temp(target);
  return true;
}
bool chargingStart() {
  if (!g_vehicle) return false;
  g_vehicle->set_charging_state(true);
  return true;
}
bool chargingStop() {
  if (!g_vehicle) return false;
  g_vehicle->set_charging_state(false);
  return true;
}
bool setChargeLimit(int percent) {
  if (!g_vehicle) return false;
  g_vehicle->set_charging_limit(percent);
  return true;
}
bool setChargeAmps(int amps) {
  if (!g_vehicle) return false;
  g_vehicle->set_charging_amps(amps);
  return true;
}

// ---------------------------------------------------------------------------
// Main loop — drive transport reconnect and periodic polling
// ---------------------------------------------------------------------------
void loop() {
  if (g_vin.length() != 17) return;
  tesla_transport::requestConnect();

  if (!g_vehicle) return;
  g_vehicle->loop();

  if (!g_connected) return;

  const uint32_t now = millis();

  if ((now - g_last_vcsec_ms) >= cfg::kBodyStatePollMs) {
    g_last_vcsec_ms = now;
    g_vehicle->vcsec_poll();
  }

  if (g_state.awake()) {
    const uint32_t ivl = g_state.charging ? cfg::kVehicleDataFastMs : cfg::kVehicleDataMs;
    if ((now - g_last_info_ms) >= ivl) {
      g_last_info_ms = now;
      g_vehicle->infotainment_poll(/*force_wake=*/true);
    }
  }
}

}  // namespace tesla_client
