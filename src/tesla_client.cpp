#include "tesla_client.h"

#include <atomic>

#include "car_server.pb.h"
#include "client.h"
#include "config.h"
#include "errors.h"
#include "signatures.pb.h"
#include "storage.h"
#include "tesla_transport.h"
#include "universal_message.pb.h"
#include "vcsec.pb.h"
#include "vin_utils.h"

namespace tesla_client {
namespace {

TeslaBLE::Client g_cli;
String g_vin;
State  g_state;
StateChangedCallback g_state_cb;
PairingCallback      g_pair_cb;

PairStage g_pair_stage   = PairStage::Idle;
uint32_t  g_pair_next_ms = 0;

bool     g_paired              = false;
bool     g_vcsec_session_ready = false;
bool     g_infotain_session_ready = false;

uint32_t g_last_body_poll_ms     = 0;
uint32_t g_last_vehicle_poll_ms  = 0;
uint32_t g_vcsec_handshake_ms    = 0;
uint32_t g_infotain_handshake_ms = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void fireStateChanged() { if (g_state_cb) g_state_cb(); }

void setPairStage(PairStage s, const char *msg) {
  g_pair_stage = s;
  if (g_pair_cb) g_pair_cb(s, msg);
}

bool sendBuilt(const uint8_t *buf, size_t len, const char *what) {
  if (!tesla_transport::isReady()) {
    log_w("Cannot send %s — transport not ready", what);
    return false;
  }
  if (!tesla_transport::sendMessage(buf, len)) {
    log_e("Transport sendMessage(%s) failed", what);
    return false;
  }
  log_i("Sent %s (%u bytes)", what, (unsigned)len);
  return true;
}

// The last domain we asked a session-info-request for — used to route the
// reply because not all Tesla firmwares set from_destination.domain in
// session-info responses (some use routing_address instead).
UniversalMessage_Domain g_pending_sessioninfo_domain = UniversalMessage_Domain_DOMAIN_BROADCAST;

// Build and send a session-info-request for `domain`.
bool requestSessionInfo(UniversalMessage_Domain domain) {
  g_pending_sessioninfo_domain = domain;
  uint8_t buf[cfg::kMaxMsgSize];
  size_t  len = 0;
  int rc = g_cli.build_session_info_request_message(domain, buf, &len);
  if (rc != 0) {
    log_e("build_session_info_request_message(%d) = %s", (int)domain,
          TeslaBLE::teslable_status_to_string(static_cast<TeslaBLE::TeslaBLE_Status_E>(rc)));
    return false;
  }
  return sendBuilt(buf, len, domain == UniversalMessage_Domain_DOMAIN_VEHICLE_SECURITY
                                  ? "VCSEC session_info_request"
                                  : "Infotainment session_info_request");
}

bool sendWhitelistMessage() {
  uint8_t buf[VCSEC_ToVCSECMessage_size];
  size_t  len = 0;
  int rc = g_cli.build_white_list_message(static_cast<Keys_Role>(TESLA_KEY_ROLE),
                                          VCSEC_KeyFormFactor_KEY_FORM_FACTOR_CLOUD_KEY, buf, &len);
  if (rc != 0) {
    log_e("build_white_list_message = %s",
          TeslaBLE::teslable_status_to_string(static_cast<TeslaBLE::TeslaBLE_Status_E>(rc)));
    return false;
  }
  return sendBuilt(buf, len, "whitelist add-key-request");
}

bool sendVcsecAction(VCSEC_RKEAction_E action, const char *name) {
  uint8_t buf[UniversalMessage_RoutableMessage_size];
  size_t  len = 0;
  int rc = g_cli.build_vcsec_action_message(action, buf, &len);
  if (rc != 0) { log_e("build_vcsec_action(%s): rc=%d", name, rc); return false; }
  return sendBuilt(buf, len, name);
}

bool sendClosure(std::function<void(VCSEC_ClosureMoveRequest &)> fill, const char *name) {
  VCSEC_ClosureMoveRequest req = {};
  fill(req);
  uint8_t buf[UniversalMessage_RoutableMessage_size];
  size_t  len = 0;
  int rc = g_cli.build_vcsec_closure_message(&req, buf, &len);
  if (rc != 0) { log_e("build_vcsec_closure(%s): rc=%d", name, rc); return false; }
  return sendBuilt(buf, len, name);
}

bool sendVehicleAction(int32_t which, const void *data, const char *name) {
  uint8_t buf[UniversalMessage_RoutableMessage_size];
  size_t  len = 0;
  int rc = g_cli.build_car_server_vehicle_action_message(buf, &len, which, data);
  if (rc != 0) { log_e("build_vehicle_action(%s): rc=%d", name, rc); return false; }
  return sendBuilt(buf, len, name);
}

// ---------------------------------------------------------------------------
// Incoming message handling
// ---------------------------------------------------------------------------
void handleVcsecReply(UniversalMessage_RoutableMessage &msg) {
  // VCSEC replies come in `protobuf_message_as_bytes`. Parse as VCSEC FromVCSECMessage.
  if (msg.which_payload != UniversalMessage_RoutableMessage_protobuf_message_as_bytes_tag)
    return;
  VCSEC_FromVCSECMessage vmsg = VCSEC_FromVCSECMessage_init_default;
  int rc = g_cli.parse_from_vcsec_message(&msg.payload.protobuf_message_as_bytes, &vmsg);
  if (rc != 0) {
    log_w("parse_from_vcsec_message rc=%d", rc);
    return;
  }

  switch (vmsg.which_sub_message) {
    case VCSEC_FromVCSECMessage_vehicleStatus_tag: {
      const auto &vs = vmsg.sub_message.vehicleStatus;
      g_state.lockState    = (int)vs.vehicleLockState;
      g_state.sleepStatus  = (int)vs.vehicleSleepStatus;
      g_state.userPresence = (int)vs.userPresence;
      if (vs.has_closureStatuses)
        g_state.chargeFlapState = (int)vs.closureStatuses.chargePort;
      g_state.lastVcsecUpdateMs = millis();
      log_i("VCSEC state: lock=%d sleep=%d presence=%d", g_state.lockState, g_state.sleepStatus,
            g_state.userPresence);
      fireStateChanged();
      break;
    }
    case VCSEC_FromVCSECMessage_commandStatus_tag: {
      const auto &cs = vmsg.sub_message.commandStatus;
      if (g_pair_stage == PairStage::Sending || g_pair_stage == PairStage::WaitingForTap) {
        if (cs.operationStatus == VCSEC_OperationStatus_E_OPERATIONSTATUS_OK) {
          log_i("Pairing accepted by vehicle!");
          g_paired = true;
          storage::setPaired(true);
          setPairStage(PairStage::Succeeded, "key accepted");
        } else if (cs.operationStatus == VCSEC_OperationStatus_E_OPERATIONSTATUS_WAIT) {
          setPairStage(PairStage::WaitingForTap, "tap your Tesla key card on the console");
        } else {
          setPairStage(PairStage::Failed, "vehicle rejected the pairing request");
        }
      } else {
        log_i("VCSEC commandStatus: op=%d", (int)cs.operationStatus);
      }
      break;
    }
    default:
      break;
  }
}

void handleInfotainmentReply(UniversalMessage_RoutableMessage &msg) {
  if (msg.which_payload != UniversalMessage_RoutableMessage_protobuf_message_as_bytes_tag)
    return;

  CarServer_Response resp = CarServer_Response_init_default;
  Signatures_SignatureData &sig = msg.sub_sigData.signature_data;
  int rc = g_cli.parse_payload_car_server_response(
      &msg.payload.protobuf_message_as_bytes, &sig, sig.which_sig_type,
      msg.signedMessageStatus.signed_message_fault, msg.flags, &resp);
  if (rc != 0) {
    log_w("parse_payload_car_server_response rc=%d", rc);
    return;
  }

  if (resp.which_response_msg == CarServer_Response_vehicleData_tag) {
    const auto &vd = resp.response_msg.vehicleData;
    if (vd.has_charge_state) {
      const auto &cs = vd.charge_state;
      if (cs.which_optional_battery_level == CarServer_ChargeState_battery_level_tag)
        g_state.batteryLevelPct = cs.optional_battery_level.battery_level;
      if (cs.which_optional_battery_range == CarServer_ChargeState_battery_range_tag)
        g_state.batteryRangeKm = cs.optional_battery_range.battery_range * 1.60934f;
      if (cs.which_optional_charger_power == CarServer_ChargeState_charger_power_tag)
        g_state.chargerPowerKw = cs.optional_charger_power.charger_power;
      if (cs.has_charging_state) {
        auto which = cs.charging_state.which_type;
        g_state.charging = (which == CarServer_ChargeState_ChargingState_Charging_tag);
      }
    }
    if (vd.has_climate_state) {
      const auto &cls = vd.climate_state;
      if (cls.which_optional_inside_temp_celsius == CarServer_ClimateState_inside_temp_celsius_tag)
        g_state.insideTempC = cls.optional_inside_temp_celsius.inside_temp_celsius;
      if (cls.which_optional_outside_temp_celsius == CarServer_ClimateState_outside_temp_celsius_tag)
        g_state.outsideTempC = cls.optional_outside_temp_celsius.outside_temp_celsius;
      if (cls.which_optional_is_climate_on == CarServer_ClimateState_is_climate_on_tag)
        g_state.climateOn = cls.optional_is_climate_on.is_climate_on;
      if (cls.which_optional_driver_temp_setting == CarServer_ClimateState_driver_temp_setting_tag)
        g_state.driverTempSetpointC = cls.optional_driver_temp_setting.driver_temp_setting;
    }
    g_state.lastInfotainUpdateMs = millis();
    log_i("Vehicle data: batt=%d%% inside=%.1fC climate=%d", g_state.batteryLevelPct,
          g_state.insideTempC, g_state.climateOn);
    fireStateChanged();
  }
}

void handleSessionInfoReply(UniversalMessage_RoutableMessage &msg) {
  if (msg.which_payload != UniversalMessage_RoutableMessage_session_info_tag) return;
  Signatures_SessionInfo si = Signatures_SessionInfo_init_default;
  int rc = g_cli.parse_payload_session_info(&msg.payload.session_info, &si);
  if (rc != 0) { log_e("parse session_info rc=%d", rc); return; }

  UniversalMessage_Domain domain = UniversalMessage_Domain_DOMAIN_BROADCAST;
  if (msg.has_from_destination &&
      msg.from_destination.which_sub_destination == UniversalMessage_Destination_domain_tag) {
    domain = msg.from_destination.sub_destination.domain;
  } else {
    domain = g_pending_sessioninfo_domain;
  }
  TeslaBLE::Peer *peer = g_cli.get_peer(domain);
  if (!peer) return;
  rc = peer->update_session(&si);
  if (rc != 0) { log_e("update_session rc=%d", rc); return; }
  if (domain == UniversalMessage_Domain_DOMAIN_VEHICLE_SECURITY) {
    g_vcsec_session_ready = true;
    log_i("VCSEC session ready (counter=%u)", (unsigned)peer->get_counter());
  } else if (domain == UniversalMessage_Domain_DOMAIN_INFOTAINMENT) {
    g_infotain_session_ready = true;
    log_i("Infotainment session ready");
  }
}

void onMessage(const uint8_t *data, size_t len) {
  UniversalMessage_RoutableMessage msg = UniversalMessage_RoutableMessage_init_default;
  int rc = g_cli.parse_universal_message_ble(const_cast<uint8_t *>(data), len, &msg);
  if (rc != 0) {
    log_w("parse_universal_message_ble rc=%d (len=%u)", rc, (unsigned)len);
    return;
  }
  if (msg.which_payload == UniversalMessage_RoutableMessage_session_info_tag) {
    handleSessionInfoReply(msg);
    return;
  }
  // Command replies: read the from-domain only when it's the union tag we expect.
  UniversalMessage_Domain from = UniversalMessage_Domain_DOMAIN_BROADCAST;
  if (msg.has_from_destination &&
      msg.from_destination.which_sub_destination == UniversalMessage_Destination_domain_tag) {
    from = msg.from_destination.sub_destination.domain;
  }
  if (from == UniversalMessage_Domain_DOMAIN_VEHICLE_SECURITY)
    handleVcsecReply(msg);
  else if (from == UniversalMessage_Domain_DOMAIN_INFOTAINMENT)
    handleInfotainmentReply(msg);
  else if (msg.which_payload == UniversalMessage_RoutableMessage_protobuf_message_as_bytes_tag)
    handleVcsecReply(msg);  // pairing responses use routing_address not domain
}

void onTransportConnection(bool up) {
  if (up) {
    g_vcsec_session_ready    = false;
    g_infotain_session_ready = false;
    g_vcsec_handshake_ms     = millis();
    if (!g_paired) {
      Serial.println("Not paired — sending key request. Tap your Tesla key card on the center console.");
      startPairing();
    } else {
      requestSessionInfo(UniversalMessage_Domain_DOMAIN_VEHICLE_SECURITY);
    }
  } else {
    Serial.println("BLE disconnected.");
    g_vcsec_session_ready    = false;
    g_infotain_session_ready = false;
  }
}

// ---------------------------------------------------------------------------
// Key init: load from NVS or generate & persist.
// ---------------------------------------------------------------------------
void ensurePrivateKey() {
  if (storage::hasPrivateKey()) {
    auto k = storage::getPrivateKey();
    if (g_cli.load_private_key(k.data(), k.size()) == 0) {
      log_i("Loaded private key from NVS (%u bytes)", (unsigned)k.size());
      return;
    }
    log_e("NVS private key failed to load — regenerating");
  }
  if (g_cli.create_private_key() != 0) {
    log_e("Failed to create private key!");
    return;
  }
  uint8_t buf[300];
  size_t  len = 0;
  if (g_cli.get_private_key(buf, sizeof(buf), &len) == 0) {
    storage::setPrivateKey(buf, len);
    log_i("Generated and persisted new private key (%u bytes)", (unsigned)len);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void begin() {
  ensurePrivateKey();
  g_paired = storage::getPaired();
  tesla_transport::setOnMessage(onMessage);
  tesla_transport::setOnConnection(onTransportConnection);
}

void setVin(const String &vin) {
  g_vin = vin;
  g_cli.set_vin(std::string(vin.c_str()));
  tesla_transport::setVin(vin);
}

bool isPaired()               { return g_paired; }
bool isSessionReady()         { return g_vcsec_session_ready; }
bool isInfotainmentReady()    { return g_infotain_session_ready; }
const State &getState()       { return g_state; }
void onStateChanged(StateChangedCallback cb) { g_state_cb = std::move(cb); }
void onPairingProgress(PairingCallback cb)   { g_pair_cb  = std::move(cb); }

void startPairing() {
  if (!tesla_transport::isReady()) {
    setPairStage(PairStage::Failed, "not connected to vehicle");
    return;
  }
  log_i("Starting key pairing — role=%d", TESLA_KEY_ROLE);
  setPairStage(PairStage::Sending, "requesting vehicle to add key");
  if (sendWhitelistMessage()) {
    g_pair_next_ms = millis() + 4000;
  } else {
    setPairStage(PairStage::Failed, "failed to send whitelist request");
  }
}

void cancelPairing() {
  if (g_pair_stage != PairStage::Idle) setPairStage(PairStage::Idle, "cancelled");
}

// Command helpers — most require an authenticated VCSEC/Infotainment session.
bool lockDoors()       { return sendVcsecAction(VCSEC_RKEAction_E_RKE_ACTION_LOCK,          "lock"); }
bool unlockDoors()     { return sendVcsecAction(VCSEC_RKEAction_E_RKE_ACTION_UNLOCK,        "unlock"); }
bool wakeVehicle()     { return sendVcsecAction(VCSEC_RKEAction_E_RKE_ACTION_WAKE_VEHICLE,  "wake"); }

bool honk() {
  CarServer_VehicleControlHonkHornAction a = CarServer_VehicleControlHonkHornAction_init_default;
  return sendVehicleAction(CarServer_VehicleAction_vehicleControlHonkHornAction_tag, &a, "honk");
}
bool flashLights() {
  CarServer_VehicleControlFlashLightsAction a = CarServer_VehicleControlFlashLightsAction_init_default;
  return sendVehicleAction(CarServer_VehicleAction_vehicleControlFlashLightsAction_tag, &a, "flashLights");
}

bool frunkOpen() {
  return sendClosure([](VCSEC_ClosureMoveRequest &r) { r.frontTrunk = VCSEC_ClosureMoveType_E_CLOSURE_MOVE_TYPE_OPEN; },
                     "frunkOpen");
}
bool trunkOpen() {
  return sendClosure([](VCSEC_ClosureMoveRequest &r) { r.rearTrunk = VCSEC_ClosureMoveType_E_CLOSURE_MOVE_TYPE_OPEN; },
                     "trunkOpen");
}
bool trunkClose() {
  return sendClosure([](VCSEC_ClosureMoveRequest &r) { r.rearTrunk = VCSEC_ClosureMoveType_E_CLOSURE_MOVE_TYPE_CLOSE; },
                     "trunkClose");
}
bool chargePortOpen() {
  CarServer_ChargePortDoorOpen a = CarServer_ChargePortDoorOpen_init_default;
  return sendVehicleAction(CarServer_VehicleAction_chargePortDoorOpen_tag, &a, "chargePortOpen");
}
bool chargePortClose() {
  CarServer_ChargePortDoorClose a = CarServer_ChargePortDoorClose_init_default;
  return sendVehicleAction(CarServer_VehicleAction_chargePortDoorClose_tag, &a, "chargePortClose");
}

bool hvacOn()  {
  CarServer_HvacAutoAction a = CarServer_HvacAutoAction_init_default;
  a.power_on = true;
  return sendVehicleAction(CarServer_VehicleAction_hvacAutoAction_tag, &a, "hvacOn");
}
bool hvacOff() {
  CarServer_HvacAutoAction a = CarServer_HvacAutoAction_init_default;
  a.power_on = false;
  return sendVehicleAction(CarServer_VehicleAction_hvacAutoAction_tag, &a, "hvacOff");
}
bool setHvacTempCelsius(float target) {
  CarServer_HvacTemperatureAdjustmentAction a = CarServer_HvacTemperatureAdjustmentAction_init_default;
  a.driver_temp_celsius    = target;
  a.passenger_temp_celsius = target;
  return sendVehicleAction(CarServer_VehicleAction_hvacTemperatureAdjustmentAction_tag, &a, "hvacTemp");
}

bool chargingStart() {
  CarServer_ChargingStartStopAction a = CarServer_ChargingStartStopAction_init_default;
  a.which_charging_action = CarServer_ChargingStartStopAction_start_tag;
  return sendVehicleAction(CarServer_VehicleAction_chargingStartStopAction_tag, &a, "chargeStart");
}
bool chargingStop() {
  CarServer_ChargingStartStopAction a = CarServer_ChargingStartStopAction_init_default;
  a.which_charging_action = CarServer_ChargingStartStopAction_stop_tag;
  return sendVehicleAction(CarServer_VehicleAction_chargingStartStopAction_tag, &a, "chargeStop");
}
bool setChargeLimit(int percent) {
  CarServer_ChargingSetLimitAction a = CarServer_ChargingSetLimitAction_init_default;
  a.percent = percent;
  return sendVehicleAction(CarServer_VehicleAction_chargingSetLimitAction_tag, &a, "chargeLimit");
}
bool setChargeAmps(int amps) {
  CarServer_SetChargingAmpsAction a = CarServer_SetChargingAmpsAction_init_default;
  a.charging_amps = amps;
  return sendVehicleAction(CarServer_VehicleAction_setChargingAmpsAction_tag, &a, "chargeAmps");
}

// ---------------------------------------------------------------------------
// Polling & state-machine loop
// ---------------------------------------------------------------------------
void loop() {
  const uint32_t now = millis();

  // 1. Nothing to do without a VIN or a connection.
  if (g_vin.length() != 17) return;
  tesla_transport::requestConnect();
  if (!tesla_transport::isReady()) return;

  // 2. Pairing retry — keep sending whitelist until the car ACKs (WAIT) or OKs.
  if ((g_pair_stage == PairStage::Sending || g_pair_stage == PairStage::WaitingForTap) &&
      (int32_t)(now - g_pair_next_ms) >= 0) {
    sendWhitelistMessage();
    g_pair_next_ms = now + 4000;
  }

  // 3. Once whitelisted, bring VCSEC & Infotainment sessions online.
  if (g_paired && !g_vcsec_session_ready && (int32_t)(now - g_vcsec_handshake_ms) > 3000) {
    g_vcsec_handshake_ms = now;
    requestSessionInfo(UniversalMessage_Domain_DOMAIN_VEHICLE_SECURITY);
  }
  if (g_vcsec_session_ready && !g_infotain_session_ready &&
      (int32_t)(now - g_infotain_handshake_ms) > 3000) {
    g_infotain_handshake_ms = now;
    requestSessionInfo(UniversalMessage_Domain_DOMAIN_INFOTAINMENT);
  }

  // 4. Cheap VCSEC body-state poll (works while asleep; does not wake).
  if (g_vcsec_session_ready && (now - g_last_body_poll_ms) >= cfg::kBodyStatePollMs) {
    g_last_body_poll_ms = now;
    uint8_t buf[UniversalMessage_RoutableMessage_size];
    size_t  len = 0;
    int rc = g_cli.build_vcsec_information_request_message(
        VCSEC_InformationRequestType_INFORMATION_REQUEST_TYPE_GET_STATUS, buf, &len);
    if (rc == 0) sendBuilt(buf, len, "VCSEC get_status");
  }

  // 5. Full infotainment data only while the car is awake.
  const uint32_t vdInterval = g_state.charging ? cfg::kVehicleDataFastMs : cfg::kVehicleDataMs;
  if (g_infotain_session_ready && g_state.awake() &&
      (now - g_last_vehicle_poll_ms) >= vdInterval) {
    g_last_vehicle_poll_ms = now;
    uint8_t buf[UniversalMessage_RoutableMessage_size];
    size_t  len = 0;
    int rc = g_cli.build_car_server_get_vehicle_data_message(
        buf, &len, CarServer_GetVehicleData_getChargeState_tag);
    if (rc == 0) sendBuilt(buf, len, "GetVehicleData(charge)");
    rc = g_cli.build_car_server_get_vehicle_data_message(
        buf, &len, CarServer_GetVehicleData_getClimateState_tag);
    if (rc == 0) sendBuilt(buf, len, "GetVehicleData(climate)");
  }
}

}  // namespace tesla_client
