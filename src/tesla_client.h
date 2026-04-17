#pragma once
#include <Arduino.h>
#include <functional>

// High-level Tesla vehicle client built on top of the transport layer.
// Owns a TeslaBLE::Client (from the vendored library) and runs the handshake,
// pairing and command-response state machines.
namespace tesla_client {

enum class PairStage {
  Idle,          // not pairing
  Sending,       // whitelist message sent, awaiting acceptance
  WaitingForTap, // vehicle replied "WAIT" — user must tap key card on console
  Succeeded,
  Failed,
};

struct State {
  // VCSEC body-controller-state fields (cheap poll, works while asleep).
  int  lockState       = -1;    // -1=unknown, 0=UNLOCKED, 1=LOCKED, 2=INTERNAL_LOCKED, 3=SELECTIVE_UNLOCKED
  int  sleepStatus     = 0;     // 0=unknown, 1=awake, 2=asleep
  int  userPresence    = 0;
  int  chargeFlapState = -1;

  // Infotainment-derived fields (only while awake).
  int   batteryLevelPct       = -1;
  float insideTempC           = NAN;
  float outsideTempC          = NAN;
  bool  climateOn             = false;
  float driverTempSetpointC   = NAN;
  float chargerPowerKw        = 0.f;
  bool  charging              = false;
  float batteryRangeKm        = NAN;

  uint32_t lastVcsecUpdateMs    = 0;
  uint32_t lastInfotainUpdateMs = 0;

  bool awake() const { return sleepStatus == 1; }
  bool locked() const { return lockState == 1 || lockState == 2; }
};

using StateChangedCallback = std::function<void()>;
using PairingCallback      = std::function<void(PairStage, const char *msg)>;

void begin();

// Vehicle identification — must be set (17-char VIN) before connect.
void setVin(const String &vin);

// Drive the state machine; call frequently from the main loop.
void loop();

bool isPaired();
bool isSessionReady();    // VCSEC session authenticated
bool isInfotainmentReady();

const State &getState();

// Register callbacks.
void onStateChanged(StateChangedCallback cb);
void onPairingProgress(PairingCallback cb);

// Start BLE key-pair request with the vehicle. The car will display
// "Tap key card to approve" on its center console; the user must tap.
// Non-blocking: call `loop()` to advance the state machine.
void startPairing();
void cancelPairing();

// --- Commands. All return true if the outbound message was queued
//     successfully (not that the car confirmed execution).
bool lockDoors();
bool unlockDoors();
bool wakeVehicle();
bool honk();
bool flashLights();
bool frunkOpen();
bool trunkOpen();
bool trunkClose();
bool chargePortOpen();
bool chargePortClose();
bool hvacOn();
bool hvacOff();
bool setHvacTempCelsius(float target);
bool chargingStart();
bool chargingStop();
bool setChargeLimit(int percent);
bool setChargeAmps(int amps);

}  // namespace tesla_client
