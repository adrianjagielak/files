// Tesla BLE ↔ HomeKit bridge for Seeed Studio XIAO ESP32-S3.
//
// On first boot:
//   1. A NIST-P256 key pair is generated and persisted to NVS.
//   2. The device connects automatically to the hardcoded Wi-Fi network and
//      applies the hardcoded VIN (both defined in config.h).
//   3. Issue `@P` over Serial to pair: the ESP32 scans for the vehicle,
//      connects, and sends an `add-key-request` over BLE. The Tesla center
//      console shows "Tap your key card to approve" — when the owner taps,
//      the vehicle whitelists the ESP32 and the firmware stores `paired=true`.
//
// Steady-state:
//   * BLE session handshake runs per domain (VCSEC + Infotainment).
//   * Cheap VCSEC `body-controller-state` polling every 10 s (does not wake).
//   * When the vehicle is already awake, full `GetVehicleData` for charge +
//     climate every 30 s (10 s while charging).
//   * HomeKit requests (lock/unlock, HVAC on/off, set temp, momentary tiles)
//     are forwarded as signed BLE commands.

#include <Arduino.h>
#include <HomeSpan.h>

#include "config.h"
#include "hk_services.h"
#include "led.h"
#include "storage.h"
#include "tesla_client.h"
#include "tesla_transport.h"

namespace {

// --- HomeSpan CLI command handlers ------------------------------------------

void cmdInfo(const char *) {
  const String vin = storage::getVin();
  const auto &s    = tesla_client::getState();
  Serial.printf("\n--- Tesla BLE Bridge ---\n");
  Serial.printf("  VIN            : %s\n", vin.c_str());
  Serial.printf("  Paired         : %s\n", tesla_client::isPaired()  ? "yes" : "no");
  Serial.printf("  Transport      : %s\n", tesla_transport::isReady()? "connected" : "disconnected");
  Serial.printf("  VCSEC session  : %s\n", tesla_client::isSessionReady() ? "ready" : "—");
  Serial.printf("  INFO  session  : %s\n", tesla_client::isInfotainmentReady() ? "ready" : "—");
  Serial.printf("  Lock state     : %d\n", s.lockState);
  Serial.printf("  Awake          : %s\n", s.awake() ? "yes" : "no");
  Serial.printf("  Battery        : %d%%\n", s.batteryLevelPct);
  Serial.printf("  Inside / outside: %.1f°C / %.1f°C\n", s.insideTempC, s.outsideTempC);
  Serial.printf("  Climate on     : %s\n", s.climateOn ? "yes" : "no");
  Serial.printf("  Charging       : %s (%.1f kW)\n", s.charging ? "yes" : "no", s.chargerPowerKw);
  Serial.printf("------------------------\n\n");
}

void cmdFactoryReset(const char *) {
  Serial.println("FACTORY RESET: erasing NVS (HomeKit + Wi-Fi + Tesla key) and rebooting.");
  delay(500);
  storage::factoryReset();
  ESP.restart();
}

void onWifiConnected() {
  Serial.println("Wi-Fi connected — starting BLE transport and Tesla client.");
  tesla_transport::begin();
  tesla_client::begin();
  storage::setVin(cfg::kVin);
  tesla_client::setVin(String(cfg::kVin));
  tesla_client::onStateChanged([]() { hk::refreshFromVehicle(); });
  tesla_client::onPairingProgress([](tesla_client::PairStage stage, const char *msg) {
    Serial.printf("[pair] %d: %s\n", (int)stage, msg ? msg : "");
    if (stage == tesla_client::PairStage::Succeeded) led::setMode(led::Mode::Heartbeat);
    else if (stage == tesla_client::PairStage::Failed) led::setMode(led::Mode::ErrorFlash);
    else led::setMode(led::Mode::FastBlink);
  });
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n" "=== Tesla BLE → HomeKit Bridge ===");

  storage::begin();
  led::begin();
  led::setMode(led::Mode::SlowBlink);

  homeSpan.setLogLevel(1);
  homeSpan.enableOTA();
  homeSpan.setPairingCode(cfg::kDefaultSetupCode);
  homeSpan.setQRID(cfg::kDefaultSetupId);
  homeSpan.setWifiCredentials(cfg::kWifiSsid, cfg::kWifiPass);
  homeSpan.setWifiCallback(onWifiConnected);

  new SpanUserCommand('I', "- print Tesla bridge status", cmdInfo);
  new SpanUserCommand('Z', "- factory reset (erase NVS and reboot)", cmdFactoryReset);

  homeSpan.begin(Category::Bridges, "Tesla BLE Bridge");
  hk::buildAccessories();
}

void loop() {
  homeSpan.poll();
  tesla_transport::loop();
  tesla_client::loop();
  led::loop();
}
