#pragma once
#include <Arduino.h>

// Firmware identity / HomeKit accessory metadata.
namespace cfg {

constexpr const char *kFwName       = "Tesla BLE Bridge";
constexpr const char *kFwVersion    = "0.1.0";
constexpr const char *kManufacturer = "ESP32-S3";
constexpr const char *kModel        = "XIAO-S3-TeslaBLE";

// Hardcoded vehicle and network credentials.
constexpr const char *kVin      = "7SAXCCE55PF393875";
constexpr const char *kWifiSsid = "Gwiezdny Jagi";
constexpr const char *kWifiPass = "Comhom-6xashi-fexgar";

// Default HomeKit pairing code used on first boot. User can override via
// the HomeSpan 'S' CLI command.
constexpr const char *kDefaultSetupCode = "46637726";
constexpr const char *kDefaultSetupId   = "TSLA";

// Xiao ESP32-S3 built-in LED (active LOW on most revisions).
constexpr int kLedPin        = 21;
constexpr bool kLedActiveLow = true;

// BOOT button — re-used as factory-reset trigger (hold ~5s).
constexpr int kBootButtonPin = 0;

// Role requested when pairing the ESP32's generated key with the vehicle.
// DRIVER is required for HVAC/charging commands and full state access.
// CHARGING_MANAGER works for wake+charging only.
// From keys.proto:
//   Keys_Role_ROLE_OWNER            = 2
//   Keys_Role_ROLE_DRIVER           = 3
//   Keys_Role_ROLE_VEHICLE_MONITOR  = 5
//   Keys_Role_ROLE_CHARGING_MANAGER = 6
#ifndef TESLA_KEY_ROLE
#define TESLA_KEY_ROLE Keys_Role_ROLE_DRIVER
#endif

// Polling intervals (milliseconds).
constexpr uint32_t kBodyStatePollMs   = 10'000;   // VCSEC body status (cheap, does not wake).
constexpr uint32_t kVehicleDataMs     = 30'000;   // Infotainment polling when awake.
constexpr uint32_t kVehicleDataFastMs = 10'000;   // While charging / active.
constexpr uint32_t kConnectRetryMs    = 5'000;
constexpr uint32_t kCommandTimeoutMs  = 5'000;

// Max protobuf sizes. Match generated .options values.
constexpr size_t kMaxMsgSize = 1024;

}  // namespace cfg
