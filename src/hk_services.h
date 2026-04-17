#pragma once
#include <HomeSpan.h>

// HomeKit accessory layout for the Tesla bridge. A single HAP Bridge
// accessory exposes:
//   1. Door lock            → LockMechanism
//   2. Climate              → HeaterCooler + CurrentTemperature (inside)
//   3. Battery              → BatteryService (SoC + charging state)
//   4. Outside temperature  → TemperatureSensor
//   5. Frunk / Trunk / Charge Port / Honk / Flash / Wake → Switch tiles
namespace hk {

void buildAccessories();

// Refresh all HomeKit characteristics from the latest tesla_client state.
// Called whenever the Tesla client signals new data.
void refreshFromVehicle();

}  // namespace hk
