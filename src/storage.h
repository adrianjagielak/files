#pragma once
#include <Arduino.h>
#include <vector>

// Persistent configuration stored in the "tesla" NVS namespace.
// Holds the VIN and the NIST-P256 private key (PEM) that acts as the
// ESP32's identity with the vehicle.
namespace storage {

void begin();

// VIN.
String  getVin();
void    setVin(const String &vin);

// Tesla private key (PEM, as produced by TeslaBLE::Client::get_private_key).
std::vector<uint8_t> getPrivateKey();
void                 setPrivateKey(const uint8_t *data, size_t len);
bool                 hasPrivateKey();

// Pairing status with the vehicle (set once the whitelist operation has
// been accepted and the first signed command succeeds).
bool getPaired();
void setPaired(bool paired);

// Wipe everything (Wi-Fi, HomeKit, Tesla keys, VIN). Called on factory reset.
void factoryReset();

}  // namespace storage
