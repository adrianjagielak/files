#include "storage.h"
#include <Preferences.h>
#include <nvs_flash.h>

namespace storage {
namespace {
constexpr const char *kNs = "tesla";
Preferences g_prefs;

void openRW() { g_prefs.begin(kNs, /*readOnly=*/false); }
void openRO() { g_prefs.begin(kNs, /*readOnly=*/true); }
void close()  { g_prefs.end(); }
}  // namespace

void begin() {
  // NVS is initialized by Arduino-ESP32's startup; nothing more needed.
}

String getVin() {
  openRO();
  String v = g_prefs.getString("vin", "");
  close();
  return v;
}

void setVin(const String &vin) {
  openRW();
  g_prefs.putString("vin", vin);
  close();
}

std::vector<uint8_t> getPrivateKey() {
  openRO();
  size_t len = g_prefs.getBytesLength("pk");
  std::vector<uint8_t> out(len);
  if (len > 0) g_prefs.getBytes("pk", out.data(), len);
  close();
  return out;
}

void setPrivateKey(const uint8_t *data, size_t len) {
  openRW();
  g_prefs.putBytes("pk", data, len);
  close();
}

bool hasPrivateKey() {
  openRO();
  bool ok = g_prefs.isKey("pk") && g_prefs.getBytesLength("pk") > 0;
  close();
  return ok;
}

bool getPaired() {
  openRO();
  bool v = g_prefs.getBool("paired", false);
  close();
  return v;
}

void setPaired(bool paired) {
  openRW();
  g_prefs.putBool("paired", paired);
  close();
}

void factoryReset() {
  // Erase every NVS namespace so HomeKit + Wi-Fi + Tesla state all go.
  nvs_flash_erase();
  nvs_flash_init();
}

}  // namespace storage
