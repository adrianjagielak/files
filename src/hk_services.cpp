#include "hk_services.h"

#include "tesla_client.h"

namespace hk {
namespace {

// ---------------------------------------------------------------------------
// Door Lock
// ---------------------------------------------------------------------------
struct LockService : Service::LockMechanism {
  Characteristic::LockCurrentState *cur;
  Characteristic::LockTargetState  *tgt;

  LockService() : Service::LockMechanism() {
    cur = new Characteristic::LockCurrentState(3);  // 3 = Unknown at boot
    tgt = new Characteristic::LockTargetState(1);
  }
  boolean update() override {
    return tgt->getNewVal() == 1 ? tesla_client::lockDoors() : tesla_client::unlockDoors();
  }
};

// ---------------------------------------------------------------------------
// Climate (HeaterCooler). Active → hvac on/off; thresholds → target temp.
// ---------------------------------------------------------------------------
struct ClimateService : Service::HeaterCooler {
  Characteristic::Active                       *active;
  Characteristic::CurrentHeaterCoolerState     *curState;
  Characteristic::TargetHeaterCoolerState      *tgtState;
  Characteristic::CurrentTemperature           *curTemp;
  Characteristic::HeatingThresholdTemperature  *heatTo;
  Characteristic::CoolingThresholdTemperature  *coolTo;

  ClimateService() : Service::HeaterCooler() {
    active   = new Characteristic::Active(0);
    curState = new Characteristic::CurrentHeaterCoolerState(0);
    tgtState = new Characteristic::TargetHeaterCoolerState(0);  // 0 = AUTO
    curTemp  = new Characteristic::CurrentTemperature(20);
    heatTo   = new Characteristic::HeatingThresholdTemperature(20);
    heatTo->setRange(15, 28, 0.5);
    coolTo   = new Characteristic::CoolingThresholdTemperature(22);
    coolTo->setRange(15, 28, 0.5);
  }

  boolean update() override {
    if (active->updated()) {
      bool on = active->getNewVal();
      (on ? tesla_client::hvacOn() : tesla_client::hvacOff());
    }
    if (heatTo->updated() || coolTo->updated()) {
      float target = (heatTo->getNewVal<float>() + coolTo->getNewVal<float>()) / 2.0f;
      tesla_client::setHvacTempCelsius(target);
    }
    return true;
  }
};

// ---------------------------------------------------------------------------
// Battery (used for SoC display).
// ---------------------------------------------------------------------------
struct BatteryService : Service::BatteryService {
  Characteristic::BatteryLevel     *level;
  Characteristic::ChargingState    *charging;
  Characteristic::StatusLowBattery *low;
  BatteryService() : Service::BatteryService() {
    level    = new Characteristic::BatteryLevel(100);
    charging = new Characteristic::ChargingState(0);
    low      = new Characteristic::StatusLowBattery(0);
  }
};

// ---------------------------------------------------------------------------
// Outside temperature sensor.
// ---------------------------------------------------------------------------
struct OutsideTempService : Service::TemperatureSensor {
  Characteristic::CurrentTemperature *temp;
  OutsideTempService() : Service::TemperatureSensor() {
    temp = new Characteristic::CurrentTemperature(0);
    temp->setRange(-40, 70);
  }
};

// ---------------------------------------------------------------------------
// Momentary action switch. HomeKit switch that auto-resets to off after
// firing the configured Tesla command.
// ---------------------------------------------------------------------------
struct MomentaryService : Service::Switch {
  Characteristic::On *on;
  const char *what;
  bool (*fn)();

  MomentaryService(const char *label, bool (*action)()) : Service::Switch(), what(label), fn(action) {
    on = new Characteristic::On(0);
  }
  boolean update() override {
    if (on->getNewVal()) {
      LOG1("Momentary action: %s\n", what);
      fn();
      // Auto-reset after a short delay (handled via loop()); for now clear immediately.
      on->setVal(false, /*notify=*/false);
    }
    return true;
  }
};

// ---------------------------------------------------------------------------
// Module-scoped pointers for state refresh.
// ---------------------------------------------------------------------------
LockService       *g_lock         = nullptr;
ClimateService    *g_climate      = nullptr;
BatteryService    *g_battery      = nullptr;
OutsideTempService *g_outsideTemp = nullptr;

void addAccessoryInfo(const char *name) {
  new Service::AccessoryInformation();
  new Characteristic::Identify();
  new Characteristic::Name(name);
  new Characteristic::Manufacturer("xiao-esp32s3");
  new Characteristic::Model("Tesla BLE Bridge");
  new Characteristic::SerialNumber("TSLA-BLE-001");
  new Characteristic::FirmwareRevision("0.1.0");
}

}  // namespace

void buildAccessories() {
  homeSpan.setLogLevel(1);

  // Accessory 1: bridge root
  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Identify();
  new Characteristic::Name("Tesla Bridge");
  new Characteristic::Manufacturer("xiao-esp32s3");
  new Characteristic::Model("Tesla BLE Bridge");
  new Characteristic::SerialNumber("TSLA-BLE-BRIDGE");
  new Characteristic::FirmwareRevision("0.1.0");

  // 2: Lock
  new SpanAccessory();
  addAccessoryInfo("Tesla Lock");
  g_lock = new LockService();

  // 3: Climate
  new SpanAccessory();
  addAccessoryInfo("Tesla Climate");
  g_climate = new ClimateService();

  // 4: Battery (separate accessory so iOS shows dedicated tile).
  new SpanAccessory();
  addAccessoryInfo("Tesla Battery");
  g_battery = new BatteryService();

  // 5: Outside temp
  new SpanAccessory();
  addAccessoryInfo("Tesla Outside");
  g_outsideTemp = new OutsideTempService();

  // Momentary switches.
  struct Momentary { const char *name; bool (*fn)(); };
  const Momentary momentaries[] = {
      {"Tesla Wake",        tesla_client::wakeVehicle},
      {"Tesla Honk",        tesla_client::honk},
      {"Tesla Flash",       tesla_client::flashLights},
      {"Tesla Frunk",       tesla_client::frunkOpen},
      {"Tesla Trunk",       tesla_client::trunkOpen},
      {"Tesla Charge Port", tesla_client::chargePortOpen},
  };
  for (const auto &m : momentaries) {
    new SpanAccessory();
    addAccessoryInfo(m.name);
    new MomentaryService(m.name, m.fn);
  }
}

void refreshFromVehicle() {
  const auto &s = tesla_client::getState();

  // Lock: Apple HomeKit LockCurrentState: 0=Unsecured, 1=Secured, 2=Jammed, 3=Unknown.
  if (g_lock) {
    int hk_lock = 3;
    if (s.lockState == 1 || s.lockState == 2) hk_lock = 1;
    else if (s.lockState == 0 || s.lockState == 3) hk_lock = 0;
    g_lock->cur->setVal(hk_lock);
  }

  // Climate.
  if (g_climate) {
    g_climate->active->setVal(s.climateOn ? 1 : 0);
    g_climate->curState->setVal(s.climateOn ? 2 : 1);  // 1=Inactive, 2=Idle/running
    if (!isnan(s.insideTempC)) g_climate->curTemp->setVal(s.insideTempC);
    if (!isnan(s.driverTempSetpointC)) {
      g_climate->heatTo->setVal(s.driverTempSetpointC);
      g_climate->coolTo->setVal(s.driverTempSetpointC);
    }
  }

  // Battery.
  if (g_battery && s.batteryLevelPct >= 0) {
    g_battery->level->setVal(s.batteryLevelPct);
    g_battery->charging->setVal(s.charging ? 1 : 0);
    g_battery->low->setVal(s.batteryLevelPct < 20 ? 1 : 0);
  }

  // Outside temp.
  if (g_outsideTemp && !isnan(s.outsideTempC)) g_outsideTemp->temp->setVal(s.outsideTempC);
}

}  // namespace hk
