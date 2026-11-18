// Cardputer-Adv cooling controller for indoor cycling and treadmill running.
#include <Arduino.h>
#include <M5Cardputer.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "cooling_math.h"
#include "fan_radio.h"
#include "zero_power.h"
#include <atomic>
#include <string>
#include <cstring>
#include <cctype>

constexpr uint32_t SCAN_MS       = 8000;
constexpr uint32_t STALE_MS      = 7000;
constexpr uint32_t UI_MS         = 200;
constexpr uint32_t MIN_SWITCH_MS = 6000;
constexpr int HR_HYST = 8;
constexpr int PWR_HYST = 20;
constexpr int MAX_DEVICES = 40;
constexpr int PAGE_COUNT = 3;

struct FoundDevice {
  char name[48];
  char address[20];
  uint8_t addressType;
  int rssi;
};

// NimBLE callbacks run on a different FreeRTOS task. The scanner list is
// copied under a short critical section before the display accesses it.
portMUX_TYPE foundMux = portMUX_INITIALIZER_UNLOCKED;
FoundDevice found[MAX_DEVICES] = {};
int foundCount = 0;
std::atomic<bool> scanning{false};

class BrowserCallbacks : public NimBLEScanCallbacks {
 public:
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    std::string addr = dev->getAddress().toString();
    std::string name = dev->getName();
    uint8_t type = dev->getAddress().getType();
    int rssi = dev->getRSSI();
    portENTER_CRITICAL(&foundMux);
    int slot = -1;
    for (int i = 0; i < foundCount; ++i) {
      if (strncmp(found[i].address, addr.c_str(), sizeof(found[i].address)) == 0) {
        slot = i;
        break;
      }
    }
    if (slot < 0 && foundCount < MAX_DEVICES) slot = foundCount++;
    if (slot >= 0) {
      FoundDevice& f = found[slot];
      snprintf(f.address, sizeof(f.address), "%s", addr.c_str());
      if (!name.empty()) snprintf(f.name, sizeof(f.name), "%s", name.c_str());
      f.addressType = type;
      f.rssi = rssi;
    }
    portEXIT_CRITICAL(&foundMux);
  }
  void onScanEnd(const NimBLEScanResults&, int) override { scanning.store(false); }
};
BrowserCallbacks browserCallbacks;

struct Preferred {
  String name;
  String address;
  uint8_t addressType = 0;
};
Preferred preferred[2];  // 0=HR, 1=PWR
Preferences prefs;
NimBLEClient* clients[2] = {nullptr, nullptr};

std::atomic<int> hrValue{-1};
std::atomic<uint32_t> hrMs{0};
portMUX_TYPE powerMux = portMUX_INITIALIZER_UNLOCKED;
PowerSample powerSample;

PowerSample readPower() {
  portENTER_CRITICAL(&powerMux);
  const PowerSample sample = powerSample;
  portEXIT_CRITICAL(&powerMux);
  return sample;
}

void clearPower() {
  portENTER_CRITICAL(&powerMux);
  powerSample = {};
  portEXIT_CRITICAL(&powerMux);
}

static void hrNotify(NimBLERemoteCharacteristic*, uint8_t* p, size_t n, bool) {
  if (n < 2) return;
  bool wide = (p[0] & 1) != 0;
  if (wide && n < 3) return;
  int v = wide ? (int)(uint16_t(p[1]) | (uint16_t(p[2]) << 8)) : int(p[1]);
  if (v > 0 && v < 250) {
    hrValue.store(v);
    hrMs.store(millis());
  }
}
static void powerNotify(NimBLERemoteCharacteristic*, uint8_t* p, size_t n, bool) {
  if (n < 4) return;
  // Cycling Power Measurement flags=uint16, instantaneous power=int16, LE.
  int16_t v = (int16_t)(uint16_t(p[2]) | (uint16_t(p[3]) << 8));
  portENTER_CRITICAL(&powerMux);
  powerSample.record(v, millis(), STALE_MS);
  portEXIT_CRITICAL(&powerMux);
}

enum class Activity : uint8_t { Cycling, Treadmill };
Activity activity = Activity::Cycling;
bool cycling() { return activity == Activity::Cycling; }

struct CoolingConfig {
  uint16_t restingHr = 49;
  uint16_t maxHr = 199;
  uint16_t ftp = 228;
  uint16_t hr2Pct = 54;
  uint16_t hr3Pct = 74;
  uint16_t pw2Pct = 60;
  uint16_t pw3Pct = 90;
  uint16_t zeroOffSeconds = 20;
  uint16_t downshiftSeconds = 20;
  uint16_t runMaxHr = 199;
  uint16_t runHr2Pct = 54;
  uint16_t runHr3Pct = 74;
};
CoolingConfig cooling;

int activeMaxHr() { return cycling() ? cooling.maxHr : cooling.runMaxHr; }
int hrStage2() {
  return CoolingMath::hrAtPercent(cooling.restingHr, activeMaxHr(),
                                 cycling() ? cooling.hr2Pct : cooling.runHr2Pct);
}
int hrStage3() {
  return CoolingMath::hrAtPercent(cooling.restingHr, activeMaxHr(),
                                 cycling() ? cooling.hr3Pct : cooling.runHr3Pct);
}
int powerStage2() { return CoolingMath::powerAtPercent(cooling.ftp, cooling.pw2Pct); }
int powerStage3() { return CoolingMath::powerAtPercent(cooling.ftp, cooling.pw3Pct); }

bool validHrProfile(int rest, int maxHr, int stage2, int stage3) {
  return maxHr >= 120 && maxHr <= 230 && maxHr > rest + 30 &&
         stage2 >= 30 && stage3 <= 99 && stage2 + 5 <= stage3;
}
bool validConfig(const CoolingConfig& c) {
  return c.restingHr >= 30 && c.restingHr <= 120 &&
         validHrProfile(c.restingHr, c.maxHr, c.hr2Pct, c.hr3Pct) &&
         validHrProfile(c.restingHr, c.runMaxHr, c.runHr2Pct, c.runHr3Pct) &&
         c.ftp >= 50 && c.ftp <= 600 && c.pw2Pct >= 20 &&
         c.pw3Pct <= 200 && c.pw2Pct + 5 <= c.pw3Pct &&
         c.zeroOffSeconds <= 600 && c.downshiftSeconds <= 600;
}

struct Setting {
  const char* key;
  const char* label;
  const char* unit;
  uint16_t CoolingConfig::*value;
};
constexpr Setting settings[] = {
  {"resthr", "Resting HR", " bpm", &CoolingConfig::restingHr},
  {"maxhr", "Max HR", " bpm", &CoolingConfig::maxHr},
  {"ftp", "FTP", " W", &CoolingConfig::ftp},
  {"hr2pct", "Fan 2 HRR", "%", &CoolingConfig::hr2Pct},
  {"hr3pct", "Fan 3 HRR", "%", &CoolingConfig::hr3Pct},
  {"pw2pct", "Fan 2 FTP", "%", &CoolingConfig::pw2Pct},
  {"pw3pct", "Fan 3 FTP", "%", &CoolingConfig::pw3Pct},
  {"zerooff", "Stop at 0 W", " s", &CoolingConfig::zeroOffSeconds},
  {"downshift", "Downshift delay", " s", &CoolingConfig::downshiftSeconds},
  {"runmaxhr", "Run max HR", " bpm", &CoolingConfig::runMaxHr},
  {"runhr2pct", "Run fan 2 HRR", "%", &CoolingConfig::runHr2Pct},
  {"runhr3pct", "Run fan 3 HRR", "%", &CoolingConfig::runHr3Pct},
};
constexpr int CyclingSettingCount = 9;
constexpr int TreadmillSettings[] = {0, 9, 10, 11, 8};

int settingsCount() {
  return cycling() ? CyclingSettingCount : sizeof(TreadmillSettings) / sizeof(int);
}

const Setting& settingAt(int index) {
  return settings[cycling() ? index : TreadmillSettings[index]];
}

void loadCooling() {
  CoolingConfig saved;
  for (const auto& setting : settings) {
    auto& value = saved.*(setting.value);
    value = prefs.getUShort(setting.key, value);
  }
  cooling = validConfig(saved) ? saved : CoolingConfig{};
  activity = prefs.getBool("treadmill", false) ? Activity::Treadmill : Activity::Cycling;
}

enum class Screen : uint8_t { Home, Browser, Test, Settings, Zones };
enum class Mode : uint8_t { Adaptive, HR, Power, Manual };
Screen screen = Screen::Home;
Mode mode = Mode::Adaptive;
int settingsSelected = 0;
bool editingSetting = false;
bool replaceOnNextDigit = false;
String editBuffer;
String settingsNotice = "ENTER to edit";
int targetSensor = 0;  // 0 = HR, 1 = PWR
String filterText;
int selected = 0;
String statusText = "Ready. B = scan BLE";
int requestedLevel = -1;  // The fan provides no feedback.
bool cyclingPaused = false;
int lastButton = -1;
uint32_t lastAttempt = 0;
int pendingLevel = -1;
uint32_t pendingSince = 0;
uint32_t lastUi = 0;
uint32_t lastHrTick = 0;
uint32_t lastPowerTick = 0;
float hrSmoothed = -1;
float powerSmoothed = -1;
bool scanForSaved = false;

// Build a stable display snapshot; never draw while holding the lock.
int snapshotDevices(FoundDevice* out) {
  portENTER_CRITICAL(&foundMux);
  int count = foundCount;
  memcpy(out, found, count * sizeof(FoundDevice));
  portEXIT_CRITICAL(&foundMux);
  return count;
}

String upper(String s) {
  s.toUpperCase();
  return s;
}
bool searchMatches(const FoundDevice& d) {
  if (filterText.isEmpty()) return true;
  String f = upper(filterText);
  return upper(String(d.name)).indexOf(f) >= 0 ||
         upper(String(d.address)).indexOf(f) >= 0;
}

int matching(const FoundDevice* snapshot, int n, int* indexes) {
  int count = 0;
  for (int i = 0; i < n; ++i) {
    if (searchMatches(snapshot[i])) indexes[count++] = i;
  }
  return count;
}

void loadPreferred() {
  prefs.begin("cardio54", false);
  for (int i = 0; i < 2; ++i) {
    String prefix = i == 0 ? "hr_" : "pw_";
    preferred[i].name = prefs.getString((prefix + "name").c_str(), "");
    preferred[i].address = prefs.getString((prefix + "addr").c_str(), "");
    preferred[i].addressType = prefs.getUChar((prefix + "type").c_str(), 0);
  }
}
void savePreferred(int i, const FoundDevice& d) {
  String prefix = i == 0 ? "hr_" : "pw_";
  preferred[i].name = String(d.name);
  preferred[i].address = String(d.address);
  preferred[i].addressType = d.addressType;
  prefs.putString((prefix + "name").c_str(), preferred[i].name);
  prefs.putString((prefix + "addr").c_str(), preferred[i].address);
  prefs.putUChar((prefix + "type").c_str(), preferred[i].addressType);
}

void startScan(bool autoConnectSaved = false) {
  NimBLEScan* s = NimBLEDevice::getScan();
  if (s->isScanning()) s->stop();
  portENTER_CRITICAL(&foundMux);
  foundCount = 0;
  memset(found, 0, sizeof(found));
  portEXIT_CRITICAL(&foundMux);
  selected = 0;
  scanForSaved = autoConnectSaved;
  scanning.store(true);
  if (!s->start(SCAN_MS, false, true)) {
    scanning.store(false);
    scanForSaved = false;
    statusText = "BLE scan start failed";
  } else {
    statusText = "Scanning BLE (8s)...";
  }
}

bool isConnected(int sensor) {
  return clients[sensor] && clients[sensor]->isConnected();
}

bool connectFound(int sensor, const FoundDevice& device, bool save) {
  if (NimBLEDevice::getScan()->isScanning()) {
    NimBLEDevice::getScan()->stop();
    scanning.store(false);
  }
  // Keep the other sensor's connection; only replace this sensor's client.
  if (clients[sensor]) {
    if (clients[sensor]->isConnected()) clients[sensor]->disconnect();
    NimBLEDevice::deleteClient(clients[sensor]);
    clients[sensor] = nullptr;
  }
  if (sensor == 0) {
    hrMs.store(0);
    hrSmoothed = -1;
  } else {
    clearPower();
    powerSmoothed = -1;
  }
  statusText = String("Connecting ") + (sensor == 0 ? "HR" : "PWR") + "...";
  Serial.printf("CONNECT %s name=%s address=%s type=%u\n",
                sensor == 0 ? "HR" : "PWR", device.name,
                device.address, (unsigned)device.addressType);
  NimBLEClient* c = NimBLEDevice::createClient();
  if (!c) {
    statusText = "BLE: client allocation failed";
    return false;
  }
  c->setConnectTimeout(7000);
  NimBLEAddress addr(std::string(device.address), device.addressType);
  if (!c->connect(addr)) {
    statusText = "BLE connection failed";
    NimBLEDevice::deleteClient(c);
    return false;
  }
  const NimBLEUUID serviceUUID(uint16_t(sensor == 0 ? 0x180D : 0x1818));
  const NimBLEUUID charUUID(uint16_t(sensor == 0 ? 0x2A37 : 0x2A63));
  NimBLERemoteService* service = c->getService(serviceUUID);
  NimBLERemoteCharacteristic* measurement = service ? service->getCharacteristic(charUUID) : nullptr;
  if (!measurement) {
    statusText = "Wrong device: no HR/Power service";
    c->disconnect();
    NimBLEDevice::deleteClient(c);
    return false;
  }
  if (!measurement->canNotify() || !measurement->subscribe(true, sensor == 0 ? hrNotify : powerNotify)) {
    statusText = "Device has no usable notifications";
    c->disconnect();
    NimBLEDevice::deleteClient(c);
    return false;
  }
  clients[sensor] = c;
  if (save) savePreferred(sensor, device);
  statusText = String(sensor == 0 ? "HR" : "PWR") + " connected: waiting for data";
  Serial.println(statusText);
  return true;
}

// Prefer exact address but allow known advertised name if address has changed.
int findSaved(const FoundDevice* devices, int n, int sensor) {
  if (preferred[sensor].address.isEmpty() && preferred[sensor].name.isEmpty()) return -1;
  for (int i = 0; i < n; ++i) {
    if (preferred[sensor].address.equalsIgnoreCase(devices[i].address)) return i;
  }
  if (!preferred[sensor].name.isEmpty()) {
    for (int i = 0; i < n; ++i) {
      if (preferred[sensor].name.equalsIgnoreCase(devices[i].name)) return i;
    }
  }
  return -1;
}

void connectSavedAfterScan() {
  scanForSaved = false;
  FoundDevice snapshot[MAX_DEVICES];
  int n = snapshotDevices(snapshot);
  for (int sensor = 0; sensor < (cycling() ? 2 : 1); ++sensor) {
    if (isConnected(sensor)) continue;
    int idx = findSaved(snapshot, n, sensor);
    if (idx >= 0) {
      if (connectFound(sensor, snapshot[idx], false)) {
        Serial.printf("Restored %s\n", sensor == 0 ? "HR" : "PWR");
      }
    } else if (!preferred[sensor].address.isEmpty()) {
      statusText = String(sensor == 0 ? "HR" : "PWR") + " not found; B to select";
    }
  }
}

void switchActivity() {
  activity = cycling() ? Activity::Treadmill : Activity::Cycling;
  prefs.putBool("treadmill", !cycling());
  mode = cycling() ? Mode::Adaptive : Mode::HR;
  settingsSelected = 0;
  targetSensor = 0;
  pendingLevel = -1;
  cyclingPaused = false;
  clearPower();
  powerSmoothed = -1;
  if (!cycling() && clients[1]) {
    NimBLEDevice::deleteClient(clients[1]);
    clients[1] = nullptr;
  }
  if ((!isConnected(0) && !preferred[0].address.isEmpty()) ||
      (cycling() && !isConnected(1) && !preferred[1].address.isEmpty())) {
    startScan(true);
  }
}

void fanRequest(int button) {
  lastButton = button;
  lastAttempt = millis();
  const bool simulate = !FanRadio::ready();
  if (!simulate && !FanRadio::send(button)) return;

  pendingLevel = -1;
  if (button > 0) {
    requestedLevel = button;
  } else if (requestedLevel >= 0) {
    requestedLevel = requestedLevel == 0 ? 1 : 0;
  }
  Serial.printf("%s button %d, target %d\n", simulate ? "Simulated" : "Sent",
                button, requestedLevel);
}

void updateCooling() {
  const PowerSample power = readPower();
  const uint32_t hrTick = hrMs.load();
  const uint32_t now = millis();
  const bool h = isConnected(0) && hrTick && now - hrTick < STALE_MS;
  const bool p = cycling() && isConnected(1) && power.fresh(now, STALE_MS);
  if (!h) {
    hrSmoothed = -1;
  } else if (hrSmoothed < 0 || hrTick != lastHrTick) {
    const int value = hrValue.load();
    hrSmoothed = hrSmoothed < 0 ? value : 0.3f * value + 0.7f * hrSmoothed;
    lastHrTick = hrTick;
  }
  if (!p) {
    powerSmoothed = -1;
  } else if (powerSmoothed < 0 || power.timestamp != lastPowerTick) {
    const int value = max(0, power.watts);
    powerSmoothed = powerSmoothed < 0 ? value : 0.3f * value + 0.7f * powerSmoothed;
    lastPowerTick = power.timestamp;
  }
  if (mode == Mode::Manual) return;

  const bool canSend = lastButton < 0 || now - lastAttempt >= MIN_SWITCH_MS;
  if (cycling() && cooling.zeroOffSeconds > 0) {
    if (p && power.watts > 0) cyclingPaused = false;
    if (p && power.zeroFor(now, cooling.zeroOffSeconds * 1000UL, STALE_MS)) {
      cyclingPaused = true;
    }
    if (cyclingPaused || (p && power.watts == 0 && requestedLevel <= 0)) {
      pendingLevel = -1;
      if (cyclingPaused && requestedLevel > 0 && (lastButton != 0 || canSend)) fanRequest(0);
      return;
    }
  }

  const int hrDemand = h ? CoolingMath::stage(hrSmoothed, hrStage2(), hrStage3(),
                                              HR_HYST, requestedLevel) : -1;
  const int powerDemand = p ? CoolingMath::stage(powerSmoothed, powerStage2(), powerStage3(),
                                                 PWR_HYST, requestedLevel) : -1;
  int desired = hrDemand;
  if (cycling() && mode == Mode::Power) desired = powerDemand;
  if (cycling() && mode == Mode::Adaptive) desired = max(hrDemand, powerDemand);
  if (desired < 1 || desired == requestedLevel) { pendingLevel = -1; return; }
  if (pendingLevel != desired) { pendingLevel = desired; pendingSince = now; }
  const uint32_t downshiftDelayMs = cooling.downshiftSeconds * 1000UL;
  if ((canSend || (requestedLevel == 0 && lastButton == 0)) &&
      (desired > requestedLevel || now - pendingSince >= downshiftDelayMs)) {
    fanRequest(desired);
  }
}

const char* modeName() {
  switch (mode) {
    case Mode::Adaptive: return "ADAPT";
    case Mode::HR:       return "HR";
    case Mode::Power:    return "POWER";
    case Mode::Manual:   return "MANUAL";
  }
  return "?";
}

// UI uses native 240x135 display, small font to fit both sensors and browser.
void line(int y, const String& s, uint16_t color = TFT_WHITE) {
  M5Cardputer.Display.setTextColor(color, TFT_BLACK);
  M5Cardputer.Display.setCursor(4, y);
  M5Cardputer.Display.print(s);
}
String compact(String s, int maxChars) {
  if ((int)s.length() <= maxChars) return s;
  return s.substring(0, maxChars - 1) + "~";
}
void drawHome() {
  const PowerSample power = readPower();
  const uint32_t hrTick = hrMs.load();
  const uint32_t now = millis();
  const bool h = isConnected(0) && hrTick && now - hrTick < STALE_MS;
  const bool p = cycling() && isConnected(1) && power.fresh(now, STALE_MS);
  line(2, cycling() ? "CARDIO54 / INDOOR CYCLING" : "CARDIO54 / TREADMILL RUN", TFT_CYAN);
  line(16, String("HR    ") + (h ? String(hrValue.load()) + " bpm" : "-- bpm") +
       (isConnected(0) ? "  [BLE]" : "  [--]"));
  if (cycling()) {
    line(29, String("POWER ") + (p ? String(power.watts) + " W" : "-- W") +
         (isConnected(1) ? "  [BLE]" : "  [--]"));
  }
  const String target = requestedLevel < 0 ? "?" :
                        requestedLevel == 0 ? "OFF" : String(requestedLevel) + "/3";
  line(43, String(modeName()) + "  TARGET: " + target, TFT_GREEN);
  line(57, FanRadio::status(), TFT_YELLOW);
  line(71, String("HR ") + hrStage2() + "/" + hrStage3() + " bpm" +
       (cycling() ? String("  PWR ") + powerStage2() + "/" + powerStage3() + "W"
                  : String("")), TFT_LIGHTGREY);
  if (cycling() && mode != Mode::Manual && cyclingPaused && cooling.zeroOffSeconds > 0) {
    line(85, "Paused - pedal to resume", TFT_YELLOW);
  }
  line(98, "M:ACTIVITY S:SETTINGS Z:ZONES");
  line(111, "B:BLE T:TEST R:RECONNECT");
  line(124, cycling() ? "A/H/P:MODE 0:POWER 1/2/3:FAN" : "A/H:HR 0:POWER 1/2/3:FAN");
}
void drawBrowser() {
  FoundDevice snapshot[MAX_DEVICES];
  int n = snapshotDevices(snapshot);
  int indexes[MAX_DEVICES];
  int nFiltered = matching(snapshot, n, indexes);
  selected = constrain(selected, 0, max(0, nFiltered - 1));
  line(2, String(targetSensor == 0 ? "SELECT HR" : "SELECT POWER") +
      (scanning.load() ? "  (scanning)" : "  (scan done)"), TFT_CYAN);
  line(17, ">" + compact(filterText.isEmpty() ? String("[type name / MAC]") : filterText, 34), TFT_YELLOW);
  if (!nFiltered) line(48, scanning.load() ? "Looking for BLE devices..." : "No matches. Fn+Enter: scan", TFT_LIGHTGREY);
  int start = (selected / PAGE_COUNT) * PAGE_COUNT;
  for (int row = 0; row < PAGE_COUNT && start + row < nFiltered; ++row) {
    int listIndex = start + row;
    const FoundDevice& dev = snapshot[indexes[listIndex]];
    const int y = 34 + row * 23;
    uint16_t color = listIndex == selected ? TFT_GREEN : TFT_WHITE;
    String name = strlen(dev.name) ? String(dev.name) : String("(unnamed)");
    line(y, String(listIndex == selected ? ">" : " ") + compact(name, 24) + " " + dev.rssi + "dB", color);
    line(y + 10, String(" ") + dev.address, TFT_DARKGREY);
  }
  line(106, String(nFiltered) + "/" + n + (cycling() ? " found  TAB:HR/PWR  ENT:pair" : " found  ENTER:pair"), TFT_WHITE);
  line(119, "Fn+;/. scroll  Fn+Ent scan", TFT_WHITE);
}
void drawTest() {
  const PowerSample power = readPower();
  const uint32_t hrTick = hrMs.load();
  const uint32_t now = millis();
  bool anyFresh = false;
  line(2, "BLE SENSOR TEST", TFT_CYAN);
  for (int i = 0; i < (cycling() ? 2 : 1); ++i) {
    int y = i == 0 ? 18 : 65;
    bool connected = isConnected(i);
    bool fresh = connected && (i == 0 ? hrTick && now - hrTick < STALE_MS
                                       : power.fresh(now, STALE_MS));
    anyFresh |= fresh;
    line(y, String(i == 0 ? "HR: " : "PWR: ") + (connected ? "CONNECTED" : "DISCONNECTED"),
         connected ? TFT_GREEN : TFT_YELLOW);
    String label = preferred[i].name.isEmpty() ? "(not saved)" : preferred[i].name;
    line(y + 12, compact(label, 36), TFT_LIGHTGREY);
    String val = fresh ? String(i == 0 ? hrValue.load() : power.watts) : "WAITING";
    line(y + 24, String(i == 0 ? "Heart rate: " : "Power: ") + val + (fresh ? (i == 0 ? " bpm" : " W") : ""));
  }
  line(109, anyFresh ? "Receiving sensor data" : compact(statusText, 36), TFT_YELLOW);
  line(123, "ESC HOME  TAB BLE BROWSER", TFT_WHITE);
}
void drawSettings() {
  line(2, cycling() ? "CYCLING SETTINGS" : "TREADMILL SETTINGS", TFT_CYAN);
  int first = (settingsSelected / 5) * 5;
  for (int row = 0; row < 5 && (first + row) < settingsCount(); ++row) {
    int i = first + row;
    const Setting& setting = settingAt(i);
    const uint16_t current = cooling.*(setting.value);
    String value = (editingSetting && i == settingsSelected) ? editBuffer + "_" : String(current);
    if (setting.value == &CoolingConfig::zeroOffSeconds && current == 0 &&
        !(editingSetting && i == settingsSelected)) value = "Disabled";
    else value += setting.unit;
    line(19 + row * 17, String(i == settingsSelected ? ">" : " ") +
         setting.label + ": " + value,
         i == settingsSelected ? TFT_GREEN : TFT_WHITE);
  }
  line(109, compact(settingsNotice, 38), TFT_YELLOW);
  line(122, editingSetting ? "Digits  ENTER:SAVE  ESC:CANCEL" : "UP/DOWN ENTER:EDIT Z:ZONES ESC", TFT_WHITE);
}

void drawZones() {
  line(2, cycling() ? "CYCLING ZONES (% HRR)" : "RUNNING ZONES (% HRR)", TFT_CYAN);
  const int pct[] = {34, 47, 60, 73, 87, 100};
  const char* labels[] = {"Z1", "Z2", "Z3", "Z4", "Z5"};
  for (int i = 0; i < 5; ++i) {
    int start = CoolingMath::hrAtPercent(cooling.restingHr, activeMaxHr(), pct[i]);
    int end = CoolingMath::hrAtPercent(cooling.restingHr, activeMaxHr(), pct[i+1]) - 1;
    if (i == 4) end = activeMaxHr();
    line(17 + 15*i, String(labels[i]) + ": " + pct[i] + "-" + pct[i+1] +
         "% = " + start + "-" + end + " bpm", TFT_WHITE);
  }
  line(96, "FAN: HR " + String(hrStage2()) + "/" + hrStage3() + " bpm", TFT_GREEN);
  if (cycling()) line(109, "FAN: PWR " + String(powerStage2()) + "/" + powerStage3() + " W", TFT_GREEN);
  line(122, "S:SETTINGS   ESC:HOME", TFT_WHITE);
}

void settingsEnter() {
  const Setting& setting = settingAt(settingsSelected);
  if (!editingSetting) {
    editBuffer = String(cooling.*(setting.value));
    editingSetting = true;
    replaceOnNextDigit = true;
    settingsNotice = "Type to replace, ENTER saves";
    return;
  }
  if (editBuffer.length() == 0) {
    settingsNotice = "Please enter a number";
    return;
  }
  long number = editBuffer.toInt();
  if (number < 0 || number > 1000) {
    settingsNotice = "Value out of range";
    return;
  }
  CoolingConfig proposed = cooling;
  proposed.*(setting.value) = static_cast<uint16_t>(number);
  if (!validConfig(proposed)) {
    settingsNotice = "Invalid values or order";
    return;
  }
  if (prefs.putUShort(setting.key, proposed.*(setting.value)) != sizeof(uint16_t)) {
    settingsNotice = "Could not save settings";
    return;
  }
  if (cooling.zeroOffSeconds != proposed.zeroOffSeconds) cyclingPaused = false;
  cooling = proposed;
  pendingLevel = -1;  // recompute using the updated thresholds
  editingSetting = false;
  settingsNotice = "Saved. Thresholds updated";
}

void drawUI() {
  auto& d = M5Cardputer.Display;
  d.fillScreen(TFT_BLACK);
  d.setTextSize(1);
  d.setTextFont(1);
  d.setTextWrap(false);
  if (screen == Screen::Home) drawHome();
  else if (screen == Screen::Browser) drawBrowser();
  else if (screen == Screen::Test) drawTest();
  else if (screen == Screen::Settings) drawSettings();
  else drawZones();
}

void browserEnter() {
  FoundDevice snapshot[MAX_DEVICES];
  int n = snapshotDevices(snapshot);
  int indices[MAX_DEVICES];
  int nFiltered = matching(snapshot, n, indices);
  if (!nFiltered) { statusText = "No BLE device selected"; return; }
  selected = constrain(selected, 0, nFiltered - 1);
  FoundDevice pick = snapshot[indices[selected]];
  // The slot determines which standard GATT service is verified after connect.
  bool ok = connectFound(targetSensor, pick, true);
  screen = Screen::Test;
  if (!ok) Serial.println(statusText);
}
void processKeyboard() {
  M5Cardputer.update();
  if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed()) return;
  Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();
  if (screen == Screen::Settings) {
    if (keys.esc) {
      if (editingSetting) {
        editingSetting = false;
        settingsNotice = "Edit cancelled";
      } else {
        screen = Screen::Home;
      }
      return;
    }
    if (editingSetting) {
      if (keys.enter) { settingsEnter(); return; }
      if (keys.del || keys.backspace) {
        replaceOnNextDigit = false;
        if (!editBuffer.isEmpty()) editBuffer.remove(editBuffer.length()-1);
        return;
      }
      if (!keys.fn && !keys.ctrl && !keys.alt) {
        for (char ch : keys.word) {
          if (ch >= '0' && ch <= '9' && editBuffer.length() < 4) {
            if (replaceOnNextDigit) { editBuffer = ""; replaceOnNextDigit = false; }
            editBuffer += ch;
          }
        }
      }
      return;
    }
    if (keys.up) { settingsSelected = max(0, settingsSelected-1); return; }
    if (keys.down) { settingsSelected = min(settingsCount()-1, settingsSelected+1); return; }
    if (keys.enter) { settingsEnter(); return; }
    for (char ch : keys.word) {
      if (ch == 'z' || ch == 'Z') { screen = Screen::Zones; return; }
    }
    return;
  }
  if (screen == Screen::Zones) {
    if (keys.esc) { screen = Screen::Home; return; }
    for (char ch : keys.word) {
      if (ch == 's' || ch == 'S') { screen = Screen::Settings; return; }
    }
    return;
  }
  if (keys.esc) { screen = Screen::Home; return; }

  if (screen == Screen::Browser) {
    if (keys.tab && cycling()) { targetSensor ^= 1; selected = 0; return; }
    if (keys.up)   { selected = max(0, selected - 1); return; }
    if (keys.down) { selected++; return; }
    if (keys.fn && keys.enter) { startScan(false); return; }
    if (keys.enter) { browserEnter(); return; }
    if (keys.del || keys.backspace) {
      if (!filterText.isEmpty()) filterText.remove(filterText.length() - 1);
      selected = 0;
      return;
    }
    if (!keys.fn && !keys.ctrl && !keys.alt) {
      for (char ch : keys.word) {
        if (ch >= 32 && ch <= 126 && filterText.length() < 40) filterText += ch;
      }
      if (keys.space && filterText.length() < 40) filterText += ' ';
      selected = 0;
    }
    return;
  }

  if (screen == Screen::Test) {
    if (keys.tab) {
      screen = Screen::Browser;
      filterText = "";
      startScan(false);
    }
    return;
  }

  // Home keys. Browsing starts with a fresh scan; the other sensor remains connected.
  for (char ch : keys.word) {
    switch (ch) {
      case 'b': case 'B':
        screen = Screen::Browser;
        filterText = "";
        startScan(false);
        return;
      case 't': case 'T': screen = Screen::Test; return;
      case 's': case 'S': screen = Screen::Settings; return;
      case 'z': case 'Z': screen = Screen::Zones; return;
      case 'r': case 'R':
        startScan(true); return;
      case 'm': case 'M': switchActivity(); return;
      case 'a': case 'A': mode = cycling() ? Mode::Adaptive : Mode::HR; pendingLevel = -1; return;
      case 'h': case 'H': mode = Mode::HR; pendingLevel = -1; break;
      case 'p': case 'P':
        if (cycling()) { mode = Mode::Power; pendingLevel = -1; }
        return;
      case '0': case '1': case '2': case '3':
        mode = Mode::Manual;
        fanRequest(ch - '0');
        return;
    }
  }
}

void setup() {
  Serial.begin(115200);
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);  // enable keyboard
  M5Cardputer.Display.setRotation(0);
  loadPreferred();
  loadCooling();
  if (!cycling()) mode = Mode::HR;
  FanRadio::begin();
  NimBLEDevice::init("Cardio54-Controller");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&browserCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
  scan->setMaxResults(0);  // callbacks populate our own bounded list
  if (!preferred[0].address.isEmpty() || (cycling() && !preferred[1].address.isEmpty())) {
    startScan(true);  // restore saved selections when available
  }
  drawUI();
}

void loop() {
  processKeyboard();
  updateCooling();
  if (scanForSaved && !scanning.load()) connectSavedAfterScan();
  uint32_t now = millis();
  if ((uint32_t)(now - lastUi) >= UI_MS) {
    lastUi = now;
    drawUI();
  }
  delay(15);
}
