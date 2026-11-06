/*
  Cardio54 Controller - Cardputer-Adv BLE scanner + Polar H10 + Wahoo KICKR CORE
  Hardware: M5Stack Cardputer-Adv with official Cap CC1101 (U219).

  Dependencies:
    - M5Cardputer 1.1.0+ (M5Unified and M5GFX dependencies)
    - NimBLE-Arduino 2.x (API checked against 2.x documentation)
  Board: M5Stack -> M5Cardputer (board package >= 3.2.2)

  This is a BLE and cooling-logic TEST firmware. There is NO RF transmission
  until we obtain legitimate pilot captures and verify the RF protocol.

  Browser:
    Home B: BLE browser (new scan)
    Browser TAB: choose HR / PWR target
    Type: filter advertised names/addresses (not a Bluetooth pairing PIN)
    Fn + ; : UP     Fn + . : DOWN
    ENTER: connect and verify required GATT service + notification
    Fn + ENTER: scan again
    Fn + ` (ESC): back home

  Home:
    A: Adaptive   H: HR only   P: Power only
    1/2/3: manual requested level (not physically sent)
    S: Settings   Z: zones   T: Test/status   R: reconnect   B: browser
  Settings:
    UP / DOWN: select field; ENTER: edit value; ENTER: save value
    ESC while editing: cancel edit; otherwise: HOME
    Z: zone preview (when not editing)

  BLE services:
    HR    0x180D, measurement 0x2A37
    Power 0x1818, measurement 0x2A63
*/

#include <Arduino.h>
#include <M5Cardputer.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "cooling_math.h"
#include <atomic>
#include <string>
#include <cstring>
#include <cctype>

// Official Cap CC1101 mapping (reserved, not used until we add RF replay).
constexpr int PIN_CC1101_CS    = 5;
constexpr int PIN_CC1101_GDO0  = 15;
constexpr int PIN_CC1101_RFSW0 = 13;
constexpr int PIN_CAP_SCK      = 40;
constexpr int PIN_CAP_MOSI     = 14;
constexpr int PIN_CAP_MISO     = 39;

constexpr uint32_t SCAN_MS       = 8000;
constexpr uint32_t STALE_MS      = 7000;
constexpr uint32_t UI_MS         = 200;
constexpr uint32_t MIN_SWITCH_MS = 6000;
constexpr uint32_t DOWN_HOLD_MS  = 20000;
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
std::atomic<int> powerValue{-1};
std::atomic<uint32_t> hrMs{0};
std::atomic<uint32_t> powerMs{0};

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
  powerValue.store(v < 0 ? 0 : int(v));
  powerMs.store(millis());
}

struct CoolingConfig {
  uint16_t restingHr = 49;
  uint16_t maxHr = 199;
  uint16_t ftp = 228;
  uint16_t hr2Pct = 54;   // HRR: 49 + 150 * 54% => 130 bpm
  uint16_t hr3Pct = 74;   // HRR: 49 + 150 * 74% => 160 bpm
  uint16_t pw2Pct = 60;   // FTP: 228 * 60% => 137 W
  uint16_t pw3Pct = 90;   // FTP: 228 * 90% => 206 W
};
CoolingConfig cooling;

int hrStage2() { return CoolingMath::hrAtPercent(cooling.restingHr, cooling.maxHr, cooling.hr2Pct); }
int hrStage3() { return CoolingMath::hrAtPercent(cooling.restingHr, cooling.maxHr, cooling.hr3Pct); }
int powerStage2() { return CoolingMath::powerAtPercent(cooling.ftp, cooling.pw2Pct); }
int powerStage3() { return CoolingMath::powerAtPercent(cooling.ftp, cooling.pw3Pct); }

bool validConfig(const CoolingConfig& c) {
  return c.restingHr >= 30 && c.restingHr <= 120 &&
         c.maxHr >= 120 && c.maxHr <= 230 &&
         c.maxHr > c.restingHr + 30 &&
         c.ftp >= 50 && c.ftp <= 600 &&
         c.hr2Pct >= 30 && c.hr3Pct <= 99 && c.hr2Pct + 5 <= c.hr3Pct &&
         c.pw2Pct >= 20 && c.pw3Pct <= 200 && c.pw2Pct + 5 <= c.pw3Pct;
}

void loadCooling() {
  CoolingConfig saved;
  saved.restingHr = prefs.getUShort("resthr", 49);
  saved.maxHr = prefs.getUShort("maxhr", 199);
  saved.ftp = prefs.getUShort("ftp", 228);
  saved.hr2Pct = prefs.getUShort("hr2pct", 54);
  saved.hr3Pct = prefs.getUShort("hr3pct", 74);
  saved.pw2Pct = prefs.getUShort("pw2pct", 60);
  saved.pw3Pct = prefs.getUShort("pw3pct", 90);
  cooling = validConfig(saved) ? saved : CoolingConfig{};
}
void saveCooling() {
  prefs.putUShort("resthr", cooling.restingHr);
  prefs.putUShort("maxhr", cooling.maxHr);
  prefs.putUShort("ftp", cooling.ftp);
  prefs.putUShort("hr2pct", cooling.hr2Pct);
  prefs.putUShort("hr3pct", cooling.hr3Pct);
  prefs.putUShort("pw2pct", cooling.pw2Pct);
  prefs.putUShort("pw3pct", cooling.pw3Pct);
}

constexpr int SETTINGS_FIELDS = 7;
const char* const settingLabels[SETTINGS_FIELDS] = {
  "Resting HR", "Max HR", "FTP", "Fan2 HRR %", "Fan3 HRR %", "Fan2 FTP %", "Fan3 FTP %"
};
uint16_t getSetting(int i) {
  switch (i) {
    case 0: return cooling.restingHr;
    case 1: return cooling.maxHr;
    case 2: return cooling.ftp;
    case 3: return cooling.hr2Pct;
    case 4: return cooling.hr3Pct;
    case 5: return cooling.pw2Pct;
    default: return cooling.pw3Pct;
  }
}
void setSetting(CoolingConfig& c, int i, uint16_t value) {
  switch (i) {
    case 0: c.restingHr = value; break;
    case 1: c.maxHr = value; break;
    case 2: c.ftp = value; break;
    case 3: c.hr2Pct = value; break;
    case 4: c.hr3Pct = value; break;
    case 5: c.pw2Pct = value; break;
    default: c.pw3Pct = value; break;
  }
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
int requestedLevel = 1;  // virtual until RF implementation
int pendingLevel = -1;
uint32_t pendingSince = 0;
uint32_t lastSwitch = 0;
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
  hrMs.store(sensor == 0 ? 0 : hrMs.load());
  powerMs.store(sensor == 1 ? 0 : powerMs.load());
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
  for (int sensor = 0; sensor < 2; ++sensor) {
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

// Cooling logic (fan output currently virtual). Thresholds are calculated
// from the editable settings and are not fixed in the source code.
void fanRequest(int level) {
  if (level == requestedLevel) return;
  requestedLevel = level;
  lastSwitch = millis();
  pendingLevel = -1;
  // TODO: send real RF when Vacmaster remote captures and semantics are known.
  Serial.printf("[VIRTUAL FAN; CC1101 NOT TRANSMITTING] request level %d\n", level);
}
void updateCooling() {
  uint32_t now = millis();
  bool h = hrMs.load() && (uint32_t)(now - hrMs.load()) < STALE_MS;
  bool p = powerMs.load() && (uint32_t)(now - powerMs.load()) < STALE_MS;
  if (h && hrMs.load() != lastHrTick) {
    int value = hrValue.load();
    hrSmoothed = hrSmoothed < 0 ? value : 0.3f * value + 0.7f * hrSmoothed;
    lastHrTick = hrMs.load();
  }
  if (p && powerMs.load() != lastPowerTick) {
    int value = powerValue.load();
    powerSmoothed = powerSmoothed < 0 ? value : 0.3f * value + 0.7f * powerSmoothed;
    lastPowerTick = powerMs.load();
  }
  if (mode == Mode::Manual) return;
  int a = h ? CoolingMath::stage(hrSmoothed, hrStage2(), hrStage3(), HR_HYST, requestedLevel) : -1;
  int b = p ? CoolingMath::stage(powerSmoothed, powerStage2(), powerStage3(), PWR_HYST, requestedLevel) : -1;
  int desired = -1;
  if (mode == Mode::HR) desired = a;
  if (mode == Mode::Power) desired = b;
  if (mode == Mode::Adaptive) desired = max(a, b);
  if (desired < 1 || desired == requestedLevel) { pendingLevel = -1; return; }
  if (pendingLevel != desired) { pendingLevel = desired; pendingSince = now; }
  if ((uint32_t)(now - lastSwitch) < MIN_SWITCH_MS) return;
  if (desired > requestedLevel || (uint32_t)(now - pendingSince) >= DOWN_HOLD_MS) {
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
  auto& d = M5Cardputer.Display;
  uint32_t now = millis();
  bool h = hrMs.load() && uint32_t(now - hrMs.load()) < STALE_MS;
  bool p = powerMs.load() && uint32_t(now - powerMs.load()) < STALE_MS;
  line(2, "CARDIO54  ADV - V0.2", TFT_CYAN);
  line(19, String("HR    ") + (h ? String(hrValue.load()) + " bpm" : "-- bpm") +
       (isConnected(0) ? "  [BLE]" : "  [--]"));
  line(35, String("POWER ") + (p ? String(powerValue.load()) + " W" : "-- W") +
       (isConnected(1) ? "  [BLE]" : "  [--]"));
  line(52, String("MODE: ") + modeName() + "  TARGET: " + requestedLevel + "/3", TFT_GREEN);
  line(68, "RF OUTPUT: DISABLED (test)", TFT_YELLOW);
  line(82, "HR " + String(hrStage2()) + "/" + hrStage3() +
       " bpm   PWR " + powerStage2() + "/" + powerStage3() + "W", TFT_LIGHTGREY);
  line(103, "S:SETTINGS Z:ZONES B:BLE T:TEST", TFT_WHITE);
  line(119, "A/H/P:MODE  1/2/3:MAN R:BLE", TFT_WHITE);
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
  line(106, String(nFiltered) + "/" + n + " found   TAB:HR/PWR   ENT:pair", TFT_WHITE);
  line(119, "Fn+;/. scroll  Fn+Ent scan", TFT_WHITE);
}
void drawTest() {
  uint32_t now = millis();
  line(2, "BLE SENSOR TEST", TFT_CYAN);
  for (int i = 0; i < 2; ++i) {
    int y = i == 0 ? 18 : 65;
    bool connected = isConnected(i);
    uint32_t tick = i == 0 ? hrMs.load() : powerMs.load();
    bool fresh = tick && (uint32_t)(now - tick) < STALE_MS;
    line(y, String(i == 0 ? "HR: " : "PWR: ") + (connected ? "CONNECTED" : "DISCONNECTED"),
         connected ? TFT_GREEN : TFT_YELLOW);
    String label = preferred[i].name.isEmpty() ? "(not saved)" : preferred[i].name;
    line(y + 12, compact(label, 36), TFT_LIGHTGREY);
    String val = fresh ? String(i == 0 ? hrValue.load() : powerValue.load()) : "WAITING";
    line(y + 24, String(i == 0 ? "Heart rate: " : "Power: ") + val + (fresh ? (i == 0 ? " bpm" : " W") : ""));
  }
  bool anyFresh = (hrMs.load() && uint32_t(now - hrMs.load()) < STALE_MS) ||
                  (powerMs.load() && uint32_t(now - powerMs.load()) < STALE_MS);
  line(109, anyFresh ? "Receiving sensor data" : compact(statusText, 36), TFT_YELLOW);
  line(123, "ESC HOME  TAB BLE BROWSER", TFT_WHITE);
}
void drawSettings() {
  line(2, "SETTINGS  (stored in flash)", TFT_CYAN);
  int first = (settingsSelected / 5) * 5;
  for (int row = 0; row < 5 && (first + row) < SETTINGS_FIELDS; ++row) {
    int i = first + row;
    String value = (editingSetting && i == settingsSelected) ? editBuffer + "_" : String(getSetting(i));
    String suffix = (i == 0 || i == 1) ? " bpm" : (i == 2 ? " W" : "%");
    line(19 + row * 17, String(i == settingsSelected ? ">" : " ") +
         settingLabels[i] + ": " + value + suffix,
         i == settingsSelected ? TFT_GREEN : TFT_WHITE);
  }
  line(109, compact(settingsNotice, 38), TFT_YELLOW);
  line(122, editingSetting ? "Digits  ENTER:SAVE  ESC:CANCEL" : "UP/DOWN ENTER:EDIT Z:ZONES ESC", TFT_WHITE);
}

void drawZones() {
  line(2, "CYCLING ZONES (% HRR)", TFT_CYAN);
  const int pct[] = {34, 47, 60, 73, 87, 100};
  const char* labels[] = {"Z1", "Z2", "Z3", "Z4", "Z5"};
  for (int i = 0; i < 5; ++i) {
    int start = CoolingMath::hrAtPercent(cooling.restingHr, cooling.maxHr, pct[i]);
    int end = CoolingMath::hrAtPercent(cooling.restingHr, cooling.maxHr, pct[i+1]) - 1;
    if (i == 4) end = cooling.maxHr;
    line(17 + 15*i, String(labels[i]) + ": " + pct[i] + "-" + pct[i+1] +
         "% = " + start + "-" + end + " bpm", TFT_WHITE);
  }
  line(96, "FAN: HR " + String(hrStage2()) + "/" + hrStage3() + " bpm", TFT_GREEN);
  line(109, "FAN: PWR " + String(powerStage2()) + "/" + powerStage3() + " W", TFT_GREEN);
  line(122, "S:SETTINGS   ESC:HOME", TFT_WHITE);
}

void settingsEnter() {
  if (!editingSetting) {
    editBuffer = String(getSetting(settingsSelected));
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
  if (number < 1 || number > 1000) {
    settingsNotice = "Value out of range";
    return;
  }
  CoolingConfig proposed = cooling;
  setSetting(proposed, settingsSelected, static_cast<uint16_t>(number));
  if (!validConfig(proposed)) {
    settingsNotice = "Invalid values or order";
    return;
  }
  cooling = proposed;
  saveCooling();
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
    if (keys.down) { settingsSelected = min(SETTINGS_FIELDS-1, settingsSelected+1); return; }
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
    if (keys.tab) { targetSensor ^= 1; selected = 0; return; }
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
      case 'a': case 'A': mode = Mode::Adaptive; pendingLevel = -1; break;
      case 'h': case 'H': mode = Mode::HR; pendingLevel = -1; break;
      case 'p': case 'P': mode = Mode::Power; pendingLevel = -1; break;
      case '1': case '2': case '3':
        mode = Mode::Manual;
        fanRequest(ch - '0');
        break;
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
  NimBLEDevice::init("Cardio54-Controller");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&browserCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
  scan->setMaxResults(0);  // callbacks populate our own bounded list
  if (!preferred[0].address.isEmpty() || !preferred[1].address.isEmpty()) {
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
