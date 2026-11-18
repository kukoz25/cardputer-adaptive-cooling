#include "fan_radio.h"

#include <Arduino.h>
#include <RadioLib.h>
#include <SD.h>
#include <SPI.h>
#include <driver/rmt.h>
#include <cstdlib>

namespace {

constexpr int CapSelect = 5;
constexpr int CapData = 15;
constexpr int CapBandSelect = 13;
constexpr int SdSelect = 12;
constexpr int NfcSelect = 6;
constexpr int SpiClock = 40;
constexpr int SpiMosi = 14;
constexpr int SpiMiso = 39;
constexpr auto Channel = RMT_CHANNEL_0;
constexpr size_t MaxTimings = 512;
constexpr int MaxDurationUs = 400000;

constexpr const char* Commands[] = {
    "/cardio54/cardio54_power_clean.sub",
    "/cardio54/cardio54_speed1_clean.sub",
    "/cardio54/cardio54_speed2_clean.sub",
    "/cardio54/cardio54_speed3_clean.sub",
};
struct Waveform {
  rmt_item32_t items[MaxTimings / 2] = {};
  size_t count = 0;
};
Waveform waveforms[4];
Module module(CapSelect, CapData, RADIOLIB_NC, RADIOLIB_NC, SPI);
CC1101 radio(&module);
bool available = false;
const char* message = "RF: simulation";

bool readWaveform(const char* path, Waveform& wave) {
  File file = SD.open(path);
  if (!file || file.size() > 8192) return false;

  bool frequencyOk = false;
  bool formatOk = false;
  bool modulationOk = false;
  size_t timings = 0;
  int totalUs = 0;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.startsWith("Frequency:")) frequencyOk = line == "Frequency: 433920000";
    if (line.startsWith("Protocol:")) formatOk = line == "Protocol: RAW";
    if (line.startsWith("Preset:")) modulationOk = line == "Preset: FuriHalSubGhzPresetOOK650Async";
    if (!line.startsWith("RAW_Data:")) continue;
    const char* cursor = line.c_str() + 9;
    while (*cursor) {
      while (*cursor == ' ' || *cursor == '\t') ++cursor;
      if (!*cursor) break;
      char* end;
      const long duration = std::strtol(cursor, &end, 10);
      const bool high = timings % 2 == 0;
      if (end == cursor || timings >= MaxTimings || duration < -32767 ||
          duration > 32767 || (high ? duration <= 0 : duration >= 0)) return false;
      const int width = abs(static_cast<int>(duration));
      totalUs += width;
      if (totalUs > MaxDurationUs) return false;
      auto& item = wave.items[timings / 2];
      if (high) {
        item.level0 = 1;
        item.duration0 = width;
      } else {
        item.level1 = 0;
        item.duration1 = width;
      }
      ++timings;
      cursor = end;
    }
  }
  wave.count = timings / 2;
  return frequencyOk && formatOk && modulationOk && timings && timings % 2 == 0;
}

}  // namespace

namespace FanRadio {

void begin() {
  pinMode(CapSelect, OUTPUT);
  digitalWrite(CapSelect, HIGH);
  pinMode(NfcSelect, OUTPUT);
  digitalWrite(NfcSelect, HIGH);
  pinMode(SdSelect, OUTPUT);
  digitalWrite(SdSelect, HIGH);
  SPI.begin(SpiClock, SpiMiso, SpiMosi, SdSelect);
  if (!SD.begin(SdSelect, SPI)) {
    message = "RF simulation: microSD missing";
    return;
  }
  for (size_t i = 0; i < 4; ++i) {
    if (!readWaveform(Commands[i], waveforms[i])) {
      Serial.printf("Missing or invalid fan command: %s\n", Commands[i]);
      message = "RF simulation: check .sub files";
      SD.end();
      return;
    }
  }
  SD.end();  // All waveforms are now in RAM.

  pinMode(CapBandSelect, OUTPUT);
  digitalWrite(CapBandSelect, LOW);
  if (radio.begin(433.92f, 4.8f, 5.0f, 270.0f, 0) != RADIOLIB_ERR_NONE ||
      radio.setOOK(true) != RADIOLIB_ERR_NONE ||
      radio.setDIOMapping(2, RADIOLIB_CC1101_GDOX_HW_TO_0 | RADIOLIB_CC1101_GDO2_INV)
          != RADIOLIB_ERR_NONE) {
    message = "RF simulation: CC1101 init failed";
    return;
  }
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(static_cast<gpio_num_t>(CapData), Channel);
  config.clk_div = 80;  // 1 tick per microsecond on the 80 MHz APB clock.
  if (rmt_config(&config) != ESP_OK || rmt_driver_install(Channel, 0, 0) != ESP_OK) {
    message = "RF simulation: RMT init failed";
    return;
  }
  available = true;
  message = "RF: ready";
}

bool ready() { return available; }
const char* status() { return message; }

bool send(int button) {
  if (!available || button < 0 || button > 3) return false;

  const bool started = radio.transmitDirectAsync() == RADIOLIB_ERR_NONE;
  const auto& wave = waveforms[button];
  const bool sent = started && rmt_write_items(Channel, wave.items, wave.count, true) == ESP_OK;
  if (radio.standby() != RADIOLIB_ERR_NONE) {
    available = false;
    message = "RF: standby failed, restart";
  } else {
    message = sent ? "RF: command sent" : "RF: transmission failed";
  }
  return sent;
}

}  // namespace FanRadio
