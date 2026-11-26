#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <stddef.h>
#include "web.h"

// ESP32 DevKit V1 / ESP32-WROOM. Verify module polarity before connecting loads.
constexpr bool RELAY_ACTIVE_LOW = true;
constexpr uint8_t RELAY_PINS[] = {16, 17, 18, 5, 22, 23};
constexpr uint8_t MODE_PIN = 27; // Closed to GND = Manual
constexpr uint8_t RED_PIN = 21;
constexpr uint8_t GREEN_PIN = 19; // Работа: основной цикл достиг этапа RUNNING.
constexpr uint8_t YELLOW_PIN = 2; // Запуск: от Старт до проверки света.
// Названия реле в HTML меняются в web.h, в массиве RELAY_NAMES.
// После изменения web.h заново загрузите скетч и обновите страницу планшета.
constexpr uint8_t AIR_PIN = 14, WATER_PIN = 4, LIGHT_PIN = 32;
OneWire airWire(AIR_PIN), waterWire(WATER_PIN);
DallasTemperature airSensor(&airWire), waterSensor(&waterWire);
uint32_t conversionSince = 0;
float rawAirTemperature = NAN, rawWaterTemperature = NAN;
const char *AP_NAME = "ESP32-Control";
const char *AP_PASSWORD = "Control1234";

struct Settings {
  uint32_t version = 2;
  float legacyFanSeconds = 15, legacyRelaySeconds = 5, legacyLightSeconds = 1;
  float lightThreshold = 50, pumpTemperature = 35;
  // Retained solely for binary compatibility with saved version 1/2 settings.
  float legacyNtcNominal = 10000, legacySeriesResistance = 10000, legacyBeta = 3950;
  float legacyNtcToGround = 1, lightInverted = 1;
  float airOffset = 0, waterOffset = 0, pumpMinimumSeconds = 0;
  float fanOffTemperature = 25;
  float overheatTemperature = 100;
};
Settings config;
Preferences preferences;
WebServer server(80);
enum Phase { IDLE, STARTING, RUNNING };
Phase phase = IDLE;
uint32_t sensorSince = 0;
uint32_t pumpSince = 0;
bool relays[6] = {}, manualMode = false;
String alarmMessage;
float airTemperature = NAN, waterTemperature = NAN, brightness = 0;

struct Button {
  Button(uint8_t buttonPin) : pin(buttonPin) {}
  uint8_t pin;
  bool raw = true, stable = true;
  uint32_t changed = 0;
  void begin() {
    pinMode(pin, INPUT_PULLUP);
    raw = stable = digitalRead(pin);
  }
  bool update(uint32_t now) {
    bool value = digitalRead(pin);
    if (value != raw) { raw = value; changed = now; }
    if (stable != raw && now - changed >= 35) {
      stable = raw;
      return !stable;
    }
    return false;
  }
};
Button buttons[] = {{25}, {26}, {33}, {13}}; // Manual 1, Manual 2, Start, Stop
Button modeSwitch = {MODE_PIN};

void setRelay(unsigned index, bool on) {
  relays[index] = on;
  digitalWrite(RELAY_PINS[index], (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}
void stopSequence() {
  phase = IDLE;
  for (unsigned i = 0; i < 3; ++i) setRelay(i, false);
  // Pump remains on until the water has cooled; invalid sensor keeps it on.
}
void trip(const String &reason) {
  stopSequence();
  alarmMessage = reason;
}
float readTemperature(DallasTemperature &sensor, OneWire &wire) {
  DeviceAddress address;
  wire.reset_search();
  if (!wire.search(address) || address[0] != 0x28 || OneWire::crc8(address, 7) != address[7]) return NAN;
  float value = sensor.getTempC(address);
  if (!isfinite(value) || value == DEVICE_DISCONNECTED_C || value < -55 || value > 125) return NAN;
  return value;
}
void updateTemperatures() {
  if (millis() - conversionSince < 800) return;
  rawAirTemperature = readTemperature(airSensor, airWire);
  rawWaterTemperature = readTemperature(waterSensor, waterWire);
  airTemperature = rawAirTemperature + config.airOffset;
  waterTemperature = rawWaterTemperature + config.waterOffset;
  airSensor.requestTemperatures();
  waterSensor.requestTemperatures();
  conversionSince = millis();
}
void sampleSensors() {
  updateTemperatures();
  uint32_t sum = 0;
  for (int i = 0; i < 8; ++i) sum += analogRead(LIGHT_PIN);
  brightness = sum * 100.0f / (8.0f * 4095.0f);
  if (config.lightInverted == 1) brightness = 100 - brightness;
}
bool startSequence() {
  sampleSensors();
  if (isfinite(waterTemperature) && waterTemperature >= config.overheatTemperature) {
    trip("Перегрев воды");
    return false;
  }
  if (!buttons[3].stable || manualMode || phase != IDLE || !alarmMessage.isEmpty() || !isfinite(waterTemperature)) return false;
  setRelay(0, true);
  setRelay(4, true);
  setRelay(5, true);
  pumpSince = millis();
  phase = STARTING;
  return true;
}
void runControl(uint32_t now) {
  if (isfinite(waterTemperature) && waterTemperature >= config.overheatTemperature)
    trip("Перегрев воды");
  if (phase != IDLE && !isfinite(waterTemperature)) trip("Ошибка датчика воды");
  const uint32_t elapsed = now - pumpSince;
  if (phase != IDLE && phase != RUNNING) {
    if (elapsed >= 21000) {
      setRelay(0, false);
      setRelay(1, false);
      sampleSensors();
      if (!isfinite(waterTemperature)) trip("Ошибка датчика воды");
      else if (waterTemperature >= config.overheatTemperature) trip("Перегрев воды");
      else if (!relays[2]) trip("Пропущен этап запуска реле 3");
      else if (brightness < config.lightThreshold) trip("Недостаточная яркость");
      else phase = RUNNING;
    } else {
      setRelay(0, elapsed < 15000);
      setRelay(1, elapsed >= 17000);
      setRelay(2, elapsed >= 19000);
    }
  }
  if (phase == IDLE) {
    if (relays[4] && isfinite(waterTemperature) &&
        waterTemperature < config.pumpTemperature && elapsed >= config.pumpMinimumSeconds * 1000)
      setRelay(4, false);
    if (relays[5] && isfinite(airTemperature) && airTemperature < config.fanOffTemperature)
      setRelay(5, false);
  }
  digitalWrite(RED_PIN, !alarmMessage.isEmpty() || !isfinite(waterTemperature));
  digitalWrite(GREEN_PIN, phase == RUNNING);
  digitalWrite(YELLOW_PIN, phase == STARTING);
}
String numberOrNull(float value) { return isfinite(value) ? String(value, 1) : String("null"); }
void sendStatus() {
  const uint32_t elapsed = millis() - pumpSince;
  const char *label = "Остановлено";
  uint32_t next = 0;
  if (phase == RUNNING) label = "Работа";
  else if (phase != IDLE) {
    if (elapsed < 15000) { label = "Реле 1"; next = 15000; }
    else if (elapsed < 17000) { label = "Пауза"; next = 17000; }
    else if (elapsed < 19000) { label = "Реле 2"; next = 19000; }
    else { label = "Проверка света"; next = 21000; }
  }
  String json = "{\"manual\":" + String(manualMode ? "true" : "false");
  json += ",\"phase\":\"" + String(label) + "\",\"alarm\":\"" + alarmMessage + "\"";
  json += ",\"air\":" + numberOrNull(airTemperature) + ",\"water\":" + numberOrNull(waterTemperature);
  float remaining = next > elapsed ? (next - elapsed) / 1000.0f : 0;
  json += ",\"remaining\":" + String(remaining, 1);
  json += ",\"cooling\":" + String(phase == IDLE && (relays[4] || relays[5]) ? "true" : "false");
  json += ",\"light\":" + String(brightness, 1) + ",\"relays\":[";
  for (unsigned i = 0; i < 6; ++i) { if (i) json += ','; json += relays[i] ? "true" : "false"; }
  json += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}
struct Field { const char *name; float Settings::*member; float minimum; float maximum; };
const Field fields[] = {
  {"light", &Settings::lightThreshold, 0, 100},
  {"pump", &Settings::pumpTemperature, -20, 120}, {"invert", &Settings::lightInverted, 0, 1},
  {"airOffset", &Settings::airOffset, -20, 20}, {"waterOffset", &Settings::waterOffset, -20, 20},
  {"pumpMin", &Settings::pumpMinimumSeconds, 0, 3600}, {"fanOff", &Settings::fanOffTemperature, -20, 120},
  {"overheat", &Settings::overheatTemperature, 1, 125}
};
bool validSettings(const Settings &candidate) {
  if (candidate.version != 2) return false;
  for (const auto &field : fields) {
    float value = candidate.*(field.member);
    if (!isfinite(value) || value < field.minimum || value > field.maximum) return false;
  }
  return candidate.lightInverted == 0 || candidate.lightInverted == 1;
}
void sendSettings() {
  String json = "{";
  for (const auto &field : fields) {
    if (json.length() > 1) json += ',';
    json += "\"" + String(field.name) + "\":" + String(config.*(field.member), 2);
  }
  server.send(200, "application/json", json + "}");
}
void saveSettings() {
  for (bool on : relays) if (on) { server.send(409, "text/plain; charset=utf-8", "Сначала дождитесь выключения всех реле"); return; }
  Settings next = config;
  for (const auto &field : fields) {
    if (!server.hasArg(field.name)) { server.send(400, "text/plain", "Missing field"); return; }
    String input = server.arg(field.name);
    char *end;
    float value = strtof(input.c_str(), &end);
    if (end == input.c_str() || *end != '\0') { server.send(400, "text/plain", "Invalid number"); return; }
    next.*(field.member) = value;
  }
  if (!validSettings(next)) { server.send(400, "text/plain", "Out of range"); return; }
  if (preferences.putBytes("settings", &next, sizeof(next)) != sizeof(next)) {
    server.send(500, "text/plain", "Save failed"); return;
  }
  config = next;
  airTemperature = rawAirTemperature + config.airOffset;
  waterTemperature = rawWaterTemperature + config.waterOffset;
  sampleSensors();
  server.send(200, "text/plain; charset=utf-8", "Настройки сохранены");
}
void command() {
  String action = server.arg("action");
  if (action == "r4") setRelay(3, !relays[3]); // Второй вентилятор, ручное управление в обоих режимах.
  else if (action == "stop") stopSequence();
  else if (action == "start") {
    if (!startSequence()) { server.send(409, "text/plain; charset=utf-8", "Нужен Auto, исправный датчик воды и сброшенный Alarm. Повторный запуск во время работы запрещён."); return; }
  } else if (action == "reset") {
    sampleSensors();
    if (isfinite(waterTemperature) && waterTemperature >= config.overheatTemperature) {
      server.send(409, "text/plain; charset=utf-8", "Температура воды выше порога перегрева"); return;
    }
    if (phase != IDLE || !isfinite(waterTemperature)) { server.send(409, "text/plain; charset=utf-8", "Остановите цикл и проверьте датчик воды"); return; }
    alarmMessage = "";
  } else if (action == "r1" || action == "r2") {
    if (!buttons[3].stable || !manualMode || !alarmMessage.isEmpty()) { server.send(409, "text/plain; charset=utf-8", "Нужен Manual без Alarm и отпущенная кнопка Стоп"); return; }
    unsigned index = action == "r1" ? 0 : 1;
    setRelay(index, !relays[index]);
  } else { server.send(400, "text/plain", "Unknown command"); return; }
  server.send(200, "text/plain", "OK");
}
void setup() {
  Serial.begin(115200);
  for (unsigned i = 0; i < 6; ++i) {
    digitalWrite(RELAY_PINS[i], RELAY_ACTIVE_LOW ? HIGH : LOW);
    pinMode(RELAY_PINS[i], OUTPUT);
    setRelay(i, false);
  }
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  for (auto &button : buttons) button.begin();
  modeSwitch.begin();
  manualMode = !modeSwitch.stable;
  preferences.begin("relay-control", false);
  Settings stored;
  size_t storedSize = preferences.getBytesLength("settings");
  // Version 1 has the same prefix and no offset/minimum-time fields.
  if (storedSize == sizeof(stored) || storedSize == offsetof(Settings, overheatTemperature) || storedSize == offsetof(Settings, fanOffTemperature) || storedSize == offsetof(Settings, airOffset)) {
    size_t bytesRead = preferences.getBytes("settings", &stored, storedSize);
    if (storedSize == offsetof(Settings, airOffset) && stored.version == 1) stored.version = 2;
    if (bytesRead == storedSize && validSettings(stored)) config = stored;
  }
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  airSensor.begin();
  waterSensor.begin();
  airSensor.setWaitForConversion(false);
  waterSensor.setWaitForConversion(false);
  airSensor.requestTemperatures();
  waterSensor.requestTemperatures();
  conversionSince = millis();
  sampleSensors();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_NAME, AP_PASSWORD);
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", WEB_PAGE); });
  server.on("/api/status", HTTP_GET, sendStatus);
  server.on("/api/settings", HTTP_GET, sendSettings);
  server.on("/api/settings", HTTP_POST, saveSettings);
  server.on("/api/command", HTTP_POST, command);
  server.begin();
  Serial.println(WiFi.softAPIP());
}
void loop() {
  uint32_t now = millis();
  modeSwitch.update(now);
  bool nextMode = !modeSwitch.stable;
  if (nextMode != manualMode) { stopSequence(); manualMode = nextMode; }
  bool pressed[4];
  for (unsigned i = 0; i < 4; ++i) pressed[i] = buttons[i].update(now);
  if (!buttons[3].stable) stopSequence();
  else {
    for (unsigned i = 0; i < 2; ++i)
      if (pressed[i] && manualMode && alarmMessage.isEmpty()) setRelay(i, !relays[i]);
    if (pressed[2]) startSequence();
  }
  if (now - sensorSince >= 200) { sensorSince = now; sampleSensors(); }
  runControl(millis());
  server.handleClient();
  // A held physical Stop overrides web commands in the same iteration.
  if (!buttons[3].stable) stopSequence();
}
