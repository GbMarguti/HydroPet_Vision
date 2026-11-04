/*
  HydraPet Vision v2.1 — perfil de simulação Wokwi / ESP32-DevKitC V4.
  Derivado de HydraPet_Vision_v2_1s.ino. Para a AI-Thinker física use o original.
  TFT no SPI padrão: SCK=18, MOSI=23, D/C=25, CS=33. RTC e PCF em SDA=26, SCL=27.
  HC-SR04 ECHO=34. SD em VSPI SCK=18, MISO=19, MOSI=23, CS=5.
  As amostras DIA e IR da Figura 41 substituem as capturas da OV2640.
*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <RTClib.h>
#include <TJpg_Decoder.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include "quadro_teste.h"
#include "FS.h"
#include <SD.h>

// ============================================================================
//  OPÇÕES DE COMPILAÇÃO
// ============================================================================
#define FW_VERSION        "2.1.0"
#define USE_DS3231        0      // 0 = DS1307 (padrão do projeto) | 1 = DS3231
#define TJPG_SWAP_BYTES   false   // writePixels() recebe RGB565 na ordem nativa do ESP32.

// ============================================================================
//  PINOS
// ============================================================================
#define TFT_CS            33
#define TFT_DC            25
#define ECHO_PIN          34
#define I2C_SDA           26
#define I2C_SCL           27
#define SD_SCK            18
#define SD_MISO           19
#define SD_MOSI           23
#define SD_CS             5

// Expansor PCF8574
#define PCF_ADDR          0x20
#define PCF_BTN_NEXT      0      // Botão 1 - Avança
#define PCF_BTN_OK        1      // Botão 2 - Confirma
#define PCF_BTN_BACK      2      // Botão 3 - Retrocede
#define PCF_TRIG          3
#define PCF_IR            4
#define PCF_LDR           5

// ============================================================================
//  PARÂMETROS GERAIS
// ============================================================================
#define LOG_PATH          "/registros.csv"
#define VISITS_PATH       "/visitas.csv"
#define PHOTO_DIR         "/fotos"
#define MAX_LOG           150            // registros mais recentes mantidos na RAM
#define LOG_TAIL_BYTES    20000UL        // trecho final do CSV lido na partida
#define DEBOUNCE_MS       30
#define LONG_PRESS_MS     1200
#define SENSOR_PERIOD_MS  60
#define SENSOR_FAULT_N    50             // ciclos seguidos sem borda no ECHO => falha do sensor
#define MIN_VALID_CM      2.0f
#define LDR_STABLE_MS     3000           // tempo para confirmar mudança claro/escuro
#define IR_PREHEAT_MS     300            // IR ligado antes do disparo (exposição automática)
#define MENU_TIMEOUT_MS   60000
#define VISITA_GAP_MS     8000           // sem presença por este tempo => visita encerrada
#define MAX_FOTOS_VISITA  6              // limite de fotos por visita
#define SD_RETRY_MS       15000

enum IrMode : uint8_t { IR_AUTO = 0, IR_ON = 1, IR_OFF = 2 };
enum BtnEvent : uint8_t { EV_NONE, EV_NEXT, EV_OK, EV_BACK, EV_OK_LONG };
enum Screen : uint8_t { SCR_HOME, SCR_MENU, SCR_LOG, SCR_VIEW, SCR_DATETIME, SCR_SETTINGS, SCR_INFO };

// ============================================================================
//  OBJETOS GLOBAIS
// ============================================================================
Adafruit_ILI9341 tft(TFT_CS, TFT_DC);
Preferences prefs;

#if USE_DS3231
RTC_DS3231 rtc;
#else
RTC_DS1307 rtc;
#endif

struct Config {
  uint8_t thresholdCm;    // distância de disparo
  uint8_t cooldownS;      // intervalo mínimo entre fotos
  uint8_t irMode;         // IrMode
  uint8_t ldrInvert;      // 0 = DO alto => escuro | 1 = invertido
  uint8_t confirmReads;   // leituras consecutivas para validar presença
} cfg;

struct LogEntry { char date[11]; char time[9]; char file[48]; uint32_t visit; };
LogEntry logs[MAX_LOG];
int  logCount    = 0;
int  totalEvents = 0;    // total de fotos registradas
int  todayCount  = 0;    // fotos de hoje
int  todayVisits = 0;    // visitas de hoje
uint32_t visitId = 0;    // contador persistente de visitas (NVS)

// Estados de hardware
bool camOk = false, sdOk = false, rtcOk = false, pcfOk = false;
bool rtcTimeSuspect = false;
bool sensorFault = false;

// Cores
uint16_t C_BG, C_PANEL, C_ACCENT, C_TEXT, C_MUTED, C_OK, C_WARN, C_ERR, C_SEL;

// ============================================================================
//  PROTÓTIPOS
// ============================================================================
LogEntry& newestLog(int i);
BtnEvent buttonsUpdate(uint8_t in);
void bootLine(const char* label, bool ok, const char* extra = "");
void handleEvent(BtnEvent ev);
void gotoScreen(Screen s);
void toast(const char* msg, uint16_t col);
DateTime getNow(bool fresh = false);

// ============================================================================
//  CONFIGURAÇÕES (NVS)
// ============================================================================
void cfgDefaults() { cfg = {15, 10, IR_AUTO, 0, 4}; }

void cfgLoad() {
  cfgDefaults();
  prefs.begin("hydrapet", true);
  cfg.thresholdCm  = prefs.getUChar("thr", cfg.thresholdCm);
  cfg.cooldownS    = prefs.getUChar("cool", cfg.cooldownS);
  cfg.irMode       = prefs.getUChar("ir", cfg.irMode);
  cfg.ldrInvert    = prefs.getUChar("ldr", cfg.ldrInvert);
  cfg.confirmReads = prefs.getUChar("conf", cfg.confirmReads);
  visitId          = prefs.getUInt("vis", 0);
  prefs.end();
}

void cfgSave() {
  prefs.begin("hydrapet", false);
  prefs.putUChar("thr", cfg.thresholdCm);
  prefs.putUChar("cool", cfg.cooldownS);
  prefs.putUChar("ir", cfg.irMode);
  prefs.putUChar("ldr", cfg.ldrInvert);
  prefs.putUChar("conf", cfg.confirmReads);
  prefs.end();
}

void visitIdSave() {
  prefs.begin("hydrapet", false);
  prefs.putUInt("vis", visitId);
  prefs.end();
}

// ============================================================================
//  EXPANSOR PCF8574
// ============================================================================
uint8_t pcfShadow = 0xE7;   // botões/LDR = 1 (entrada), TRIG = 0, IR = 0
uint8_t pcfIn     = 0xFF;

bool pcfWrite(uint8_t v) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

uint8_t pcfRead(bool &ok) {
  ok = (Wire.requestFrom((uint8_t)PCF_ADDR, (uint8_t)1) == 1);
  return ok ? Wire.read() : 0xFF;
}

void pcfSetBit(uint8_t bit, bool v) {
  if (v) pcfShadow |= (1 << bit); else pcfShadow &= ~(1 << bit);
  pcfWrite(pcfShadow);
}

// ============================================================================
//  RTC
// ============================================================================
DateTime getNow(bool fresh) {
  static DateTime cache;
  static uint32_t t = 0;
  if (fresh || t == 0 || millis() - t >= 250) {
    if (rtcOk) cache = rtc.now();
    else       cache = DateTime(2026, 1, 1, 0, 0, 0) + TimeSpan(millis() / 1000);
    t = millis() | 1;
  }
  return cache;
}

uint8_t daysInMonth(int y, int m) {
  static const uint8_t d[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) return 29;
  return d[m - 1];
}

// ============================================================================
//  CARTÃO SD E BANCO DE REGISTROS
// ============================================================================
uint64_t sdTotalMB = 0;
uint64_t sdUsedB   = 0;      // ocupação: lida na montagem e estimada a cada foto

void sdReadCapacity() {
  sdTotalMB = SD.totalBytes() / (1024ULL * 1024ULL);
  sdUsedB   = SD.usedBytes();          // pode levar segundos: só na montagem
}

bool initSD() {
  if (!SD.begin(SD_CS, SPI, 4000000)) return false;
  if (SD.cardType() == CARD_NONE) return false;
  if (!SD.exists(PHOTO_DIR)) SD.mkdir(PHOTO_DIR);
  return true;
}

void addLogEntry(const char* d, const char* t, const char* f, uint32_t v) {
  if (logCount == MAX_LOG) {
    memmove(&logs[0], &logs[1], sizeof(LogEntry) * (MAX_LOG - 1));
    logCount--;
  }
  strlcpy(logs[logCount].date, d, sizeof(logs[0].date));
  strlcpy(logs[logCount].time, t, sizeof(logs[0].time));
  strlcpy(logs[logCount].file, f, sizeof(logs[0].file));
  logs[logCount].visit = v;
  logCount++;
}

static void todayString(char* out, size_t n) {
  DateTime d = getNow();
  snprintf(out, n, "%04d-%02d-%02d", d.year(), d.month(), d.day());
}

void recountToday() {                      // fotos de hoje (entre as mantidas na RAM)
  char today[11];
  todayString(today, sizeof(today));
  int c = 0;
  for (int i = 0; i < logCount; i++) if (strcmp(logs[i].date, today) == 0) c++;
  todayCount = c;
}

void recountVisitsToday() {                // visitas distintas de hoje com foto registrada
  char today[11];
  todayString(today, sizeof(today));
  int c = 0;
  uint32_t prev = 0xFFFFFFFFUL;
  for (int i = 0; i < logCount; i++) {
    if (strcmp(logs[i].date, today) != 0) continue;
    if (logs[i].visit == 0 || logs[i].visit != prev) c++;
    prev = logs[i].visit;
  }
  todayVisits = c;
}

// Linha: data;hora;arquivo;distancia;ir;visita   (as linhas da v2.0 têm só 5 campos)
static void parseCsvLine(char* line) {
  if (!isdigit((unsigned char)line[0])) return;            // ignora cabeçalho
  char* fld[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
  int n = 0;
  fld[n++] = line;
  for (char* p = line; *p && n < 6; p++) {
    if (*p == ';') { *p = 0; fld[n++] = p + 1; }
  }
  if (n < 3) return;
  uint32_t v = (n >= 6 && fld[5]) ? (uint32_t)strtoul(fld[5], nullptr, 10) : 0;
  addLogEntry(fld[0], fld[1], fld[2], v);
}

// Conta as linhas do CSV com buffer fixo (sem String); só é chamada na montagem
static uint32_t countCsvLines(File &f, bool &hasHeader) {
  uint8_t buf[512];
  uint32_t nl = 0;
  bool first = true;
  hasHeader = false;
  f.seek(0);
  for (;;) {
    int n = (int)f.read(buf, sizeof(buf));
    if (n <= 0) break;
    if (first) { first = false; hasHeader = !isdigit(buf[0]); }
    for (int i = 0; i < n; i++) if (buf[i] == '\n') nl++;
  }
  return nl;
}

void loadLog() {
  logCount = 0; totalEvents = 0;
  File f = SD.open(LOG_PATH);
  if (f) {
    bool hdr = false;
    uint32_t nl = countCsvLines(f, hdr);
    uint32_t sub = hdr ? 1 : 0;
    totalEvents = (nl > sub) ? (int)(nl - sub) : 0;

    uint32_t sz = f.size();
    char line[160];
    f.seek(0);
    if (sz > LOG_TAIL_BYTES) {                              // lê só o final do arquivo
      f.seek(sz - LOG_TAIL_BYTES);
      f.readBytesUntil('\n', line, sizeof(line) - 1);       // descarta linha parcial
    }
    while (f.available()) {
      size_t n = f.readBytesUntil('\n', line, sizeof(line) - 1);
      line[n] = 0;
      if (n && line[n - 1] == '\r') line[--n] = 0;
      if (n < 12) continue;
      parseCsvLine(line);
    }
    f.close();
  }
  recountToday();
  recountVisitsToday();
}

// distância com vírgula decimal (Excel pt-BR); vazia quando inválida
void appendCsv(const char* d, const char* t, const char* path, float dist, bool ir, uint32_t visit) {
  bool isNew = !SD.exists(LOG_PATH);
  File f = SD.open(LOG_PATH, FILE_APPEND);
  if (!f) return;
  if (isNew) f.println("data;hora;arquivo;distancia_cm;ir;visita");
  char ds[12] = "";
  if (dist >= 0) {
    snprintf(ds, sizeof(ds), "%.1f", dist);
    for (char* p = ds; *p; p++) if (*p == '.') *p = ',';
  }
  f.printf("%s;%s;%s;%s;%d;%lu\n", d, t, path, ds, ir ? 1 : 0, (unsigned long)visit);
  f.close();
}

// ============================================================================
//  DISPLAY: HELPERS
// ============================================================================
bool tftOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (y >= tft.height() || x >= tft.width()) return false;
  tft.startWrite();
  tft.setAddrWindow(x, y, w, h);
  tft.writePixels(bitmap, (uint32_t)w * h);
  tft.endWrite();
  return true;
}

// Desenha um JPEG ajustando a escala (1, 1/2, 1/4 ou 1/8) para caber em 320x240
void drawJpgFit(const uint8_t* buf, uint32_t len) {
  uint16_t w = 0, h = 0;
  uint8_t sc = 1;
  if (TJpgDec.getJpgSize(&w, &h, buf, len) == JDR_OK) {
    while ((w / sc > 320 || h / sc > 240) && sc < 8) sc *= 2;
  }
  TJpgDec.setJpgScale(sc);
  TJpgDec.drawJpg(0, 0, buf, len);
}

void initColors() {
  C_BG     = tft.color565(10, 14, 26);
  C_PANEL  = tft.color565(24, 32, 54);
  C_ACCENT = tft.color565(0, 168, 232);
  C_TEXT   = 0xFFFF;
  C_MUTED  = tft.color565(140, 150, 175);
  C_OK     = tft.color565(46, 204, 113);
  C_WARN   = tft.color565(255, 193, 7);
  C_ERR    = tft.color565(231, 76, 60);
  C_SEL    = tft.color565(0, 96, 150);
}

void txt(int16_t x, int16_t y, const char* s, uint8_t sz, uint16_t fg, uint16_t bg) {
  tft.setTextSize(sz);
  tft.setTextColor(fg, bg);
  tft.setCursor(x, y);
  tft.print(s);
}

void txtC(int16_t cx, int16_t y, const char* s, uint8_t sz, uint16_t fg, uint16_t bg) {
  txt(cx - (int)strlen(s) * 6 * sz / 2, y, s, sz, fg, bg);
}

void txtR(int16_t rx, int16_t y, const char* s, uint8_t sz, uint16_t fg, uint16_t bg) {
  txt(rx - (int)strlen(s) * 6 * sz, y, s, sz, fg, bg);
}

void pill(int x, int y, int w, const char* s, uint16_t fill, uint16_t fg) {
  tft.fillRoundRect(x, y, w, 18, 5, fill);
  txtC(x + w / 2, y + 5, s, 1, fg, fill);
}

int lastHeaderMin = -1;
void drawHeaderClock() {
  DateTime n = getNow();
  char b[8];
  snprintf(b, sizeof(b), "%02d:%02d", n.hour(), n.minute());
  txt(252, 7, b, 2, C_TEXT, C_PANEL);
  lastHeaderMin = n.minute();
}

void drawHeader(const char* title) {
  tft.fillRect(0, 0, 320, 28, C_PANEL);
  tft.fillRect(0, 28, 320, 2, C_ACCENT);
  txt(10, 7, title, 2, C_TEXT, C_PANEL);
  drawHeaderClock();
}

void drawFooter(const char* a, const char* b, const char* c) {   // esquerda = Btn3 | meio = Btn2 | direita = Btn1
  tft.fillRect(0, 212, 320, 28, C_BG);
  const char* L[3] = {a, b, c};
  for (int i = 0; i < 3; i++) {
    if (!L[i] || !L[i][0]) continue;
    int x = 6 + i * 106;
    tft.fillRoundRect(x, 216, 102, 20, 6, C_PANEL);
    tft.drawRoundRect(x, 216, 102, 20, 6, C_MUTED);
    txtC(x + 51, 222, L[i], 1, C_TEXT, C_PANEL);
  }
}

Screen screen = SCR_HOME;
bool needRedraw = true;
bool fullClear  = true;
uint32_t lastActivity = 0;

uint32_t toastUntil = 0;
void toast(const char* m, uint16_t col) {
  toastUntil = millis() + 2500;
  if (screen == SCR_VIEW) return;
  tft.fillRoundRect(10, 184, 300, 26, 8, col);
  txtC(160, 191, m, 2, C_BG, col);
}

// ============================================================================
//  ESTADO: SENSOR, LUZ, IR, VISITA, CAPTURA
// ============================================================================
// --- Ultrassom por interrupção ---
volatile uint32_t echoRise = 0, echoWidth = 0, echoEdges = 0;
volatile bool echoDone = false;
void IRAM_ATTR echoISR() {
  if (gpio_get_level((gpio_num_t)ECHO_PIN)) {
    echoRise = micros();
    echoEdges = echoEdges + 1;            // contagem de bordas de subida (diagnóstico do sensor)
  } else if (echoRise) {
    echoWidth = micros() - echoRise;
    echoDone = true;
    echoRise = 0;
  }
}

bool     waitingEcho   = false;
uint32_t lastTrig      = 0;
uint32_t edgesAtTrig   = 0;
uint8_t  noEdgeCount   = 0;
float    lastDistance  = -1;
uint8_t  hits          = 0;
bool     captureRequested = false;
uint32_t lastPhoto     = 0;

// --- Visita ---
bool     inVisit = false;
uint32_t visitStartMs = 0, visitLastSeen = 0;
uint32_t curVisitId = 0;
uint8_t  photosThisVisit = 0;
char     visitDate[11] = "", visitTime[9] = "";

// --- LDR / IR ---
bool     isDark = false, ldrPending = false, ldrFirst = true;
uint32_t ldrSince = 0;
bool     irState = false;
uint32_t irHoldUntil = 0;

// --- Pré-visualização após captura ---
bool     previewActive = false;
uint32_t previewUntil  = 0;

void setIR(bool on) {
  if (on == irState) return;
  irState = on;
  pcfSetBit(PCF_IR, on);
}

void irTask() {
  bool want = (cfg.irMode == IR_ON) ||
              (cfg.irMode == IR_AUTO && isDark && (int32_t)(irHoldUntil - millis()) > 0);
  setIR(want);
}

void lightUpdate(uint8_t in) {
  bool pinHigh = (in >> PCF_LDR) & 1;
  bool dark = cfg.ldrInvert ? !pinHigh : pinHigh;
  uint32_t now = millis();
  if (ldrFirst) { ldrFirst = false; isDark = ldrPending = dark; ldrSince = now; return; }
  if (dark != ldrPending) { ldrPending = dark; ldrSince = now; }
  if (ldrPending != isDark && now - ldrSince >= LDR_STABLE_MS) isDark = ldrPending;
}

void startVisit() {
  DateTime dt = getNow(true);
  inVisit = true;
  visitStartMs = visitLastSeen = millis();
  photosThisVisit = 0;
  visitId++;
  curVisitId = visitId;
  todayVisits++;
  snprintf(visitDate, sizeof(visitDate), "%04d-%02d-%02d", dt.year(), dt.month(), dt.day());
  snprintf(visitTime, sizeof(visitTime), "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
  visitIdSave();
}

void endVisit() {
  inVisit = false;
  if (!sdOk) return;
  uint32_t dur = (visitLastSeen - visitStartMs) / 1000UL;
  bool isNew = !SD.exists(VISITS_PATH);
  File f = SD.open(VISITS_PATH, FILE_APPEND);
  if (!f) return;
  if (isNew) f.println("visita;data;hora_inicio;duracao_s;fotos");
  f.printf("%lu;%s;%s;%lu;%u\n", (unsigned long)curVisitId, visitDate, visitTime,
           (unsigned long)dur, (unsigned)photosThisVisit);
  f.close();
}

void visitTask() {
  if (inVisit && millis() - visitLastSeen > VISITA_GAP_MS) endVisit();
}

void processDistance(float d) {
  lastDistance = d;
  bool inRange = (d >= MIN_VALID_CM && d <= cfg.thresholdCm);
  bool near2x  = (d >= MIN_VALID_CM && d <= cfg.thresholdCm * 2.0f);
  if (near2x && isDark) irHoldUntil = millis() + 2500;      // aquece o IR antes do disparo
  if (inRange) { if (hits < 255) hits++; visitLastSeen = millis(); } else hits = 0;
  if (hits >= cfg.confirmReads) {
    if (!inVisit) startVisit();
    if (photosThisVisit < MAX_FOTOS_VISITA &&
        millis() - lastPhoto > (uint32_t)cfg.cooldownS * 1000UL) {
      captureRequested = true;
    }
  }
}

void sensorTask() {
  if (!pcfOk) return;
  uint32_t now = millis();
  if (!waitingEcho) {
    if (now - lastTrig >= SENSOR_PERIOD_MS) {
      echoDone = false; echoRise = 0;
      edgesAtTrig = echoEdges;
      pcfSetBit(PCF_TRIG, true);
      delayMicroseconds(12);
      pcfSetBit(PCF_TRIG, false);
      lastTrig = now;
      waitingEcho = true;
    }
  } else {
    if (echoDone) {
      waitingEcho = false;
      noEdgeCount = 0; sensorFault = false;
      float d = echoWidth * 0.0343f / 2.0f;
      processDistance(d > 400 ? -1 : d);
    } else if (now - lastTrig > 35) {          // timeout (~6 m)
      waitingEcho = false;
      if (echoEdges == edgesAtTrig) {          // o ECHO nem subiu: sensor ausente ou travado
        if (noEdgeCount < 255) noEdgeCount++;
        if (noEdgeCount >= SENSOR_FAULT_N) sensorFault = true;
      } else {                                 // ECHO subiu, mas sem objeto no alcance
        noEdgeCount = 0; sensorFault = false;
      }
      processDistance(-1);
    }
  }
}

// Autoteste na partida: o HC-SR04 sempre eleva o ECHO após o TRIG (mesmo sem objeto)
bool sensorSelfTest() {
  if (!pcfOk) return false;
  for (int i = 0; i < 3; i++) {
    uint32_t e0 = echoEdges;
    echoDone = false; echoRise = 0;
    pcfSetBit(PCF_TRIG, true);
    delayMicroseconds(12);
    pcfSetBit(PCF_TRIG, false);
    uint32_t t0 = millis();
    while (millis() - t0 < 45 && echoEdges == e0) delay(1);
    if (echoEdges != e0) {
      delay(45);                               // deixa o pulso terminar
      echoDone = false; echoRise = 0;
      return true;
    }
    delay(60);
  }
  return false;
}

// ============================================================================
//  BOTÕES (via PCF8574) - debounce + repetição + pressão longa
// ============================================================================
struct BtnState {
  bool stable = false, lastRaw = false, longFired = false;
  uint32_t rawSince = 0, pressedAt = 0, lastRepeat = 0;
} btn[3];
const uint8_t btnBit[3] = {PCF_BTN_NEXT, PCF_BTN_OK, PCF_BTN_BACK};

BtnEvent buttonsUpdate(uint8_t in) {
  uint32_t now = millis();
  BtnEvent ev = EV_NONE;
  for (int i = 0; i < 3; i++) {
    BtnState &b = btn[i];
    bool raw = !((in >> btnBit[i]) & 1);          // ativo em nível baixo
    if (raw != b.lastRaw) { b.lastRaw = raw; b.rawSince = now; }
    if (now - b.rawSince >= DEBOUNCE_MS && raw != b.stable) {
      b.stable = raw;
      if (raw) {
        b.pressedAt = now; b.lastRepeat = now; b.longFired = false;
        if (i == 0) ev = EV_NEXT; else if (i == 2) ev = EV_BACK;
      } else if (i == 1 && !b.longFired) {
        ev = EV_OK;
      }
    } else if (b.stable && raw) {
      if (i != 1 && now - b.pressedAt > 500 && now - b.lastRepeat > 120) {
        b.lastRepeat = now;
        ev = (i == 0) ? EV_NEXT : EV_BACK;        // auto-repetição ao segurar
      }
      if (i == 1 && !b.longFired && now - b.pressedAt > LONG_PRESS_MS) {
        b.longFired = true;
        ev = EV_OK_LONG;
      }
    }
  }
  return ev;
}

uint32_t lastPoll = 0, lastPcfRetry = 0;
void pcfTask() {
  uint32_t now = millis();
  if (!pcfOk && now - lastPcfRetry < 2000) return;
  if (pcfOk && now - lastPoll < 10) return;
  lastPoll = now;
  bool ok;
  uint8_t in = pcfRead(ok);
  if (!ok) { pcfOk = false; lastPcfRetry = now; return; }
  if (!pcfOk) { pcfOk = true; pcfWrite(pcfShadow); }
  pcfIn = in;
  lightUpdate(in);
  handleEvent(buttonsUpdate(in));
}

// ============================================================================
//  CAPTURA E REGISTRO
// ============================================================================
void showPreview(const char* t, const uint8_t* jpeg, size_t bytes) {
  drawJpgFit(jpeg, bytes);
  tft.fillRect(0, 206, 320, 34, C_OK);
  txt(8, 210, "FOTO SALVA", 2, C_BG, C_OK);
  txtR(312, 210, t, 2, C_BG, C_OK);
  if (irState) txt(8, 228, "modo infravermelho", 1, C_BG, C_OK);
  previewActive = true;
  previewUntil  = millis() + 3000;
}

void doCapture() {
  captureRequested = false;
  lastPhoto = millis();
  if (!camOk) { toast("Camera com erro", C_ERR); return; }
  if (!sdOk)  { toast("SD ausente/erro", C_ERR); return; }

  bool useIR = (cfg.irMode == IR_ON) || (cfg.irMode == IR_AUTO && isDark);
  if (useIR && !irState) { setIR(true); delay(IR_PREHEAT_MS); }
  irHoldUntil = millis() + 1500;

  DateTime dt = getNow(true);
  char d[11], t[9], dir[32], path[64];
  snprintf(d, sizeof(d), "%04d-%02d-%02d", dt.year(), dt.month(), dt.day());
  snprintf(t, sizeof(t), "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
  snprintf(dir, sizeof(dir), PHOTO_DIR "/%s", d);
  if (!SD.exists(dir)) SD.mkdir(dir);
  snprintf(path, sizeof(path), "%s/gato_%04d%02d%02d_%02d%02d%02d.jpg", dir,
           dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
  if (SD.exists(path)) {
    snprintf(path, sizeof(path), "%s/gato_%04d%02d%02d_%02d%02d%02d_%lu.jpg", dir,
             dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second(), (unsigned long)(millis() % 1000));
  }

  const uint8_t* jpeg = useIR ? quadroIR : quadroDia;
  size_t bytes = useIR ? sizeof(quadroIR) : sizeof(quadroDia);
  bool ok = false;
  File f = SD.open(path, FILE_WRITE);
  if (f) {
    ok = (f.write(jpeg, bytes) == bytes);
    f.close();
  }

  if (ok) {
    uint32_t vid = inVisit ? curVisitId : 0;
    appendCsv(d, t, path, lastDistance, useIR, vid);
    addLogEntry(d, t, path, vid);
    totalEvents++;
    if (photosThisVisit < 255) photosThisVisit++;
    sdUsedB += ((uint64_t)bytes + 16383ULL) / 16384ULL * 16384ULL;   // estimativa (cluster de 16 KB)
    recountToday();
    if (screen == SCR_HOME) showPreview(t, jpeg, bytes);
    else if (screen != SCR_VIEW) needRedraw = true;
  } else {
    sdOk = false;                                  // força nova tentativa de montagem
    toast("Erro ao gravar no SD", C_ERR);
  }
}

// ============================================================================
//  TELAS
// ============================================================================
// ---------- HOME ----------
int hSec = -1, hDist = -9999, hToday = -1, hTotal = -1, hStatus = -1;
int hSd = -1, hIr = -1, hDark = -1;
uint32_t hDistT = 0;

void drawCard(int x, const char* label, const char* val, uint16_t col) {
  tft.fillRoundRect(x, 112, 96, 60, 8, C_PANEL);
  txt(x + 8, 118, label, 1, C_MUTED, C_PANEL);
  txtC(x + 48, 140, val, 3, col, C_PANEL);
}

void drawHomeStatic() {
  drawHeader("HydraPet Vision");
  drawFooter("", "2  MENU", "");
  hSec = hDist = hToday = hTotal = hStatus = hSd = hIr = hDark = -1;
}

void updateHome() {
  DateTime n = getNow();
  char b[24];
  if (n.second() != hSec) {
    hSec = n.second();
    snprintf(b, sizeof(b), "%02d:%02d:%02d", n.hour(), n.minute(), n.second());
    txt(14, 42, b, 4, C_TEXT, C_BG);
    if (rtcOk) {
      snprintf(b, sizeof(b), "%02d/%02d/%04d", n.day(), n.month(), n.year());
      txt(14, 82, b, 2, C_MUTED, C_BG);
    } else {
      txt(14, 82, "RTC NAO ENCONTRADO", 1, C_ERR, C_BG);
    }
    if (n.minute() != lastHeaderMin) drawHeaderClock();
  }
  int sd = sdOk ? 1 : 0;
  if (sd != hSd) { hSd = sd; pill(226, 42, 84, sdOk ? "SD OK" : "SD ERRO", sdOk ? C_OK : C_ERR, C_BG); }
  int irs = irState ? 2 : (cfg.irMode == IR_OFF ? 0 : 1);
  if (irs != hIr) {
    hIr = irs;
    const char* s = irs == 2 ? "IR LIGADO" : (cfg.irMode == IR_OFF ? "IR OFF" : (cfg.irMode == IR_ON ? "IR FIXO" : "IR AUTO"));
    pill(226, 64, 84, s, irs == 2 ? C_WARN : C_PANEL, irs == 2 ? C_BG : C_TEXT);
  }
  int dk = isDark ? 1 : 0;
  if (dk != hDark) { hDark = dk; pill(226, 86, 84, isDark ? "ESCURO" : "CLARO", isDark ? C_SEL : C_PANEL, C_TEXT); }

  if (todayVisits != hToday) { hToday = todayVisits; snprintf(b, sizeof(b), "%d", todayVisits); drawCard(10, "VISITAS HOJE", b, C_ACCENT); }
  if (totalEvents != hTotal) { hTotal = totalEvents; snprintf(b, sizeof(b), "%d", totalEvents); drawCard(112, "FOTOS", b, C_TEXT); }
  int dshow = (lastDistance >= MIN_VALID_CM) ? (int)(lastDistance + 0.5f) : -1;
  if (dshow != hDist && millis() - hDistT > 250) {
    hDist = dshow; hDistT = millis();
    if (dshow < 0) snprintf(b, sizeof(b), "--"); else snprintf(b, sizeof(b), "%d", dshow);
    drawCard(214, "DIST. (cm)", b, (dshow >= 0 && dshow <= cfg.thresholdCm) ? C_WARN : C_OK);
  }

  int st = 0;
  if (!camOk) st = 3; else if (!sdOk) st = 2; else if (!pcfOk) st = 4; else if (sensorFault) st = 5;
  else if (hits >= cfg.confirmReads) st = 1;
  if (st != hStatus) {
    hStatus = st;
    uint16_t col = (st == 0) ? C_OK : (st == 1 ? C_WARN : C_ERR);
    const char* m = st == 0 ? "Monitorando..." : st == 1 ? "Gato detectado!" :
                    st == 2 ? "ERRO: cartao SD" : st == 3 ? "ERRO: camera" :
                    st == 4 ? "ERRO: expansor I/O" : "ERRO: sensor HC-SR04";
    tft.fillRoundRect(10, 182, 300, 26, 8, C_PANEL);
    tft.fillCircle(28, 195, 6, col);
    txt(44, 188, m, 2, C_TEXT, C_PANEL);
  }
}

// ---------- MENU ----------
const char* MENU_ITEMS[5] = {"Registros de fotos", "Ajustar data e hora", "Configuracoes", "Info do sistema", "Voltar ao inicio"};
int8_t menuIdx = 0;

void drawMenuRow(int i) {
  int y = 38 + i * 34;
  bool sel = (i == menuIdx);
  tft.fillRoundRect(10, y, 300, 30, 8, sel ? C_SEL : C_PANEL);
  if (sel) tft.drawRoundRect(10, y, 300, 30, 8, C_ACCENT);
  tft.fillCircle(30, y + 15, 11, sel ? C_ACCENT : C_BG);
  char n[3]; snprintf(n, sizeof(n), "%d", i + 1);
  txtC(30, y + 8, n, 2, sel ? C_BG : C_TEXT, sel ? C_ACCENT : C_BG);
  txt(52, y + 8, MENU_ITEMS[i], 2, C_TEXT, sel ? C_SEL : C_PANEL);
}

void drawMenu() {
  drawHeader("Menu");
  for (int i = 0; i < 5; i++) drawMenuRow(i);
  drawFooter("3  ANTERIOR", "2  ABRIR", "1  PROXIMO");
}

// ---------- REGISTROS ----------
int logSel = 0, logTop = 0, viewIdx = 0;
LogEntry& newestLog(int i) { return logs[logCount - 1 - i]; }

void drawLog() {
  char h[24];
  snprintf(h, sizeof(h), "Registros (%d)", totalEvents);
  drawHeader(h);
  for (int r = 0; r < 5; r++) {
    int idx = logTop + r;
    int y = 38 + r * 34;
    if (idx > logCount) break;
    bool sel = (idx == logSel);
    tft.fillRoundRect(10, y, 300, 30, 8, sel ? C_SEL : C_PANEL);
    if (sel) tft.drawRoundRect(10, y, 300, 30, 8, C_ACCENT);
    uint16_t bg = sel ? C_SEL : C_PANEL;
    if (idx == logCount) {
      txt(20, y + 8, "< Voltar", 2, C_TEXT, bg);
    } else {
      LogEntry &e = newestLog(idx);
      char num[8], row[24];
      snprintf(num, sizeof(num), "#%d", totalEvents - idx);
      snprintf(row, sizeof(row), "%c%c/%c%c %s", e.date[8], e.date[9], e.date[5], e.date[6], e.time);
      tft.fillRoundRect(16, y + 6, 48, 18, 5, sel ? C_ACCENT : C_BG);
      txtC(40, y + 11, num, 1, sel ? C_BG : C_TEXT, sel ? C_ACCENT : C_BG);
      txt(74, y + 8, row, 2, C_TEXT, bg);
    }
  }
  if (logCount == 0) txt(20, 120, "Nenhum registro ainda", 1, C_MUTED, C_BG);
  drawFooter("3  ANTERIOR", "2  ABRIR", "1  PROXIMO");
}

uint8_t* loadFile(const char* path, size_t &len) {
  File f = SD.open(path);
  if (!f) return nullptr;
  len = f.size();
  uint8_t* buf = (uint8_t*)(psramFound() ? ps_malloc(len) : malloc(len));
  if (!buf) { f.close(); return nullptr; }
  size_t r = f.read(buf, len);
  f.close();
  if (r != len) { free(buf); return nullptr; }
  return buf;
}

void drawView() {
  tft.fillScreen(C_BG);
  LogEntry &e = newestLog(viewIdx);
  size_t len = 0;
  uint8_t* buf = sdOk ? loadFile(e.file, len) : nullptr;
  if (buf) { drawJpgFit(buf, len); free(buf); }
  else txtC(160, 110, "Foto nao encontrada", 2, C_ERR, C_BG);
  tft.fillRect(0, 214, 320, 26, C_BG);
  char s[40];
  snprintf(s, sizeof(s), "#%d  %c%c/%c%c/%c%c%c%c  %s", totalEvents - viewIdx,
           e.date[8], e.date[9], e.date[5], e.date[6], e.date[0], e.date[1], e.date[2], e.date[3], e.time);
  txt(6, 218, s, 1, C_TEXT, C_BG);
  txt(6, 229, "3 < mais nova   2 lista   mais antiga > 1", 1, C_MUTED, C_BG);
}

// ---------- DATA E HORA ----------
int dtVal[5];           // dia, mês, ano, hora, minuto
int dtField = 0;        // 0..4 valores | 5 = ações
int dtAction = 0;       // 0 = salvar | 1 = cancelar
const int dtX[5] = {20, 100, 180, 20, 100};
const int dtY[5] = {60, 60, 60, 132, 132};
const int dtW[5] = {64, 64, 120, 64, 64};

void drawDtField(int i) {
  bool sel = (dtField == i);
  tft.fillRoundRect(dtX[i], dtY[i], dtW[i], 44, 8, sel ? C_SEL : C_PANEL);
  if (sel) tft.drawRoundRect(dtX[i], dtY[i], dtW[i], 44, 8, C_WARN);
  char b[8];
  snprintf(b, sizeof(b), i == 2 ? "%04d" : "%02d", dtVal[i]);
  txtC(dtX[i] + dtW[i] / 2, dtY[i] + 12, b, 3, C_TEXT, sel ? C_SEL : C_PANEL);
}

void drawDtActions() {
  bool act = (dtField == 5);
  tft.fillRoundRect(20, 182, 130, 26, 8, (act && dtAction == 0) ? C_OK : C_PANEL);
  tft.fillRoundRect(170, 182, 130, 26, 8, (act && dtAction == 1) ? C_ERR : C_PANEL);
  txtC(85, 188, "SALVAR", 2, (act && dtAction == 0) ? C_BG : C_TEXT, (act && dtAction == 0) ? C_OK : C_PANEL);
  txtC(235, 188, "CANCELAR", 2, (act && dtAction == 1) ? C_BG : C_TEXT, (act && dtAction == 1) ? C_ERR : C_PANEL);
}

void loadDtFromRtc() {
  DateTime n = getNow(true);
  dtVal[0] = n.day(); dtVal[1] = n.month(); dtVal[2] = n.year();
  dtVal[3] = n.hour(); dtVal[4] = n.minute();
  if (dtVal[2] < 2024) dtVal[2] = 2026;
  dtField = 0; dtAction = 0;
}

void drawDateTime() {
  drawHeader("Data e Hora");
  txt(20, 44, "DATA", 1, C_MUTED, C_BG);
  txt(20, 116, "HORA", 1, C_MUTED, C_BG);
  txt(88, 74, "/", 2, C_MUTED, C_BG);
  txt(168, 74, "/", 2, C_MUTED, C_BG);
  txt(88, 146, ":", 2, C_MUTED, C_BG);
  for (int i = 0; i < 5; i++) drawDtField(i);
  drawDtActions();
  drawFooter("3  DIMINUIR", "2  CONFIRMAR", "1  AUMENTAR");
}

int wrapv(int v, int lo, int hi) { if (v > hi) return lo; if (v < lo) return hi; return v; }

void dtChange(int delta) {
  switch (dtField) {
    case 0: dtVal[0] = wrapv(dtVal[0] + delta, 1, daysInMonth(dtVal[2], dtVal[1])); break;
    case 1: dtVal[1] = wrapv(dtVal[1] + delta, 1, 12); break;
    case 2: dtVal[2] = wrapv(dtVal[2] + delta, 2024, 2099); break;
    case 3: dtVal[3] = wrapv(dtVal[3] + delta, 0, 23); break;
    case 4: dtVal[4] = wrapv(dtVal[4] + delta, 0, 59); break;
  }
  int dim = daysInMonth(dtVal[2], dtVal[1]);
  if (dtVal[0] > dim) dtVal[0] = dim;
}

// ---------- CONFIGURAÇÕES ----------
#define CFG_ITEMS 7
int cfgSel = 0, cfgTop = 0;
bool cfgEditing = false;
const char* CFG_LABEL[CFG_ITEMS] = {"Distancia de deteccao", "Intervalo entre fotos", "Iluminacao IR",
                                    "Polaridade do LDR", "Filtro de presenca", "Restaurar padroes", "Voltar"};

void cfgValueStr(int i, char* b, size_t n) {
  b[0] = 0;
  switch (i) {
    case 0: snprintf(b, n, "%d cm", cfg.thresholdCm); break;
    case 1: snprintf(b, n, "%d s", cfg.cooldownS); break;
    case 2: snprintf(b, n, "%s", cfg.irMode == IR_AUTO ? "AUTO" : cfg.irMode == IR_ON ? "LIGADO" : "DESLIG."); break;
    case 3: snprintf(b, n, "%s", cfg.ldrInvert ? "INVERTIDA" : "NORMAL"); break;
    case 4: snprintf(b, n, "%d leit.", cfg.confirmReads); break;
  }
}

void drawSettings() {
  drawHeader("Configuracoes");
  for (int r = 0; r < 6; r++) {
    int i = cfgTop + r;
    if (i >= CFG_ITEMS) break;
    int y = 36 + r * 29;
    bool sel = (i == cfgSel);
    uint16_t bg = sel ? C_SEL : C_PANEL;
    tft.fillRoundRect(10, y, 300, 26, 7, bg);
    if (sel) tft.drawRoundRect(10, y, 300, 26, 7, cfgEditing ? C_WARN : C_ACCENT);
    txt(18, y + 9, CFG_LABEL[i], 1, C_TEXT, bg);
    char v[16]; cfgValueStr(i, v, sizeof(v));
    if (v[0]) txtR(302, y + 6, v, 2, (sel && cfgEditing) ? C_WARN : C_ACCENT, bg);
  }
  if (cfgEditing) drawFooter("3  DIMINUIR", "2  SALVAR", "1  AUMENTAR");
  else            drawFooter("3  ANTERIOR", "2  EDITAR", "1  PROXIMO");
}

void cfgAdjust(int delta) {
  switch (cfgSel) {
    case 0: cfg.thresholdCm  = constrain((int)cfg.thresholdCm + delta, 5, 60); break;
    case 1: cfg.cooldownS    = constrain((int)cfg.cooldownS + delta, 3, 120); break;
    case 2: cfg.irMode       = wrapv((int)cfg.irMode + delta, 0, 2); break;
    case 3: cfg.ldrInvert    = cfg.ldrInvert ? 0 : 1; break;
    case 4: cfg.confirmReads = constrain((int)cfg.confirmReads + delta, 2, 10); break;
  }
}

// ---------- INFO ----------
uint32_t infoT = 0;

void drawInfoBody() {
  tft.fillRect(0, 32, 320, 178, C_BG);
  char b[48]; int y = 36;
  auto line = [&](const char* k, const char* v, uint16_t col) {
    txt(12, y, k, 1, C_MUTED, C_BG); txt(140, y, v, 1, col, C_BG); y += 14;
  };
  line("Firmware", "HydraPet Vision v" FW_VERSION, C_TEXT);
  line("Camera (simulada)", camOk ? "JPEG de teste" : "ERRO JPEG", camOk ? C_TEXT : C_ERR);
  snprintf(b, sizeof(b), "%s", psramFound() ? "PSRAM ativa" : "sem PSRAM");
  line("Memoria de imagem", b, psramFound() ? C_OK : C_WARN);
  line("RTC", rtcOk ? (rtcTimeSuspect ? "OK (hora a ajustar)" : "OK") : "ERRO", rtcOk ? C_OK : C_ERR);
  line("Expansor PCF8574", pcfOk ? "OK" : "ERRO", pcfOk ? C_OK : C_ERR);
  line("Sensor HC-SR04", sensorFault ? "SEM ECO (falha)" : "OK", sensorFault ? C_ERR : C_OK);
  if (sdOk) snprintf(b, sizeof(b), "~%llu / %llu MB", (unsigned long long)(sdUsedB / (1024ULL * 1024ULL)),
                     (unsigned long long)sdTotalMB);
  else snprintf(b, sizeof(b), "ausente");
  line("Cartao SD", b, sdOk ? C_OK : C_ERR);
  snprintf(b, sizeof(b), "%d  (hoje: %d)", totalEvents, todayCount);
  line("Fotos registradas", b, C_TEXT);
  snprintf(b, sizeof(b), "%lu  (hoje: %d)", (unsigned long)visitId, todayVisits);
  line("Visitas", b, C_TEXT);
  snprintf(b, sizeof(b), "%s  | IR %s", isDark ? "ESCURO" : "CLARO", irState ? "ligado" : "desligado");
  line("Luminosidade", b, C_TEXT);
  snprintf(b, sizeof(b), "%u KB livres", (unsigned)(ESP.getFreeHeap() / 1024));
  line("Memoria RAM", b, C_TEXT);
  uint32_t s = millis() / 1000;
  snprintf(b, sizeof(b), "%02u:%02u:%02u", (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
  line("Ligado ha", b, C_TEXT);
}

void drawInfo() {
  drawHeader("Sistema");
  drawInfoBody();
  drawFooter("3  VOLTAR", "2  VOLTAR", "");
  infoT = millis();
}

// ---------- DESPACHO ----------
void drawScreen() {
  if (fullClear) tft.fillScreen(C_BG); else if (screen != SCR_VIEW) tft.fillRect(0, 32, 320, 180, C_BG);
  fullClear = false;
  switch (screen) {
    case SCR_HOME:     drawHomeStatic(); break;
    case SCR_MENU:     drawMenu(); break;
    case SCR_LOG:      drawLog(); break;
    case SCR_VIEW:     drawView(); break;
    case SCR_DATETIME: drawDateTime(); break;
    case SCR_SETTINGS: drawSettings(); break;
    case SCR_INFO:     drawInfo(); break;
  }
}

void gotoScreen(Screen s) {
  screen = s;
  needRedraw = true;
  fullClear = true;
  lastHeaderMin = -1;
}

// ============================================================================
//  TRATAMENTO DOS BOTÕES POR TELA
// ============================================================================
void handleEvent(BtnEvent ev) {
  if (ev == EV_NONE) return;
  lastActivity = millis();

  if (previewActive) { previewActive = false; needRedraw = true; return; }
  if (ev == EV_OK_LONG) { cfgEditing = false; if (screen != SCR_HOME) gotoScreen(SCR_HOME); return; }

  switch (screen) {
    case SCR_HOME:
      if (ev == EV_OK) { menuIdx = 0; gotoScreen(SCR_MENU); }
      break;

    case SCR_MENU: {
      int8_t old = menuIdx;
      if (ev == EV_NEXT) menuIdx = (menuIdx + 1) % 5;
      else if (ev == EV_BACK) menuIdx = (menuIdx + 4) % 5;
      else if (ev == EV_OK) {
        switch (menuIdx) {
          case 0: logSel = 0; logTop = 0; gotoScreen(SCR_LOG); break;
          case 1: loadDtFromRtc(); gotoScreen(SCR_DATETIME); break;
          case 2: cfgSel = 0; cfgTop = 0; cfgEditing = false; gotoScreen(SCR_SETTINGS); break;
          case 3: gotoScreen(SCR_INFO); break;
          case 4: gotoScreen(SCR_HOME); break;
        }
        break;
      }
      if (old != menuIdx) { drawMenuRow(old); drawMenuRow(menuIdx); }
      break;
    }

    case SCR_LOG:
      if (ev == EV_NEXT) logSel = (logSel + 1) % (logCount + 1);
      else if (ev == EV_BACK) logSel = (logSel + logCount) % (logCount + 1);
      else if (ev == EV_OK) {
        if (logSel >= logCount) { gotoScreen(SCR_MENU); break; }
        viewIdx = logSel; gotoScreen(SCR_VIEW); break;
      }
      if (logSel < logTop) logTop = logSel;
      if (logSel >= logTop + 5) logTop = logSel - 4;
      needRedraw = true;
      break;

    case SCR_VIEW:
      if (ev == EV_NEXT && viewIdx < logCount - 1) { viewIdx++; needRedraw = true; }
      else if (ev == EV_BACK && viewIdx > 0) { viewIdx--; needRedraw = true; }
      else if (ev == EV_OK) { logSel = viewIdx; if (logSel < logTop) logTop = logSel; if (logSel >= logTop + 5) logTop = logSel - 4; gotoScreen(SCR_LOG); }
      break;

    case SCR_DATETIME:
      if (dtField < 5) {
        if (ev == EV_NEXT || ev == EV_BACK) {
          dtChange(ev == EV_NEXT ? 1 : -1);
          drawDtField(dtField);
          if (dtField == 1 || dtField == 2) drawDtField(0);   // dia pode ter sido ajustado
        } else if (ev == EV_OK) {
          int old = dtField; dtField++;
          drawDtField(old);
          if (dtField < 5) drawDtField(dtField); else drawDtActions();
        }
      } else {
        if (ev == EV_NEXT || ev == EV_BACK) { dtAction ^= 1; drawDtActions(); }
        else if (ev == EV_OK) {
          if (dtAction == 0) {
            if (rtcOk) {
              rtc.adjust(DateTime(dtVal[2], dtVal[1], dtVal[0], dtVal[3], dtVal[4], 0));
              getNow(true); recountToday(); recountVisitsToday(); rtcTimeSuspect = false;
            }
            gotoScreen(SCR_MENU);
            drawScreen(); needRedraw = false;
            toast(rtcOk ? "Data e hora salvas!" : "RTC nao encontrado", rtcOk ? C_OK : C_ERR);
          } else gotoScreen(SCR_MENU);
        }
      }
      break;

    case SCR_SETTINGS:
      if (!cfgEditing) {
        if (ev == EV_NEXT) cfgSel = (cfgSel + 1) % CFG_ITEMS;
        else if (ev == EV_BACK) cfgSel = (cfgSel + CFG_ITEMS - 1) % CFG_ITEMS;
        else if (ev == EV_OK) {
          if (cfgSel == 6) { gotoScreen(SCR_MENU); break; }
          if (cfgSel == 5) { cfgDefaults(); cfgSave(); toast("Padroes restaurados", C_OK); break; }
          cfgEditing = true;
        }
        if (cfgSel < cfgTop) cfgTop = cfgSel;
        if (cfgSel >= cfgTop + 6) cfgTop = cfgSel - 5;
      } else {
        if (ev == EV_NEXT) cfgAdjust(+1);
        else if (ev == EV_BACK) cfgAdjust(-1);
        else if (ev == EV_OK) { cfgEditing = false; cfgSave(); }
      }
      needRedraw = true;
      break;

    case SCR_INFO:
      if (ev == EV_OK || ev == EV_BACK) gotoScreen(SCR_MENU);
      break;
  }
}

// ============================================================================
//  TAREFAS PERIÓDICAS DE UI
// ============================================================================
int lastDay = -1;
void dayWatch() {                         // recalcula os contadores diários na virada do dia
  int d = getNow().day();
  if (d == lastDay) return;
  if (lastDay != -1) { recountToday(); recountVisitsToday(); }
  lastDay = d;
}

void uiTask() {
  uint32_t now = millis();

  dayWatch();
  if (toastUntil && (int32_t)(now - toastUntil) >= 0) { toastUntil = 0; needRedraw = true; }
  if (needRedraw) { needRedraw = false; drawScreen(); }

  if (screen != SCR_HOME && now - lastActivity > MENU_TIMEOUT_MS) gotoScreen(SCR_HOME);

  switch (screen) {
    case SCR_HOME:
      if (previewActive) { if ((int32_t)(now - previewUntil) >= 0) { previewActive = false; needRedraw = true; } }
      else if (!toastUntil) updateHome();
      break;
    case SCR_INFO:
      if (now - infoT > 1000) { infoT = now; drawInfoBody(); }
      break;
    case SCR_VIEW: break;
    default:
      if (getNow().minute() != lastHeaderMin) drawHeaderClock();
      break;
  }
}

// A remontagem do SD bloqueia o laço por centenas de ms: só ocorre fora de uma visita
void sdRetryTask() {
  static uint32_t t = 0;
  if (sdOk || inVisit || millis() - t < SD_RETRY_MS) return;
  t = millis();
  SD.end();
  sdOk = initSD();
  if (sdOk) { sdReadCapacity(); loadLog(); }
}

// ============================================================================
//  SETUP
// ============================================================================
int bootY = 118;
void bootLine(const char* label, bool ok, const char* extra) {
  Serial.printf("[BOOT] %s: %s\n", label, ok ? "OK" : "ERRO");
  txt(40, bootY, label, 1, C_TEXT, C_BG);
  txtR(280, bootY, ok ? "OK" : "ERRO", 1, ok ? C_OK : C_ERR, C_BG);
  if (extra[0]) txt(150, bootY, extra, 1, C_MUTED, C_BG);
  bootY += 14;
}

void setup() {

  Serial.begin(115200);
  Serial.println("[HydraPet TFT-R2] Inicio do sketch");
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  Serial.println("[TFT] Inicializando ILI9341");
  tft.begin(8000000);
  tft.setRotation(1);                         // paisagem 320x240
  initColors();
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 320, 14, 0x03FF);
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(TJPG_SWAP_BYTES);
  TJpgDec.setCallback(tftOutput);

  txtC(160, 30, "HydraPet", 4, C_ACCENT, C_BG);
  txtC(160, 68, "Vision", 3, C_TEXT, C_BG);
  txtC(160, 98, "v" FW_VERSION, 1, C_MUTED, C_BG);

  Serial.println("[TFT] Desenho de abertura enviado");
  cfgLoad();

  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  Wire.setTimeOut(30);
  uint16_t largura = 0, altura = 0;
  camOk = TJpgDec.getJpgSize(&largura, &altura, quadroDia, sizeof(quadroDia)) == JDR_OK
          && largura == 320 && altura == 240
          && TJpgDec.getJpgSize(&largura, &altura, quadroIR, sizeof(quadroIR)) == JDR_OK
          && largura == 320 && altura == 240;
  bootLine("Camera (simulada)", camOk, "JPEG de teste");

  // --- Expansor de I/O ---
  Wire.beginTransmission(PCF_ADDR);
  pcfOk = (Wire.endTransmission() == 0);
  if (pcfOk) pcfWrite(pcfShadow);              // IR desligado, TRIG baixo, botões em modo entrada
  bootLine("Expansor PCF8574", pcfOk);

  // --- RTC ---
  rtcOk = rtc.begin();
  if (rtcOk) {
#if USE_DS3231
    if (rtc.lostPower()) { rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); rtcTimeSuspect = true; }
#else
    if (!rtc.isrunning()) { rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); rtcTimeSuspect = true; }
#endif
  }
  bootLine("Relogio RTC", rtcOk, rtcTimeSuspect ? "ajuste a hora" : "");

  // O cartão e o TFT compartilham o SPI, cada um com seu CS.
  Serial.println("[SD] Montando cartao");
  sdOk = initSD();
  if (sdOk) {
    txt(40, bootY, "Lendo cartao SD...", 1, C_MUTED, C_BG);
    sdReadCapacity();
    loadLog();
    tft.fillRect(40, bootY, 240, 10, C_BG);
  }
  char ex[24]; snprintf(ex, sizeof(ex), "%d fotos", totalEvents);
  bootLine("Cartao SD", sdOk, sdOk ? ex : "");

  // --- Sensor ultrassônico (ECHO por interrupção) + autoteste ---
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);
  bool sensOk = sensorSelfTest();
  sensorFault = !sensOk;
  bootLine("Sensor HC-SR04", sensOk, sensOk ? "" : "sem eco");

  // --- Leitura inicial do LDR ---
  if (pcfOk) { bool ok; uint8_t in = pcfRead(ok); if (ok) { pcfIn = in; lightUpdate(in); } }

  lastPhoto = millis() - (uint32_t)cfg.cooldownS * 1000UL - 1;   // primeira foto liberada
  delay(1200);
  lastActivity = millis();
  gotoScreen(SCR_HOME);
  Serial.println("[HydraPet] Monitoramento iniciado");
}

// ============================================================================
//  LOOP PRINCIPAL (cooperativo; pausas bloqueantes pontuais descritas no relatório)
// ============================================================================
void loop() {
  pcfTask();                 // botões (10 ms) + LDR
  sensorTask();              // ultrassom por interrupção (60 ms)
  visitTask();               // encerra a visita após VISITA_GAP_MS sem presença
  irTask();                  // acende/apaga LEDs IR conforme modo e luminosidade
  if (captureRequested) doCapture();
  uiTask();                  // telas / relógio / status / virada do dia
  sdRetryTask();             // remonta o SD se foi removido (fora de visitas)
  delay(1);
}
