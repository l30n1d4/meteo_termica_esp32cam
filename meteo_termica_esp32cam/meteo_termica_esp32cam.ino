/*
  ============================================================================
   CRUSCOTTO METEO SU STAMPANTE TERMICA - versione ESP32-CAM (AI-Thinker)
   Stampante: CSN-A2-T (Adafruit 597)
  ============================================================================
   Funzionamento
   - Stampa COMPLETA ogni mattina alle 08:00 e all'accensione / ritorno della
     corrente / tasto RST:
       * condizioni attuali
       * oggi a fasce: mattino e pomeriggio se è mattina; dal pomeriggio in poi
         solo la parte che resta (pomeriggio, oppure sera)
       * domani (giornata + mattino/pomeriggio in breve)
       * pioggia ora per ora su 4 giorni, con * nelle ore di neve
       * sintesi di oggi e domani + quando è attesa la prossima pioggia
       * batteria
   - Pulsante esterno su IO14 (predisposto, facoltativo): stampa RIDOTTA,
     solo le condizioni attuali.
   - Tra una stampa e l'altra va in deep sleep (risveglio da timer interno,
     al massimo ogni 6h per risincronizzare l'orologio). La stampante viene
     accesa da un relè solo per il tempo della stampa.
   - In fondo allo scontrino: tensione e carica stimata della batteria, con
     avviso quando è ora di ricaricare. Batteria quasi vuota: stampa un
     ultimo avviso e smette di stampare finché non viene ricaricata.

   Collegamenti (tutti sulla fila pin 5V-GND-IO12-IO13-IO15-IO14-IO2-IO4)
   - 5V / GND <- uscita 5V del modulo carica + boost
   - IO13 -> RX della stampante (il TX della stampante NON va collegato)
   - IO15 <- partitore 100k/100k dal + batterie (lettura tensione)
   - IO2  -> 4,7k -> base BC337; collettore -> IN1 modulo relè HL-52S;
             emettitore -> GND. IN2 del modulo su VCC.
   - Relè 1: COM -> +5V, NO -> + stampante. GND stampante sempre al GND comune.
   - IO14 -> pulsante -> GND (facoltativo: stampa solo le condizioni attuali)
   - IO12, IO4: liberi

   Programmazione (Arduino IDE)
   - Core "esp32" by Espressif, scheda "AI Thinker ESP32-CAM"
   - Caricare con la base ESP32-CAM-MB, poi staccarla.
   - Librerie: "Adafruit Thermal Printer Library", "ArduinoJson" 7.x
  ============================================================================
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Adafruit_Thermal.h>
#include <time.h>
#include <stdarg.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"

// ============================ TIPI DI DATI ==================================
// Tutti i tipi stanno qui, subito dopo gli #include: l'IDE Arduino aggiunge da
// solo le dichiarazioni delle funzioni in un punto del file che può essere
// molto in alto, e a quel punto i tipi devono essere già definiti.
struct CurrentWx {
  float temp, feels, precip, snow, pressure, wind, gusts;
  int hum, clouds, code, windDir;
};

struct DayWx {
  int y, m, d, wday, code, pprob;
  float tmax, tmin, psum, snow, windMax, uv;
  char sunrise[6], sunset[6];
};

// Riassunto di una fascia oraria (mattino, pomeriggio, sera)
struct SlotWx { bool ok; int code, pmax; float tmin, tmax, rain, snow, wind; };

// ============================ CONFIGURAZIONE ================================
const char* WIFI_SSID     = "WIFI_SSID";
const char* WIFI_PASSWORD = "WIFI_PASSWORD";

// Località: nome da stampare + coordinate decimali (da Google Maps: tasto destro sul punto)
const char* LOCATION_NAME = "Roma";
const float LATITUDE      = 41.9028;
const float LONGITUDE     = 12.4964;

// Fuso orario (POSIX per l'orologio, IANA url-encoded per Open-Meteo)
const char* TZ_POSIX = "CET-1CEST,M3.5.0,M10.5.0/3";   // Italia, con ora legale
const char* TZ_API   = "Europe%2FRome";

const int   MORNING_HOUR        = 8;     // unica stampa automatica del giorno
const int   MORNING_FROM        = 6;     // fasce della giornata: mattino 6-12,
const int   AFTERNOON_FROM      = 12;    //   pomeriggio 12-18,
const int   EVENING_FROM        = 18;    //   sera 18-24
const int   RAIN_HINT_PROB      = 30;    // % giornaliera oltre cui segnalare la "prossima pioggia"
const int   RAIN_PROB_THRESHOLD = 50;    // % oraria oltre cui "piove"
const float RAIN_MM_THRESHOLD   = 1.0;   // mm/giorno oltre cui "piove"
const float SNOW_CM_DAY         = 0.1;   // cm/giorno oltre cui "nevica"
const float SNOW_CM_HOUR        = 0.05;  // cm/ora oltre cui l'ora conta come nevosa

#define PRINTER_BAUD 19200   // vedi la pagina di autotest (tasto feed premuto all'accensione); alcune sono a 9600
#define PRINTER_HEAT 140     // aumenta (fino a ~200) per una stampa più scura
#define COLS 31              // caratteri per riga usati (la testina ne fa 32)

// Pin ESP32-CAM (liberi senza microSD e senza fotocamera)
#define PRINTER_TX_PIN 13    // -> RX stampante
#define RELAY_PIN       2    // -> BC337 -> IN1 relè (alto = stampante accesa)
#define BATT_PIN       15    // <- partitore della batteria (ADC2: letto prima del WiFi)
#define BUTTON_PIN     14    // pulsante verso GND, sveglia dal deep sleep
#define FLASH_LED_PIN   4    // LED flash della scheda: lo teniamo spento

#define PRINTER_BOOT_MS 2000 // attesa dopo l'accensione del relè prima di inviare dati

// Batteria (3 celle Li-ion in parallelo)
// Partitore: R1 dal + batterie a IO15, R2 da IO15 a GND.
// Sulla ESP32-CAM, IO15 ha anche una resistenza di pull-up da 47k verso 3,3 V
// (linea della microSD, saldata sulla scheda): il calcolo ne tiene conto.
const float BATT_R1      = 100000.0;  // ohm
const float BATT_R2      = 100000.0;  // ohm
const float BATT_RPU     = 47000.0;   // pull-up della scheda su IO15
const float BATT_V33     = 3.30;      // tensione del pin 3V3 (misurabile col tester)
const float BATT_CAL     = 1.00;      // ritocco finale: (tensione tester) / (tensione nel log)
const float BATT_SOON    = 3.55;  // sotto: "RICARICARE PRESTO"
const float BATT_NOW     = 3.40;  // sotto: "RICARICARE ORA"
const float BATT_EMPTY   = 3.20;  // sotto: niente più stampe (protegge le celle)
const float BATT_RESUME  = 3.50;  // sopra: riprende a funzionare dopo la ricarica
// ============================================================================

#define FC_DAYS    7                // giorni di previsione scaricati (per la "prossima pioggia")
#define CHART_DAYS 4                // giorni nel grafico orario
#define FC_HOURS   (CHART_DAYS * 24)

HardwareSerial printerSerial(1);
Adafruit_Thermal printer(&printerSerial);

// Sopravvive al deep sleep (si azzera a ogni accensione/reset)
RTC_DATA_ATTR int32_t lastSlot = -1;
RTC_DATA_ATTR bool    battEmpty = false;   // batteria scarica: avviso già stampato

float battV = 0;     // tensione batteria calcolata a questo risveglio
float battPinV = 0;  // tensione letta direttamente su IO15 (per il log)

// ---------------------------- Dati meteo ------------------------------------
CurrentWx cur;
DayWx     days[FC_DAYS];

int   hProb[FC_HOURS];
int   hCode[FC_HOURS];
float hTemp[FC_HOURS];
float hRain[FC_HOURS];
float hSnow[FC_HOURS];
float hWind[FC_HOURS];
int   nHours = 0;

// ---------------------------- Testi italiani --------------------------------
const char* DAY_FULL[]  = {"Domenica","Lunedì","Martedì","Mercoledì","Giovedì","Venerdì","Sabato"};
const char* DAY_ABBR[]  = {"Dom","Lun","Mar","Mer","Gio","Ven","Sab"};
const char* MONTHS[]    = {"gennaio","febbraio","marzo","aprile","maggio","giugno",
                           "luglio","agosto","settembre","ottobre","novembre","dicembre"};

// ======================= UTF-8 -> CP437 (font stampante) ====================
uint8_t mapCodepoint(uint16_t cp) {
  switch (cp) {
    case 0x00E0: return 0x85;  // à
    case 0x00E8: return 0x8A;  // è
    case 0x00E9: return 0x82;  // é
    case 0x00EC: return 0x8D;  // ì
    case 0x00F2: return 0x95;  // ò
    case 0x00F9: return 0x97;  // ù
    case 0x00C0: return 'A';
    case 0x00C8: case 0x00C9: return 'E';
    case 0x00B0: return 0xF8;  // °
    case 0x00B7: return 0xFA;  // ·
    case 0x00B1: return 0xF1;  // ±
    case 0x00BB: return 0xAF;  // »
    case 0x00AB: return 0xAE;  // «
    case 0x2265: return 0xF2;  // ≥
    case 0x2264: return 0xF3;  // ≤
    case 0x2591: return 0xB0;  // ░
    case 0x2592: return 0xB1;  // ▒
    case 0x2593: return 0xB2;  // ▓
    case 0x2588: return 0xDB;  // █
    case 0x2500: return 0xC4;  // ─
    case 0x2550: return 0xCD;  // ═
    case 0x2551: return 0xBA;  // ║
    case 0x2554: return 0xC9;  // ╔
    case 0x2557: return 0xBB;  // ╗
    case 0x255A: return 0xC8;  // ╚
    case 0x255D: return 0xBC;  // ╝
    case 0x2019: return '\'';
    default:     return '?';
  }
}

size_t toCP437(const char* in, char* out, size_t outSize) {
  size_t o = 0;
  const uint8_t* p = (const uint8_t*)in;
  while (*p && o < outSize - 1) {
    uint8_t c = p[0];
    if (c < 0x80) { out[o++] = (char)c; p++; continue; }
    if ((c & 0xE0) == 0xC0 && p[1]) {
      uint16_t cp = ((c & 0x1F) << 6) | (p[1] & 0x3F);
      out[o++] = (char)mapCodepoint(cp); p += 2; continue;
    }
    if ((c & 0xF0) == 0xE0 && p[1] && p[2]) {
      uint16_t cp = ((c & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F);
      out[o++] = (char)mapCodepoint(cp); p += 3; continue;
    }
    out[o++] = '?'; p++;
  }
  out[o] = 0;
  return o;
}

// ============================ Helper di stampa ==============================
void logTime(const char* prefix, time_t t) {
  struct tm lt; localtime_r(&t, &lt);
  Serial.printf("%s%s %02d/%02d/%04d %02d:%02d:%02d\n", prefix, DAY_FULL[lt.tm_wday],
                lt.tm_mday, lt.tm_mon + 1, lt.tm_year + 1900, lt.tm_hour, lt.tm_min, lt.tm_sec);
}

void writeBytes(const char* s, int n) { for (int i = 0; i < n; i++) printer.write((uint8_t)s[i]); }

void pr(const char* utf8) {
  static char buf[256];
  toCP437(utf8, buf, sizeof(buf));
  writeBytes(buf, strlen(buf));
}

void prf(const char* fmt, ...) {
  char tmp[256];
  va_list ap; va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  pr(tmp);
}

void hline(uint8_t ch) { for (int i = 0; i < COLS; i++) printer.write(ch); printer.write('\n'); }

// Testo lungo con a-capo sulle parole (indent = spazi a inizio di ogni riga)
void prWrapIndent(const char* utf8, int indent) {
  static char buf[512];
  toCP437(utf8, buf, sizeof(buf));
  int w = COLS - indent;
  int len = strlen(buf), start = 0;
  while (start < len) {
    while (buf[start] == ' ') start++;
    if (start >= len) break;
    for (int k = 0; k < indent; k++) printer.write(' ');
    if (len - start <= w) { writeBytes(buf + start, len - start); printer.write('\n'); break; }
    int cut = start + w, sp = cut;
    while (sp > start && buf[sp] != ' ') sp--;
    if (sp == start) sp = cut;
    writeBytes(buf + start, sp - start); printer.write('\n');
    start = sp;
  }
}

void prWrap(const char* utf8) { prWrapIndent(utf8, 0); }

// Frase della sintesi: va su carta e, come sommario, anche sul seriale
void say(const char* utf8) {
  prWrap(utf8);
  Serial.printf("  > %s\n", utf8);
}

void boxTop()    { printer.write(0xC9); for (int i = 0; i < COLS - 2; i++) printer.write(0xCD); printer.write(0xBB); printer.write('\n'); }
void boxBottom() { printer.write(0xC8); for (int i = 0; i < COLS - 2; i++) printer.write(0xCD); printer.write(0xBC); printer.write('\n'); }

void boxRow(const char* label, const char* value) {
  char l[40], v[40], line[80];
  toCP437(label, l, sizeof(l));
  toCP437(value, v, sizeof(v));
  snprintf(line, sizeof(line), " %-11s%s", l, v);
  int n = strlen(line);
  if (n > COLS - 2) n = COLS - 2;
  printer.write(0xBA);
  writeBytes(line, n);
  for (int i = n; i < COLS - 2; i++) printer.write(' ');
  printer.write(0xBA); printer.write('\n');
}

// ======================== Alimentazione stampante ===========================
bool printerReady = false;

// Accende il relè, avvia la seriale e inizializza la stampante
void ensurePrinter() {
  if (printerReady) return;
  Serial.println("Relè ON: accendo la stampante");
  digitalWrite(RELAY_PIN, HIGH);
  printerSerial.begin(PRINTER_BAUD, SERIAL_8N1, -1, PRINTER_TX_PIN);   // solo TX
  delay(PRINTER_BOOT_MS);
  printer.begin(PRINTER_HEAT);
  printer.setDefault();
  printer.setCodePage(CODEPAGE_CP437);
  printerReady = true;
}

// Aspetta che finisca di stampare, abbassa la linea dati e spegne il relè.
// La linea dati deve stare BASSA: se restasse alta alimenterebbe la stampante
// spenta attraverso il suo ingresso.
void printerPowerOff() {
  if (printerReady) {
    printerSerial.flush();
    delay(2500);                        // la testina finisce le ultime righe
    printerSerial.end();
    Serial.println("Relè OFF: stampante spenta");
  }
  pinMode(PRINTER_TX_PIN, OUTPUT);
  digitalWrite(PRINTER_TX_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  printerReady = false;
}

// ============================== Batteria ====================================
// Legge la tensione delle batterie (media di 32 letture). Va chiamata PRIMA
// di accendere il WiFi: l'ADC2 (IO15) non funziona con il WiFi attivo.
float readBattery() {
  // IO15 è un pin di avvio: all'accensione ha la resistenza di pull-up interna
  // attiva (~30-50k verso 3,3 V), che "alza" il partitore e falsa la lettura.
  // La spengo esplicitamente prima di misurare.
  pinMode(BATT_PIN, INPUT);
  gpio_pullup_dis((gpio_num_t)BATT_PIN);
  gpio_pulldown_dis((gpio_num_t)BATT_PIN);
  rtc_gpio_pullup_dis((gpio_num_t)BATT_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)BATT_PIN);
  delay(20);                                   // lascia assestare il partitore
  analogReadMilliVolts(BATT_PIN);              // prima lettura scartata
  uint32_t sum = 0;
  for (int i = 0; i < 32; i++) { sum += analogReadMilliVolts(BATT_PIN); delay(2); }
  battPinV = (sum / 32.0f) / 1000.0f;
  // Nodo IO15: (Vbat - Vpin)/R1 + (V33 - Vpin)/Rpu = Vpin/R2  ->  ricavo Vbat
  float vbat = BATT_R1 * (battPinV * (1.0f / BATT_R1 + 1.0f / BATT_R2 + 1.0f / BATT_RPU)
                          - BATT_V33 / BATT_RPU);
  return vbat * BATT_CAL;
}

// Carica stimata da tensione a riposo (curva tipica Li-ion, indicativa)
int battPercent(float v) {
  static const float V[] = {3.00, 3.30, 3.40, 3.50, 3.60, 3.70, 3.80, 3.90, 4.00, 4.10, 4.20};
  static const int   P[] = {   0,    2,    5,   10,   20,   37,   52,   66,   79,   90,  100};
  if (v <= V[0]) return 0;
  for (int i = 1; i < 11; i++)
    if (v <= V[i]) return P[i - 1] + (int)((v - V[i - 1]) / (V[i] - V[i - 1]) * (P[i] - P[i - 1]));
  return 100;
}

// "3.92" -> "3,92"
void fmtVolt(char* out, size_t n, float v) {
  snprintf(out, n, "%.2f", v);
  for (char* p = out; *p; p++) if (*p == '.') *p = ',';
}

// ============================ Meteo: testi ==================================
const char* wxText(int c) {
  switch (c) {
    case 0:  return "Sereno";
    case 1:  return "Poco nuvoloso";
    case 2:  return "Parz. nuvoloso";
    case 3:  return "Coperto";
    case 45: case 48: return "Nebbia";
    case 51: case 53: case 55: return "Pioviggine";
    case 56: case 57: return "Pioviggine gelata";
    case 61: return "Pioggia debole";
    case 63: return "Pioggia";
    case 65: return "Pioggia forte";
    case 66: case 67: return "Pioggia gelata";
    case 71: return "Neve debole";
    case 73: return "Neve";
    case 75: return "Neve forte";
    case 77: return "Nevischio";
    case 80: return "Rovesci deboli";
    case 81: return "Rovesci";
    case 82: return "Rovesci violenti";
    case 85: case 86: return "Rovesci di neve";
    case 95: return "Temporale";
    case 96: case 99: return "Temporale e grandine";
    default: return "n.d.";
  }
}
bool isSnowCode(int c) { return (c >= 71 && c <= 77) || c == 85 || c == 86; }

const char* windDirText(int deg) {
  static const char* dirs[] = {"N","NE","E","SE","S","SO","O","NO"};
  return dirs[((int)((deg + 22.5) / 45.0)) & 7];
}

int weekdayOf(int y, int m, int d) {
  struct tm t = {};
  t.tm_year = y - 1900; t.tm_mon = m - 1; t.tm_mday = d; t.tm_hour = 12; t.tm_isdst = -1;
  mktime(&t);
  return t.tm_wday;
}

bool isRainyDay(int i) { return days[i].pprob >= RAIN_PROB_THRESHOLD || days[i].psum >= RAIN_MM_THRESHOLD; }
bool isSnowDay(int i)  { return days[i].snow >= SNOW_CM_DAY || isSnowCode(days[i].code); }

bool chartHasSnow() {
  for (int i = 0; i < nHours; i++) if (hSnow[i] >= SNOW_CM_HOUR) return true;
  return false;
}

// Fascia oraria di pioggia nel giorno d (0=oggi) a partire da fromHour.
// Le ore di neve non contano qui: le segnala snowWindow.
bool rainWindow(int d, int fromHour, int& h1, int& h2, int& maxP) {
  h1 = -1; h2 = -1; maxP = 0;
  for (int h = fromHour; h < 24; h++) {
    int i = d * 24 + h;
    if (i >= nHours) break;
    if (hSnow[i] >= SNOW_CM_HOUR) continue;
    int p = hProb[i];
    if (p > maxP) maxP = p;
    if (p >= RAIN_PROB_THRESHOLD) { if (h1 < 0) h1 = h; h2 = h; }
  }
  return h1 >= 0;
}

// Fascia oraria di neve nel giorno d a partire da fromHour (tot = cm accumulati)
bool snowWindow(int d, int fromHour, int& h1, int& h2, float& tot) {
  h1 = -1; h2 = -1; tot = 0;
  for (int h = fromHour; h < 24; h++) {
    int i = d * 24 + h;
    if (i >= nHours) break;
    tot += hSnow[i];
    if (hSnow[i] >= SNOW_CM_HOUR) { if (h1 < 0) h1 = h; h2 = h; }
  }
  return h1 >= 0;
}

// ============================ Rete e dati ===================================
bool connectWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > 20000) { Serial.println("WiFi: timeout"); return false; }
    delay(250);
  }
  Serial.printf("WiFi ok, IP %s\n", WiFi.localIP().toString().c_str());
  return true;
}

bool syncTime() {
  configTzTime(TZ_POSIX, "pool.ntp.org", "time.google.com");
  uint32_t t0 = millis();
  while (time(nullptr) < 1700000000) {
    if (millis() - t0 > 15000) { Serial.println("NTP: timeout"); return false; }
    delay(200);
  }
  return true;
}

bool parseWeather(Stream& s) {
  JsonDocument filter;
  filter["current"] = true;
  filter["daily"] = true;
  filter["hourly"]["precipitation_probability"] = true;
  filter["hourly"]["temperature_2m"] = true;
  filter["hourly"]["snowfall"] = true;
  filter["hourly"]["precipitation"] = true;
  filter["hourly"]["weather_code"] = true;
  filter["hourly"]["wind_speed_10m"] = true;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, s, DeserializationOption::Filter(filter));
  if (err) { Serial.printf("JSON: %s\n", err.c_str()); return false; }

  JsonObject c = doc["current"];
  if (c.isNull()) return false;
  cur.temp     = c["temperature_2m"] | 0.0f;
  cur.feels    = c["apparent_temperature"] | 0.0f;
  cur.hum      = c["relative_humidity_2m"] | 0;
  cur.precip   = c["precipitation"] | 0.0f;
  cur.snow     = c["snowfall"] | 0.0f;
  cur.code     = c["weather_code"] | 0;
  cur.clouds   = c["cloud_cover"] | 0;
  cur.pressure = c["pressure_msl"] | 0.0f;
  cur.wind     = c["wind_speed_10m"] | 0.0f;
  cur.windDir  = c["wind_direction_10m"] | 0;
  cur.gusts    = c["wind_gusts_10m"] | 0.0f;

  JsonObject d = doc["daily"];
  for (int i = 0; i < FC_DAYS; i++) {
    const char* t = d["time"][i] | "2000-01-01";
    sscanf(t, "%d-%d-%d", &days[i].y, &days[i].m, &days[i].d);
    days[i].wday    = weekdayOf(days[i].y, days[i].m, days[i].d);
    days[i].code    = d["weather_code"][i] | 0;
    days[i].tmax    = d["temperature_2m_max"][i] | 0.0f;
    days[i].tmin    = d["temperature_2m_min"][i] | 0.0f;
    days[i].psum    = d["precipitation_sum"][i] | 0.0f;
    days[i].snow    = d["snowfall_sum"][i] | 0.0f;
    days[i].pprob   = d["precipitation_probability_max"][i] | 0;
    days[i].windMax = d["wind_speed_10m_max"][i] | 0.0f;
    days[i].uv      = d["uv_index_max"][i] | 0.0f;
    const char* sr = d["sunrise"][i] | "";
    const char* ss = d["sunset"][i] | "";
    if (strlen(sr) >= 16) { memcpy(days[i].sunrise, sr + 11, 5); days[i].sunrise[5] = 0; } else strcpy(days[i].sunrise, "--:--");
    if (strlen(ss) >= 16) { memcpy(days[i].sunset,  ss + 11, 5); days[i].sunset[5]  = 0; } else strcpy(days[i].sunset,  "--:--");
  }

  JsonArray hp = doc["hourly"]["precipitation_probability"];
  JsonArray ht = doc["hourly"]["temperature_2m"];
  JsonArray hs = doc["hourly"]["snowfall"];
  JsonArray hr = doc["hourly"]["precipitation"];
  JsonArray hc = doc["hourly"]["weather_code"];
  JsonArray hw = doc["hourly"]["wind_speed_10m"];
  nHours = hp.size() < FC_HOURS ? hp.size() : FC_HOURS;
  for (int i = 0; i < nHours; i++) {
    hProb[i] = hp[i] | 0;
    hCode[i] = hc[i] | 0;
    hTemp[i] = ht[i] | 0.0f;
    hRain[i] = hr[i] | 0.0f;
    hSnow[i] = hs[i] | 0.0f;
    hWind[i] = hw[i] | 0.0f;
  }

  Serial.printf("Meteo ricevuto: %.1f°C, %s, umidità %d%%, %.0f mbar\n",
                cur.temp, wxText(cur.code), cur.hum, cur.pressure);
  for (int i = 0; i < 3; i++)
    Serial.printf("  %s %02d/%02d: %s, %.0f°/%.0f°, pioggia %d%% (%.1f mm)%s\n",
                  DAY_ABBR[days[i].wday], days[i].d, days[i].m, wxText(days[i].code),
                  days[i].tmin, days[i].tmax, days[i].pprob, days[i].psum,
                  isSnowDay(i) ? ", NEVE" : "");
  return true;
}

bool fetchWeatherVia(bool secure) {
  String url = String(secure ? "https" : "http") +
    "://api.open-meteo.com/v1/forecast?latitude=" + String(LATITUDE, 4) +
    "&longitude=" + String(LONGITUDE, 4) +
    "&current=temperature_2m,relative_humidity_2m,apparent_temperature,precipitation,snowfall,"
    "weather_code,cloud_cover,pressure_msl,wind_speed_10m,wind_direction_10m,wind_gusts_10m"
    "&hourly=temperature_2m,precipitation_probability,precipitation,snowfall,weather_code,wind_speed_10m"
    "&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_sum,snowfall_sum,"
    "precipitation_probability_max,wind_speed_10m_max,sunrise,sunset,uv_index_max"
    "&timezone=" + TZ_API + "&forecast_days=" + String(FC_DAYS);

  WiFiClient plain;
  WiFiClientSecure tls;
  tls.setInsecure();               // niente verifica certificato: dati pubblici
  HTTPClient http;
  http.useHTTP10(true);            // evita il chunked encoding -> parsing in streaming
  http.setTimeout(15000);
  bool begun = secure ? http.begin(tls, url) : http.begin(plain, url);
  if (!begun) return false;
  int code = http.GET();
  Serial.printf("%s GET -> %d\n", secure ? "HTTPS" : "HTTP", code);
  bool ok = (code == HTTP_CODE_OK) && parseWeather(http.getStream());
  http.end();
  return ok;
}

bool fetchWeather() {
  for (int attempt = 0; attempt < 2; attempt++) {
    if (fetchWeatherVia(true))  return true;   // HTTPS
    if (fetchWeatherVia(false)) return true;   // ripiego HTTP
    delay(2000);
  }
  return false;
}

// ============================ Stampa ========================================
// Riassunto di una fascia oraria [h0, h1) del giorno d (0 = oggi), dai dati orari
SlotWx slotWx(int d, int h0, int h1) {
  SlotWx r = {false, 0, 0, 999, -999, 0, 0, 0};
  int cnt[100] = {0};
  int worstWet = -1;                         // codice di precipitazione più intenso
  for (int h = h0; h < h1; h++) {
    int i = d * 24 + h;
    if (i < 0 || i >= nHours) continue;
    r.ok = true;
    int c = hCode[i];
    if (c >= 0 && c < 100) cnt[c]++;
    if (c >= 51 && c > worstWet) worstWet = c;
    if (hProb[i] > r.pmax) r.pmax = hProb[i];
    if (hTemp[i] < r.tmin) r.tmin = hTemp[i];
    if (hTemp[i] > r.tmax) r.tmax = hTemp[i];
    if (hWind[i] > r.wind) r.wind = hWind[i];
    r.rain += hRain[i];
    r.snow += hSnow[i];
  }
  // Se in quella fascia è prevista precipitazione significativa, la mostro;
  // altrimenti il "cielo" più frequente
  if (worstWet >= 0 && (r.pmax >= RAIN_HINT_PROB || r.rain >= 0.2)) {
    r.code = worstWet;
  } else {
    int best = 0;
    for (int c = 0; c < 51; c++) if (cnt[c] > cnt[best]) best = c;
    r.code = best;
  }
  return r;
}

// Sottotitolo: ─ TESTO ─────────
void subTitle(const char* utf8) {
  char tc[48];
  toCP437(utf8, tc, sizeof(tc));
  int n = strlen(tc);
  if (n > COLS - 3) n = COLS - 3;
  printer.boldOn();
  printer.write(0xC4); printer.write(' '); writeBytes(tc, n); printer.write(' ');
  for (int k = n + 3; k < COLS; k++) printer.write(0xC4);
  printer.write('\n');
  printer.boldOff();
}

// Titolo di sezione in negativo, centrato
void sectionTitle(const char* utf8) {
  printer.feed(1);
  printer.justify('C'); printer.inverseOn(); printer.boldOn();
  prf(" %s \n", utf8);
  printer.inverseOff(); printer.boldOff();
  printer.justify('L');
}

// Una fascia di oggi: 2 righe (+ vento se forte, + neve se c'è)
void printSlot(const char* name, int h0, int h1) {
  SlotWx w = slotWx(0, h0, h1);
  if (!w.ok) return;
  char t[48];
  snprintf(t, sizeof(t), "%s %d-%d", name, h0, h1);
  subTitle(t);
  if ((int)(w.tmin + 0.5f) == (int)(w.tmax + 0.5f)) prf(" %s, %.0f°\n", wxText(w.code), w.tmax);
  else                                              prf(" %s, %.0f/%.0f°\n", wxText(w.code), w.tmin, w.tmax);
  prf(" %s %d%% · %.1f mm\n", w.snow >= SNOW_CM_HOUR ? "Precip." : "Pioggia", w.pmax, w.rain);
  if (w.snow >= SNOW_CM_HOUR) { printer.boldOn(); prf(" Neve %.1f cm\n", w.snow); printer.boldOff(); }
  if (w.wind >= 20) prf(" Vento fino a %.0f km/h\n", w.wind);
}

// Barra oraria: 24 caratteri, uno per ora: ·░▒▓█ = probabilità pioggia, * = neve
void hourBar(int d, const char* label) {
  char lab[16];
  toCP437(label, lab, sizeof(lab));
  int n = strlen(lab);
  writeBytes(lab, n);
  for (int i = n; i < 6; i++) printer.write(' ');
  for (int h = 0; h < 24; h++) {
    int i = d * 24 + h;
    uint8_t ch;
    if (i < nHours && hSnow[i] >= SNOW_CM_HOUR) {
      ch = '*';
    } else {
      int p = (i < nHours) ? hProb[i] : 0;
      ch = p < 10 ? 0xFA : p < 30 ? 0xB0 : p < 60 ? 0xB1 : p < 80 ? 0xB2 : 0xDB;
    }
    printer.write(ch);
  }
  printer.write('\n');
}

const char* barLabel(int i) { return i == 0 ? "Oggi" : DAY_ABBR[days[i].wday]; }

// Intestazione: titolo, data e ora, tutto centrato
void printHeader(const struct tm& lt) {
  printer.justify('C');
  printer.setSize('M'); printer.inverseOn(); printer.boldOn();
  prf(" METEO %s \n", LOCATION_NAME);
  printer.inverseOff(); printer.boldOff(); printer.setSize('S');
  prf("%s %d %s %d\n", DAY_FULL[lt.tm_wday], lt.tm_mday, MONTHS[lt.tm_mon], lt.tm_year + 1900);
  prf("ore %02d:%02d\n", lt.tm_hour, lt.tm_min);
  hline(0xCD);
}

// Sezione 1: condizioni attuali (temperatura grande + tabella con cornice)
void printCurrent() {
  printer.justify('C');
  printer.boldOn(); pr("CONDIZIONI ATTUALI\n"); printer.boldOff();
  printer.setSize('L'); prf("%.1f°C\n", cur.temp); printer.setSize('S');
  prf("%s\n", wxText(cur.code));
  printer.justify('L');
  char v[32];
  printer.setLineHeight(24);   // righe attaccate: la cornice esce continua, senza tratteggio
  boxTop();
  snprintf(v, sizeof(v), "%.1f°C", cur.feels);                       boxRow("Percepita", v);
  snprintf(v, sizeof(v), "%d%%", cur.hum);                           boxRow("Umidità", v);
  snprintf(v, sizeof(v), "%.0f km/h da %s", cur.wind, windDirText(cur.windDir)); boxRow("Vento", v);
  snprintf(v, sizeof(v), "%.0f km/h", cur.gusts);                    boxRow("Raffiche", v);
  snprintf(v, sizeof(v), "%.0f mbar", cur.pressure);                 boxRow("Pressione", v);
  snprintf(v, sizeof(v), "%d%%", cur.clouds);                        boxRow("Nuvolosità", v);
  snprintf(v, sizeof(v), "%.1f mm", cur.precip);                     boxRow("Precipit.", v);
  if (cur.snow > 0) { snprintf(v, sizeof(v), "%.1f cm/h", cur.snow); boxRow("Neve", v); }
  boxRow("Alba", days[0].sunrise);
  boxRow("Tramonto", days[0].sunset);
  snprintf(v, sizeof(v), "%.0f (max oggi)", days[0].uv);             boxRow("Indice UV", v);
  boxBottom();
  printer.setLineHeight(30);   // interlinea normale
}

// Riga finale sulla batteria, con avviso in negativo quando serve ricaricare
void printBatteryFooter() {
  char vs[12];
  fmtVolt(vs, sizeof(vs), battV);
  printer.feed(1);
  printer.justify('C');
  if (battV < BATT_NOW) {
    printer.inverseOn(); printer.boldOn();
    pr(" RICARICARE ORA \n");
    printer.inverseOff(); printer.boldOff();
  } else if (battV < BATT_SOON) {
    printer.inverseOn(); printer.boldOn();
    pr(" RICARICARE PRESTO \n");
    printer.inverseOff(); printer.boldOff();
    pr("restano pochi giorni\n");
  }
  prf("Batteria %s V · ~%d%%\n", vs, battPercent(battV));
}

// Stampa ridotta (pulsante IO14): solo intestazione e condizioni attuali
void printCurrentOnly() {
  ensurePrinter();
  printer.wake();
  time_t now = time(nullptr);
  struct tm lt; localtime_r(&now, &lt);
  Serial.println("Stampa in corso: solo condizioni attuali");
  printHeader(lt);
  printCurrent();
  printBatteryFooter();
  printer.feed(3);
  Serial.println("Stampa completata.");
}

// Frasi di sintesi su neve e pioggia di un giorno, a partire da fromHour.
// wet = true se è attesa pioggia (> RAIN_HINT_PROB) o neve.
void rainSentence(const char* who, int d, int fromHour, bool& wet) {
  char s[200];
  int r1, r2, mp, s1, s2;
  float sn;
  bool rainWin = rainWindow(d, fromHour, r1, r2, mp);   // ore senza neve
  bool snowWin = snowWindow(d, fromHour, s1, s2, sn);
  wet = rainWin || mp > RAIN_HINT_PROB || snowWin;
  if (snowWin) {
    snprintf(s, sizeof(s), "%s NEVE tra le %d e le %d, circa %.1f cm.", who, s1, s2 + 1, sn);
    say(s);
  }
  if (rainWin)
    snprintf(s, sizeof(s), "%s pioggia probabile tra le %d e le %d (fino al %d%%): porta l'ombrello!", who, r1, r2 + 1, mp);
  else if (mp > RAIN_HINT_PROB)
    snprintf(s, sizeof(s), "%s possibile qualche goccia (max %d%%): ombrello per sicurezza.", who, mp);
  else if (!snowWin)
    snprintf(s, sizeof(s), "%s niente pioggia: ombrello a casa.", who);
  else
    return;
  say(s);
}

// Stampa completa: attuali, oggi a fasce, domani, grafico orario, sintesi, batteria
void printReport() {
  ensurePrinter();
  printer.wake();
  time_t now = time(nullptr);
  struct tm lt; localtime_r(&now, &lt);
  int hr = lt.tm_hour;
  char s[200];
  Serial.printf("Stampa in corso: completa (ore %02d)\n", hr);

  printHeader(lt);
  printCurrent();

  // ---------------- OGGI, a fasce ----------------
  snprintf(s, sizeof(s), "OGGI · %s %d/%d", DAY_FULL[days[0].wday], days[0].d, days[0].m);
  sectionTitle(s);
  if (hr < AFTERNOON_FROM) {                         // mattina: mattino + pomeriggio
    printSlot("Mattino", hr < MORNING_FROM ? MORNING_FROM : hr, AFTERNOON_FROM);
    printSlot("Pomeriggio", AFTERNOON_FROM, EVENING_FROM);
  } else if (hr < EVENING_FROM) {                    // dal pomeriggio: solo il resto del pomeriggio
    printSlot("Pomeriggio", hr, EVENING_FROM);
  } else {                                           // sera: quello che resta della giornata
    printSlot("Sera", hr, 24);
  }

  // ---------------- DOMANI ----------------
  snprintf(s, sizeof(s), "DOMANI · %s %d/%d", DAY_FULL[days[1].wday], days[1].d, days[1].m);
  sectionTitle(s);
  prf(" %s\n", wxText(days[1].code));
  prf(" Min %.0f° · Max %.0f°\n", days[1].tmin, days[1].tmax);
  prf(" %s %d%% · %.1f mm\n", isSnowDay(1) ? "Precip." : "Pioggia", days[1].pprob, days[1].psum);
  if (isSnowDay(1)) { printer.boldOn(); prf(" Neve %.1f cm\n", days[1].snow); printer.boldOff(); }
  prf(" Vento max %.0f km/h\n", days[1].windMax);
  SlotWx m = slotWx(1, MORNING_FROM, AFTERNOON_FROM);
  SlotWx p = slotWx(1, AFTERNOON_FROM, EVENING_FROM);
  if (m.ok) { snprintf(s, sizeof(s), "Mattino: %s, %d%%", wxText(m.code), m.pmax); prWrapIndent(s, 1); }
  if (p.ok) { snprintf(s, sizeof(s), "Pomeriggio: %s, %d%%", wxText(p.code), p.pmax); prWrapIndent(s, 1); }

  // ---------------- Pioggia (e neve) ora per ora ----------------
  bool snowChart = chartHasSnow();
  printer.feed(1);
  printer.boldOn();
  pr(snowChart ? "Pioggia e neve ora per ora\n" : "Probabilità pioggia ora per ora\n");
  printer.boldOff();
  for (int i = 0; i < CHART_DAYS; i++) hourBar(i, barLabel(i));
  pr("      0     6     12    18   23\n");
  pr("      ·<10 ░<30 ▒<60 ▓<80 █>80\n");
  if (snowChart) pr("      * = neve\n");

  // ---------------- SINTESI ----------------
  sectionTitle("IN SINTESI");
  int h1, h2;
  float sn;
  if (snowWindow(0, hr, h1, h2, sn) || snowWindow(1, 0, h1, h2, sn) || isSnowDay(1)) {
    printer.justify('C'); printer.inverseOn(); printer.boldOn();
    pr(" *** NEVE IN ARRIVO! *** \n");
    printer.inverseOff(); printer.boldOff(); printer.justify('L');
    Serial.println("  > *** NEVE IN ARRIVO! ***");
  }
  bool wetToday, wetTomorrow;
  rainSentence("OGGI", 0, hr, wetToday);
  rainSentence("DOMANI", 1, MORNING_FROM, wetTomorrow);
  wetTomorrow = wetTomorrow || days[1].pprob > RAIN_HINT_PROB;

  // Prossima pioggia oltre domani (probabilità giornaliera > 30%)
  int next = -1;
  for (int i = 2; i < FC_DAYS; i++) if (days[i].pprob > RAIN_HINT_PROB) { next = i; break; }
  const char* lead = (wetToday || wetTomorrow) ? "Poi" : "Prossima";
  if (next > 0)
    snprintf(s, sizeof(s), "%s %s tra %d giorni (%s %d/%d), probabilità %d%%.",
             lead, isSnowDay(next) ? "neve" : "pioggia", next,
             DAY_ABBR[days[next].wday], days[next].d, days[next].m, days[next].pprob);
  else
    snprintf(s, sizeof(s), "%s asciutto fino a %s %d/%d.",
             (wetToday || wetTomorrow) ? "Poi" : "Niente pioggia:",
             DAY_ABBR[days[FC_DAYS - 1].wday], days[FC_DAYS - 1].d, days[FC_DAYS - 1].m);
  say(s);

  printBatteryFooter();
  printer.feed(3);
  Serial.println("Stampa completata.");
}

// Ultimo avviso prima di fermarsi per batteria scarica (non serve il WiFi)
void printBatteryEmpty() {
  char vs[12];
  fmtVolt(vs, sizeof(vs), battV);
  Serial.printf("Stampa avviso batteria scarica (%s V)\n", vs);
  ensurePrinter();
  printer.justify('C');
  printer.setSize('M'); printer.inverseOn(); printer.boldOn();
  pr(" BATTERIA SCARICA \n");
  printer.inverseOff(); printer.boldOff(); printer.setSize('S');
  prf("Batteria %s V\n", vs);
  prWrap("Stampe sospese: ricaricare dalla presa USB. Riparte da solo a ricarica avvenuta.");
  printer.feed(3);
}

void printError(const char* msg) {
  Serial.printf("Stampa messaggio di errore: %s\n", msg);
  ensurePrinter();
  printer.wake();
  printer.justify('C'); printer.boldOn();
  prf("METEO %s\n", LOCATION_NAME);
  printer.boldOff();
  hline(0xC4);
  printer.justify('L');
  prWrap(msg);
  printBatteryFooter();
  printer.feed(3);
}

// ======================== Pianificazione e sonno ============================
time_t targetAt(const struct tm& base, int dayOff, int hour) {
  struct tm t = base;
  t.tm_mday += dayOff; t.tm_hour = hour; t.tm_min = 0; t.tm_sec = 0; t.tm_isdst = -1;
  return mktime(&t);
}

// Stampa automatica precedente e successiva (una sola al giorno, alle MORNING_HOUR)
void findTargets(time_t now, time_t& prevT, time_t& nextT) {
  struct tm lt; localtime_r(&now, &lt);
  prevT = 0; nextT = 0;
  for (int d = -1; d <= 1; d++) {
    time_t T = targetAt(lt, d, MORNING_HOUR);
    if (T <= now) { if (T > prevT) prevT = T; }
    else if (nextT == 0 || T < nextT) nextT = T;
  }
}

int32_t slotId(time_t T) {
  struct tm t; localtime_r(&T, &t);
  return t.tm_year * 400 + t.tm_yday;
}

void sleepFor(uint32_t secs) {
  const uint32_t cap = 6UL * 3600UL;   // si risveglia comunque ogni 6h per rimettere l'ora
  if (secs > cap) secs = cap;
  if (secs < 10) secs = 10;
  Serial.printf("Deep sleep per %u s (%uh %02um)\n", (unsigned)secs, (unsigned)(secs / 3600), (unsigned)((secs % 3600) / 60));
  if (time(nullptr) > 1700000000) logTime("Risveglio previsto verso: ", time(nullptr) + secs);
  Serial.flush();

  // Stampante spenta: relè aperto e linea dati bassa, "congelati" durante il sonno
  printerPowerOff();
  rtc_gpio_hold_en((gpio_num_t)PRINTER_TX_PIN);
  rtc_gpio_hold_en((gpio_num_t)RELAY_PIN);
  // LED flash sicuramente spento
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  rtc_gpio_hold_en((gpio_num_t)FLASH_LED_PIN);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup((uint64_t)secs * 1000000ULL);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);   // pulsante premuto = livello basso
  rtc_gpio_pullup_en((gpio_num_t)BUTTON_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)BUTTON_PIN);
  esp_deep_sleep_start();
}

// ================================ SETUP =====================================
void setup() {
  // Relè spento e linea dati bassa subito, poi rilascia i pin "congelati" nel sonno
  pinMode(RELAY_PIN, OUTPUT);      digitalWrite(RELAY_PIN, LOW);
  pinMode(PRINTER_TX_PIN, OUTPUT); digitalWrite(PRINTER_TX_PIN, LOW);
  pinMode(FLASH_LED_PIN, OUTPUT);  digitalWrite(FLASH_LED_PIN, LOW);
  rtc_gpio_hold_dis((gpio_num_t)RELAY_PIN);
  rtc_gpio_hold_dis((gpio_num_t)PRINTER_TX_PIN);
  rtc_gpio_hold_dis((gpio_num_t)FLASH_LED_PIN);

  Serial.begin(115200);
  delay(50);

  // Timer = risveglio programmato; pulsante IO14 = stampa ridotta;
  // accensione / ritorno corrente / tasto RST = stampa completa
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool buttonWake = (cause == ESP_SLEEP_WAKEUP_EXT0);
  bool userBoot   = (cause != ESP_SLEEP_WAKEUP_TIMER);
  const char* causeTxt =
    cause == ESP_SLEEP_WAKEUP_TIMER ? "timer (controllo programmato)" :
    cause == ESP_SLEEP_WAKEUP_EXT0  ? "pulsante IO14" :
                                      "accensione o tasto RST";
  Serial.printf("\n### Meteo %s - avvio: %s\n", LOCATION_NAME, causeTxt);

  // ---- Batteria: lettura prima del WiFi (ADC2) ----
  battV = readBattery();
  char vs[12]; fmtVolt(vs, sizeof(vs), battV);
  Serial.printf("Batteria: %s V (~%d%%)  [pin IO15: %.3f V, BATT_CAL %.3f]\n",
                vs, battPercent(battV), battPinV, BATT_CAL);

  if (battEmpty && battV >= BATT_RESUME) {
    Serial.println("Batteria ricaricata: riprendo il funzionamento normale");
    battEmpty = false;
  }
  if (battEmpty || battV < BATT_EMPTY) {
    // Avviso stampato una sola volta (o a ogni RST/pulsante); poi solo sonno lungo
    if (!battEmpty || userBoot) printBatteryEmpty();
    battEmpty = true;
    Serial.println("Batteria scarica: niente WiFi né stampe, ricontrollo tra 6 ore");
    sleepFor(6UL * 3600UL);
    return;
  }

  if (!(connectWiFi() && syncTime())) {
    if (userBoot) printError("Connessione WiFi o sincronizzazione orario non riuscita. Riprovo tra 10 minuti.");
    sleepFor(600);
    return;
  }

  time_t now = time(nullptr);
  logTime("Ora attuale: ", now);
  time_t prevT, nextT;
  findTargets(now, prevT, nextT);
  time_t dueT = 0;
  if (nextT - now <= 180) dueT = nextT;                                      // sveglia appena prima dell'orario
  else if (now - prevT <= 1800 && slotId(prevT) != lastSlot) dueT = prevT;   // in ritardo o tentativo fallito

  if (dueT) {
    Serial.printf("Decisione: stampa programmata delle %02d:00 (completa)\n", MORNING_HOUR);
    bool ok = fetchWeather();
    if (time(nullptr) < dueT) Serial.printf("Attendo l'orario esatto (%ld s)...\n", (long)(dueT - time(nullptr)));
    while (time(nullptr) < dueT) delay(250);
    if (ok) {
      printReport();
      lastSlot = slotId(dueT);
    } else {
      if (userBoot) printError("Impossibile scaricare i dati meteo. Riprovo tra 5 minuti.");
      sleepFor(300);
      return;
    }
  } else if (buttonWake) {
    Serial.println("Decisione: pulsante, stampa solo condizioni attuali");
    if (fetchWeather()) printCurrentOnly();
    else printError("Impossibile scaricare i dati meteo da Open-Meteo.");
  } else if (userBoot) {
    Serial.println("Decisione: accensione/RST, stampa completa");
    if (fetchWeather()) printReport();
    else printError("Impossibile scaricare i dati meteo da Open-Meteo.");
  } else {
    Serial.println("Decisione: nessuna stampa (risveglio intermedio per rimettere l'ora)");
  }

  now = time(nullptr);
  findTargets(now, prevT, nextT);
  logTime("Prossima stampa programmata: ", nextT);
  uint32_t s = (uint32_t)(nextT - now);
  // Si sveglia un po' prima (l'orologio del deep sleep sbaglia di qualche %)
  uint32_t sl = (s > 200) ? s - 120 - s / 25 : 30;
  sleepFor(sl);
}

void loop() {}
