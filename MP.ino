#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <Wire.h>
#include "RTClib.h"
#include <EEPROM.h>

// ---------- RFID pins (Mega 2560 SPI) ----------
#define SS_PIN     53   // RC522 SDA/SS
#define RST_PIN     9   // RC522 RST

// ---------- IO pins ----------
#define SERVO_PIN   6
#define BUZZER_PIN  7
#define BUTTON_PIN  4   // Momentary button: INPUT_PULLUP; LOW = pressed

// ---------- Objects ----------
MFRC522 mfrc522(SS_PIN, RST_PIN);
Servo doorServo;
RTC_DS3231 rtc;

// ---------- AUTHORIZED UID (4 bytes) ----------
byte authorizedUID[4] = {0xB0, 0x9E, 0x58, 0x37};

// ---------- DOOR STATE ----------
bool doorUnlocked = false;

// ---------- 5s Idle Countdown (pre-held rule only) ----------
bool countdownActive = false;
unsigned long countdownStart = 0;
const unsigned long unlockDuration = 5000UL; // 5 seconds
unsigned long lastBeep = 0;
int beepSecond = 0;

// ---------- Button debounce / edges ----------
const unsigned long DEBOUNCE_MS = 25;
int  lastStableBtn     = HIGH;  // debounced current
int  prevStableBtn     = HIGH;  // debounced previous (for edge detection)
unsigned long lastBtnChangeMs = 0;

// ---------- Pre-held marker (button held BEFORE RFID) ----------
bool buttonHeldBeforeRFID = false;

// ---------- EEPROM monthly logging ----------
#define LOG_ENTRY_SIZE     8
#define LOGS_PER_MONTH     20
#define MONTH_BLOCK_SIZE   (1 + 1 + (LOG_ENTRY_SIZE * LOGS_PER_MONTH))
#define EEPROM_MONTH_BASE  10  // Monthly log storage starts here

// ---------- Countdown mode ----------
enum CountdownMode : uint8_t { COUNTDOWN_NONE = 0, COUNTDOWN_PREHELD = 1 };
CountdownMode countdownMode = COUNTDOWN_NONE;

// ---------- Servo ramp settings ----------
const int SERVO_LOCK_POS    = 0;    // adjust to your lock angle
const int SERVO_UNLOCK_POS  = 90;   // adjust to your unlock angle
const int SERVO_STEP_DEG    = 2;    // 1–3 = smoother
const int SERVO_STEP_DELAY  = 10;   // ms per step (5–15 smooth)
#define SERVO_DETACH_IDLE   1       // 1=detach after move; 0=keep attached

// ===================== TIMEZONE / DST CONFIG =====================
// Store RTC as UTC and convert to local when displaying.
#define RTC_IS_UTC          1
// Prompt the user at boot to enter current local time (AM/PM). Zone prompt occurs only if not stored.
#define PROMPT_TIME_ON_BOOT 1

// ===================== TIMEZONE DEFINITIONS =====================
struct ZoneSpec {
  const char* name;       // Human name
  const char* abbrStd;    // e.g., "EST"
  const char* abbrDst;    // e.g., "EDT" (nullptr if no DST)
  int stdOffsetHours;     // Standard offset (UTC + this many hours). US zones are negative.
  bool observesDST;       // Whether zone observes US/Canada DST rule
};

// Current zone + index (index persisted in EEPROM)
ZoneSpec currentZone = { "Eastern", "EST", "EDT", -5, true };
int currentZoneIndex = 1; // 1..8 (see table below)

// ===================== EEPROM (Zone Persistence) =====================
// We reserve a few bytes at EEPROM addresses 0..3 for zone persistence.
// This doesn't collide with monthly logs that start at EEPROM_MONTH_BASE (10).
#define EE_TZ_SIG0   0
#define EE_TZ_SIG1   1
#define EE_TZ_VER    2
#define EE_TZ_INDEX  3
#define TZ_SIGNATURE0 'T'
#define TZ_SIGNATURE1 'Z'
#define TZ_VERSION    1

void saveZoneToEEPROM(int idx) {
  EEPROM.update(EE_TZ_SIG0, TZ_SIGNATURE0);
  EEPROM.update(EE_TZ_SIG1, TZ_SIGNATURE1);
  EEPROM.update(EE_TZ_VER,  TZ_VERSION);
  EEPROM.update(EE_TZ_INDEX, (uint8_t)idx);
}

bool loadZoneFromEEPROM() {
  if (EEPROM.read(EE_TZ_SIG0) == TZ_SIGNATURE0 &&
      EEPROM.read(EE_TZ_SIG1) == TZ_SIGNATURE1 &&
      EEPROM.read(EE_TZ_VER ) == TZ_VERSION) {
    int idx = EEPROM.read(EE_TZ_INDEX);
    return (idx >= 1 && idx <= 8);
  }
  return false;
}

int readZoneIndexFromEEPROM() {
  return EEPROM.read(EE_TZ_INDEX);
}

// ===================== Helpers (zone table + lookup) =====================
bool sContainsCI(const String &hay, const char* needle) {
  String n = String(needle); n.toLowerCase();
  String h = hay; h.toLowerCase();
  return h.indexOf(n) >= 0;
}

bool getZoneByIndex(int idx, ZoneSpec &out) {
  switch (idx) {
    case 1: out = {"Eastern",  "EST", "EDT", -5, true};  return true;
    case 2: out = {"Central",  "CST", "CDT", -6, true};  return true;
    case 3: out = {"Mountain", "MST", "MDT", -7, true};  return true;
    case 4: out = {"Mountain (no DST/AZ)", "MST", nullptr, -7, false}; return true;
    case 5: out = {"Pacific",  "PST", "PDT", -8, true};  return true;
    case 6: out = {"Alaska",   "AKST","AKDT",-9, true};  return true;
    case 7: out = {"Hawaii",   "HST", nullptr,-10, false}; return true;
    case 8: out = {"Atlantic", "AST", "ADT", -4, true};  return true;
    default: return false;
  }
}

bool getZoneByName(const String &txt, ZoneSpec &out, int &idxOut) {
  String s = txt; s.trim(); s.toLowerCase();
  if (s.length() == 0) return false;

  if (sContainsCI(s, "eastern") || sContainsCI(s, "est") || s == "et") {
    idxOut = 1; return getZoneByIndex(1, out);
  } else if (sContainsCI(s, "central") || sContainsCI(s, "cst") || s == "ct") {
    idxOut = 2; return getZoneByIndex(2, out);
  } else if (sContainsCI(s, "arizona") || sContainsCI(s, "no dst")) {
    idxOut = 4; return getZoneByIndex(4, out);
  } else if (sContainsCI(s, "mountain") || sContainsCI(s, "mst") || sContainsCI(s, "mdt") || s == "mt") {
    idxOut = 3; return getZoneByIndex(3, out);
  } else if (sContainsCI(s, "pacific") || sContainsCI(s, "pst") || sContainsCI(s, "pdt") || s == "pt") {
    idxOut = 5; return getZoneByIndex(5, out);
  } else if (sContainsCI(s, "alaska") || sContainsCI(s, "akst") || sContainsCI(s, "akdt") || s == "akt") {
    idxOut = 6; return getZoneByIndex(6, out);
  } else if (sContainsCI(s, "hawaii") || sContainsCI(s, "hst") || s == "ht") {
    idxOut = 7; return getZoneByIndex(7, out);
  } else if (sContainsCI(s, "atlantic") || sContainsCI(s, "ast") || sContainsCI(s, "adt") || s == "at") {
    idxOut = 8; return getZoneByIndex(8, out);
  }
  return false;
}

// ===================== DST COMPUTATION (US/Canada rule) =====================
// RTClib dayOfTheWeek(): 0=Sunday, 6=Saturday.
int nthSunday(int year, int month, int n) {
  DateTime firstOfMonth(year, month, 1, 0, 0, 0);
  int dowFirst = firstOfMonth.dayOfTheWeek(); // 0=Sunday
  int firstSunday = (dowFirst == 0) ? 1 : (8 - dowFirst);
  return firstSunday + 7 * (n - 1);
}
int firstSunday(int year, int month) { return nthSunday(year, month, 1); }

// For UTC -> local: evaluate boundaries using local STANDARD time in the selected zone.
bool isDST_US_fromUTC(DateTime utc, const ZoneSpec& z) {
  if (!z.observesDST) return false;
  DateTime localStd = utc + TimeSpan(z.stdOffsetHours * 3600); // standard time
  int y = localStd.year();

  int dstStartDay = nthSunday(y, 3, 2);  // 2nd Sunday in March, 02:00 local
  DateTime dstStartLocal(y, 3, dstStartDay, 2, 0, 0);

  int dstEndDay = firstSunday(y, 11);    // 1st Sunday in Nov, 02:00 local
  DateTime dstEndLocal(y, 11, dstEndDay, 2, 0, 0);

  return (localStd >= dstStartLocal) && (localStd < dstEndLocal);
}

// For local -> UTC conversion when user types local wall time in that zone.
bool isDST_US_fromLocalWall(DateTime localWall, const ZoneSpec& z) {
  if (!z.observesDST) return false;
  int y = localWall.year();
  int dstStartDay = nthSunday(y, 3, 2);
  DateTime dstStartLocal(y, 3, dstStartDay, 2, 0, 0);
  int dstEndDay = firstSunday(y, 11);
  DateTime dstEndLocal(y, 11, dstEndDay, 2, 0, 0);
  return (localWall >= dstStartLocal) && (localWall < dstEndLocal);
}

// ===================== TIME / LOG HELPERS =====================
DateTime getLocalTime() {
  DateTime now = rtc.now();
#if RTC_IS_UTC
  // RTC is UTC: apply selected zone (with DST if applicable)
  bool dst = isDST_US_fromUTC(now, currentZone);
  int offsetHours = currentZone.stdOffsetHours + (dst ? 1 : 0);
  return now + TimeSpan(offsetHours * 3600);
#else
  // If you store local time directly (not recommended)
  return now;
#endif
}

// Zero-padding helper for nicer prints
void print2(int v) { if (v < 10) Serial.print('0'); Serial.print(v); }

// Current zone abbreviation based on DST
const char* currentZoneAbbrev(bool dst) {
  if (dst && currentZone.abbrDst) return currentZone.abbrDst;
  return currentZone.abbrStd ? currentZone.abbrStd : "";
}

// Print with AM/PM + zone
void printTime(const char* label) {
  DateTime nowUTC = rtc.now();
  bool dst = isDST_US_fromUTC(nowUTC, currentZone);
  DateTime t = getLocalTime();

  int h24 = t.hour();
  int h12 = h24 % 12; if (h12 == 0) h12 = 12;
  const char* ampm = (h24 < 12) ? "AM" : "PM";

  if (label && label[0]) { Serial.print(label); Serial.print(" "); }
  Serial.print(t.year()); Serial.print("-");
  print2(t.month());  Serial.print("-");
  print2(t.day());    Serial.print(" ");
  Serial.print(h12);  Serial.print(":");
  print2(t.minute()); Serial.print(":");
  print2(t.second()); Serial.print(" ");
  Serial.print(ampm); Serial.print(" ");
  Serial.println(currentZoneAbbrev(dst));
}

int getMonthBlock(int month) {
  return EEPROM_MONTH_BASE + (month - 1) * MONTH_BLOCK_SIZE;
}

void clearMonthBlock(int month) {
  int base = getMonthBlock(month);
  EEPROM.update(base, month);
  EEPROM.update(base + 1, 0);
  for (int i = 0; i < LOG_ENTRY_SIZE * LOGS_PER_MONTH; i++) {
    EEPROM.update(base + 2 + i, 0);
  }
}

void ensureMonthInitialized(int month) {
  int base = getMonthBlock(month);
  if (EEPROM.read(base) != month) clearMonthBlock(month);
}

void logEvent(DateTime t, byte eventType) {
  int month = t.month();
  ensureMonthInitialized(month);

  int base = getMonthBlock(month);
  int count = EEPROM.read(base + 1);
  if (count >= LOGS_PER_MONTH) count = LOGS_PER_MONTH - 1;

  int addr = base + 2 + (count * LOG_ENTRY_SIZE);

  uint16_t year = t.year();
  EEPROM.update(addr + 0, (year >> 8) & 0xFF);
  EEPROM.update(addr + 1, year & 0xFF);
  EEPROM.update(addr + 2, t.month());
  EEPROM.update(addr + 3, t.day());
  EEPROM.update(addr + 4, t.hour());   // stored as 24h local in selected zone
  EEPROM.update(addr + 5, t.minute());
  EEPROM.update(addr + 6, t.second());
  EEPROM.update(addr + 7, eventType);  // 1=UNLOCK, 2=LOCK, 3=DENIED

  EEPROM.update(base + 1, count + 1);
  Serial.println("Event logged.");
}

// ===================== SERIAL INPUT HELPERS =====================
String readLine() {
  String s = "";
  while (true) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue; // ignore CR
      if (c == '\n') break;
      s += c;
    }
  }
  return s;
}

bool parseIntSafe(const String &s, int &out) {
  String t = s; t.trim();
  if (t.length() == 0) return false;
  bool neg = (t.charAt(0) == '-');
  int start = (neg || t.charAt(0) == '+') ? 1 : 0;
  for (int i = start; i < t.length(); ++i) {
    if (!isDigit(t.charAt(i))) return false;
  }
  out = t.toInt();
  return true;
}

// Parses "YYYY-MM-DD hh:mm:ss AM/PM" into DateTime (local wall time)
bool parseLocal_AMPM(const String& line, DateTime& outLocal) {
  // Expected format: YYYY-MM-DD hh:mm:ss AM/PM
  int sp1 = line.indexOf(' ');
  if (sp1 < 0) return false;
  int sp2 = line.indexOf(' ', sp1 + 1);
  if (sp2 < 0) return false;

  String date = line.substring(0, sp1);
  String time = line.substring(sp1 + 1, sp2);
  String ampm = line.substring(sp2 + 1);

  // Normalize AM/PM
  ampm.trim();
  ampm.toUpperCase();
  if (!(ampm == "AM" || ampm == "PM")) return false;

  // Parse date YYYY-MM-DD
  int d1 = date.indexOf('-');
  int d2 = date.indexOf('-', d1 + 1);
  if (d1 < 0 || d2 < 0) return false;
  int year  = date.substring(0, d1).toInt();
  int month = date.substring(d1 + 1, d2).toInt();
  int day   = date.substring(d2 + 1).toInt();

  // Parse time hh:mm:ss
  int c1 = time.indexOf(':');
  int c2 = time.indexOf(':', c1 + 1);
  if (c1 < 0 || c2 < 0) return false;
  int hh = time.substring(0, c1).toInt();
  int mm = time.substring(c1 + 1, c2).toInt();
  int ss = time.substring(c2 + 1).toInt();

  // Basic validation
  if (year < 2000 || month < 1 || month > 12 || day < 1 || day > 31) return false;
  if (hh < 1 || hh > 12 || mm < 0 || mm > 59 || ss < 0 || ss > 59) return false;

  // 12h -> 24h
  if (ampm == "AM") {
    if (hh == 12) hh = 0;
  } else { // PM
    if (hh != 12) hh += 12;
  }

  outLocal = DateTime(year, month, day, hh, mm, ss);
  return true;
}

// ===================== PROMPTS: TIME ZONE + TIME =====================
void promptSelectZone() {
  Serial.println();
  Serial.println("=== Select Your Time Zone ===");
  Serial.println("Type a number or a name (e.g., 'Eastern', 'Arizona', 'Pacific'):");
  Serial.println("  1) Eastern (EST/EDT)     [DST: yes]");
  Serial.println("  2) Central (CST/CDT)     [DST: yes]");
  Serial.println("  3) Mountain (MST/MDT)    [DST: yes]");
  Serial.println("  4) Mountain (no DST/AZ)  [DST: no]");
  Serial.println("  5) Pacific (PST/PDT)     [DST: yes]");
  Serial.println("  6) Alaska (AKST/AKDT)    [DST: yes]");
  Serial.println("  7) Hawaii (HST)          [DST: no]");
  Serial.println("  8) Atlantic (AST/ADT)    [DST: yes]");
  Serial.print("Your choice: ");

  while (!Serial.available()) { /* wait */ }
  String choice = readLine(); choice.trim();

  ZoneSpec z;
  int idx;
  if (parseIntSafe(choice, idx) && getZoneByIndex(idx, z)) {
    currentZone = z;
    currentZoneIndex = idx;
  } else if (getZoneByName(choice, z, idx)) {
    currentZone = z;
    currentZoneIndex = idx;
  } else {
    Serial.println("Unrecognized input. Defaulting to Eastern.");
    getZoneByIndex(1, currentZone);
    currentZoneIndex = 1;
  }

  saveZoneToEEPROM(currentZoneIndex);

  Serial.print("Selected zone: ");
  Serial.print(currentZone.name);
  Serial.print(" (");
  Serial.print(currentZone.abbrStd ? currentZone.abbrStd : "");
  if (currentZone.observesDST && currentZone.abbrDst) {
    Serial.print("/");
    Serial.print(currentZone.abbrDst);
  }
  Serial.println(")");
}

void promptAndSetTimeFromUser() {
  Serial.println();
  Serial.println("=== Set Current Local Time (AM/PM) ===");
  Serial.println("Enter current local time for your selected zone in the format:");
  Serial.println("  YYYY-MM-DD hh:mm:ss AM/PM");
  Serial.println("Example:");
  Serial.println("  2026-03-12 09:45:30 AM");
  Serial.print("Now enter time: ");

  while (!Serial.available()) { /* wait */ }
  String line = readLine();
  line.trim();

  DateTime localWall(2000,1,1,0,0,0);
  if (!parseLocal_AMPM(line, localWall)) {
    Serial.println("Time parse failed. Please reset and try again.");
    return;
  }

  bool dst = isDST_US_fromLocalWall(localWall, currentZone);

#if RTC_IS_UTC
  // Convert Local -> UTC:
  // local = UTC + offset  =>  UTC = local - offset
  int offsetHoursLocal = currentZone.stdOffsetHours + (dst ? 1 : 0);
  DateTime utc = localWall - TimeSpan(offsetHoursLocal * 3600);
  rtc.adjust(utc);
  Serial.print("RTC set to UTC based on local input. DST=");
  Serial.println(dst ? "ON" : "OFF");
#else
  rtc.adjust(localWall);
  Serial.print("RTC set to local time. DST=");
  Serial.println(dst ? "ON" : "OFF");
#endif

  Serial.print("Local time set: ");
  printTime("");
  Serial.println("Time set complete.");
}

// ===================== Button (debounced) =====================
int readButtonDebounced() {
  int raw = digitalRead(BUTTON_PIN);
  if (raw != lastStableBtn) {
    unsigned long now = millis();
    if (now - lastBtnChangeMs >= DEBOUNCE_MS) {
      lastStableBtn = raw;
      lastBtnChangeMs = now;
    }
  }
  return lastStableBtn; // LOW = pressed
}

// ===================== Buzzer helpers =====================
void beepOnce() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(80);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepSuccess() {
  for (int i = 0; i < 40; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delayMicroseconds(400);
    digitalWrite(BUZZER_PIN, LOW);  delayMicroseconds(600);
  }
}

void beepDenied() {
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < 40; i++) {
      digitalWrite(BUZZER_PIN, HIGH); delayMicroseconds(400);
      digitalWrite(BUZZER_PIN, LOW);  delayMicroseconds(600);
    }
    delay(120);
  }
}

// ===================== RFID AUTH =====================
bool isAuthorized(byte *uid, byte uidSize) {
  if (uidSize != 4) return false;
  for (byte i = 0; i < 4; i++)
    if (uid[i] != authorizedUID[i]) return false;
  return true;
}

// ===================== Servo (ramped) =====================
void servoEnsureAttached() {
#if SERVO_DETACH_IDLE
  if (!doorServo.attached()) doorServo.attach(SERVO_PIN);
#endif
}
void servoDetachIfIdle() {
#if SERVO_DETACH_IDLE
  if (doorServo.attached()) doorServo.detach();
#endif
}

void servoGoTo(int targetDeg) {
  servoEnsureAttached();
  targetDeg = constrain(targetDeg, 0, 180);
  int current = doorServo.read(); // approximate; ok for ramping
  int step = (targetDeg >= current) ? SERVO_STEP_DEG : -SERVO_STEP_DEG;
  while (current != targetDeg) {
    current += step;
    if ((step > 0 && current > targetDeg) || (step < 0 && current < targetDeg))
      current = targetDeg;
    doorServo.write(current);
    delay(SERVO_STEP_DELAY);
  }
  servoDetachIfIdle();
}

// ===================== Countdown control =====================
void startIdleCountdown(CountdownMode mode) {
  countdownMode   = mode;
  countdownActive = true;
  countdownStart  = millis();
  lastBeep        = millis();
  beepSecond      = 0;
  if (mode == COUNTDOWN_PREHELD)
    Serial.println("Idle countdown (PRE-HELD) started: 5s to auto-lock.");
}

void cancelIdleCountdown(const char* reason) {
  if (countdownActive) {
    countdownActive = false;
    countdownMode = COUNTDOWN_NONE;
    Serial.print("Idle countdown cancelled: ");
    Serial.println(reason);
  }
}

// ===================== Door actions =====================
void unlockDoor(bool startPreHeldCountdown) {
  DateTime t = getLocalTime();
  printTime("UNLOCK at");
  doorUnlocked = true;

  // Smooth unlock
  servoGoTo(SERVO_UNLOCK_POS);

  if (startPreHeldCountdown) {
    startIdleCountdown(COUNTDOWN_PREHELD);
  } else {
    countdownActive = false;
    countdownMode = COUNTDOWN_NONE;
  }

  buttonHeldBeforeRFID = false;
  logEvent(t, 1); // 1 = UNLOCK
}

void lockDoor() {
  DateTime t = getLocalTime();
  printTime("LOCK at");

  doorUnlocked = false;
  servoGoTo(SERVO_LOCK_POS);

  countdownActive = false;
  countdownMode = COUNTDOWN_NONE;
  buttonHeldBeforeRFID = false;
  logEvent(t, 2); // 2 = LOCK
}

// ===================== Button logic (edges) =====================
// Rules:
// - During countdown: releasing the button cancels the countdown immediately.
// - If unlocked and pressed: lock immediately.
// - Pre-held rule: holding the button before RFID scan triggers 5s unlock window.
void handleButtonLogic() {
  // Debounce + edge detect
  int btn = readButtonDebounced();         // LOW = pressed, HIGH = released
  bool fell = (prevStableBtn == HIGH && btn == LOW);  // press edge
  bool rose = (prevStableBtn == LOW  && btn == HIGH); // release edge
  prevStableBtn = btn;

  // Pre-held marking (while locked)
  if (!doorUnlocked && btn == LOW) {
    buttonHeldBeforeRFID = true;
  }

  // If the button is RELEASED during countdown → stop the countdown now
  if (rose && countdownActive) {
    cancelIdleCountdown("button released during countdown");
    return;
  }

  // If door is unlocked and button is PRESSED → immediate lock
  if (fell && doorUnlocked) {
    if (countdownActive) cancelIdleCountdown("button pressed -> immediate lock");
    Serial.println("Button pressed -> Immediate LOCK.");
    lockDoor();
  }
}

// ============================ SETUP ============================
void setup() {
  Serial.begin(9600);
  while (!Serial) { /* wait for USB serial (on native USB boards) */ }

  Serial.println("RFID Door Lock — Timezone + AM/PM — Release cancels countdown; press locks");

  // RTC init
  if (!rtc.begin()) {
    Serial.println("RTC NOT FOUND!");
    while (1) { delay(100); }
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power; time needs to be set.");
  }

  // Load persisted zone if present; otherwise prompt and store
  if (loadZoneFromEEPROM()) {
    int idx = readZoneIndexFromEEPROM();
    ZoneSpec z;
    if (getZoneByIndex(idx, z)) {
      currentZone = z;
      currentZoneIndex = idx;
      Serial.print("Restored zone from EEPROM: ");
      Serial.println(currentZone.name);
    } else {
      Serial.println("EEPROM zone index invalid; prompting for zone.");
      promptSelectZone();
    }
  } else {
    promptSelectZone();
  }

  // Prompt the user to set time at boot if requested (or if RTC lost power)
  if (PROMPT_TIME_ON_BOOT || rtc.lostPower()) {
    promptAndSetTimeFromUser();
  }

  // Keep Mega in SPI master mode
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);

  // RFID init
  SPI.begin();
  mfrc522.PCD_Init();

  // IO
#if !SERVO_DETACH_IDLE
  doorServo.attach(SERVO_PIN);
#endif
  // Start locked
  servoEnsureAttached();
  doorServo.write(SERVO_LOCK_POS);
  servoDetachIfIdle();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastStableBtn = digitalRead(BUTTON_PIN);
  prevStableBtn = lastStableBtn;
  lastBtnChangeMs = millis();

  Serial.println("System Ready.");
  Serial.print("Current local: ");
  printTime("");
}

// ============================= LOOP ============================
void loop() {
  handleButtonLogic();

  // ---------- 5s Countdown ----------
  if (countdownActive) {
    // Tick only when countdown is active (release cancels it above)
    if (millis() - lastBeep >= 1000UL) {
      lastBeep = millis();
      beepOnce();
      beepSecond++;
      int remain = (int)(unlockDuration / 1000UL) - beepSecond;
      if (remain < 0) remain = 0;
      Serial.print("Countdown: "); Serial.println(remain);
    }
    if (millis() - countdownStart >= unlockDuration) {
      Serial.println("Idle timeout — auto-locking.");
      lockDoor();
    }
  }

  // ---------- RFID check ----------
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    printTime("Card scanned at");

    // Special rule: pre-held if button was held before AND is still held now
    int btnNow = readButtonDebounced();
    bool preHeldNow = (buttonHeldBeforeRFID && (btnNow == LOW));

    if (isAuthorized(mfrc522.uid.uidByte, mfrc522.uid.size)) {
      Serial.println("ACCESS GRANTED");
      beepSuccess();
      unlockDoor(preHeldNow);
    } else {
      Serial.println("ACCESS DENIED");
      beepDenied();
      DateTime t = getLocalTime();
      logEvent(t, 3); // 3 = DENIED
      // remain locked; keep buttonHeldBeforeRFID as-is for a future good scan
    }

    // end session cleanly
  
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  }
}