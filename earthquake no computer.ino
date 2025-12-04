#include <Arduino.h>
#include "driver/ledc.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

// =====================================================
// OLED
// =====================================================
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// =====================================================
// BUTTONS
// =====================================================
#define BTN_UP      25
#define BTN_DOWN    26
#define BTN_ENTER   33
#define BTN_BACK    32

// =====================================================
// MOTOR + ENCODER
// =====================================================
#define PWM_PIN      18
#define PHOTO_PIN    34

#define PWM_CHANNEL  LEDC_CHANNEL_0
#define PWM_TIMER    LEDC_TIMER_0
#define PWM_MODE     LEDC_HIGH_SPEED_MODE
#define PWM_RES      LEDC_TIMER_8_BIT
#define PWM_FREQ     5000

// REAL encoder value
#define PULSES_PER_REV 30   // 30 divisions per revolution

volatile unsigned long pulseCount = 0;

float targetHz   = 0.0;
float measuredHz = 0.0;
int   duty       = 0;

unsigned long lastMeasure = 0;
unsigned long lastUpdate  = 0;  // for optional serial failsafe

// --- Tuning parameters (original logic) ---
const float KP               = 20.0;
const int   MIN_DUTY         = 15;
const int   MAX_DUTY         = 100;
const float LOW_SPEED_BOOST  = 8.0;
const float FILTER_ALPHA     = 0.6;

// =====================================================
// MENU / STATE
// =====================================================
enum ModeState {
  MENU_MAIN,

  MENU_SET_MENU,
  MENU_SET_EDIT,

  MENU_RAMP_MENU,
  MENU_RAMP_EDIT,

  MENU_LIVE_MENU,
  MENU_LIVE_EDIT,

  RUN_SET,
  RUN_RAMP,
  RUN_LIVE,

  RUN_SET_STOPPED,
  RUN_RAMP_STOPPED,
  RUN_LIVE_STOPPED,

  RESTART_SET_CONFIRM,
  RESTART_RAMP_CONFIRM,
  RESTART_LIVE_CONFIRM
};

ModeState state = MENU_MAIN;

// --------- Settings (stored in flash) ----------
const float   DEFAULT_SET_FREQ    = 3.3f;
const float   DEFAULT_LIVE_FREQ   = 1.0f;
const float   DEFAULT_RAMP_START  = 0.5f;
const float   DEFAULT_RAMP_END    = 3.3f;
const float   DEFAULT_RAMP_RATE   = 0.1f;   // Hz/s

float   setFreq    = DEFAULT_SET_FREQ;     // Set Frequency (pre-set mode)
float   liveFreq   = DEFAULT_LIVE_FREQ;    // Live Frequency (adjust while running)
float   rampStartF = DEFAULT_RAMP_START;   // Ramp start frequency
float   rampEndF   = DEFAULT_RAMP_END;     // Ramp end frequency
float   rampRate   = DEFAULT_RAMP_RATE;    // Hz/s

const float MIN_HZ = 0.5f;
const float MAX_HZ = 3.3f;

// menu navigation
int menuIndex       = 0;   // used for MAIN / SET / LIVE menus
int rampEditIndex   = 0;   // 0=start, 1=end, 2=rate, 3=run
unsigned long lastButtonTime = 0;
const unsigned long BTN_DEBOUNCE = 180; // ms

// run timing / stop snapshot
unsigned long runStartMillis  = 0;
unsigned long rampStartMillis = 0;

float stoppedFreq = 0.0f;
float stoppedTime = 0.0f;

// flash
Preferences prefs;

// =====================================================
// HELPERS
// =====================================================
void IRAM_ATTR pulseISR() { pulseCount++; }

void setDuty(int dutyPercent) {
  if (dutyPercent < 0) dutyPercent = 0;
  if (dutyPercent > 100) dutyPercent = 100;
  uint32_t dutyValue = (255 * dutyPercent) / 100;
  ledc_set_duty(PWM_MODE, PWM_CHANNEL, dutyValue);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL);
  duty = dutyPercent;
}

bool btnPressed(int pin) {
  return digitalRead(pin) == LOW;
}

bool btnClick(int pin) {
  if (!btnPressed(pin)) return false;
  unsigned long now = millis();
  if (now - lastButtonTime < BTN_DEBOUNCE) return false;
  lastButtonTime = now;
  return true;
}

float clampFreq(float f) {
  if (f < MIN_HZ) f = MIN_HZ;
  if (f > MAX_HZ) f = MAX_HZ;
  return f;
}

void centerText(const String &s, int y) {
  display.setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((OLED_WIDTH - w) / 2, y);
  display.print(s);
}

// =====================================================
// FLASH (NVS) STORAGE
// =====================================================
void saveSettings() {
  prefs.putFloat("setFreq",   setFreq);
  prefs.putFloat("liveFreq",  liveFreq);
  prefs.putFloat("rStart",    rampStartF);
  prefs.putFloat("rEnd",      rampEndF);
  prefs.putFloat("rRate",     rampRate);
}

void loadSettings() {
  setFreq    = prefs.getFloat("setFreq",   DEFAULT_SET_FREQ);
  liveFreq   = prefs.getFloat("liveFreq",  DEFAULT_LIVE_FREQ);
  rampStartF = prefs.getFloat("rStart",    DEFAULT_RAMP_START);
  rampEndF   = prefs.getFloat("rEnd",      DEFAULT_RAMP_END);
  rampRate   = prefs.getFloat("rRate",     DEFAULT_RAMP_RATE);

  // sanity clamp
  setFreq    = clampFreq(setFreq);
  liveFreq   = clampFreq(liveFreq);
  rampStartF = clampFreq(rampStartF);
  rampEndF   = clampFreq(rampEndF);
  if (rampRate < 0.05f) rampRate = 0.05f;
  if (rampRate > 1.0f)  rampRate = 1.0f;
}

// =====================================================
// MOTOR CONTROL (30 PPR, original logic)
// =====================================================
void updateMotorControl() {
  unsigned long now = millis();

  // Closed-loop measurement every 500 ms
  if (now - lastMeasure >= 500) {
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    // 0.5 s window, 30 pulses/rev
    float rawHz = (count / (float)PULSES_PER_REV) / 0.5f;
    measuredHz  = FILTER_ALPHA * measuredHz + (1.0f - FILTER_ALPHA) * rawHz;
    lastMeasure = now;

    // Control logic
    if (targetHz > 0.0f) {
      float error = targetHz - measuredHz;
      float gain  = KP;
      if (targetHz < 0.2f) gain += LOW_SPEED_BOOST;

      int newDuty = duty + error * gain;
      if (newDuty < MIN_DUTY)    newDuty = MIN_DUTY;
      if (targetHz < 0.05f)      newDuty = MIN_DUTY + 3;
      if (newDuty > MAX_DUTY)    newDuty = MAX_DUTY;
      setDuty(newDuty);
    } else {
      setDuty(0);
    }

    // Optional serial monitor for debugging
    Serial.print("M:");
    Serial.println(measuredHz, 3);
  }

  // Optional serial failsafe if you ever use PC control again
  if (lastUpdate != 0 && now - lastUpdate > 3000) {
    targetHz = 0;
    setDuty(0);
  }
}

// =====================================================
// OLED SCREENS (ALL SIZE 2, NO MENU TITLES, 1dp)
// =====================================================

void showMainMenu() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  // tight spacing: 0, 22, 44
  display.setCursor(0, 0);
  display.print(menuIndex == 0 ? "> " : "  ");
  display.print("Set");

  display.setCursor(0, 22);
  display.print(menuIndex == 1 ? "> " : "  ");
  display.print("Live");

  display.setCursor(0, 44);
  display.print(menuIndex == 2 ? "> " : "  ");
  display.print("Ramp");

  display.display();
}

void showSetMenuScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  // 2 options: Freq, Run
  display.setCursor(0, 0);
  display.print(menuIndex == 0 ? "> " : "  ");
  display.print("F:");
  display.print(setFreq, 1);

  display.setCursor(0, 24);
  display.print(menuIndex == 1 ? "> " : "  ");
  display.print("Run");

  display.display();
}

void showSetEditScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  centerText(String(setFreq, 1) + "Hz", 24);

  display.display();
}

void showLiveMenuScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  // 2 options: Target, Run
  display.setCursor(0, 0);
  display.print(menuIndex == 0 ? "> " : "  ");
  display.print("T:");
  display.print(liveFreq, 1);

  display.setCursor(0, 24);
  display.print(menuIndex == 1 ? "> " : "  ");
  display.print("Run");

  display.display();
}

void showLiveEditScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  centerText(String(liveFreq, 1) + "Hz", 24);

  display.display();
}

void showRampMenu() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  // Layout: 0, 16, 32, 48
  display.setCursor(0, 0);
  display.print(rampEditIndex == 0 ? "> " : "  ");
  display.print("St:");
  display.print(rampStartF, 1);

  display.setCursor(0, 16);
  display.print(rampEditIndex == 1 ? "> " : "  ");
  display.print("En:");
  display.print(rampEndF, 1);

  display.setCursor(0, 32);
  display.print(rampEditIndex == 2 ? "> " : "  ");
  display.print("Rt:");
  display.print(rampRate, 1);

  display.setCursor(0, 48);
  display.print(rampEditIndex == 3 ? "> Run" : "  Run");

  display.display();
}

void showRampEditScreen(const char *label, float value, const char *unit) {
  (void)label; // label intentionally unused (screen kept clean)

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  centerText(String(value, 1) + unit, 24);

  display.display();
}

// RUN screens: SET / RAMP → only actual frequency and time
void showRunSetScreen() {
  float elapsed = (millis() - runStartMillis) / 1000.0f;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  display.setCursor(0, 8);
  display.print("f:");
  display.print(measuredHz, 1);

  display.setCursor(0, 40);
  display.print("t:");
  display.print(elapsed, 1);
  display.print("s");

  display.display();
}

void showRunRampScreen() {
  float elapsed = (millis() - runStartMillis) / 1000.0f;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  display.setCursor(0, 8);
  display.print("f:");
  display.print(measuredHz, 1);

  display.setCursor(0, 40);
  display.print("t:");
  display.print(elapsed, 1);
  display.print("s");

  display.display();
}

// LIVE run screen: actual f, time, and target
void showRunLiveScreen() {
  float elapsed = (millis() - runStartMillis) / 1000.0f;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  display.setCursor(0, 0);
  display.print("f:");
  display.print(measuredHz, 1);

  display.setCursor(0, 22);
  display.print("t:");
  display.print(elapsed, 1);
  display.print("s");

  display.setCursor(0, 44);
  display.print("T:");
  display.print(liveFreq, 1);

  display.display();
}

// STOPPED screens: show frozen frequency & time only
void showStoppedSetScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  display.setCursor(0, 8);
  display.print("f:");
  display.print(stoppedFreq, 1);

  display.setCursor(0, 40);
  display.print("t:");
  display.print(stoppedTime, 1);
  display.print("s");

  display.display();
}

void showStoppedRampScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  display.setCursor(0, 8);
  display.print("f:");
  display.print(stoppedFreq, 1);

  display.setCursor(0, 40);
  display.print("t:");
  display.print(stoppedTime, 1);
  display.print("s");

  display.display();
}

void showStoppedLiveScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  display.setCursor(0, 8);
  display.print("f:");
  display.print(stoppedFreq, 1);

  display.setCursor(0, 40);
  display.print("t:");
  display.print(stoppedTime, 1);
  display.print("s");

  display.display();
}

// Restart confirm screens: short text
void showRestartSetConfirm() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  centerText("Restart?", 0);
  centerText("Ent=Yes", 24);
  centerText("Back=Menu", 48);

  display.display();
}

void showRestartRampConfirm() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  centerText("Restart?", 0);
  centerText("Ent=Yes", 24);
  centerText("Back=Menu", 48);

  display.display();
}

void showRestartLiveConfirm() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  centerText("Restart?", 0);
  centerText("Ent=Yes", 24);
  centerText("Back=Menu", 48);

  display.display();
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(9600);

  // OLED
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  centerText("SHAKER", 8);
  centerText("CTRL",   32);
  display.display();
  delay(1000);

  // Buttons
  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_BACK,  INPUT_PULLUP);

  // PWM
  ledc_timer_config_t timerConf = {
    .speed_mode = PWM_MODE,
    .duty_resolution = PWM_RES,
    .timer_num = PWM_TIMER,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timerConf);

  ledc_channel_config_t channelMotor = {
    .gpio_num = PWM_PIN,
    .speed_mode = PWM_MODE,
    .channel = PWM_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = PWM_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channelMotor);

  // Encoder
  pinMode(PHOTO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), pulseISR, FALLING);

  setDuty(0);

  // Flash
  prefs.begin("shaker", false);
  loadSettings();

  menuIndex = 0;
  state = MENU_MAIN;
  showMainMenu();
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  // 1) Motor control (encoder + PID)
  updateMotorControl();

  // 2) Menu & Buttons / Screens
  switch (state) {

    // -----------------------------
    // MAIN MENU: Set, Live, Ramp
    // -----------------------------
    case MENU_MAIN:
      showMainMenu();

      if (btnClick(BTN_UP)) {
        menuIndex = (menuIndex + 2) % 3;  // up cycles backwards
      }
      if (btnClick(BTN_DOWN)) {
        menuIndex = (menuIndex + 1) % 3;
      }

      if (btnClick(BTN_ENTER)) {
        if (menuIndex == 0) {
          // Set menu
          menuIndex = 0;
          state = MENU_SET_MENU;
          showSetMenuScreen();
        } else if (menuIndex == 1) {
          // Live menu
          menuIndex = 0;
          state = MENU_LIVE_MENU;
          showLiveMenuScreen();
        } else { // menuIndex == 2 => Ramp
          rampEditIndex = 0;
          state         = MENU_RAMP_MENU;
          showRampMenu();
        }
      }
      break;

    // -----------------------------
    // SET MENU (Freq + Run)
// -----------------------------
    case MENU_SET_MENU:
      showSetMenuScreen();

      if (btnClick(BTN_UP)) {
        menuIndex = (menuIndex + 1) % 2;  // toggle 0/1
      }
      if (btnClick(BTN_DOWN)) {
        menuIndex = (menuIndex + 1) % 2;
      }

      if (btnClick(BTN_BACK)) {
        state = MENU_MAIN;
        menuIndex = 0;
        showMainMenu();
      }

      if (btnClick(BTN_ENTER)) {
        if (menuIndex == 0) {
          // Edit frequency
          state = MENU_SET_EDIT;
          showSetEditScreen();
        } else {
          // Run set mode
          setFreq        = clampFreq(setFreq);
          targetHz       = setFreq;
          runStartMillis = millis();
          lastUpdate     = 0;
          state          = RUN_SET;
        }
      }
      break;

    // -----------------------------
    // SET FREQUENCY EDIT SCREEN
    // -----------------------------
    case MENU_SET_EDIT:
      showSetEditScreen();

      if (btnPressed(BTN_UP)) {
        setFreq += 0.1f;
        setFreq = clampFreq(setFreq);
        saveSettings();
        showSetEditScreen();
        delay(120);
      }
      if (btnPressed(BTN_DOWN)) {
        setFreq -= 0.1f;
        setFreq = clampFreq(setFreq);
        saveSettings();
        showSetEditScreen();
        delay(120);
      }

      if (btnClick(BTN_BACK) || btnClick(BTN_ENTER)) {
        // Return to Set menu
        menuIndex = 0;
        state = MENU_SET_MENU;
        showSetMenuScreen();
      }
      break;

    // -----------------------------
    // RAMP MENU
    // -----------------------------
    case MENU_RAMP_MENU:
      showRampMenu();

      if (btnClick(BTN_UP)) {
        rampEditIndex = (rampEditIndex + 3) % 4; // 0..3
      }
      if (btnClick(BTN_DOWN)) {
        rampEditIndex = (rampEditIndex + 1) % 4;
      }

      if (btnClick(BTN_BACK)) {
        state = MENU_MAIN;
        menuIndex = 0;
        showMainMenu();
      }

      if (btnClick(BTN_ENTER)) {
        // 0=start, 1=end, 2=rate, 3=run
        if (rampEditIndex == 0) {
          state = MENU_RAMP_EDIT;
          showRampEditScreen("Start", rampStartF, "Hz");
        } else if (rampEditIndex == 1) {
          state = MENU_RAMP_EDIT;
          showRampEditScreen("End", rampEndF, "Hz");
        } else if (rampEditIndex == 2) {
          state = MENU_RAMP_EDIT;
          showRampEditScreen("Rate", rampRate, "Hz/s");
        } else { // Run
          rampStartMillis = millis();
          runStartMillis  = rampStartMillis;
          targetHz        = rampStartF;
          saveSettings();
          lastUpdate      = 0;
          state           = RUN_RAMP;
        }
      }
      break;

    // -----------------------------
    // RAMP EDIT (start / end / rate)
// -----------------------------
    case MENU_RAMP_EDIT:
      if (rampEditIndex == 0) { // Start
        showRampEditScreen("Start", rampStartF, "Hz");
        if (btnPressed(BTN_UP)) {
          rampStartF += 0.1f;
          rampStartF = clampFreq(rampStartF);
          if (rampStartF > rampEndF) rampEndF = rampStartF;
          saveSettings();
          delay(120);
        }
        if (btnPressed(BTN_DOWN)) {
          rampStartF -= 0.1f;
          rampStartF = clampFreq(rampStartF);
          saveSettings();
          delay(120);
        }
      } else if (rampEditIndex == 1) { // End
        showRampEditScreen("End", rampEndF, "Hz");
        if (btnPressed(BTN_UP)) {
          rampEndF += 0.1f;
          rampEndF = clampFreq(rampEndF);
          saveSettings();
          delay(120);
        }
        if (btnPressed(BTN_DOWN)) {
          rampEndF -= 0.1f;
          rampEndF = clampFreq(rampEndF);
          if (rampEndF < rampStartF) rampStartF = rampEndF;
          saveSettings();
          delay(120);
        }
      } else { // Rate
        showRampEditScreen("Rate", rampRate, "Hz/s");
        if (btnPressed(BTN_UP)) {
          rampRate += 0.05f;
          if (rampRate > 1.0f) rampRate = 1.0f;
          saveSettings();
          delay(120);
        }
        if (btnPressed(BTN_DOWN)) {
          rampRate -= 0.05f;
          if (rampRate < 0.05f) rampRate = 0.05f;
          saveSettings();
          delay(120);
        }
      }

      if (btnClick(BTN_BACK) || btnClick(BTN_ENTER)) {
        state = MENU_RAMP_MENU;
        showRampMenu();
      }
      break;

    // -----------------------------
    // LIVE MENU (Target + Run)
// -----------------------------
    case MENU_LIVE_MENU:
      showLiveMenuScreen();

      if (btnClick(BTN_UP)) {
        menuIndex = (menuIndex + 1) % 2;  // toggle
      }
      if (btnClick(BTN_DOWN)) {
        menuIndex = (menuIndex + 1) % 2;
      }

      if (btnClick(BTN_BACK)) {
        state = MENU_MAIN;
        menuIndex = 0;
        showMainMenu();
      }

      if (btnClick(BTN_ENTER)) {
        if (menuIndex == 0) {
          state = MENU_LIVE_EDIT;
          showLiveEditScreen();
        } else {
          liveFreq      = clampFreq(liveFreq);
          targetHz       = liveFreq;
          runStartMillis = millis();
          lastUpdate     = 0;
          state          = RUN_LIVE;
        }
      }
      break;

    // -----------------------------
    // LIVE EDIT (target)
// -----------------------------
    case MENU_LIVE_EDIT:
      showLiveEditScreen();

      if (btnPressed(BTN_UP)) {
        liveFreq += 0.1f;
        liveFreq = clampFreq(liveFreq);
        saveSettings();
        showLiveEditScreen();
        delay(120);
      }
      if (btnPressed(BTN_DOWN)) {
        liveFreq -= 0.1f;
        liveFreq = clampFreq(liveFreq);
        saveSettings();
        showLiveEditScreen();
        delay(120);
      }

      if (btnClick(BTN_BACK) || btnClick(BTN_ENTER)) {
        menuIndex = 0;
        state = MENU_LIVE_MENU;
        showLiveMenuScreen();
      }
      break;

    // -----------------------------
    // RUN SET FREQUENCY
    // -----------------------------
    case RUN_SET:
      showRunSetScreen();

      // live adjust setFreq while running
      if (btnPressed(BTN_UP)) {
        setFreq += 0.1f;
        setFreq = clampFreq(setFreq);
        targetHz = setFreq;
        saveSettings();
        delay(120);
      }
      if (btnPressed(BTN_DOWN)) {
        setFreq -= 0.1f;
        setFreq = clampFreq(setFreq);
        targetHz = setFreq;
        saveSettings();
        delay(120);
      }

      // STOP → capture snapshot and show stopped
      if (btnClick(BTN_BACK) || btnClick(BTN_ENTER)) {
        targetHz    = 0;
        setDuty(0);
        stoppedTime = (millis() - runStartMillis) / 1000.0f;
        stoppedFreq = measuredHz;
        state       = RUN_SET_STOPPED;
        showStoppedSetScreen();
      }
      break;

    // -----------------------------
    // RUN RAMP
    // -----------------------------
    case RUN_RAMP: {
      // compute ramp target
      float t   = (millis() - rampStartMillis) / 1000.0f;
      float df  = rampEndF - rampStartF;
      float dir = (df >= 0.0f) ? 1.0f : -1.0f;
      float tot = fabs(df) / rampRate;

      float f;
      if (t <= tot) {
        f = rampStartF + dir * rampRate * t;
      } else {
        f = rampEndF;
      }
      f = clampFreq(f);
      targetHz = f;

      showRunRampScreen();

      // STOP
      if (btnClick(BTN_BACK) || btnClick(BTN_ENTER)) {
        targetHz    = 0;
        setDuty(0);
        stoppedTime = (millis() - runStartMillis) / 1000.0f;
        stoppedFreq = measuredHz;
        state       = RUN_RAMP_STOPPED;
        showStoppedRampScreen();
      }
    }
    break;

    // -----------------------------
    // RUN LIVE FREQUENCY
    // -----------------------------
    case RUN_LIVE:
      // Live adjust liveFreq while running
      if (btnPressed(BTN_UP)) {
        liveFreq += 0.1f;
        liveFreq = clampFreq(liveFreq);
        targetHz = liveFreq;
        saveSettings();
        delay(120);
      }
      if (btnPressed(BTN_DOWN)) {
        liveFreq -= 0.1f;
        liveFreq = clampFreq(liveFreq);
        targetHz = liveFreq;
        saveSettings();
        delay(120);
      }

      showRunLiveScreen();

      // STOP
      if (btnClick(BTN_BACK) || btnClick(BTN_ENTER)) {
        targetHz    = 0;
        setDuty(0);
        stoppedTime = (millis() - runStartMillis) / 1000.0f;
        stoppedFreq = measuredHz;
        state       = RUN_LIVE_STOPPED;
        showStoppedLiveScreen();
      }
      break;

    // -----------------------------
    // SET STOPPED SCREEN
    // -----------------------------
    case RUN_SET_STOPPED:
      showStoppedSetScreen();

      if (btnClick(BTN_BACK)) {
        state = MENU_MAIN;
        menuIndex = 0;
        showMainMenu();
      }
      if (btnClick(BTN_ENTER)) {
        state = RESTART_SET_CONFIRM;
        showRestartSetConfirm();
      }
      break;

    // -----------------------------
    // RAMP STOPPED SCREEN
    // -----------------------------
    case RUN_RAMP_STOPPED:
      showStoppedRampScreen();

      if (btnClick(BTN_BACK)) {
        state = MENU_MAIN;
        menuIndex = 0;
        showMainMenu();
      }
      if (btnClick(BTN_ENTER)) {
        state = RESTART_RAMP_CONFIRM;
        showRestartRampConfirm();
      }
      break;

    // -----------------------------
    // LIVE STOPPED SCREEN
    // -----------------------------
    case RUN_LIVE_STOPPED:
      showStoppedLiveScreen();

      if (btnClick(BTN_BACK)) {
        state = MENU_MAIN;
        menuIndex = 0;
        showMainMenu();
      }
      if (btnClick(BTN_ENTER)) {
        state = RESTART_LIVE_CONFIRM;
        showRestartLiveConfirm();
      }
      break;

    // -----------------------------
    // RESTART CONFIRM - SET
    // -----------------------------
    case RESTART_SET_CONFIRM:
      showRestartSetConfirm();

      if (btnClick(BTN_BACK)) {
        state = MENU_MAIN;
        menuIndex = 0;
        showMainMenu();
      }
      if (btnClick(BTN_ENTER)) {
        // restart SET mode with same setFreq
        setFreq        = clampFreq(setFreq);
        targetHz       = setFreq;
        runStartMillis = millis();
        state          = RUN_SET;
      }
      break;

    // -----------------------------
    // RESTART CONFIRM - RAMP
    // -----------------------------
    case RESTART_RAMP_CONFIRM:
      showRestartRampConfirm();

      if (btnClick(BTN_BACK)) {
        state = MENU_MAIN;
        menuIndex = 0;
        showMainMenu();
      }
      if (btnClick(BTN_ENTER)) {
        // restart RAMP with same start/end/rate
        rampStartMillis = millis();
        runStartMillis  = rampStartMillis;
        targetHz        = rampStartF;
        state           = RUN_RAMP;
      }
      break;

    // -----------------------------
    // RESTART CONFIRM - LIVE
    // -----------------------------
    case RESTART_LIVE_CONFIRM:
      showRestartLiveConfirm();

      if (btnClick(BTN_BACK)) {
        state = MENU_MAIN;
        menuIndex = 0;
        showMainMenu();
      }
      if (btnClick(BTN_ENTER)) {
        // restart LIVE mode with same liveFreq
        liveFreq       = clampFreq(liveFreq);
        targetHz       = liveFreq;
        runStartMillis = millis();
        state          = RUN_LIVE;
      }
      break;

    default:
      state = MENU_MAIN;
      menuIndex = 0;
      showMainMenu();
      break;
  }

  // 3) Optional: simple serial numeric control if you plug in a PC
  if (Serial.available()) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in == "STOP!" || in == "0") {
      targetHz = 0;
      setDuty(0);
      state = MENU_MAIN;
      menuIndex = 0;
      showMainMenu();
    } else {
      float val = in.toFloat();
      if (val > 0.0f) {
        targetHz       = clampFreq(val);
        runStartMillis = millis();
        state          = RUN_SET;
      }
    }
    lastUpdate = millis();
  }
}
