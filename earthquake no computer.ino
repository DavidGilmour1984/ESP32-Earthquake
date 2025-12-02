#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------------------------------------------------------------------
// OLED
// ---------------------------------------------------------------------------
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// ---------------------------------------------------------------------------
// BUTTONS
// ---------------------------------------------------------------------------
#define BTN_UP      25
#define BTN_DOWN    26
#define BTN_ENTER   33
#define BTN_BACK    32

// ---------------------------------------------------------------------------
// MOTOR + ENCODER
// ---------------------------------------------------------------------------
#define MOTOR_PWM_PIN  18
#define ENCODER_PIN    34

#define MOTOR_PWM_CHANNEL 0
#define MOTOR_PWM_FREQ    5000
#define MOTOR_PWM_RES     8

// ---------------------------------------------------------------------------
// LIMITS
// ---------------------------------------------------------------------------
const float MAX_ALLOWED_HZ = 3.3;

// ---------------------------------------------------------------------------
// CLOSED-LOOP MOTOR CONTROL VARIABLES
// ---------------------------------------------------------------------------
volatile unsigned long pulseCount = 0;

float targetHz = 0.0;
float measuredHz = 0.0;
unsigned long lastMeasure = 0;
int duty = 0;

const float KP = 20.0;
const int MIN_DUTY = 15;
const int MAX_DUTY = 100;
const float LOW_SPEED_BOOST = 8.0;
const float FILTER_ALPHA = 0.6;

// Stall detection
unsigned long stallStart = 0;
float lastMeasuredHz_forStall = 0;

// Debounce timer (menus only)
unsigned long lastMenuPress = 0;
const unsigned long MENU_DEBOUNCE = 500;

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------
void IRAM_ATTR encoderISR() { pulseCount++; }

void setDuty(int dp) {
    if (dp < 0) dp = 0;
    if (dp > 100) dp = 100;
    int raw = map(dp,0,100,0,255);
    ledcWrite(MOTOR_PWM_CHANNEL, raw);
    duty = dp;
}

// ---------------------------------------------------------------------------
// MENU VARIABLES
// ---------------------------------------------------------------------------
enum State {
  MAIN_MENU,

  FREQ_MENU,
  FREQ_EDIT,
  FREQ_RUN,

  RAMP_MENU,
  RAMP_EDIT,
  RAMP_RUN,

  TIMER_MENU,
  TIMER_UP_MENU,
  TIMER_UP_EDIT_FREQ,
  TIMER_UP_RUN,

  TIMER_DOWN_MENU,
  TIMER_DOWN_EDIT_FREQ,
  TIMER_DOWN_EDIT_TIME,
  TIMER_DOWN_RUN,

  LIVE_MENU,
  LIVE_RUN,

  STALL_SCREEN
};

State state = MAIN_MENU;

float freq_value = 1.0;
float ramp_start = 1.0;
float ramp_end   = 3.3;
float ramp_rate  = 1.0;

float timer_freq = 1.0;
float timer_time = 5.0;

int menu_index = 0;
int scroll_offset = 0;

unsigned long runStart;
unsigned long rampStart;

// ---------------------------------------------------------------------------
// HELPERS
// ---------------------------------------------------------------------------
bool pressed(int pin) { return digitalRead(pin) == LOW; }

bool menuPress(int pin) {
  if (!pressed(pin)) return false;
  if (millis() - lastMenuPress < MENU_DEBOUNCE) return false;
  lastMenuPress = millis();
  return true;
}

float clampHz(float x) {
  if (x < 0.1) x = 0.1;
  if (x > MAX_ALLOWED_HZ) x = MAX_ALLOWED_HZ;
  return x;
}

void resetCursor() { menu_index = 0; scroll_offset = 0; }

void updateScroll(int count) {
  if (menu_index < scroll_offset) scroll_offset = menu_index;
  if (menu_index >= scroll_offset + 2) scroll_offset = menu_index - 1;
}

void centerText(String s, int y) {
  int16_t x1,y1;
  uint16_t w,h;
  display.getTextBounds(s,0,0,&x1,&y1,&w,&h);
  display.setCursor((OLED_WIDTH - w)/2, y);
  display.print(s);
}

void renderScrollingMenu(String title, String items[], int count) {
  display.clearDisplay();
  centerText(title, 0);
  display.drawLine(0,16,128,16,WHITE);

  for (int i=0;i<2;i++) {
    int idx = scroll_offset + i;
    if (idx >= count) break;
    display.setCursor(0, 26 + i*20);
    display.print(idx == menu_index ? "> " : "  ");
    display.print(items[idx]);
  }

  display.display();
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(2);
  display.setTextColor(WHITE);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);

  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(MOTOR_PWM_PIN, MOTOR_PWM_CHANNEL);
  ledcWrite(MOTOR_PWM_CHANNEL, 0);

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, FALLING);

  display.clearDisplay();
  centerText("SHAKER",10);
  centerText("CTRL",34);
  display.display();
  delay(1200);
}

// ===========================================================================
// MAIN LOOP
// ===========================================================================
void loop() {

// ---------------------------------------------------------------------------
// CLOSED-LOOP SPEED MEASUREMENT
// ---------------------------------------------------------------------------
if (millis() - lastMeasure >= 500) {

    noInterrupts();
    unsigned long cnt = pulseCount;
    pulseCount = 0;
    interrupts();

    float rawHz = (cnt / 60.0) / 0.5;
    measuredHz = FILTER_ALPHA * measuredHz + (1.0-FILTER_ALPHA)*rawHz;

    lastMeasure = millis();

    // Stall detection
    if (targetHz > 0.1) {
        if (measuredHz > lastMeasuredHz_forStall + 0.02) {
            stallStart = millis(); // reset stall timer on improvement
            lastMeasuredHz_forStall = measuredHz;
        } else {
            if (millis() - stallStart > 1500 && duty > MIN_DUTY + 5) {
                targetHz = 0;
                setDuty(0);
                state = STALL_SCREEN;
            }
        }
    }

    if (targetHz > 0.0) {
        float error = targetHz - measuredHz;
        float gain = KP;
        if (targetHz < 0.2) gain += LOW_SPEED_BOOST;

        int newDuty = duty + error * gain;
        if (newDuty < MIN_DUTY) newDuty = MIN_DUTY;
        if (targetHz < 0.05) newDuty = MIN_DUTY + 3;
        if (newDuty > MAX_DUTY) newDuty = MAX_DUTY;

        setDuty(newDuty);
    } else {
        setDuty(0);
    }
}


// ===========================================================================
// STALL SCREEN
// ===========================================================================
if (state == STALL_SCREEN) {
    display.clearDisplay();
    centerText("STALL!",10);
    centerText("STOPPED",34);
    display.display();

    if (pressed(BTN_ENTER) || pressed(BTN_BACK)) {
        state = MAIN_MENU;
        resetCursor();
        delay(300);
    }
    return;
}


// ===========================================================================
// MAIN MENU
// ===========================================================================
if (state == MAIN_MENU) {

    String items[]={"Freq","Ramp","Timer","Live"};
    int count=4;

    updateScroll(count);
    renderScrollingMenu("MAIN",items,count);

    if (menuPress(BTN_UP))   {menu_index=(menu_index-1+count)%count;}
    if (menuPress(BTN_DOWN)) {menu_index=(menu_index+1)%count;}

    if (menuPress(BTN_ENTER)) {
        if (menu_index==0) state=FREQ_MENU;
        if (menu_index==1) state=RAMP_MENU;
        if (menu_index==2) state=TIMER_MENU;
        if (menu_index==3) state=LIVE_MENU;
        resetCursor();
    }
}


// ===========================================================================
// FREQUENCY MENU
// ===========================================================================
if (state == FREQ_MENU) {

    String items[]={"F:"+String(freq_value,1),"Go"};
    int count=2;

    updateScroll(count);
    renderScrollingMenu("FREQ",items,count);

    if (menuPress(BTN_UP))   {menu_index=(menu_index-1+count)%count;}
    if (menuPress(BTN_DOWN)) {menu_index=(menu_index+1)%count;}
    if (menuPress(BTN_BACK)) {state=MAIN_MENU;}

    if (menuPress(BTN_ENTER)) {
        if (menu_index==0) state=FREQ_EDIT;
        if (menu_index==1) {
            targetHz=freq_value;
            runStart=millis();
            stallStart=millis();
            lastMeasuredHz_forStall=measuredHz;
            state=FREQ_RUN;
        }
    }
}


// ===========================================================================
// FREQ EDIT (NO DEBOUNCE)
// ===========================================================================
if (state == FREQ_EDIT) {

    float original=freq_value;

    display.clearDisplay();
    centerText("Edit F",0);
    centerText(String(freq_value,1),28);
    display.display();

    if (pressed(BTN_UP))   {freq_value+=0.1; freq_value=clampHz(freq_value); delay(120);}
    if (pressed(BTN_DOWN)) {freq_value-=0.1; freq_value=clampHz(freq_value); delay(120);}

    if (pressed(BTN_ENTER)) {state=FREQ_MENU;}
    if (pressed(BTN_BACK))  {freq_value=original; state=FREQ_MENU;}
}


// ===========================================================================
// FREQ RUN
// ===========================================================================
if (state == FREQ_RUN) {

    display.clearDisplay();
    centerText("RUN F",0);
    centerText("T "+String(freq_value,1),20);
    centerText("A "+String(measuredHz,1),40);
    display.display();

    if (pressed(BTN_BACK)||pressed(BTN_ENTER)) {
        targetHz=0;
        state=FREQ_MENU;
    }
}


// ===========================================================================
// RAMP MENU
// ===========================================================================
if (state == RAMP_MENU) {

    String items[]={
      "St:"+String(ramp_start,1),
      "En:"+String(ramp_end,1),
      "Rt:"+String(ramp_rate,1),
      "Go"
    };
    int count=4;

    updateScroll(count);
    renderScrollingMenu("RAMP",items,count);

    if (menuPress(BTN_UP))   {menu_index=(menu_index-1+count)%count;}
    if (menuPress(BTN_DOWN)) {menu_index=(menu_index+1)%count;}
    if (menuPress(BTN_BACK)) {state=MAIN_MENU;}

    if (menuPress(BTN_ENTER)) {
        if (menu_index<3) state=RAMP_EDIT;
        if (menu_index==3) {
            rampStart=millis();
            lastMeasuredHz_forStall=measuredHz;
            stallStart=millis();
            state=RAMP_RUN;
        }
    }
}


// ===========================================================================
// RAMP EDIT (no debounce)
// ===========================================================================
if (state == RAMP_EDIT) {

    float *ptr =
      (menu_index==0)?&ramp_start:
      (menu_index==1)?&ramp_end:
                       &ramp_rate;

    float orig=*ptr;

    display.clearDisplay();
    centerText("Edit",0);
    centerText(String(*ptr,1),28);
    display.display();

    if (ptr!=&ramp_rate) {
      if (pressed(BTN_UP))   {*ptr+=0.1; *ptr=clampHz(*ptr); delay(120);}
      if (pressed(BTN_DOWN)) {*ptr-=0.1; *ptr=clampHz(*ptr); delay(120);}
    } else {
      if (pressed(BTN_UP))   {*ptr+=0.1; delay(120);}
      if (pressed(BTN_DOWN)) {*ptr-=0.1; if(*ptr<0.1)*ptr=0.1; delay(120);}
    }

    if (pressed(BTN_ENTER)) {state=RAMP_MENU;}
    if (pressed(BTN_BACK))  {*ptr=orig; state=RAMP_MENU;}
}


// ===========================================================================
// RAMP RUN
// ===========================================================================
if (state == RAMP_RUN) {

    float elapsed=(millis()-rampStart)/1000.0;
    float df=ramp_end-ramp_start;
    float tot=fabs(df)/ramp_rate;

    float f = (elapsed<=tot) ?
               (ramp_start+(df>0?1:-1)*ramp_rate*elapsed) :
                ramp_end;

    f=clampHz(f);
    targetHz=f;

    display.clearDisplay();
    centerText("RMP",0);
    centerText("T "+String(f,1),20);
    centerText("A "+String(measuredHz,1),40);
    display.display();

    if (pressed(BTN_BACK)||pressed(BTN_ENTER)) {
      targetHz=0; state=RAMP_MENU;
    }
}


// ===========================================================================
// TIMER MENU
// ===========================================================================
if (state == TIMER_MENU) {

    String items[]={"Up","Down"};
    int count=2;

    updateScroll(count);
    renderScrollingMenu("TIME",items,count);

    if (menuPress(BTN_UP))   {menu_index=(menu_index-1+count)%count;}
    if (menuPress(BTN_DOWN)) {menu_index=(menu_index+1)%count;}
    if (menuPress(BTN_BACK)) {state=MAIN_MENU;}

    if (menuPress(BTN_ENTER)) {
      if(menu_index==0) state=TIMER_UP_MENU;
      if(menu_index==1) state=TIMER_DOWN_MENU;
      resetCursor();
    }
}


// ===========================================================================
// TIMER UP MENU
// ===========================================================================
if (state == TIMER_UP_MENU) {

    String items[]={"F:"+String(timer_freq,1),"Go"};
    int count=2;

    updateScroll(count);
    renderScrollingMenu("UP",items,count);

    if (menuPress(BTN_UP))   {menu_index=(menu_index-1+count)%count;}
    if (menuPress(BTN_DOWN)) {menu_index=(menu_index+1)%count;}
    if (menuPress(BTN_BACK)) {state=TIMER_MENU;}

    if (menuPress(BTN_ENTER)) {
      if (menu_index==0) state=TIMER_UP_EDIT_FREQ;
      if (menu_index==1) {
        targetHz=timer_freq;
        runStart=millis();
        stallStart=millis();
        lastMeasuredHz_forStall=measuredHz;
        state=TIMER_UP_RUN;
      }
    }
}


// ===========================================================================
// TIMER UP EDIT (no debounce)
// ===========================================================================
if (state == TIMER_UP_EDIT_FREQ) {

    float orig=timer_freq;

    display.clearDisplay();
    centerText("F",0);
    centerText(String(timer_freq,1),28);
    display.display();

    if (pressed(BTN_UP))   {timer_freq+=0.1; timer_freq=clampHz(timer_freq); delay(120);}
    if (pressed(BTN_DOWN)) {timer_freq-=0.1; timer_freq=clampHz(timer_freq); delay(120);}

    if (pressed(BTN_ENTER)) {state=TIMER_UP_MENU;}
    if (pressed(BTN_BACK))  {timer_freq=orig; state=TIMER_UP_MENU;}
}


// ===========================================================================
// TIMER UP RUN
// ===========================================================================
if (state == TIMER_UP_RUN) {

    display.clearDisplay();
    centerText("UP",0);
    centerText("T "+String(timer_freq,1),20);
    centerText("A "+String(measuredHz,1),40);
    display.display();

    if (pressed(BTN_BACK)||pressed(BTN_ENTER)) {
      targetHz=0; state=TIMER_UP_MENU;
    }
}


// ===========================================================================
// TIMER DOWN MENU
// ===========================================================================
if (state == TIMER_DOWN_MENU) {

    String items[]={
      "F:"+String(timer_freq,1),
      "T:"+String(timer_time,1),
      "Go"
    };
    int count=3;

    updateScroll(count);
    renderScrollingMenu("DOWN",items,count);

    if (menuPress(BTN_UP))   {menu_index
