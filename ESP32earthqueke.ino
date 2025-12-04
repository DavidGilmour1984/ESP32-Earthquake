#include <Arduino.h>
#include "driver/ledc.h"

#define PWM_PIN      18
#define PHOTO_PIN    34

#define PWM_CHANNEL  LEDC_CHANNEL_0
#define PWM_TIMER    LEDC_TIMER_0
#define PWM_MODE     LEDC_HIGH_SPEED_MODE
#define PWM_RES      LEDC_TIMER_8_BIT
#define PWM_FREQ     5000

// REAL: 30 pulses per revolution
#define PULSES_PER_REV 30

volatile unsigned long pulseCount = 0;
float targetHz = 0.0;
float measuredHz = 0.0;
int duty = 0;

// ----- FILTERING -----
const float FILTER_ALPHA = 0.60;

// ----- CONTROL -----
const float KP = 20.0;
const int MIN_DUTY = 15;
const int MAX_DUTY = 100;
const float LOW_SPEED_BOOST = 8.0;

// ----- FLAGS -----
volatile bool hardStop = false;
unsigned long lastUpdate = 0;

// ---------------------------------------------------------------------------
// INTERRUPT
// ---------------------------------------------------------------------------
void IRAM_ATTR pulseISR() { pulseCount++; }

// ---------------------------------------------------------------------------
// SET DUTY
// ---------------------------------------------------------------------------
void setDuty(int d) {
  if (d < 0) d = 0;
  if (d > 100) d = 100;
  uint32_t dv = (255 * d) / 100;
  ledc_set_duty(PWM_MODE, PWM_CHANNEL, dv);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL);
  duty = d;
}

// ---------------------------------------------------------------------------
// SERIAL THREAD (CORE 0)
// ---------------------------------------------------------------------------
void serialTask(void *parameter) {
  for (;;) {
    if (Serial.available()) {
      String in = Serial.readStringUntil('\n');

      // HARD STOP OVERRIDE
      if (in == "STOP!") {
        hardStop = true;
        targetHz = 0;
        setDuty(0);
        continue;
      }

      hardStop = false;
      targetHz = in.toFloat();
      lastUpdate = millis();
    }
    vTaskDelay(1);
  }
}

// ---------------------------------------------------------------------------
// MOTOR CONTROL THREAD (CORE 1)
// ---------------------------------------------------------------------------
void controlTask(void *parameter) {

  unsigned long lastMeasure = 0;

  for (;;) {

    // HARD STOP ACTIVE
    if (hardStop) {
      setDuty(0);
      vTaskDelay(1);
      continue;
    }

    // 3-second timeout (your ramp smoothing behaviour)
    if (millis() - lastUpdate > 3000) {
      targetHz = 0;
    }

    // ----- MEASUREMENT WINDOW: 200 ms -----
    if (millis() - lastMeasure >= 200) {

      noInterrupts();
      unsigned long count = pulseCount;
      pulseCount = 0;
      interrupts();

      // convert to Hz: (pulses / pulses_per_rev) per 0.2 s → ×5
      float rawHz = (count / (float)PULSES_PER_REV) * 5.0;

      measuredHz =
        FILTER_ALPHA * measuredHz + (1 - FILTER_ALPHA) * rawHz;

      lastMeasure = millis();

      // STOP behaviour
      if (targetHz <= 0.0) {
        setDuty(0);
        Serial.print("M:");
        Serial.println(measuredHz, 3);
        continue;
      }

      // ----- PID exactly like your original -----
      float error = targetHz - measuredHz;
      float gain = KP;
      if (targetHz < 0.2) gain += LOW_SPEED_BOOST;

      int newDuty = duty + error * gain;

      if (newDuty < MIN_DUTY) newDuty = MIN_DUTY;
      if (targetHz < 0.05) newDuty = MIN_DUTY + 3;
      if (newDuty > MAX_DUTY) newDuty = MAX_DUTY;

      setDuty(newDuty);

      Serial.print("M:");
      Serial.println(measuredHz, 3);
    }

    vTaskDelay(1);
  }
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {

  Serial.begin(9600);

  pinMode(PHOTO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), pulseISR, FALLING);

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

  setDuty(0);
  lastUpdate = millis();

  // --------------------------------------------------------------
  // MOVE SERIAL LISTENER TO CORE 0
  // MOVE MOTOR CONTROL TO CORE 1
  // --------------------------------------------------------------
  xTaskCreatePinnedToCore(serialTask,  "SerialTask",  4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 1, NULL, 1);
}

// ---------------------------------------------------------------------------
void loop() {}
