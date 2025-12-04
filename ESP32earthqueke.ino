#include <Arduino.h>
#include "driver/ledc.h"

#define PWM_PIN      18
#define PHOTO_PIN    34
#define LED_PIN      2

#define PWM_CHANNEL  LEDC_CHANNEL_0
#define PWM_TIMER    LEDC_TIMER_0
#define PWM_MODE     LEDC_HIGH_SPEED_MODE
#define PWM_RES      LEDC_TIMER_8_BIT
#define PWM_FREQ     5000  // PWM frequency for motor drive

volatile unsigned long pulseCount = 0;
unsigned long lastUpdate = 0;
float targetHz = 0.0;
float measuredHz = 0.0;
int duty = 0;

// --- Tuning parameters ---
const float KP = 20.0;          // proportional gain
const int MIN_DUTY = 15;        // minimum drive to overcome friction
const int MAX_DUTY = 100;       // maximum drive
const float LOW_SPEED_BOOST = 8.0; // extra gain below 0.2 Hz
const float FILTER_ALPHA = 0.6; // low-pass filter for measured speed

void IRAM_ATTR pulseISR() { pulseCount++; }

void setDuty(int dutyPercent) {
  if (dutyPercent < 0) dutyPercent = 0;
  if (dutyPercent > 100) dutyPercent = 100;
  uint32_t dutyValue = (255 * dutyPercent) / 100;
  ledc_set_duty(PWM_MODE, PWM_CHANNEL, dutyValue);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL);
  duty = dutyPercent;
}

void setup() {
  Serial.begin(9600);

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

  pinMode(PHOTO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), pulseISR, FALLING);

  setDuty(0);
  lastUpdate = millis();
}

void loop() {
  // --- Receive frequency commands ---
  if (Serial.available()) {
    String in = Serial.readStringUntil('\n');
    targetHz = in.toFloat();
    lastUpdate = millis();
  }

  // --- Measurement every 500 ms ---
  static unsigned long lastMeasure = 0;
  if (millis() - lastMeasure >= 500) {
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    // ✅ Corrected for 60 pulses per revolution
    float rawHz = (count / 60.0) / 0.5;
    measuredHz = FILTER_ALPHA * measuredHz + (1.0 - FILTER_ALPHA) * rawHz;
    lastMeasure = millis();

    // --- Control logic ---
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

    Serial.print("M:");
    Serial.println(measuredHz, 3);
  }

  // --- Failsafe ---
  if (millis() - lastUpdate > 3000) {
    targetHz = 0;
    setDuty(0);
  }
}
