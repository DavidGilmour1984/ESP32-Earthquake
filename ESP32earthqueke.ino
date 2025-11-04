#include <Arduino.h>
#include "driver/ledc.h"

#define PWM_PIN        15
#define LED_PIN        2
#define PWM_CHANNEL_M  LEDC_CHANNEL_0
#define PWM_CHANNEL_L  LEDC_CHANNEL_1
#define PWM_TIMER      LEDC_TIMER_0
#define PWM_MODE       LEDC_HIGH_SPEED_MODE
#define PWM_FREQ       500
#define PWM_RES        LEDC_TIMER_8_BIT

unsigned long lastUpdate = 0;
int currentDuty = 0;

void setDuty(int dutyPercent) {
  if (dutyPercent < 0) dutyPercent = 0;
  if (dutyPercent > 100) dutyPercent = 100;
  currentDuty = dutyPercent;

  uint32_t dutyValue = (255 * dutyPercent) / 100;

  ledc_set_duty(PWM_MODE, PWM_CHANNEL_M, dutyValue);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL_M);

  ledc_set_duty(PWM_MODE, PWM_CHANNEL_L, dutyValue);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL_L);
}

void setup() {
  Serial.begin(9600);

  ledc_timer_config_t timerConf = {
    .speed_mode       = PWM_MODE,
    .duty_resolution  = PWM_RES,
    .timer_num        = PWM_TIMER,
    .freq_hz          = PWM_FREQ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timerConf);

  ledc_channel_config_t channelMotor = {
    .gpio_num       = PWM_PIN,
    .speed_mode     = PWM_MODE,
    .channel        = PWM_CHANNEL_M,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = PWM_TIMER,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&channelMotor);

  ledc_channel_config_t channelLED = {
    .gpio_num       = LED_PIN,
    .speed_mode     = PWM_MODE,
    .channel        = PWM_CHANNEL_L,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = PWM_TIMER,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&channelLED);

  setDuty(0);
  lastUpdate = millis();
}

void loop() {
  // Receive duty updates
  if (Serial.available()) {
    String in = Serial.readStringUntil('\n');
    int duty = in.toInt();
    setDuty(duty);
    lastUpdate = millis();
  }

  // Failsafe: if no command received for >300 ms -> stop
  if (millis() - lastUpdate > 300) {
    setDuty(0);
  }
}
