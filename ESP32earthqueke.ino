#include <Arduino.h>
#include "driver/ledc.h"

// PWM parameters
#define PWM_PIN        15
#define PWM_CHANNEL    LEDC_CHANNEL_0
#define PWM_TIMER      LEDC_TIMER_0
#define PWM_MODE       LEDC_HIGH_SPEED_MODE
#define PWM_FREQ       500     // 500 Hz
#define PWM_RES        LEDC_TIMER_8_BIT

int currentDuty = 30; // start at 30%

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

  ledc_channel_config_t channelConf = {
    .gpio_num       = PWM_PIN,
    .speed_mode     = PWM_MODE,
    .channel        = PWM_CHANNEL,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = PWM_TIMER,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&channelConf);
}

void setDuty(int dutyPercent) {
  if (dutyPercent < 0) dutyPercent = 0;
  if (dutyPercent > 100) dutyPercent = 100;

  uint32_t dutyValue = (255 * dutyPercent) / 100;
  ledc_set_duty(PWM_MODE, PWM_CHANNEL, dutyValue);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int duty = input.toInt();
    setDuty(duty);
  }
}
