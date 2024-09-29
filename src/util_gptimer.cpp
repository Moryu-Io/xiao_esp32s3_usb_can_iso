#include <Arduino.h>
#include "util_gptimer.hpp"

static hw_timer_t* _gptimer;

void init_debug_timer(hw_timer_t* _p_timer) {
  // Enable timer
  _gptimer = _p_timer;
}

void start_debug_cnt() {
  timerStart(_gptimer); /* Enable timer */
}

void stop_debug_cnt() {
  timerStop(_gptimer); /* Disable timer */
}

uint32_t get_debug_cnt() {
  return timerRead(_gptimer);
}

